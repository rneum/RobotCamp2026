#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// Pin Declarations
const int MotIn1 = 5;
const int MotIn2 = 4;
const int pwm = 9;

const int encA = 2;
const int encB = 3;

Encoder encoder(encB, encA);

// Encoder resolution
const float encoderResolution = 48.0;

// Mechanism dimensions [cm]
const float rh = 8.5;   // handle length
const float rp = 0.5;   // motor pulley radius
const float rs = 7.5;   // sector radius

// Variables for kinematics
float pos = 0;      // encoder position (radians)
float xh = 0;       // position of the handle [we will calculate it in cm in code below]
float xh_prev = 0;  // previous handle position

// velocity using 1st-order low-pass filter
float vh = 0;         // handle velocity
float vh_prev = 0;    // previous velocity
float dt = 0.001;  // time steps of 1ms (1kHz control loop)

// define "r" based on cutoff frequency
float fc = 20;                //cutoff frequency [Hz]
float r = exp(-2*PI*fc*dt);   //filter constant

float tPulley = 0;  //motor pulley angle (radians)
float tS = 0;       //sector angle (radians)

float motorCommand = 0;

// Selected virtual environment
char mode = '0';

void setup() {
  pinMode(MotIn1, OUTPUT);
  pinMode(MotIn2, OUTPUT);
  pinMode(pwm, OUTPUT);

  digitalWrite(MotIn1, HIGH);
  digitalWrite(MotIn2, LOW);

  // Configure control interrupt with Timer2 to make 1kHz control loop
  cli();  //disable interrupts

  TCCR2A = 0;
  TCCR2B = 0;
  
  // waveform generation mode 2 -- clear timer on compare match (CTC) mode
  TCCR2A |= (1 << WGM21);

  // Prescaler = 64
  TCCR2B |= (1 << CS22);

  OCR2A = 249;
  // freq = freq_cpu / (prescaler * (1 + OCR2A))
  // freq = 16,000,000 / (64 * (1 + 249)) = 1000 Hz or 1 kHz
  // no multiplication by factor of 2 in denominator as Timer2 is single-slope (Timer1 is dual-slope)

  // enable interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();   // enable interrupts


  Serial.begin(115200);
}

void setMotor(int command) {

  // Limit to PWM range
  if (command > 255) command = 255;
  if (command < -255) command = -255;

  if (command >= 0) {
    digitalWrite(MotIn1, HIGH);
    digitalWrite(MotIn2, LOW);
    analogWrite(pwm, command);
  } else {
    digitalWrite(MotIn1, LOW);
    digitalWrite(MotIn2, HIGH);
    analogWrite(pwm, -command);
  }
}

void loop() {

  // Read encoder
  pos = encoder.read();

  // Kinematics
  tPulley = (pos / encoderResolution) * 2.0 * PI;
  tS = (tPulley * rp) / rs;

  // Handle displacement [cm]
  xh = (rh * rp * tPulley) / rs;

  // Update mode if a serial command arrives
  if (Serial.available()) {
    mode = Serial.read();
  }

  // Compute force continuously
  switch (mode) {

    // No movement/ zeroing
    case '0':
    {
      motorCommand = 0;
      encoder.write(0);
      pos = 0;
      break;
    }

    // Virtual spring
    case '1':
    {
      const float k = 20.0;
      motorCommand = -k * xh;
      break;
    }

    // Virtual wall
    case '2':
    {
      const float x_wall = 2.0;
      const float k_wall = 200.0;

      if (xh > x_wall) {
        motorCommand = -k_wall * (xh - x_wall);
      } else {
        motorCommand = 0;
      }
      break;
    }

    // Virtual damper
    case '3':
    {
      const float b = 8;      //damping coefficient
      motorCommand = -b * vh;  //multiplied by velocity
      break;
    }

  }

Serial.println(xh);

}

//Haptic loop running at 1kHz using an interrupt service routine (ISR) for reliable calling
ISR(TIMER2_COMPA_vect) {

  // read encoder
  pos = encoder.read();

  tPulley = (pos / encoderResolution) * 2 * PI; //angular position of motor pulley in radians

  tS = (tPulley * rp) / rs; //angular position of sector pulley in radians

  //determine handle position in cm
  xh = (rh * rp * tPulley) / rs;
  
  // handle velocity (cm/s) using a 1st-order low pass filter with constant r defined in setup()
  vh = r * vh_prev + (1 - r) * (xh - xh_prev) / dt;

  // update "previous" terms for next loop
  xh_prev = xh;
  vh_prev = vh;

  // command motor based on duty cycle (-255 to 255)
  setMotor(int(motorCommand));
}