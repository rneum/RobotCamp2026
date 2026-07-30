#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

//PWM MUST BE ON PIN 9 for this version to run (PWM is configured on Timer1 which controls pin 9 and 10 pwm)

//This code is equivalent to all_environments.ino except with fast PWM enabled
//PWM commands are now scaled to 400 instead of 255

/*
=======================
Haptic Modes:
=======================
0 - Reset
1 - Spring
2 - Wall
3 - Wall with Damping
4 - Damper
5 - Nonlinear Friction
6 - Bump
7 - Better Bump
8 - Bump and Valley
9 - Hard Surface
A - Mass Spring Damper
========================
========================
*/


// Pin Declarations
const int MotIn1 = 5;
const int MotIn2 = 4;
const int pwm = 9;

const int encA = 2;
const int encB = 3;

Encoder encoder(encB, encA);

// Encoder / Geometry
const float encoderResolution = 48.0;

const float rh = 8.5;  // handle length [cm]
const float rp = 0.5;  // motor pulley radius [cm]
const float rs = 7.5;  // sector radius [cm]

// State Variables
volatile float pos = 0;

volatile float xh = 0;
volatile float xh_prev = 0;

volatile float vh = 0;
volatile float vh_prev = 0;

float tPulley = 0;
float tS = 0;

// Timing / Filtering
const float dt = 0.001;
const float fc = 20.0;
const float r = exp(-2.0 * PI * fc * dt);

// Force Command
volatile float motorCommand = 0;

// Environment Selection
char mode = '0';


// Evironment variables that get reset with 0 mode
float x_msd = 0.5;
float v_msd = 0;
float a_msd = 0;

float xi_msd = 0.5;

float Decay_Sine_Amplitude = 0;
int Decay_In_Wall_Flag = 0;
float Decay_Sine_Time = 0;
// ===================================================
// Setup
// ===================================================
void setup() {
  //initialize motor outputs
  pinMode(MotIn1, OUTPUT);
  pinMode(MotIn2, OUTPUT);
  pinMode(pwm, OUTPUT);

  //initialize motor direction
  digitalWrite(MotIn1, HIGH);
  digitalWrite(MotIn2, LOW);



  Serial.begin(115200);

  //set up PWM on Timer1 so it's faster and we don't hear the annoying hum anymore
  //this stuff is more in the weeds, please consult the ATmega328/P datasheet for more detailed info (p. ~170)
  //set phase and frequency correct pwm at 20kHz:
  TCCR1A = (1 << COM1A1);               //non-inverting PWM on OC1A (pin 9)
  TCCR1B = (1 << WGM13) | (1 << CS10);  //waveform generation mode 8 (phase/freq correct, ICR1 top), prescaler = 1
  ICR1 = 400;                           //ICR1 defines TOP in WGM8
  OCR1A = 0;                            //initialize at 0 duty cycle
  // the results of the above code is:
  // freq_PWM = freq_CPU / (2 * prescaler * TOP)
  // freq_PWM = 16,000,000 / (2 * 1 * 400) = 20,000 or 20 kHz

  // Setup 1kHz ISR
  cli();

  TCCR2A = 0;
  TCCR2B = 0;

  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22);

  OCR2A = 249;

  TIMSK2 |= (1 << OCIE2A);

  sei();
}

// ===================================================
// Motor Driver
// ===================================================
void setMotor(int command) {

  if (command > 400) command = 400;
  if (command < -400) command = -400;

  if (command >= 0) {
    digitalWrite(MotIn1, HIGH);
    digitalWrite(MotIn2, LOW);
    OCR1A = command;
  } else {
    digitalWrite(MotIn1, LOW);
    digitalWrite(MotIn2, HIGH);
    OCR1A = -command;
  }
}

// ===================================================
// Main Loop
// ===================================================
void loop() {

  if (Serial.available()) {
    mode = Serial.read();
  }

  switch (mode) {

    // =======================
    // Reset
    // =======================
    case '0':
      {
        motorCommand = 0;

        encoder.write(0);
        pos = 0;

        x_msd = xi_msd;
        v_msd = 0;
        a_msd = 0;

        Decay_In_Wall_Flag = 0;
        Decay_Sine_Time = 0;
        Decay_Sine_Amplitude = 0;

        break;
      }

    // =======================
    // Spring
    // =======================
    case '1':
      {
        const float k = 80.0;
        motorCommand = -k * xh;
        break;
      }

    // =======================
    // Wall
    // =======================
    case '2':
      {
        const float x_wall = 2.0;
        const float k_wall = 200.0;
        if (xh > x_wall)
          motorCommand = -k_wall * (xh - x_wall);
        else
          motorCommand = 0;

        break;
      }

    // =======================
    // Wall with Damping
    // =======================
    case '3':
      {
        const float x_wall_damped = 2.0;
        const float k_wall_damped = 200.0;
        const float b_wall_damped = 5.0;

        if (xh > x_wall_damped) {
          motorCommand =
            -k_wall_damped * (xh - x_wall_damped)
            - b_wall_damped * vh;
        } else {
          motorCommand = 0;
        }

        break;
      }

    // =======================
    // Damper
    // =======================
    case '4':
      {
        const float b = 8.0;
        motorCommand = -b * vh;
        break;
      }

    // =======================
    // Nonlinear Friction
    // =======================
    case '5':
      {
        const float Fc = 80.0;
        const float vth = 0.2;
        const float k = Fc / vth;

        if (vh > vth) {
          motorCommand = -Fc;
        } else if (vh < -vth) {
          motorCommand = Fc;
        } else {
          motorCommand = -k * vh;
        }

        break;
      }

    // =======================
    // Bump
    // =======================
    case '6':
      {
        float bump_location = 2.0;
        float bump_width = 1.0;
        float bump_force = 150.0;

        if (abs(xh - bump_location) < bump_width / 2.0) {
          motorCommand = -bump_force;
        } else {
          motorCommand = 0;
        }

        break;
      }

    // =======================
    // Better Bump
    // =======================
    case '7':
      {
        float bump_location = 2.0;
        float bump_width = 1.0;
        float bump_force = 150.0;

        if ((xh > bump_location - bump_width / 2.0) && (xh < bump_location)) {

          motorCommand = -bump_force;
        } else if ((xh >= bump_location) && (xh < bump_location + bump_width / 2.0)) {

          motorCommand = bump_force;
        } else {

          motorCommand = 0;
        }

        break;
      }

    // =======================
    // Bump and Valley
    // =======================
    case '8':
      {
        float bump_location = 2.0;
        float bump_length = 3.0;
        float bump_height = 2.0;
        float bump_k = 200.0;

        float valley_location = -2.0;
        float valley_length = 3.0;
        float valley_height = 2.0;
        float valley_k = 150.0;

        if ((xh <= (bump_location + bump_length / 2.0)) && (xh >= (bump_location - bump_length / 2.0))) {

          motorCommand =
            bump_k * bump_height * cos(PI / bump_length * (xh - bump_location)) * sin(PI / bump_length * (xh - bump_location));
        } else if ((xh <= (valley_location + valley_length / 2.0)) && (xh >= (valley_location - valley_length / 2.0))) {

          motorCommand =
            -valley_k * valley_height * cos(PI / valley_length * (xh - valley_location)) * sin(PI / valley_length * (xh - valley_location));
        } else {

          motorCommand = 0;
        }

        break;
      }

    // =======================
    // Hard Surface
    // =======================
    case '9':
      {
        const float x_wall_hard = 2.0;
        const float k_wall_hard = 200.0;

        const float Decay_Sine_Amplitude_Scale = 5.0;
        const float Decay_Sine_Amplitude_Max = 30.0;

        const float Decay_Sine_Frequency = 200.0;
        const float Decay_Sine_Exponent = -20.0;

        if (xh > x_wall_hard) {

          if (Decay_In_Wall_Flag == 0) {

            Decay_Sine_Time = 0;

            Decay_Sine_Amplitude =
              Decay_Sine_Amplitude_Scale * fabs(vh);

            if (Decay_Sine_Amplitude > Decay_Sine_Amplitude_Max) {
              Decay_Sine_Amplitude =
                Decay_Sine_Amplitude_Max;
            }

            Decay_In_Wall_Flag = 1;
          }

          motorCommand =
            -exp(Decay_Sine_Exponent * Decay_Sine_Time)
            * Decay_Sine_Amplitude
            * sin(2.0 * PI * Decay_Sine_Frequency * Decay_Sine_Time);

          motorCommand +=
            -k_wall_hard * (xh - x_wall_hard);

          Decay_Sine_Time += dt;
        } else {

          Decay_In_Wall_Flag = 0;
          Decay_Sine_Time = 0;
          Decay_Sine_Amplitude = 0;
          motorCommand = 0;
        }

        break;
      }

    // =======================
    // Mass Spring Damper
    // =======================
    // *try zeroing your handle all the way to the left before running this one for better effect
    case 'A':
      {
        float m_msd = 1.0;    //mass
        float b_msd = 0.2;    //damping coefficient
        float k_msd = 50.0;   //spring stiffness
        float k_user = 100.0; //collision stiffness

        // simple integration to get velocity and position
        v_msd += a_msd * dt;
        x_msd += v_msd * dt;

        if (xh > x_msd) {
          //contact
          a_msd =
            (k_user * (xh - x_msd)
             + k_msd * (xi_msd - x_msd)
             - b_msd * v_msd)
            / m_msd;

          motorCommand =
            k_user * (x_msd - xh);
        } else {
          //no contact
          motorCommand = 0;

          a_msd =
            (k_msd * (xi_msd - x_msd)
             - b_msd * v_msd)
            / m_msd;
        }

        break;
      }
  }

  Serial.print("xh: ");
  Serial.print(xh);
  Serial.print(" ");
  Serial.print("PWM: ");
  Serial.println(motorCommand);

  delay(5);
}

// ===================================================
// 1 kHz Interrupt Service Routine
// ===================================================

ISR(TIMER2_COMPA_vect) {

  pos = encoder.read();

  tPulley =
    (pos / encoderResolution) * 2.0 * PI;

  tS =
    (tPulley * rp) / rs;

  xh =
    (rh * rp * tPulley) / rs;

  vh =
    r * vh_prev + (1.0 - r) * (xh - xh_prev) / dt;

  xh_prev = xh;
  vh_prev = vh;

  setMotor((int)motorCommand);
}