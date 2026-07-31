#include <Encoder.h>
#define ENCODER_OPTIMIZE_INTERUPTS

const int MotIn1 = 5;
const int MotIn2 = 4;
const int pwm = 9;

const int encA = 2;
const int encB = 3;

const int epsilon = 1;

int speed = 0;
float deg = 0.0;
double dampeningThreshold = 7.5;

union FloatConverter {
  byte b[4];
  float f;
} input;

Encoder encoder(encA, encB);


void setup() {

  //set up PWM on Timer1 so it's faster and we don't hear the annoying hum anymore
  //this stuff is more in the weeds, please consult the ATmega328/P datasheet for more detailed info (p. ~170)
  //set phase and frequency correct pwm at 20kHz:
  TCCR1A = (1 << COM1A1);               //non-inverting PWM on OC1A (pin 9)
  TCCR1B = (1 << WGM13) | (1 << CS10);  //waveform generation mode 8 (phase/freq correct, ICR1 top), prescaler = 1
  ICR1 = 400;                           //ICR1 defines TOP in WGM8
  OCR1A = 0; 
  // the results of the above code is:
  // freq_PWM = freq_CPU / (2 * prescaler * TOP)
  // freq_PWM = 16,000,000 / (2 * 1 * 400) = 20,000 or 20 kHz
  pinMode(MotIn1, OUTPUT);
  pinMode(MotIn2, OUTPUT);
  pinMode(pwm, OUTPUT);

  digitalWrite(MotIn1, LOW);
  digitalWrite(MotIn2, HIGH);
  Serial.begin(115200);

  //setMotorSpeed(100);

  //Serial.println("mkdir -p /home/horns758/Desktop/ArduinoTriggered");
}

void loop() {

  //Serial.println(encoder.read() / 48.0 * 360.0 * 5.0 / 85.0);
  //Serial.println("Hello World!");
  //delay(1000);
  if (Serial.available() >=4) {
    for (int i = 0; i< 4; i++) {
      input.b[i] = Serial.read();
    }
    //Serial.print("Redefined deg to ");
    Serial.println(input.f);
    deg = input.f;
  }
  
  //Serial.println(deg,4);
  runToPosition(-30 *deg, 210);
}

void setMotorSpeed(int speed) {
  if (speed < -255) {
    speed = -255;
  }
  if (speed > 255) {
    speed = 255;
  }
  if (speed == 0) {
    analogWrite(pwm,0);
    return;
  }
  digitalWrite(MotIn1, speed > 0 ? HIGH : LOW);
  digitalWrite(MotIn2, speed > 0 ? LOW : HIGH);
  analogWrite(pwm, speed > 0 ? speed : -speed);
}

void runToPosition(float deg, int speed) {
  if (abs((encoder.read() / 48.0 * 360.0 * 5.0 / 85.0) - deg) > epsilon) {
    double diff = abs((encoder.read() / 48.0 * 360.0 * 5.0 / 85.0) - deg);
    //Serial.println(diff);
    setMotorSpeed((encoder.read() / 48.0 * 360.0 * 5.0 / 85.0) < deg ? -speed : speed);
    //setMotorSpeed( (encoder.read() / 48.0 * 360.0 * 5.0 / 85.0) < deg ? -speed*255*(diff < dampeningThreshold ? sqrt(diff)/dampeningThreshold : 1.0): speed*255*(diff < dampeningThreshold ? sqrt(diff)/dampeningThreshold : 1.0));

  } else {
    setMotorSpeed(0);
  }

}

