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

// Variables
float pos = 0;
float xh = 0;

float tPulley = 0;
float tS = 0;

float motorCommand = 0;

// Selected virtual environment
char mode = '0';

void setup() {
  pinMode(MotIn1, OUTPUT);
  pinMode(MotIn2, OUTPUT);
  pinMode(pwm, OUTPUT);

  digitalWrite(MotIn1, HIGH);
  digitalWrite(MotIn2, LOW);

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

  }

  // Send command to motor, cast as an integer
  setMotor((int)motorCommand);
}