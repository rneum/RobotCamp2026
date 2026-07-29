#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// =======================
// Pin Declarations
// =======================
const int MotIn1 = 5;
const int MotIn2 = 4;
const int pwm = 9;

const int encA = 2;
const int encB = 3;

Encoder encoder(encB, encA);

// =======================
// Mechanism Parameters
// =======================
const float encoderResolution = 48.0;

const float rh = 8.5;   // handle length [cm]
const float rp = 0.5;   // motor pulley radius [cm]
const float rs = 7.5;   // sector radius [cm]

// =======================
// State Variables
// =======================
volatile float pos = 0;

volatile float xh = 0;
volatile float xh_prev = 0;

volatile float vh = 0;
volatile float vh_prev = 0;

float tPulley = 0;
float tS = 0;

// =======================
// Timing / Filtering
// =======================
const float dt = 0.001;      // 1 kHz

const float fc = 20.0;
const float r = exp(-2.0 * PI * fc * dt);

// =======================
// Motor Command
// =======================
volatile float motorCommand = 0;

// =======================
// Environment Selection
// =======================
// 0 = Reset
// 1 = Spring
// 2 = Wall
// 3 = Damper
// 4 = Nonlinear Friction
// 5 = Hard Surface
// 6 = Mass-Spring-Damper
char mode = '0';

// =======================
// Virtual Wall Parameters
// =======================
const float x_wall = 2.0;     // cm
const float k_wall = 200.0;

// =======================
// Nonlinear Friction
// =======================
const float b_nonlinear = 10.0;
const float dxh_threshold = 0.05;
const float b_scaling = 2000.0;

// =======================
// Hard Surface
// =======================
const float E_Value = 2.71828;

const float Decay_Sine_Amplitude_Scale = 5.0;
const float Decay_Sine_Amplitude_Max = 10.0;

float Decay_Sine_Amplitude = 0;

const float Decay_Sine_Frequency = 150.0;
const float Decay_Sine_Exponent = -20.0;

int Decay_In_Wall_Flag = 0;
float Decay_Sine_Time = 0;

// =======================
// Mass-Spring-Damper
// =======================
float m_msd = 3.0;
float b_msd = 1.0;
float k_msd = 20.0;
float k_user = 100.0;

float x_msd = 0.5;
float v_msd = 0;
float a_msd = 0;

float xi_msd = 0.5;

// ===================================================
// Setup
// ===================================================
void setup() {

  pinMode(MotIn1, OUTPUT);
  pinMode(MotIn2, OUTPUT);
  pinMode(pwm, OUTPUT);

  digitalWrite(MotIn1, HIGH);
  digitalWrite(MotIn2, LOW);

  Serial.begin(115200);

  // --------------------------
  // Timer2 setup for 1 kHz ISR
  // --------------------------
  cli();

  TCCR2A = 0;
  TCCR2B = 0;

  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22);

  OCR2A = 249;

  TIMSK2 |= (1 << OCIE2A);

  sei();

  Serial.println("Longhorn Hapkit Ready");
  Serial.println("0=Reset");
  Serial.println("1=Spring");
  Serial.println("2=Wall");
  Serial.println("3=Damper");
  Serial.println("4=Nonlinear Friction");
  Serial.println("5=Hard Surface");
  Serial.println("6=Mass-Spring-Damper");
}

// ===================================================
// Motor Driver
// ===================================================
void setMotor(int command) {

  if (command > 255) command = 255;
  if (command < -255) command = -255;

  if (command >= 0) {
    digitalWrite(MotIn1, HIGH);
    digitalWrite(MotIn2, LOW);
    analogWrite(pwm, command);
  }
  else {
    digitalWrite(MotIn1, LOW);
    digitalWrite(MotIn2, HIGH);
    analogWrite(pwm, -command);
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
    // Virtual Wall
    // =======================
    case '2':
    {
      if (xh > x_wall) {
        motorCommand = -k_wall * (xh - x_wall);
      }
      else {
        motorCommand = 0;
      }
      break;
    }

    // =======================
    // Damper
    // =======================
    case '3':
    {
      const float b = 8.0;
      motorCommand = -b * vh;
      break;
    }

    // =======================
    // Nonlinear Friction
    // =======================
    case '4':
    {
      if ((vh < dxh_threshold) &&
          (vh > -dxh_threshold)) {

        motorCommand = -b_nonlinear * vh;
      }
      else {

        if (vh < 0) {
          motorCommand =
            b_nonlinear * dxh_threshold
            + (-b_nonlinear / b_scaling)
            * (vh + dxh_threshold);
        }
        else {
          motorCommand =
            -b_nonlinear * dxh_threshold
            + (-b_nonlinear / b_scaling)
            * (vh - dxh_threshold);
        }
      }

      break;
    }

    // =======================
    // Hard Surface
    // =======================
    case '5':
    {
      if (xh > x_wall) {

        if (Decay_In_Wall_Flag == 0) {

          Decay_Sine_Time = 0;

          Decay_Sine_Amplitude =
            Decay_Sine_Amplitude_Scale * fabs(vh);

          if (Decay_Sine_Amplitude >
              Decay_Sine_Amplitude_Max) {

            Decay_Sine_Amplitude =
              Decay_Sine_Amplitude_Max;
          }

          Decay_In_Wall_Flag = 1;
        }

        motorCommand =
          -exp(Decay_Sine_Exponent *
               Decay_Sine_Time)
          * Decay_Sine_Amplitude
          * sin(2.0 * PI *
                Decay_Sine_Frequency *
                Decay_Sine_Time);

        motorCommand +=
          -k_wall * (xh - x_wall);

        Decay_Sine_Time += dt;
      }
      else {

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
    case '6':
    {
      v_msd += a_msd * dt;
      x_msd += v_msd * dt;

      if (xh > x_msd) {

        a_msd =
          (k_user * (xh - x_msd)
          + k_msd * (xi_msd - x_msd)
          - b_msd * v_msd)
          / m_msd;

        motorCommand =
          k_user * (x_msd - xh);
      }
      else {

        motorCommand = 0;

        a_msd =
          (k_msd * (xi_msd - x_msd)
          - b_msd * v_msd)
          / m_msd;
      }

      break;
    }
  }

  Serial.print(xh);
  Serial.print(",");
  Serial.print(vh);
  Serial.print(",");
  Serial.println(motorCommand);

  delay(5);
}

// ===================================================
// 1 kHz Haptic Loop ISR
// ===================================================
ISR(TIMER2_COMPA_vect)
{
  pos = encoder.read();

  tPulley =
    (pos / encoderResolution)
    * 2.0 * PI;

  tS = (tPulley * rp) / rs;

  xh =
    (rh * rp * tPulley)
    / rs;

  vh =
    r * vh_prev +
    (1.0 - r)
    * (xh - xh_prev)
    / dt;

  xh_prev = xh;
  vh_prev = vh;

  setMotor((int)motorCommand);
}