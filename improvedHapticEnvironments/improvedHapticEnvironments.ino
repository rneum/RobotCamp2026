//IF YOU USE THIS CODE, MAKE SURE THE PWMout WIRE IS MOVED TO PIN 9
//Write all serial messages in loop(), do not do serial communication in ISR

//Updated version of the hapkit's basic environments code using higher frequency PWM and ISR for haptic loop
//Ann Majewicz Fey and Ross Neuman 2/13/26


#include <Encoder.h>
#define ENCODER_OPTIMIZE_INTERRUPTS

// Pin Declarations
const int MotIn1 = 5;   //motor login pin 1
const int MotIn2 = 4;   //motor logic pin 2
const int PWMout = 9;   //PWM signal to control voltage to motor (pins 9 and 10 are on timer1 (16bit))

const int encA = 2;  //encoder output "A"
const int encB = 3;  //encoder output "B"

Encoder encoder(encB, encA);  //the encoder library uses the pulses from the two encoder outputs to get one 48 CPR angular positon

float encoderResolution = 48;  //encoder has 48 counts per revolution (CPR)
float pos = 0; //encoder position in ticks (initialized at zero)

// Change rh to reflect the handle you printed!
float rh = 8.5;  // length of your handle[cm]
float rp = 0.5;  // radius of motor pulley[cm]
float rs = 7.5;  // radius of sector[cm]

// Kinematics variables
float xh = 0;      // position of the handle [we will calculate it in cm in code below]
float xh_prev = 0; // previous handle position

float tPulley;     // motor pulley angle (radians)
float tS;          // sector angle (radians)

// velocity using 1st-order low-pass filter
float vh1 = 0;         // handle velocity
float vh1_prev = 0;    // previous velocity

float dt = 0.001;  // time steps of 1ms (1kHz control loop)

// define "r" based on cutoff frequency
float fc = 40;                //cutoff frequency [Hz]
float r = exp(-2*PI*fc*dt); //filter constant

float duty; //duty cycle for PWM (-400 to 400)

// enumerate the different possible haptic modes
enum hModes{
  ZERO,
  SPRING,
  DAMPER,
  SPRING_DAMPER,
  WALL,
  BUMP_VALLEY,
  TEXTURE
};

// pick which mode is there from the start
volatile hModes hapticMode = SPRING;
//volatile lets the compiler know this value may change due to outside factors (e.g. keyboard press)

void setup() {
  //****DO NOT CHANGE*****
  //set up PWM on Timer1 so it's faster and we don't hear the annoying hum anymore
  //this stuff is more in the weeds, please consult the ATmega328/P datasheet for more detailed info (p. ~170)
  //set phase and frequency correct pwm at 20kHz:
  TCCR1A = (1 << COM1A1); //clears OC1 on compare match when up-counting, sets on match when down-counting
  TCCR1B = (1 << WGM13) | (1 << CS10); //waveform generation mode 8 (phase/freq correct, ICR1 top), prescaler = 1
  ICR1 = 400; //ICR1 defines TOP in WGM8
  // the results of the above code is:
  // freq_PWM = freq_CPU / (2 * prescaler * TOP)
  // freq_PWM = 16,000,000 / (2 * 1 * 400) = 20,000 or 20 kHz

  // Set motor controlling pins to OUTPUT mode
  pinMode(MotIn1, OUTPUT);
  pinMode(MotIn2, OUTPUT);
  pinMode(PWMout, OUTPUT);

  // Initalize motor direction and set to 0 (no spin)
  digitalWrite(MotIn1, HIGH);
  digitalWrite(MotIn2, LOW);
  OCR1A = 0;  //write to this register for PWM later instead of "analogwrite"

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

  Serial.begin(115200);  //begin serial communication
}

void setMotor(int motCommand) {
  //motCommand range: -400 to 400
  //Limit values to this range
  if (motCommand > 400) {
    motCommand = 400;
  }
  if (motCommand < -400) {
    motCommand = -400;
  }

  //command motor speed and direction based on motCommand
  if(motCommand > 0) {
    digitalWrite(MotIn1, HIGH);
    digitalWrite(MotIn2, LOW);
    OCR1A = motCommand;
  }
  if(motCommand < 0) {
    digitalWrite(MotIn1, LOW);
    digitalWrite(MotIn2, HIGH);
    OCR1A = -motCommand;
  }
}

// write each haptic environment as a function that returns the desired duty cycle for PWM
// writing as functions can help make the code easier to parse (plus you can collapse them in the IDE)

// simple spring
float spring(float x){
  float k = 80;
  return -k * x;
}

// simple damper
float damper(float v){
  float b = 8;
  return -b * v;
}

// spring plus damping
float springDamper(float x, float v){
  float k = 80;
  float b = 7;
  return -k * x - b * v;
}

// wall with damping
float wall(float x, float v) {
  float x_wall = 2;   // wall location (cm)
  float k_wall = 200; // wall stiffness
  float b_wall = 8;  // wall damping (try increasing this to feel some instability vibrations)
  if (x > x_wall)
  {
    return -k_wall * (x - x_wall)  + -b_wall*(v);
  }
  else 
  {
    return 0;
  }
}

// Bump and Valley
float bumpValley(float x){
  float bump_location = 2;
  float bump_length = 3;
  float bump_height = 2;
  float bump_k = 200;
  float valley_location = -2;
  float valley_length = 3;
  float valley_height = 2;
  float valley_k = 200;

  if ((x <= (bump_location + bump_length / 2)) && (x >= (bump_location - bump_length / 2))) {
    // The handle is on the bump
    return bump_k * bump_height * cos(PI / bump_length * (x - bump_location)) * sin(PI / bump_length * (x - bump_location));
  } else if ((x <= (valley_location + valley_length / 2)) && (x >= (valley_location - valley_length / 2))) {
    // The handle is in the valley
    return -valley_k * valley_height * cos(PI / valley_length * (x - valley_location)) * sin(PI / valley_length * (x - valley_location));
  } else {
    // the handle is on flat ground
    return 0;
  }

}

//Texture using sinusoid
float texture(float x, float v){
  float w = 0.5; //width of damping area (cm)
  float b = 7;  //damping constant
  if (sin((PI * x) / w)> 0) {
    return -b * v;
  }
  else{
    return 0;
  }
}

// The "superloop" below can now just be used for things like serial communication
void loop() {

  Serial.println(duty); //print a line to the serial monitor

  // allow serial monitor to read for a new mode and switch accordingly
  if(Serial.available()){
    char s = Serial.read();
    if (s == '0') hapticMode = ZERO;
    if (s == '1') hapticMode = SPRING;
    if (s == '2') hapticMode = DAMPER;
    if (s == '3') hapticMode = SPRING_DAMPER;
    if (s == '4') hapticMode = WALL;
    if (s == '5') hapticMode = BUMP_VALLEY;
    if (s == '6') hapticMode = TEXTURE;
  }
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
  vh1 = r * vh1_prev + (1 - r) * (xh - xh_prev) / dt;

  // update "previous" terms for next loop
  xh_prev = xh;
  vh1_prev = vh1;

  // switch statement to determine which haptic mode we will use
  switch(hapticMode){
    case ZERO:
      duty = 0;
      encoder.write(0);
      pos = 0;
      break;
    case SPRING:
      duty = spring(xh);
      break;
    case DAMPER:
      duty = damper(vh1);
      break;
    case SPRING_DAMPER:
      duty = springDamper(xh, vh1);
      break;
    case WALL:
      duty = wall(xh, vh1);
      break;
    case BUMP_VALLEY:
      duty = bumpValley(xh);
      break;
    case TEXTURE:
      duty = texture(xh,vh1);
      break;
  }

  // command motor based on duty cycle (-400 to 400)
  setMotor(duty);
}
