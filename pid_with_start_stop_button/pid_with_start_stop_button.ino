#define ENCODER_OPTIMIZE_INTERRUPTS

#include <AccelStepper.h>
#include <PID_v1.h>
#include <Encoder.h>

bool on = false;

// Pin Definitions
const int dirPin = 8;         // Direction pin for the stepper motor
const int pulsePin = 9;       // Pulse pin for the stepper motor
const int encoderPinA = 2;    // Encoder pin A
const int encoderPinB = 3;    // Encoder pin B

const int stop_button = 10;  // STOP BUTTON
const int start_button = 11;  // START Button
const int START_LED = 7;   
const int OFF_LED = 6;   
const int POLE_LED = 4;     
const int PID_LED = 5;  



// Stepper Motor and Driver Settings
const bool inverted = true;
const int stepMode = 3;
const int stepModes[6][5] = {
  {5, 1, 0, 0, 0},
  {10, 2, 1, 0, 0},
  {20, 4, 0, 1, 0},
  {40, 8, 1, 1, 0},
  {80, 16, 0, 0, 1},
  {160, 32, 1, 1, 1}
};

double stepsPerMM;
int outputDir;
AccelStepper stepper(AccelStepper::DRIVER, pulsePin, dirPin);

// PID Control Variables
double setPoint, input, output, scaleFactor;
double kp, ki, kd;
PID pid(&input, &output, &setPoint, kp, ki, kd, DIRECT);

// Encoder Setup
Encoder encoder(encoderPinA, encoderPinB);

void setup() {
  // Set PID parameters based on inversion status
  if (inverted) {
    outputDir = -1;
    kp = 55.00;
    ki = 100.00;
    kd = 0;
    scaleFactor = -1 / 37.5;

    
  } else {
    outputDir = -1;
    kp = 55.00;
    ki = 600.00;
    kd = 0.9;
    scaleFactor = -1 / 100;
  }

  digitalWrite(OFF_LED, HIGH);
  digitalWrite(PID_LED, HIGH);
  pid.SetTunings(kp, ki, kd);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-1200, 1200);

  stepsPerMM = stepModes[stepMode][0];
  stepper.setMaxSpeed(20000);       // Set a reasonable max speed
  stepper.setMinPulseWidth(5);

  pinMode(start_button, INPUT_PULLUP);
  pinMode(stop_button, INPUT_PULLUP);
  pinMode(START_LED, OUTPUT);
  pinMode(OFF_LED, OUTPUT);
  pinMode(POLE_LED, OUTPUT);
  pinMode(PID_LED, OUTPUT);
 

  setPoint = 0;
  encoder.write(0);

  // Wait for the encoder to reach zero position
  while (encoder.read() < 0) {
    // NOOP
  }
}

void loop() {
  // Read encoder count and calculate angle
  int count = encoder.read();
  double angle = count * (360.0 / 2000.0); // Convert encoder count to angle

  if (digitalRead(start_button) == LOW){
    on = true;
    digitalWrite(OFF_LED, LOW);
    digitalWrite(START_LED, HIGH);
  }
  else if (digitalRead(stop_button) == LOW){
    on = false;
    digitalWrite(OFF_LED, HIGH);
    digitalWrite(START_LED, LOW);
  }
  
  if (on == true){
      // Check if the angle exceeds -30 or 30 degrees
    if (angle < -20|| angle > 20) {
      stepper.setSpeed(0);  // Stop the motor
      stepper.runSpeed();   // Apply the stop command
      return;               // Exit the loop early
    }
  
    // Continue with PID control if angle is within -30 to 30 range
    input = scaleFactor * stepper.currentPosition() / stepsPerMM + 200 * sin(angle * (PI / 180));
    pid.Compute();
  
    // Set motor speed and direction based on PID output
    stepper.setSpeed(outputDir * output);
    stepper.runSpeed();
    }
  }
 
