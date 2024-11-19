#include <AccelStepper.h>
#include <Encoder.h>

// Pin Definitions
const int dirPin = 8;         // Direction pin for the stepper motor
const int pulsePin = 9;       // Pulse pin for the stepper motor
const int limitSwitch1 = 12;  // First limit switch
const int limitSwitch2 = 13;  // Second limit switch
const int encoderPinA = 2;    // Encoder pin A
const int encoderPinB = 3;    // Encoder pin B

// Stepper Motor Setup
AccelStepper stepper(AccelStepper::DRIVER, pulsePin, dirPin);

// Speed and Acceleration Settings
const float maxSpeed = 1500;       // Max speed in steps per second
const float acceleration = 300;    // Acceleration in steps per second^2

// State Variables
bool movingTowardsLimit1 = true;
int limitSwitch1Count = 0;
int limitSwitch2Count = 0;
long position1 = 0;
long position2 = 0;
long midpoint = 0;
bool zeroInitialized = false;

// Encoder Setup
Encoder encoder(encoderPinA, encoderPinB);

// Pole Placement Parameters
const float M = 0.250;   // Cart mass (kg)
const float m = 0.05;    // Pendulum mass (kg)
const float g = 9.81;    // Gravitational acceleration (m/s^2)
const float l = 0.42;    // Pendulum length (m)
const float b = 0.1;     // Damping/friction coefficient (Ns/m)

// Feedback Gain Matrix (from MATLAB)
const float K[4] = {2.5, 3.0, 8.0, 1.5}; // Gains for [x1, x2, x3, x4]

// State Variables for Pole Placement
double x1 = 0.0; // Cart position
double x2 = 0.0; // Cart velocity
double x3 = 0.0; // Pendulum angle
double x4 = 0.0; // Pendulum angular velocity
double u = 0.0;  // Control input

// Timing Variables
unsigned long lastControlTime = 0;
const unsigned long controlInterval = 10; // Control loop interval in ms

void setup() {
  // Stepper Motor Settings
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);

  // Configure Limit Switches
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);

  // Initialize Encoder
  encoder.write(0);

  // Start moving towards the first limit switch
  stepper.setSpeed(maxSpeed);
}

void loop() {
  if (!zeroInitialized) {
    initializeZeroPosition();
  } else {
    runPolePlacementControl();
  }
}

// Zero Position Initialization
void initializeZeroPosition() {
  if (digitalRead(limitSwitch1) == LOW && digitalRead(limitSwitch2) == LOW) {
    stepper.setSpeed(0);
    stepper.stop();
    while (true) {
      delay(1000);
    }
  }

  if (digitalRead(limitSwitch1) == LOW && movingTowardsLimit1) {
    limitSwitch1Count++;
    if (limitSwitch1Count == 3) {
      position1 = stepper.currentPosition();
      if (limitSwitch2Count == 3) calculateMidpointAndMove();
    }
    movingTowardsLimit1 = false;
    stepper.setSpeed(-maxSpeed);
    delay(200);
  } else if (digitalRead(limitSwitch2) == LOW && !movingTowardsLimit1) {
    limitSwitch2Count++;
    if (limitSwitch2Count == 3) {
      position2 = stepper.currentPosition();
      if (limitSwitch1Count == 3) calculateMidpointAndMove();
    }
    movingTowardsLimit1 = true;
    stepper.setSpeed(maxSpeed);
    delay(200);
  }

  stepper.runSpeed();
}

void calculateMidpointAndMove() {
  midpoint = (position1 + position2) / 2;

  stepper.moveTo(midpoint);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  zeroInitialized = true;
  encoder.write(0);
}

// Pole Placement Control Loop
void runPolePlacementControl() {
  unsigned long currentTime = millis();

  if (currentTime - lastControlTime >= controlInterval) {
    lastControlTime = currentTime;

    // Read Encoder Data
    int encoderCount = encoder.read();
    x3 = encoderCount * (360.0 / 2400.0); // Convert encoder counts to angle in degrees
    x4 = (x3 - x2) / (controlInterval / 1000.0); // Estimate angular velocity

    // Control Logic
    u = -(K[0] * x1 + K[1] * x2 + K[2] * x3 + K[3] * x4); // Compute control input
    applyControl(u);
  }
}

// Apply control input to the motor
void applyControl(double u) {
  if (u > 0) {
    stepper.setSpeed(constrain(u, 0, maxSpeed));
  } else {
    stepper.setSpeed(constrain(u, -maxSpeed, 0));
  }
  stepper.runSpeed();
}
