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
const float maxSpeed = 1500;       // Set the desired max speed in steps per second
const float acceleration = 300;    // Acceleration in steps per second^2

// State Variables
bool movingTowardsLimit1 = true;   // Track the direction of movement
int limitSwitch1Count = 0;         // Count how many times limit switch 1 is triggered
int limitSwitch2Count = 0;         // Count how many times limit switch 2 is triggered
long position1 = 0;                // Position recorded at limit switch 1
long position2 = 0;                // Position recorded at limit switch 2
long midpoint = 0;                 // Calculated midpoint
bool zeroInitialized = false;      // Flag for zero position initialization

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
   Serial.begin(9600);

  // Set up stepper motor settings
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);

  // Configure limit switches as inputs with pull-up resistors
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);

  // Initialize Encoder
  encoder.write(0);

  Serial.println("Initializing zero position...");
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
    Serial.println("Both limit switches triggered. Stopping motor.");
    stepper.setSpeed(0);
    stepper.stop();
    while (true) {
      delay(1000);
    }
  }

  if (digitalRead(limitSwitch1) == LOW && movingTowardsLimit1) {
    Serial.println("LimitSwitch1 triggered.");
    limitSwitch1Count++;
    if (limitSwitch1Count == 3) {
      position1 = stepper.currentPosition();
      Serial.print("Position1 recorded: ");
      Serial.println(position1);
      if (limitSwitch2Count == 3) calculateMidpointAndMove();
    }
    stepper.setSpeed(-maxSpeed);
    
    movingTowardsLimit1 = false;
    delay(200);
  } else if (digitalRead(limitSwitch2) == LOW && !movingTowardsLimit1) {
    Serial.println("LimitSwitch2 triggered.");
    limitSwitch2Count++;
    if (limitSwitch2Count == 3) {
      position2 = stepper.currentPosition();
      Serial.print("Position2 recorded: ");
      Serial.println(position2);
      if (limitSwitch1Count == 3) calculateMidpointAndMove();
    }
    stepper.setSpeed(maxSpeed);
    movingTowardsLimit1 = true;
    delay(200);
  }

  stepper.runSpeed();
}

void calculateMidpointAndMove() {
  midpoint = (position1 + position2) / 2;
  Serial.print("Midpoint calculated: ");
  Serial.println(midpoint);

  stepper.moveTo(midpoint);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  Serial.println("Midpoint reached. Zero position initialized.");
  zeroInitialized = true;
  encoder.write(0); // Reset encoder to zero at midpoint
}

// Pole Placement Control Loop
void runPolePlacementControl() {
  unsigned long currentTime = millis();

  if (currentTime - lastControlTime >= controlInterval) {
    lastControlTime = currentTime;

    // Read Encoder Data
    int encoderCount = encoder.read();
    x3 = encoderCount * (360.0 / 2400.0); // Convert encoder counts to angle in degrees
    x4 = (x3 - x2) / (controlInterval / 1000.0); // Estimate angular velocity (deg/s)

    // Control Logic
    u = -(K[0] * x1 + K[1] * x2 + K[2] * x3 + K[3] * x4); // Compute control input
    applyControl(u);

   // Serial.print("Cart Position: ");
  //  Serial.print(x1);
    //Serial.print(" | Pendulum Angle: ");
   // Serial.print(x3);
   // Serial.print(" | Control Input (u): ");
   // Serial.println(u);
  }
}

// Apply control input to the motor
void applyControl(double u) {
  if (u > 0) {
    stepper.setSpeed(constrain(u, 0, maxSpeed)); // Apply positive speed
  } else {
    stepper.setSpeed(constrain(u, -maxSpeed, 0)); // Apply negative speed
  }
  stepper.runSpeed();
}
