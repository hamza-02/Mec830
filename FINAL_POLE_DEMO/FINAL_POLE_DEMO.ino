#include <AccelStepper.h>
#include <Encoder.h>

// Pin Definitions
const int dirPin = 8;         // Direction pin for the stepper motor
const int pulsePin = 9;       // Pulse pin for the stepper motor
const int limitSwitch1 = 12;  // First limit switch
const int limitSwitch2 = 13;  // Second limit switch
const int encoder_pin_a = 2;    // Encoder pin A
const int encoder_pin_b = 3;    // Encoder pin B
const int stop_button = 10;    // STOP button
const int start_button = 11;   // START button
const int start_led = 7;   
const int off_led = 6;   
const int pole_led = 4;     
const int pid_led = 5;  
bool is_on = false;

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
Encoder encoder(encoder_pin_a, encoder_pin_b);

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

  pinMode(start_button, INPUT_PULLUP);
  pinMode(stop_button, INPUT_PULLUP);
  pinMode(start_led, OUTPUT);
  pinMode(off_led, OUTPUT);
  pinMode(pole_led, OUTPUT);
  pinMode(pid_led, OUTPUT);

  Serial.begin(1000000);
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
  digitalWrite(off_led, HIGH);
  digitalWrite(start_led, LOW);
  digitalWrite(pole_led, HIGH);
}

void loop() {

   while(digitalRead(start_button) == LOW & digitalRead(stop_button) == LOW ) {
  //setting up the set point 
    is_on = false;
    digitalWrite(start_led, LOW);
    digitalWrite(pid_led, HIGH);
    digitalWrite(pole_led, HIGH);
    digitalWrite(start_led, HIGH);
    digitalWrite(off_led, HIGH);
    delay(500);
    digitalWrite(pid_led, LOW);
    digitalWrite(pole_led, LOW);
    digitalWrite(start_led, LOW);
    digitalWrite(off_led, LOW);
    delay(500);
    encoder.write(0);
  }

  if (digitalRead(start_button) == LOW & digitalRead(stop_button) == HIGH)  {
    is_on = true;
    digitalWrite(pole_led, HIGH);
    digitalWrite(off_led, LOW);
    digitalWrite(start_led, HIGH);
  }
  else if (digitalRead(stop_button) == LOW  & digitalRead(start_button) == HIGH) {
    is_on = false;
    digitalWrite(off_led, HIGH);
    digitalWrite(start_led, LOW);
    digitalWrite(pole_led, HIGH);
  }

  if (is_on) {
    // Continuously run the stepper for smooth motion
    stepper.runSpeed();
    Serial.println(stepper.currentPosition());
  
    if (!zeroInitialized) {
      initializeZeroPosition();
    } else {
      // Perform pole placement control logic
      executePolePlacementControl();
      }
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
    if (limitSwitch1Count == 2) {
      position1 = stepper.currentPosition();
        Serial.println(stepper.currentPosition());
      if (limitSwitch2Count == 2) calculateMidpointAndMove();
    }
    movingTowardsLimit1 = false;
    stepper.setSpeed(-maxSpeed);
    delay(200);
  } else if (digitalRead(limitSwitch2) == LOW && !movingTowardsLimit1) {
    limitSwitch2Count++;
    if (limitSwitch2Count == 2) {
      position2 = stepper.currentPosition();
      if (limitSwitch1Count == 2) calculateMidpointAndMove();
    }
    movingTowardsLimit1 = true;
    stepper.setSpeed(maxSpeed);
     Serial.println(stepper.currentPosition());
    delay(200);
  }
}
void calculateMidpointAndMove() {
  midpoint = (position1 + position2) / 2;

  // Move the cart to the midpoint
  stepper.moveTo(midpoint);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  // Set the system's zero position to the midpoint
  zeroInitialized = true;
  encoder.write(0);           // Reset encoder to zero
  stepper.setCurrentPosition(0); // Reset the stepper's position to zero
  x1 = 0.0;                   // Reset cart position for control logic
}

// Pole Placement Control Logic
void executePolePlacementControl() {
    unsigned long currentTime = millis();

    if (currentTime - lastControlTime >= controlInterval) {
        lastControlTime = currentTime;

        // Update cart position (x1) using the stepper's current position
        double previousX1 = x1; // Store the previous position
      // x1 = stepper.currentPosition(); // Get the current position relative to zero
     

        // Update cart velocity (x2)
       // x2 = (x1 - previousX1) / (controlInterval / 1000.0); // Velocity in steps/sec

        // Update pendulum angle (x3) and angular velocity (x4)
        int encoderCount = encoder.read();
        x3 = encoderCount * (360.0 / 2400.0); // Convert encoder counts to angle in degrees
        x4 = (x3 - x2) / (controlInterval / 1000.0); // Estimate angular velocity

        // Compute control input (u)
        u = (K[0] * x1 + K[1] * x2 + K[2] * x3 + K[3] * x4);

        // Apply the control input to the motor
        applyControl(u);
    }
}


// Apply Control Input
void applyControl(double u) {
  Serial.println(x1);
  Serial.println(u);
  stepper.setSpeed(constrain(4*u, -maxSpeed, maxSpeed));
}
