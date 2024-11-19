#include <AccelStepper.h>

// Pin Definitions
const int dirPin = 8;         // Direction pin for the stepper motor
const int pulsePin = 9;       // Pulse pin for the stepper motor
const int limitSwitch1 = 12;  // First limit switch
const int limitSwitch2 = 13;  // Second limit switch

// Stepper Motor Setup
AccelStepper stepper(AccelStepper::DRIVER, pulsePin, dirPin);

// Speed and Acceleration Settings
const float maxSpeed = 1500;       // Set the desired max speed in steps per second
const float acceleration = 300;    // Acceleration in steps per second^2

// State variables
bool movingTowardsLimit1 = true;   // Track the direction of movement
int limitSwitch1Count = 0;         // Count hits for limit switch 1
int limitSwitch2Count = 0;         // Count hits for limit switch 2
long position1 = 0;                // Position at limit switch 1
long position2 = 0;                // Position at limit switch 2
bool midpointCalculated = false;   // Flag to indicate midpoint calculation

void setup() {
  Serial.begin(9600);

  // Set up stepper motor settings
  stepper.setMaxSpeed(maxSpeed);       // Set the maximum speed
  stepper.setAcceleration(acceleration); // Set the acceleration
  stepper.setSpeed(maxSpeed);          // Start moving towards limit switch 1

  // Configure limit switches as inputs with pull-up resistors
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);

  Serial.println("Motor setup complete, starting movement.");
}

void loop() {
  if (midpointCalculated) {
    return; // Stop processing once the midpoint is reached
  }

  // Check if both limit switches are triggered
  if (digitalRead(limitSwitch1) == LOW && digitalRead(limitSwitch2) == LOW) {
    Serial.println("Both limit switches triggered. Stopping motor.");
    stepper.setSpeed(0);  // Stop the motor
    stepper.stop();       // Immediately stop the motor
    while (true) {
      delay(1000);  // Halt the program if both limit switches are pressed
    }
  }

  // Check limit switches based on the direction of movement
  if (digitalRead(limitSwitch1) == LOW && movingTowardsLimit1) {
    Serial.print("Limit switch 1 triggered at position: ");
    Serial.println(stepper.currentPosition()); // Print the motor position

    limitSwitch1Count++; // Increment hit count for limit switch 1

    if (limitSwitch1Count == 3) {
      position1 = stepper.currentPosition(); // Record the position at limit switch 1
      if (limitSwitch2Count == 3) {
        calculateMidpointAndMove();
        return;
      }
    }

    stepper.setSpeed(-maxSpeed);        // Reverse direction to move away from limit switch 1
    movingTowardsLimit1 = false;        // Update direction state
    delay(200);
  } 
  else if (digitalRead(limitSwitch2) == LOW && !movingTowardsLimit1) {
    Serial.print("Limit switch 2 triggered at position: ");
    Serial.println(stepper.currentPosition()); // Print the motor position

    limitSwitch2Count++; // Increment hit count for limit switch 2

    if (limitSwitch2Count == 3) {
      position2 = stepper.currentPosition(); // Record the position at limit switch 2
      if (limitSwitch1Count == 3) {
        calculateMidpointAndMove();
        return;
      }
    }

    stepper.setSpeed(maxSpeed);         // Reverse direction to move away from limit switch 2
    movingTowardsLimit1 = true;         // Update direction state
    delay(200);
  }

  // Run the motor at the set speed
  stepper.runSpeed();  // Run the motor at the current speed
}

// Function to calculate the midpoint and move to it
void calculateMidpointAndMove() {
  long midpoint = (position1 + position2) / 2; // Calculate the midpoint
  Serial.print("Midpoint calculated: ");
  Serial.println(midpoint);

  // Move to the midpoint
  stepper.moveTo(midpoint);
  while (stepper.distanceToGo() != 0) {
    stepper.run(); // Continue moving until the midpoint is reached
  }

  Serial.println("Midpoint reached. System stopped.");
  stepper.stop(); // Stop the motor
  midpointCalculated = true; // Set the flag to stop further execution
  while (true) {
    delay(1000); // Halt the program
  }
}
