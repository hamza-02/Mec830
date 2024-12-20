#define ENCODER_OPTIMIZE_INTERRUPTS
#include <AccelStepper.h>
#include <PID_v1.h>
#include <Encoder.h>

bool is_on = false;
bool set_point_initilized =false;

// Pin Definitions
const int dir_pin = 8;         // Direction pin for the stepper motor
const int pulse_pin = 9;       // Pulse pin for the stepper motor
const int encoder_pin_a = 2;   // Encoder pin A
const int encoder_pin_b = 3;   // Encoder pin B
const int stop_button = 10;    // STOP button
const int start_button = 11;   // START button
const int start_led = 7;   
const int off_led = 6;   
const int pole_led = 4;     
const int pid_led = 5;  
const int limitSwitch1 = 12;  // First limit switch
const int limitSwitch2 = 13;  // Second limit switch

// Stepper Motor and Driver Settings
const bool inverted = true;
const int step_mode = 3;
const int step_modes[6][5] = {
  {5, 1, 0, 0, 0},
  {10, 2, 1, 0, 0},
  {20, 4, 0, 1, 0},
  {40, 8, 1, 1, 0},
  {80, 16, 0, 0, 1},
  {160, 32, 1, 1, 1}
};

double steps_per_mm;
int output_dir;
AccelStepper stepper(AccelStepper::DRIVER, pulse_pin, dir_pin);

// PID Control Variables
double set_point, input, output, scale_factor;
double kp, ki, kd;
PID pid(&input, &output, &set_point, kp, ki, kd, DIRECT);

// Encoder Setup
Encoder encoder(encoder_pin_a, encoder_pin_b);

void setup() {

  output_dir = -1;
  kp = 50.00;
  ki = 700.00;
  kd =0.1;
  scale_factor = -1 / 37.5;


  digitalWrite(off_led, HIGH);
  digitalWrite(pid_led, HIGH);
  pid.SetTunings(kp, ki, kd);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-1200, 1200);

  steps_per_mm = step_modes[step_mode][0];
  stepper.setMaxSpeed(15000);       // Set a reasonable max speed
  stepper.setMinPulseWidth(5);

  pinMode(start_button, INPUT_PULLUP);
  pinMode(stop_button, INPUT_PULLUP);
  pinMode(start_led, OUTPUT);
  pinMode(off_led, OUTPUT);
  pinMode(pole_led, OUTPUT);
  pinMode(pid_led, OUTPUT);

 
  set_point = 0;
  encoder.write(0);


}

void loop() {
  // Read encoder count and calculate angle
  int count = encoder.read();
  double angle = count * (360.0 / 2400.0); // Convert encoder count to angle
  
  
    
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
    digitalWrite(pid_led, HIGH);
    digitalWrite(off_led, LOW);
    digitalWrite(start_led, HIGH);
  }
  else if (digitalRead(stop_button) == LOW  & digitalRead(start_button) == HIGH) {
    is_on = false;
    digitalWrite(off_led, HIGH);
    digitalWrite(start_led, LOW);
  }
  
  if (is_on) {
    // Check if the angle exceeds -30 or 30 degrees
    if (angle < -20 || angle > 20 ) {
      stepper.setSpeed(0);  // Stop the motor
      stepper.runSpeed();   // Apply the stop command
      return;               // Exit the loop early
    }

    // Continue with PID control if angle is within -30 to 30 range
    input = 5*angle;
    pid.Compute();

    // Set motor speed and direction based on PID output
    stepper.setSpeed(output_dir * output);
    stepper.runSpeed();
  }
}
