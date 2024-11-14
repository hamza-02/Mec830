const int button1Pin = 10;  // Button 1 pin for stop
const int button2Pin = 11;  // Button 2 start
const int led1Pin = 3;     // got to test and see what pins are for what
const int led2Pin = 4;   
const int led1Pin = 5;     
const int led2Pin = 6;    


void setup() {
  pinMode(button1Pin, INPUT_PULLUP);  // Set button 1 as input with internal pull-up resistor
  pinMode(button2Pin, INPUT_PULLUP);  // Set button 2 as input with internal pull-up resistor
  pinMode(led1Pin, OUTPUT);           // Set LED 1 as output
  pinMode(led2Pin, OUTPUT);           // Set LED 2 as output
  Serial.begin(9600);                 // Initialize serial communication
}

void loop() {
  // Check if button 1 is pressed
  if (digitalRead(button1Pin) == LOW) { // Button is active-low (pressed when LOW)
    digitalWrite(led1Pin, HIGH);        // Turn on LED 1
    Serial.println("Button 1 pressed"); // Print message to serial
  } else {
    digitalWrite(led1Pin, LOW);         // Turn off LED 1 when button is not pressed
  }

  // Check if button 2 is pressed
  if (digitalRead(button2Pin) == LOW) { // Button is active-low (pressed when LOW)
    digitalWrite(led2Pin, HIGH);        // Turn on LED 2
    Serial.println("Button 2 pressed"); // Print message to serial
  } else {
    digitalWrite(led2Pin, LOW);         // Turn off LED 2 when button is not pressed
  }
}
