const int button1Pin = 10;  // STOP BUTTON
const int button2Pin = 11;  // START Button
const int START_LED = 6;   
const int OFF_LED = 7;   
const int POLE_LED = 4;     
const int PID_LED = 5;   


void setup() {
  pinMode(button1Pin, INPUT_PULLUP);  
  pinMode(button2Pin, INPUT_PULLUP);  
  pinMode(START_LED, OUTPUT);           
  pinMode(OFF_LED, OUTPUT);    
  pinMode(POLE_LED, OUTPUT);           
  pinMode(PID_LED, OUTPUT);  
  digitalWrite(OFF_LED, LOW);     
  Serial.begin(9600);    
  
            
}

void loop() {
  // Check if button 1 is pressed
  if (digitalRead(button1Pin) == LOW) { // Button is active-low (pressed when LOW)
    digitalWrite(START_LED, HIGH);        // Turn on LED 1
    Serial.println("Button 1 pressed"); // Print message to serial
  } else {
   // digitalWrite(START_LED, LOW);         // Turn off LED 1 when button is not pressed
  }

  // Check if button 2 is pressed
  if (digitalRead(button2Pin) == LOW) { // Button is active-low (pressed when LOW)
    digitalWrite(OFF_LED, HIGH);        // Turn on LED 2
    Serial.println("Button 2 pressed"); // Print message to serial
  } else {
   // digitalWrite(OFF_LED, LOW);         // Turn off LED 2 when button is not pressed
  }

  if (digitalRead(button2Pin) == LOW & digitalRead(button1Pin) == LOW) { // Button is active-low (pressed when LOW)
    digitalWrite(PID_LED, HIGH);  
    digitalWrite(POLE_LED, HIGH);// Turn on LED 2
    Serial.println("Both pressed"); // Print message to serial
  } else {
   // digitalWrite(PID_LED, LOW);  
   // digitalWrite(POLE_LED, LOW);// Turn on LED 2

  }
}
