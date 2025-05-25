#include <Wire.h>
// Define pin assignment
const int buttonPin = 7; // The button is connected to pin 2
int horn = 0;
void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set pin mode
  pinMode(buttonPin, INPUT); // The button pin is an input
}

void loop() {
  // Read button state
  int buttonState = digitalRead(buttonPin);
  int anabutt = analogRead(buttonPin);
  if (buttonState == 1){
    horn = 1;
    Serial.println("donky");
  } else {
    Serial.println("fuck not work"); 
  }

  Serial.println(buttonState);
  Serial.println(horn);
  Serial.println(anabutt);
  delay(250);
}

