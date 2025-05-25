#include <Wire.h>
// Define the states of the state machine
enum State {
  STATE_IDLE,
  STATE_BUTTON_PRESSED,
};

// Define the initial state
State currentState = STATE_IDLE;

// Define pin numbers
const int buttonPin1 = 6 ;
const int buttonPin2 = 7 ;

// Variables to store button state
bool buttonState1 = false;
bool lastButtonState1 = false;
bool buttonState2 = false;
bool lastButtonState2 = false;

void setup() {
  pinMode(buttonPin1, INPUT_PULLUP); // Set button pin as input with internal pull-up resistor
  pinMode(buttonPin2, INPUT_PULLUP);
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  // Read the button state
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);
  // State machine logic
  switch(currentState) {
    case STATE_IDLE:
      if (buttonState1 == HIGH && lastButtonState1 == LOW) {
        currentState = STATE_BUTTON_PRESSED;
        
      }
      Serial.println("Idle");
      break;
      
    case STATE_BUTTON_PRESSED:
      if (buttonState2 == LOW && lastButtonState2 == HIGH) {
        currentState = STATE_IDLE;
        
      }
      Serial.println("Pressed");
      break;
  }

  // Store the current button state for comparison in the next loop iteration
  lastButtonState1 = buttonState1;
  lastButtonState2 = buttonState2;
}
