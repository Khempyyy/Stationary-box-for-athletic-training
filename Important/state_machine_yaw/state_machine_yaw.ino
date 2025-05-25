#include <Wire.h>
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

// Define the states of the state machine
enum State {
  STATE_IDLE,
  STATE_BUTTON_PRESSED,
};

// Define the initial state
State currentState = STATE_IDLE;

// Define pin numbers
const int buttonPin1 = 7 ;
const int buttonPin2 = 8 ;

// Variables to store button state
bool buttonState1 = false;
bool lastButtonState1 = false;
bool buttonState2 = false;
bool lastButtonState2 = false;

int Yaw;
int Yaw_remem = 0;
int previous_Yaw;
const int LED_PIN0 = 0;
int different_Yaw = 0;
const int BUTTON_PIN = 7;
int different_Yaw_remem = 0;

void setup() {
  pinMode(buttonPin1, INPUT_PULLUP); // Set button pin as input with internal pull-up resistor
  pinMode(buttonPin2, INPUT_PULLUP);
  Serial.begin(9600); // Initialize serial communication
  compass.init();
  pinMode(LED_PIN0, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
}

void loop() {
  // Read the button state
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);
  // State machine logic
  compass.read();
  Yaw = compass.getAzimuth();
  different_Yaw = abs(Yaw - previous_Yaw); 
  switch(currentState) {
    case STATE_IDLE:
      if (buttonState1 == HIGH && lastButtonState1 == LOW) {
        currentState = STATE_BUTTON_PRESSED;
        Yaw_remem = Yaw;
      }
      Serial.println(different_Yaw);
      if (different_Yaw > 3) {
        digitalWrite(LED_PIN0, HIGH);
      } else {
        digitalWrite(LED_PIN0, LOW);   
      }
      Serial.println("Free mode");
      previous_Yaw = Yaw;
      delay(500);
      break;
      
    case STATE_BUTTON_PRESSED:
      if (buttonState2 == LOW && lastButtonState2 == HIGH) {
        currentState = STATE_IDLE;
        
      }
      different_Yaw_remem = abs(Yaw - Yaw_remem);
      if (abs(different_Yaw_remem) > 3) {
        digitalWrite(LED_PIN0, HIGH);
      } else {
        digitalWrite(LED_PIN0, LOW);
      }  
      Serial.println("Yaw Remem =");
      Serial.print(Yaw_remem);
      Serial.println("Remember mode");
      break;
  }

  // Store the current button state for comparison in the next loop iteration
  lastButtonState1 = buttonState1;
  lastButtonState2 = buttonState2;
}
