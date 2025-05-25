#include <Wire.h>
const int tactileSwitchPin = 3;  
const int ledPin = 0;    
bool my_boolean = true;
int buttonPushCounter = 0;  
int buttonState = 0;        
int lastButtonState = 0;

void setup() {
  pinMode(tactileSwitchPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  buttonState = digitalRead(tactileSwitchPin);

  if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {
      buttonPushCounter++;
    }
    delay(50);
  }

  lastButtonState = buttonState;
  
  if (buttonPushCounter % 2 == 0) {
    digitalWrite(ledPin, HIGH);
    my_boolean = true;
  } else {
    digitalWrite(ledPin, LOW);
    my_boolean = false;
  }
}
