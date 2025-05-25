#include <Wire.h>
#include <QMC5883LCompass.h>
#include "LSM6DS3.h"
#include <VL53L1X.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);
QMC5883LCompass compass;
VL53L1X sensor;

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

// Define LED
const int LED_PIN0 = 0;
const int LED_PIN1 = 1;
const int LED_PIN2 = 2;
const int LED_PIN3 = 3;


// Variables to store button state
bool buttonState1 = false;
bool lastButtonState1 = false;
bool buttonState2 = false;
bool lastButtonState2 = false;

// Variable for Pitch and Roll Calculate
float X = 0;
float Y = 0;
float Z = 0;
float Roll = 0;
float Pitch = 0;
float theta = 0;
float different_Roll = 0;
float different_Pitch = 0;
float different_Roll_remem = 0;
float different_Pitch_remem = 0;
float Roll_remem = 0;
float Pitch_remem = 0;
float previous_Roll;
float previous_Pitch;
// Variable for Yaw Calculate
int Yaw;
int Yaw_remem = 0;
int previous_Yaw;
int different_Yaw = 0;
int different_Yaw_remem = 0;

// Variable for Z axis
float different_range = 0;
float previous_Z_axis_range;
float Z_axis_range = 0;
float different_range_remem = 0;
float Z_axis_range_remem = 0;

void setup() {
  pinMode(buttonPin1, INPUT_PULLUP); // Set button pin as input with internal pull-up resistor
  pinMode(buttonPin2, INPUT_PULLUP);
  Serial.begin(9600); // Initialize serial communication
  compass.init();

  Wire.begin();
  Wire.setClock(400000); // Use 400 kHz I2C

  while (!Serial);
    //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) {
      Serial.println("Device error");
  } else {
      Serial.println("Device OK!");
  }
  pinMode(LED_PIN0, OUTPUT);
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);
}

void loop() {
  // Read the button state
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);
  //Roll Pitch data
  X = myIMU.readFloatAccelX();
  Y = myIMU.readFloatAccelY();
  Z = myIMU.readFloatAccelZ();
  Pitch = atan2(-X, sqrt(Y*Y + Z*Z)) * 180/3.14;
  Roll = atan2(Y, Z) * 180/3.14;

  //compass data
  compass.read();
  Yaw = compass.getAzimuth();

  //distance data
  sensor.read();
  Z_axis_range = sensor.ranging_data.range_mm;

  switch(currentState) {
    case STATE_IDLE:
      if (buttonState1 == HIGH && lastButtonState1 == LOW) {
        currentState = STATE_BUTTON_PRESSED;
        Yaw_remem = Yaw;
        Roll_remem = Roll;
        Pitch_remem = Pitch;
        Z_axis_range_remem = Z_axis_range;
      }      
      
      different_Yaw = abs(Yaw - previous_Yaw); 
      different_Roll = abs(Roll - previous_Roll);
      different_Pitch = abs(Pitch - previous_Pitch);
      different_range = abs(Z_axis_range - previous_Z_axis_range);

      if (different_Pitch > 3) {
        digitalWrite(LED_PIN0, HIGH);
      } else {
        digitalWrite(LED_PIN0, LOW);   
      }

      if (different_Roll > 3) {
        digitalWrite(LED_PIN1, HIGH);
      } else {
        digitalWrite(LED_PIN1, LOW);   
      }

      if (different_Yaw > 6) {
        digitalWrite(LED_PIN2, HIGH);
      } else {
        digitalWrite(LED_PIN2, LOW);   
      }

      if (different_range > 40) {
        digitalWrite(LED_PIN3, HIGH);
      } else {
        digitalWrite(LED_PIN3, LOW);   
      }
      
      Serial.println("Free");
      Serial.println(different_Pitch);
      Serial.println(different_Roll);
      Serial.println(different_Yaw);
      Serial.println(different_range);
      previous_Yaw = Yaw;
      previous_Roll = Roll;
      previous_Pitch = Pitch;
      previous_Z_axis_range = Z_axis_range;
      delay(500);
      break;
      
    case STATE_BUTTON_PRESSED:
      if (buttonState2 == LOW && lastButtonState2 == HIGH) {
        currentState = STATE_IDLE;
        
      }
      different_Yaw_remem = abs(Yaw - Yaw_remem);
      different_Roll_remem = abs(Roll - Roll_remem);
      different_Pitch_remem = abs(Pitch - Pitch_remem);
      different_range_remem = abs(Z_axis_range - Z_axis_range_remem);

      if (abs(different_Pitch_remem) > 3) {
        digitalWrite(LED_PIN0, HIGH);
      } else {
        digitalWrite(LED_PIN0, LOW);
      }  

      if (abs(different_Roll_remem) > 3) {
        digitalWrite(LED_PIN1, HIGH);
      } else {
        digitalWrite(LED_PIN1, LOW);
      }  

      if (abs(different_Yaw_remem) > 6) {
        digitalWrite(LED_PIN2, HIGH);
      } else {
        digitalWrite(LED_PIN2, LOW);
      }  
      

      if (abs(different_range_remem) > 40) {
        digitalWrite(LED_PIN3, HIGH);
      } else {
        digitalWrite(LED_PIN3, LOW);
      }

      Serial.println("Remem");
      Serial.println(different_Pitch_remem);
      Serial.println(different_Roll_remem);
      Serial.println(different_Yaw_remem);
      Serial.println(different_range_remem);
      break;
  }

  // Store the current button state for comparison in the next loop iteration
  lastButtonState1 = buttonState1;
  lastButtonState2 = buttonState2;
}
