#include <Wire.h>
#include <VL53L1X.h>
#include <QMC5883LCompass.h>
#include <LSM6DS3.h>
// Define the states of the state machine

VL53L1X sensor;
QMC5883LCompass compass;
LSM6DS3 myIMU(I2C_MODE, 0x6A);

enum State {
  STATE_IDLE,
  STATE_BUTTON_PRESSED,
};

// Define the initial state
State currentState = STATE_IDLE;

// Define pin numbers
const int buttonPin1 = 7 ;
const int buttonPin2 = 8 ;
const int LED_PIN0 = 0;
const int LED_PIN1 = 1;
const int LED_PIN2 = 2;
const int LED_PIN3 = 3;

// Variables to store button state
bool buttonState1 = false;
bool lastButtonState1 = false;
bool buttonState2 = false;
bool lastButtonState2 = false;

const int NUM_SAMPLES = 12;
float distance_readings[NUM_SAMPLES];
int current_index = 0;
unsigned long previous_millis = 0;
unsigned long interval = 1000;
float previous_moving_average = 0;

const int WINDOW_SIZE = 10;
float X = 0;
float Y = 0;
float Z = 0;
float Roll = 0;
float Pitch = 0;
float theta = 0;
const int numReadings = 10;
float rollReadings[numReadings];
float pitchReadings[numReadings];
int rollIndex = 0;
int pitchIndex = 0;
float rollTotal = 0;
float pitchTotal = 0;
float previousAzimuth = 0;
float dis_remem = 0;
float azimuth = 0;
float azimuth_remem = 0;
float Count_test = 0;
float Roll_remem = 0;
float Pitch_remem = 0;
float distance = 0;
float currentAzimuth = 0;
float moving_average = 0;
unsigned long current_millis = 0;
float diff_azimuth = 0;
float pitchAvg = 0;
float rollAvg = 0;
float difference_distance = 0;

void setup() {
  pinMode(buttonPin1, INPUT_PULLUP); // Set button pin as input with internal pull-up resistor
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(LED_PIN0, OUTPUT);
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);
  Serial.begin(9600); // Initialize serial communication
  Wire.begin();
  Wire.setClock(400000);

  compass.init();

  sensor.setTimeout(500);


  sensor.setDistanceMode(VL53L1X::Medium);
  sensor.setMeasurementTimingBudget(200000);
  sensor.startContinuous(200);

  while (!Serial);
  //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("Device OK!");
  }
  
}

void loop() {
  // Read the button state
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);
  
  // State machine logic
  switch(currentState) {
    case STATE_IDLE:
      if (buttonState1 == LOW && lastButtonState1 == HIGH) {
        currentState = STATE_BUTTON_PRESSED;
        Count_test += 1;
        dis_remem = distance;
        azimuth_remem = currentAzimuth;
        Roll_remem = Roll;
        Pitch_remem = Pitch;
      }

      X = myIMU.readFloatAccelX();
      Y = myIMU.readFloatAccelY();
      Z = myIMU.readFloatAccelZ();
      Pitch = atan2(-X, sqrt(Y*Y + Z*Z)) * 180/3.14;
      Roll = atan2(Y, Z) * 180/3.14;
      rollTotal -= rollReadings[rollIndex];
      pitchTotal -= pitchReadings[pitchIndex];
      rollReadings[rollIndex] = Roll;
      pitchReadings[pitchIndex] = Pitch;
      rollTotal += Roll;
      pitchTotal += Pitch;
      rollIndex = (rollIndex + 1) % numReadings;
      pitchIndex = (pitchIndex + 1) % numReadings;
      rollAvg = rollTotal / numReadings;
      pitchAvg = pitchTotal / numReadings;

      compass.read();
      currentAzimuth = compass.getAzimuth();
      diff_azimuth = previousAzimuth - currentAzimuth;
      previousAzimuth = currentAzimuth;

      sensor.read();
      distance = sensor.ranging_data.range_mm;
      distance_readings[current_index] = distance;
      current_index = (current_index + 1) % NUM_SAMPLES;
      moving_average = calculateMovingAverage();
      current_millis = millis();

      difference_distance = moving_average - previous_moving_average;
      previous_moving_average = moving_average;
      previous_millis = current_millis;

      if (abs(difference_distance) > 40) {
        digitalWrite(LED_PIN3, HIGH);
      } else {        
        digitalWrite(LED_PIN3, LOW);
      }
      // Roll notification
      if (abs(Roll - rollAvg) > 3) {
        digitalWrite(LED_PIN1, HIGH); // Turn on LED
      } else {
        digitalWrite(LED_PIN1, LOW); // Turn off LED
      }
      // Pitch notification
      if (abs(Pitch - pitchAvg) > 3) {
        digitalWrite(LED_PIN2, HIGH); // Turn on LED
      } else {
        digitalWrite(LED_PIN2, LOW); // Turn off LED
      }
      // Yaw-move
      if (abs(diff_azimuth) > 3) {
        digitalWrite(LED_PIN0, HIGH);
      } else {
        digitalWrite(LED_PIN0, LOW);
      }

      Serial.println("Free mode");
      Serial.print("R: ");
      Serial.print(Roll - rollAvg);
      Serial.print(" P: ");
      Serial.print(Pitch - pitchAvg);
      Serial.print(" Y: ");
      Serial.print(diff_azimuth);
      Serial.print(" D: ");
      Serial.println(difference_distance);
      break;
      
    case STATE_BUTTON_PRESSED:
      if (buttonState2 == HIGH && lastButtonState2 == LOW) {
        currentState = STATE_IDLE;
        
      }
      
      Serial.println(Count_test);
      Serial.println("Pressed");
      Serial.println("Rememposition mode");
      Serial.print(" Roll: ");
      Serial.print(Roll_remem);
      Serial.print(" Pitch: ");
      Serial.print(Pitch_remem);
      Serial.print(" Yaw: ");
      Serial.print(azimuth_remem);
      Serial.print(" Distance from Ground: ");
      Serial.println(dis_remem);

           //if (abs(distance - dis_remem) > 40) {
     //  digitalWrite(LED_PIN3, HIGH);
    //   delay(1000);
    // } else {  
    //   digitalWrite(LED_PIN3, LOW);
    // }
      //Roll notification
      if (abs(Roll-Roll_remem) > 3) {
        digitalWrite(LED_PIN1, HIGH); // Turn on LED
      } else {
        digitalWrite(LED_PIN1, LOW); // Turn off LED
      }
      //Pitch notification
      if (abs(Pitch - Pitch_remem) > 3) {
        digitalWrite(LED_PIN2, HIGH); // Turn on LED
      } else {
        digitalWrite(LED_PIN2, LOW); // Turn off LED
      }

      //Yaw-move
      if (abs(azimuth_remem - currentAzimuth) > 3) {
        digitalWrite(LED_PIN0, HIGH);
      } else {
        digitalWrite(LED_PIN0, LOW);
      }

      if (buttonState2 == HIGH && lastButtonState2 == LOW) {
        currentState = STATE_IDLE;
        
      }else{
        currentState = STATE_BUTTON_PRESSED;
      }
      break;
  }

  // Store the current button state for comparison in the next loop iteration
  lastButtonState1 = buttonState1;
  lastButtonState2 = buttonState2;
}

float calculateMovingAverage() {
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += distance_readings[i];
  }
  return sum / NUM_SAMPLES;
}

