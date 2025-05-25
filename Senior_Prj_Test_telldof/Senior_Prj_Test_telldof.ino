#include "LSM6DS3.h"
#include "Wire.h"
#include <VL53L1X.h>
#include <QMC5883LCompass.h>

const int LED_PIN0 = 0;
const int LED_PIN1 = 1;
const int LED_PIN2 = 2;
//const int LED_PIN3 = 3;
const int LED_PIN6 = 6;
const int LED_PIN7 = 7;
//const int LED_PIN8 = 8;

LSM6DS3 myIMU(I2C_MODE, 0x6A);
VL53L1X sensor;

const int NUM_SAMPLES = 12;
float distance_readings[NUM_SAMPLES];
int current_index = 0;
unsigned long previous_millis = 0;
unsigned long interval = 1000;
float previous_moving_average = 0;

const int WINDOW_SIZE = 10;
const unsigned long SAMPLE_INTERVAL = 1000;
QMC5883LCompass compass;
int yValues[WINDOW_SIZE];
int currentIndex = 0;
unsigned long lastSampleTime = 0;
int prevMovingAverage = 0;
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
int previousAzimuth = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (myIMU.begin() != 0) {
      Serial.println("Device error");
    } else {
      Serial.println("Device OK!");
  }


  //pinMode(LED_PIN0, OUTPUT);
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  //pinMode(LED_PIN3, OUTPUT);
  pinMode(LED_PIN6, OUTPUT);
  pinMode(LED_PIN7, OUTPUT);
  //pinMode(LED_PIN8, OUTPUT);
  
  
  Wire.begin();
  Wire.setClock(400000);
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);
  compass.init();
}

void loop() {

  //Y-axis Roll-move and Pitch-move
  //if (abs(myIMU.readFloatAccelY()) > 0.2){
  //  digitalWrite(LED_PIN0, HIGH);
  //}else{
  //  digitalWrite(LED_PIN0, LOW);
  //}

  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatAccelY());


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
  float rollAvg = rollTotal / numReadings;
  float pitchAvg = pitchTotal / numReadings;

  if (abs(Roll - rollAvg) > 3) {
      digitalWrite(LED_PIN1, HIGH); // Turn on LED
      delay(1000);
  } else {
      digitalWrite(LED_PIN1, LOW); // Turn off LED
  }

  // Check if the difference exceeds 10 for Roll and pitchAvg
  if (abs(Pitch - pitchAvg) > 3) {
      digitalWrite(LED_PIN2, HIGH); // Turn on LED
      delay(1000);
  } else {
      digitalWrite(LED_PIN2, LOW); // Turn off LED
  }

  //Serial.print(" diffRoll = ");
  //Serial.println(Roll - rollAvg);
  //Serial.print(" diffPitch = ");
  //Serial.println(Pitch - pitchAvg);


  //Z-axis
  sensor.read();
  float distance = sensor.ranging_data.range_mm;
  distance_readings[current_index] = distance;
  current_index = (current_index + 1) % NUM_SAMPLES;
  float moving_average = calculateMovingAverage();
  unsigned long current_millis = millis();
  if (current_millis - previous_millis >= interval) {
    float difference_distance = moving_average - previous_moving_average;
    Serial.print("VL53L1X Difference between current and 1 second ago moving average: "); 
    Serial.println(difference_distance);
    previous_moving_average = moving_average;
    previous_millis = current_millis;
    if (abs(difference_distance) > 40) {
      digitalWrite(LED_PIN7, HIGH);
      delay(1000);
      digitalWrite(LED_PIN7, LOW);
    }
  }

  
  //Yaw-move
  int currentAzimuth;
  compass.read();
  currentAzimuth = compass.getAzimuth();
  Serial.print("A: ");
  Serial.print(currentAzimuth);
  Serial.println();
  if (abs(currentAzimuth - previousAzimuth) > 3) {
    digitalWrite(LED_PIN6, HIGH);
    delay(1000);
    digitalWrite(LED_PIN6, LOW);
  }
  previousAzimuth = currentAzimuth;
  delay(250);

  Serial.print(" currentAzimuth = ");
  Serial.println(currentAzimuth);

  delay(1000);
}


float calculateMovingAverage() {
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += distance_readings[i];
  }
  return sum / NUM_SAMPLES;
}
