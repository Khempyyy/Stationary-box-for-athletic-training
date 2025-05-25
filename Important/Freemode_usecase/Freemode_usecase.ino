#include "LSM6DS3.h"
#include "Wire.h"
#include <VL53L1X.h>
#include <QMC5883LCompass.h>
//IMU
const int LED_PIN0 = 0;
const int LED_PIN1 = 1;
const int LED_PIN2 = 2;
const int LED_PIN3 = 3;

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
QMC5883LCompass compass;
VL53L1X sensor;

//Compass

float X = 0;
float Y = 0;
float Z = 0;
float Roll = 0;
float Pitch = 0;
float theta = 0;
const int numReadings = 10; // Number of readings to average
float rollReadings[numReadings];
float pitchReadings[numReadings];
int rollIndex = 0;
int pitchIndex = 0;
float rollTotal = 0;
float pitchTotal = 0;
float previousAzimuth = 0;
float different_range = 0;
float previous_Z_axis_range = 0;
float Z_axis_range;

float z_distance_remem;
float azimuth_remem;
float Roll_remem;
float Pitch_remem;

const int ButtonPin = 7;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);
  //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) {
      Serial.println("Device error");
    } else {
      Serial.println("Device OK!");
  }
  pinMode(LED_PIN0, OUTPUT); // ตั้งค่าขาพิน LED เป็น OUTPUT
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);
  pinMode(ButtonPin, INPUT);
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

  // Move to the next index
  rollIndex = (rollIndex + 1) % numReadings;
  pitchIndex = (pitchIndex + 1) % numReadings;

  // Calculate the moving average
  float rollAvg = rollTotal / numReadings;
  float pitchAvg = pitchTotal / numReadings;

  // Roll notification
  if (abs(Roll - rollAvg) > 3) {
    digitalWrite(LED_PIN1, HIGH); // Turn on LED
  } else {
    digitalWrite(LED_PIN1, LOW); // Turn off LED
  }
  // Pitch notification
  if (abs(Pitch - pitchAvg) > 3) {
    digitalWrite(LED_PIN0, HIGH); // Turn on LED
  } else {
    digitalWrite(LED_PIN0, LOW); // Turn off LED
  }
  
  
  //Z-axis calculate MA
  sensor.read();
  Z_axis_range = sensor.ranging_data.range_mm;
  different_range = abs(Z_axis_range - previous_Z_axis_range);
  previous_Z_axis_range = Z_axis_range;

  if (abs(different_range) > 40) {
      digitalWrite(LED_PIN3, HIGH);
  } else {    
      digitalWrite(LED_PIN3, LOW);
  }
  
  // Read compass values
  compass.read();
  
  // Return Azimuth reading
  float currentAzimuth = compass.getAzimuth();
  float diff_azimuth = previousAzimuth - currentAzimuth;
  previousAzimuth = currentAzimuth;

  if (abs(diff_azimuth) > 3) {
    digitalWrite(LED_PIN2, HIGH);
  } else {
    digitalWrite(LED_PIN2, LOW);
  }

  Serial.println("Free mode");
  Serial.print("R: ");
  Serial.print(Roll - rollAvg);
  Serial.print(" P: ");
  Serial.print(Pitch - pitchAvg);
  Serial.print(" Y: ");
  Serial.print(diff_azimuth);
  Serial.print(" D: ");
  Serial.println(different_range);


  delay(250);
}

