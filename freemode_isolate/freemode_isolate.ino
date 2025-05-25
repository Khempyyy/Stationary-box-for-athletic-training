#include <Wire.h>
#include <VL53L1X.h>
#include <QMC5883LCompass.h>
#include <LSM6DS3.h>

VL53L1X sensor;
QMC5883LCompass compass;
LSM6DS3 myIMU(I2C_MODE, 0x6A);

const int LED_PIN0 = 0;
const int LED_PIN1 = 1;
const int LED_PIN2 = 2;
const int LED_PIN3 = 3;
const int tactileSwitchPin1 = 7; // Change this to your switch pin
const int tactileSwitchPin2 = 8;
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
int previousAzimuth = 0;
float dis_remem = 0;
float azimuth = 0;
int button1 = 0;
int button2 = 0;
int lastbutton1 = 0;
int lastbutton2 = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Use 400 kHz I2C

  pinMode(LED_PIN0, OUTPUT);
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);

  compass.init();

  sensor.setTimeout(500);

  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

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

  pinMode(tactileSwitchPin1, INPUT); // Enable internal pull-up resistor
  pinMode(tactileSwitchPin2, INPUT);
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
  rollIndex = (rollIndex + 1) % numReadings;
  pitchIndex = (pitchIndex + 1) % numReadings;
  float rollAvg = rollTotal / numReadings;
  float pitchAvg = pitchTotal / numReadings;

  //calculate azimuth
  compass.read();
  float currentAzimuth = compass.getAzimuth();
  float diff_azimuth = previousAzimuth - currentAzimuth;
  previousAzimuth = currentAzimuth;

  //Z-axis calculate MA
  sensor.read();
  float distance = sensor.ranging_data.range_mm;
  distance_readings[current_index] = distance;
  current_index = (current_index + 1) % NUM_SAMPLES;
  float moving_average = calculateMovingAverage();
  unsigned long current_millis = millis();
  
  float difference_distance = moving_average - previous_moving_average;
  previous_moving_average = moving_average;
  previous_millis = current_millis;
  delay(250);

  button1 = digitalRead(tactileSwitchPin1);
  button2 = digitalRead(tactileSwitchPin2);


  if (abs(difference_distance) > 40) {
    digitalWrite(LED_PIN3, HIGH);
    delay(1000);
  } else {  
    digitalWrite(LED_PIN3, LOW);
    }
    // Roll notification
  if (abs(Roll - rollAvg) > 3) {
    digitalWrite(LED_PIN1, HIGH); // Turn on LED
    delay(1000);
  } else {
    digitalWrite(LED_PIN1, LOW); // Turn off LED
  }
  // Pitch notification
  if (abs(Pitch - pitchAvg) > 3) {
    digitalWrite(LED_PIN2, HIGH); // Turn on LED
    delay(1000);
  } else {
    digitalWrite(LED_PIN2, LOW); // Turn off LED
  }
  // Yaw-move
  if (abs(diff_azimuth) > 3) {
    digitalWrite(LED_PIN0, HIGH);
    delay(1000);
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

}

float calculateMovingAverage() {
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += distance_readings[i];
  }
  return sum / NUM_SAMPLES;
}
