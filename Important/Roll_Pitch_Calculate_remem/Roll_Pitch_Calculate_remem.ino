#include <Wire.h>
#include <LSM6DS3.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

const int LED_PIN0 = 0;
const int LED_PIN1 = 1;
const int BUTTON_PIN = 7;

float X = 0;
float Y = 0;
float Z = 0;
float Roll = 0;
float Pitch = 0;
float theta = 0;
//const int numReadings = 10;
//float rollReadings[numReadings];
//float pitchReadings[numReadings];
//int rollIndex = 0;
//int pitchIndex = 0;
//float rollTotal = 0;
//float pitchTotal = 0;

//float different_Roll = 0;
//float different_Pitch = 0;

float different_Roll_remem = 0;
float different_Pitch_remem = 0;

float Roll_remem = 0;
float Pitch_remem = 0;


void setup() {
  Serial.begin(115200);
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
  pinMode(BUTTON_PIN, INPUT);
}

void loop() {
  X = myIMU.readFloatAccelX();
  Y = myIMU.readFloatAccelY();
  Z = myIMU.readFloatAccelZ();
  Pitch = atan2(-X, sqrt(Y*Y + Z*Z)) * 180/3.14;
  Roll = atan2(Y, Z) * 180/3.14;
  //rollTotal -= rollReadings[rollIndex];
  //pitchTotal -= pitchReadings[pitchIndex];
  //rollReadings[rollIndex] = Roll;
  //pitchReadings[pitchIndex] = Pitch;
  //rollTotal += Roll;
  //pitchTotal += Pitch;
  //rollIndex = (rollIndex + 1) % numReadings;
  //pitchIndex = (pitchIndex + 1) % numReadings;
  //float rollAvg = rollTotal / numReadings;
  //float pitchAvg = pitchTotal / numReadings;

  //different_Roll = abs(Roll - rollAvg);
  //different_Pitch = abs(Pitch - pitchAvg);

  Serial.print("R: ");
  Serial.print(Roll);
  Serial.println();
  Serial.print("P: ");
  Serial.print(Pitch);
  Serial.println();
  Serial.print("R_remem: ");
  Serial.print(Roll_remem);
  Serial.println();
  Serial.print("P_remem: ");
  Serial.print(Pitch_remem);
  Serial.println();
  // Serial.print("R_Avg: ");
  // Serial.print(rollAvg);
  // Serial.println();
  // Serial.print("P_Avg: ");
  // Serial.print(pitchAvg);
  // Serial.println();

  if (digitalRead(BUTTON_PIN) == HIGH) { 
    Roll_remem = Roll;
    Pitch_remem = Pitch;
  }
  different_Roll_remem = abs(Pitch - Pitch_remem);
  different_Pitch_remem = abs(Pitch - Roll_remem); 

  if (abs(different_Roll_remem) > 3) {
    digitalWrite(LED_PIN0, HIGH);

  } else {
    digitalWrite(LED_PIN0, LOW);
  }  

  if (abs(different_Pitch_remem) > 3) {
    digitalWrite(LED_PIN1, HIGH);

  } else {
    digitalWrite(LED_PIN1, LOW);
  }  
  delay(250);
}
