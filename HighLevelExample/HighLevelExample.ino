#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>
const int LED_PIN0 = 0;
const int LED_PIN1 = 1;

// Create an instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A); // I2C device address 0x6A
float X = 0;
float Y = 0;
float Z = 0;
float Roll = 0;
float Pitch = 0;
float theta = 0;

// Constants for moving average
const int numReadings = 10; // Number of readings to average
float rollReadings[numReadings];
float pitchReadings[numReadings];
int rollIndex = 0;
int pitchIndex = 0;
float rollTotal = 0;
float pitchTotal = 0;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(9600);
    while (!Serial)
        ;

    pinMode(LED_PIN0, OUTPUT);
    pinMode(LED_PIN1, OUTPUT);
    // Call .begin() to configure the IMUs
    if (myIMU.begin() != 0)
    {
        Serial.println("Device error");
    }
    else
    {
        Serial.println("Device OK!");
    }
}

void loop()
{

    X = myIMU.readFloatAccelX();
    Y = myIMU.readFloatAccelY();
    Z = myIMU.readFloatAccelZ();
    Pitch = atan2(-X, sqrt(Y * Y + Z * Z)) * 180 / 3.14;
    Roll = atan2(Y, Z) * 180 / 3.14;

    // Add the new readings to the total
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
    Serial.print(" Roll = ");
    Serial.print(Roll);
    Serial.print(" Pitch = ");
    Serial.print(Pitch);
    Serial.print(" MA_Roll = ");
    Serial.print(rollAvg);
    Serial.print(" MA_Pitch = ");
    Serial.println(pitchAvg);


    if (abs(Roll - rollAvg) > 3) {
        digitalWrite(LED_PIN0, HIGH); // Turn on LED
    } else {
        digitalWrite(LED_PIN0, LOW); // Turn off LED
    }

    // Check if the difference exceeds 10 for Roll and pitchAvg
    if (abs(Roll - pitchAvg) > 3) {
        digitalWrite(LED_PIN1, HIGH); // Turn on LED
    } else {
        digitalWrite(LED_PIN1, LOW); // Turn off LED
    }
    delay(1000);
}
