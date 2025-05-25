#include "LSM6DS3.h"
#include "Wire.h"
const int LED_PIN = 6; // กำหนดขาพิน 6 สำหรับ LED
//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

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
    pinMode(LED_PIN, OUTPUT); // ตั้งค่าขาพิน LED เป็น OUTPUT
}

void loop() {
    //Accelerometer
    Serial.print("\nAccelerometer:\n");
    Serial.print(" X1 = ");
    Serial.println(myIMU.readFloatAccelX(), 4);
    Serial.print(" Y1 = ");
    Serial.println(myIMU.readFloatAccelY(), 4);
    Serial.print(" Z1 = ");
    Serial.println(myIMU.readFloatAccelZ(), 4);

    //Gyroscope
    Serial.print("\nGyroscope:\n");
    Serial.print(" X1 = ");
    Serial.println(myIMU.readFloatGyroX(), 4);
    Serial.print(" Y1 = ");
    Serial.println(myIMU.readFloatGyroY(), 4);
    Serial.print(" Z1 = ");
    Serial.println(myIMU.readFloatGyroZ(), 4);

    delay(1000);
    if (abs(myIMU.readFloatAccelX()) > 0.2 ||
      abs(myIMU.readFloatAccelY()) > 0.2 ||
      abs(myIMU.readFloatAccelZ()) > 1.5) {
    digitalWrite(LED_PIN, HIGH); 
    } else {
    digitalWrite(LED_PIN, LOW); 
    }

    if (abs(myIMU.readFloatGyroZ()) > 5) {
    digitalWrite(LED_PIN, HIGH); 
    } else {
    digitalWrite(LED_PIN, LOW); 
    }
    //if (abs(myIMU.readFloatGyroX()) > 0.9 ||
    //  abs(myIMU.readFloatGyroY()) > 3 ||
    //  abs(myIMU.readFloatGyroZ()) > 0.5) {
    //digitalWrite(LED_PIN, HIGH); 
    //} else {
    //digitalWrite(LED_PIN, LOW); 
    //}


}