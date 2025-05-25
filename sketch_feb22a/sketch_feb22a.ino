#include <Wire.h>
#include <VL53L1X.h>
#include <QMC5883LCompass.h>

const int tactileSwitchPin = 2; // Change this to your switch pin
VL53L1X sensor;
QMC5883LCompass compass;

uint16_t distance;
uint16_t azimuth;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Use 400 kHz I2C

  compass.init();

  sensor.setTimeout(500);

  if (!sensor.init()) 
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Medium);
  sensor.setMeasurementTimingBudget(200000);
  sensor.startContinuous(200);

  pinMode(tactileSwitchPin, INPUT_PULLUP); // Enable internal pull-up resistor
}

void loop() {
  int a;
  int d;
  int c;
  compass.read();
  c = compass.getAzimuth();
  Serial.print("D: ");
  Serial.print(sensor.read());
  Serial.print(" A: ");
  Serial.println(c);

  if (digitalRead(tactileSwitchPin) == LOW) { // Switch pressed
    distance = sensor.read();
    azimuth = compass.getAzimuth();
  }
  // Display readings for debugging (optional)
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" Azimuth: ");
  Serial.println(azimuth);
  //delay(250);
}


