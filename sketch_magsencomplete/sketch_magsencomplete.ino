#include <QMC5883LCompass.h>
#include "Wire.h"

QMC5883LCompass compass;
const int LED_PIN0 = 0;
float previousAzimuth = 0;



void setup() {
  Serial.begin(9600);
  compass.init();
  pinMode(LED_PIN0, OUTPUT);
  
}

void loop() {
  compass.read();
  
  // Return Azimuth reading
  float currentAzimuth = compass.getAzimuth();
  float diff_azimuth = previousAzimuth - currentAzimuth;
  previousAzimuth = currentAzimuth;

  if (abs(diff_azimuth) > 3) {
    digitalWrite(LED_PIN0, HIGH);
    delay(1000);
    digitalWrite(LED_PIN0, LOW);
  }
  Serial.println(diff_azimuth);
  delay(250);
}
