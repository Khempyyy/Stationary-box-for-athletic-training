
#include <QMC5883LCompass.h>

QMC5883LCompass compass;
int previousAzimuth = 0;

const int LED_PIN = 6;


void setup() {
  Serial.begin(9600);
  compass.init();
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  int currentAzimuth;
  
  // Read compass values
  compass.read();

  // Return Azimuth reading
  currentAzimuth = compass.getAzimuth();
  
  Serial.print("A: ");
  Serial.print(currentAzimuth);
  Serial.println();

  if (abs(currentAzimuth - previousAzimuth) > 3) {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
  }
  
  // อัพเดทค่า Azimuth ที่เก่า
  previousAzimuth = currentAzimuth;
  delay(250);
}
