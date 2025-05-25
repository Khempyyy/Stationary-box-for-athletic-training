#include <QMC5883LCompass.h>
int Yaw;
QMC5883LCompass compass;
int previous_Yaw = 0;
const int LED_PIN2 = 2;
int different_Yaw = 0;
void setup() {
  Serial.begin(9600);
  compass.init();
  pinMode(LED_PIN2, OUTPUT);
}

void loop() {
  compass.read();

  Yaw = compass.getAzimuth();
  different_Yaw = abs(Yaw - previous_Yaw);

  if (abs(different_Yaw) > 3) {
    digitalWrite(LED_PIN2, HIGH);

  } else {
    digitalWrite(LED_PIN2, LOW);
  }  

  Serial.print("Yaw: ");
  Serial.print(Yaw);
  Serial.println();
  Serial.print("diff_Yaw: ");
  Serial.print(different_Yaw);
  Serial.println();
  previous_Yaw = Yaw; 
  delay(250);
}