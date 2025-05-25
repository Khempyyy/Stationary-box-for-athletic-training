#include <QMC5883LCompass.h>
int Yaw;
QMC5883LCompass compass;
int Yaw_remem = 0;
int previous_Yaw = 0;
const int LED_PIN0 = 0;
int different_Yaw = 0;
const int BUTTON_PIN = 7;
int different_Yaw_remem = 0;

void setup() {
  Serial.begin(9600);
  compass.init();
  pinMode(LED_PIN0, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
}

void loop() {
  compass.read();

  Yaw = compass.getAzimuth();
  different_Yaw_remem = abs(Yaw - Yaw_remem);
  
  if (digitalRead(BUTTON_PIN) == HIGH) { 
    Yaw_remem = Yaw;
  }  

  if (abs(different_Yaw_remem) > 3) {
    digitalWrite(LED_PIN0, HIGH);
  } else {
    digitalWrite(LED_PIN0, LOW);
  }  

  Serial.print("Yaw: ");
  Serial.print(Yaw);
  Serial.println();
  Serial.print("Yaw_remem: ");
  Serial.print(Yaw_remem);
  Serial.println();  
  Serial.print("diff_Yaw: ");
  Serial.print(different_Yaw_remem);
  Serial.println();
  
}