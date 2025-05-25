#include <Wire.h>
#include <VL53L1X.h>
float different_range_remem = 0;
float previous_Z_axis_range = 0;
float Z_axis_range;
float Z_axis_range_remem = 0;
VL53L1X sensor;
const int LED_PIN0 = 0;
const int BUTTON_PIN = 7;

void setup()
{
  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);

  pinMode(LED_PIN0, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
}

void loop()
{
  sensor.read();

  Z_axis_range = sensor.ranging_data.range_mm;
  different_range_remem = abs(Z_axis_range - Z_axis_range_remem);

  if (digitalRead(BUTTON_PIN) == HIGH) { 
    Z_axis_range_remem = Z_axis_range;
  }  

  if (abs(different_range_remem) > 50) {
    digitalWrite(LED_PIN0, HIGH);

  } else {
    digitalWrite(LED_PIN0, LOW);
  }  

  Serial.print("Z: ");
  Serial.print(Z_axis_range);
  Serial.println();
  Serial.print("Z_remem: ");
  Serial.print(Z_axis_range_remem);
  Serial.println();
  Serial.print("diff_Z_remem: ");
  Serial.print(Z_axis_range_remem);
  Serial.println();

}