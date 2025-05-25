#include <Wire.h>
#include <VL53L1X.h>
float different_range = 0;
float previous_Z_axis_range = 0;
float Z_axis_range;
VL53L1X sensor;
const int LED_PIN0 = 0;

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
}

void loop()
{
  sensor.read();

  Z_axis_range = sensor.ranging_data.range_mm;
  different_range = abs(Z_axis_range - previous_Z_axis_range);

  if (abs(different_range) > 50) {
    digitalWrite(LED_PIN0, HIGH);
  } else {
    digitalWrite(LED_PIN0, LOW);
  }  

  Serial.print("Z: ");
  Serial.print(Z_axis_range);
  Serial.println();
  Serial.print("diff_Z: ");
  Serial.print(different_range);
  Serial.println();

  previous_Z_axis_range = Z_axis_range;
  delay(250);
}