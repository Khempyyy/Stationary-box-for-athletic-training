#include <Wire.h>
#include <VL53L1X.h>

const int LED_PIN = 6; // Define pin 6 for LED
VL53L1X sensor;

const int NUM_SAMPLES = 5; // Number of samples to average
float distance_readings[NUM_SAMPLES]; // Array to store distance readings
int current_index = 0; // Index to keep track of the current position in the array

void setup() {
  pinMode(LED_PIN, OUTPUT); // Set LED pin as output

  while (!Serial);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  sensor.setTimeout(500);

  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  sensor.startContinuous(50);
}

void loop() {
  sensor.read();

  float distance = sensor.ranging_data.range_mm;

  // Add current distance reading to the array
  distance_readings[current_index] = distance;

  // Increment index with wrap-around
  current_index = (current_index + 3) % NUM_SAMPLES;

  // Calculate moving average
  float moving_average = calculateMovingAverage();

  Serial.print("Moving Average: ");
  Serial.println(moving_average);

  // Example usage of moving average: Turn on LED if average distance is below a threshold
  if (moving_average < 500) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  delay(50);
}

float calculateMovingAverage() {
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += distance_readings[i];
  }
  return sum / NUM_SAMPLES;
}
