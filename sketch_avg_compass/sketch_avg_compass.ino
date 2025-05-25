#include <Wire.h>
#include <QMC5883LCompass.h>

const int LED_PIN = 6;
const int WINDOW_SIZE = 10; // Adjust window size as needed
const unsigned long SAMPLE_INTERVAL = 1000; // Interval in milliseconds

QMC5883LCompass compass;
int yValues[WINDOW_SIZE];
int currentIndex = 0;
unsigned long lastSampleTime = 0;
int prevMovingAverage = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  int x, y, z;
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  // Update rolling window
  yValues[currentIndex] = y;
  currentIndex = (currentIndex + 1) % WINDOW_SIZE;

  // Calculate current moving average
  int sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sum += yValues[i];
  }
  int currentMovingAverage = sum / WINDOW_SIZE;

  // Calculate time difference
  unsigned long currentTime = millis();
  unsigned long timeDifference = currentTime - lastSampleTime;

  // If one second has passed, calculate the difference
  if (timeDifference >= SAMPLE_INTERVAL) {
    int difference = currentMovingAverage - prevMovingAverage;
    Serial.print("Difference between current and 1 second ago moving average (Y direction): ");
    Serial.println(difference);
    
    // Update previous moving average
    prevMovingAverage = currentMovingAverage;
    
    // Reset sample time
    lastSampleTime = currentTime;
  }

  delay(100); // Adjust as needed
}
