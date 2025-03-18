#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Initialize ADXL345
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Hysteresis parameters
const float centralThreshold = 2.25;  // Center point (m/s^2)
const float windowWidth = 1.5;        // Width of hysteresis window (m/s^2)

float upperThreshold = 0;
float lowerThreshold = 0;
bool movementDetected = false;

// Moving average parameters
const int windowSize = 10;
float accelXBuffer[windowSize];
int bufferIndex = 0;
float sum = 0;

void setup() {
  Serial.begin(115200);

  if (!accel.begin()) {
    Serial.println("Failed to find ADXL345 sensor!");
    while (1);
  }

  accel.setRange(ADXL345_RANGE_16_G);
  Serial.println("ADXL345 Initialized!");
  
  // Calculate dynamic thresholds
  upperThreshold = centralThreshold + (windowWidth / 2);
  lowerThreshold = centralThreshold - (windowWidth / 2);

  // Initialize moving average buffer
  for (int i = 0; i < windowSize; i++) {
    accelXBuffer[i] = 0;
  }

  Serial.print("Upper Threshold: ");
  Serial.println(upperThreshold);
  Serial.print("Lower Threshold: ");
  Serial.println(lowerThreshold);
}

void loop() {
  sensors_event_t event;
  accel.getEvent(&event);
  
  float rawAccelX = event.acceleration.x;

  // Moving Average Filter
  sum -= accelXBuffer[bufferIndex];      // Remove oldest value
  accelXBuffer[bufferIndex] = rawAccelX; // Add new value
  sum += rawAccelX;
  bufferIndex = (bufferIndex + 1) % windowSize;
  
  float filteredAccelX = sum / windowSize;

  // Hysteresis Logic (based on central threshold and window width)
  if (!movementDetected && filteredAccelX > upperThreshold) {
    movementDetected = true;
    Serial.println("Movement Detected!");
  } else if (movementDetected && filteredAccelX < lowerThreshold) {
    movementDetected = false;
    Serial.println("Movement Ended.");
  }

  // Debug output
  Serial.print("Raw X: ");
  Serial.print(rawAccelX);
  Serial.print(" | Filtered X: ");
  Serial.println(filteredAccelX);
  
  delay(100); // Sampling interval
}
