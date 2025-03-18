#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>

// Initialize ADXL345
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Moving average parameters
const int windowSize = 10;
float accelXBuffer[windowSize];
float accelYBuffer[windowSize];
float accelZBuffer[windowSize];

int bufferIndex = 0;
float sumX = 0, sumY = 0, sumZ = 0;

void setup() {
  Serial.begin(115200);

  if (!accel.begin()) {
    Serial.println("Failed to find ADXL345 sensor!");
    while (1);
  }

  accel.setRange(ADXL345_RANGE_16_G);
  Serial.println("ADXL345 Initialized!");

  // Initialize moving average buffers
  for (int i = 0; i < windowSize; i++) {
    accelXBuffer[i] = 0;
    accelYBuffer[i] = 0;
    accelZBuffer[i] = 0;
  }
}

void loop() {
  sensors_event_t event;
  accel.getEvent(&event);

  // Read raw acceleration
  float rawX = event.acceleration.x;
  float rawY = event.acceleration.y;
  float rawZ = event.acceleration.z;

  // Moving Average Filter for X
  sumX -= accelXBuffer[bufferIndex];
  accelXBuffer[bufferIndex] = rawX;
  sumX += rawX;

  // Moving Average Filter for Y
  sumY -= accelYBuffer[bufferIndex];
  accelYBuffer[bufferIndex] = rawY;
  sumY += rawY;

  // Moving Average Filter for Z
  sumZ -= accelZBuffer[bufferIndex];
  accelZBuffer[bufferIndex] = rawZ;
  sumZ += rawZ;

  bufferIndex = (bufferIndex + 1) % windowSize;

  float filteredX = sumX / windowSize;
  float filteredY = sumY / windowSize;
  float filteredZ = sumZ / windowSize;

  // Calculate Euler angles (pitch & roll)
  float roll = atan2(filteredY, filteredZ) * 180.0 / PI;
  float pitch = atan2(-filteredX, sqrt(filteredY * filteredY + filteredZ * filteredZ)) * 180.0 / PI;

  // Print results
  Serial.print("Filtered X: ");
  Serial.print(filteredX);
  Serial.print(" | Y: ");
  Serial.print(filteredY);
  Serial.print(" | Z: ");
  Serial.print(filteredZ);
  Serial.print(" | Pitch: ");
  Serial.print(pitch, 2);
  Serial.print(" deg | Roll: ");
  Serial.print(roll, 2);
  Serial.println(" deg");

  delay(100); // Sampling interval
}
