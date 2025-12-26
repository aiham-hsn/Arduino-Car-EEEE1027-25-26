// The following code is modified from https://matthewtechub.com/2024/09/06/slope-detection-by-gyro-accelerometer/

// Calculate and Print Slope in X and Y Directions with MPU6050
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(20); // Wait for Serial Monitor to open

  // Try to initialize the MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(20);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set the accelerometer range
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  // Set the gyro range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // Set the filter bandwidth
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(200);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate tilt angles in degrees
  float angleX = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
  float angleY = atan2(-a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;

  // Normalize angles to a range of -90° to 90°
  if (angleX > 90) angleX = 180 - angleX;
  if (angleX < -90) angleX = -180 - angleX;
  
  if (angleY > 90) angleY = 180 - angleY;
  if (angleY < -90) angleY = -180 - angleY;

  // Calculate slopes for X and Y axes
  float slopeX = tan(angleX * DEG_TO_RAD);  // Convert angle to radians and calculate slope
  float slopeY = tan(angleY * DEG_TO_RAD);  // Convert angle to radians and calculate slope

  // Print tilt angles and slopes
  Serial.print("TiltAngleX:");
  Serial.print(angleX);
  Serial.print(",");
  Serial.print("SlopeX:");
  Serial.print(slopeX);
  Serial.print(",");
  Serial.print("TiltAngleY:");
  Serial.print(angleY);
  Serial.print(",");
  Serial.print("SlopeY:");
  Serial.print(slopeY);
  Serial.println("");

  delay(100);
}
