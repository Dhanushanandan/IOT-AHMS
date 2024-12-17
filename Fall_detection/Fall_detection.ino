#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Threshold values for detecting falls and stabilization
#define FALL_THRESHOLD 200    // Threshold for fall detection (adjust based on your environment)
#define STABILIZATION_THRESHOLD 20 // Threshold for stabilization
#define STABILIZATION_TIME 1000    // Time (ms) to confirm stabilization

bool fallDetected = false;
unsigned long stabilizationStartTime = 0;

void setup() 
{
  Serial.begin(115200);

  // Initialize MPU6050
  Serial.println("Initialize MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope
  mpu.calibrateGyro();

  // Set threshold sensitivity
  mpu.setThreshold(3);

  // Check settings
  checkSettings();
}

void checkSettings()
{
  Serial.println();
  Serial.print(" * Sleep Mode:        ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Clock Source:      ");
  switch (mpu.getClockSource()) {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }

  Serial.print(" * Gyroscope:         ");
  switch (mpu.getScale()) {
    case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
    case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
  }

  Serial.print(" * Gyroscope offsets: ");
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getGyroOffsetZ());

  Serial.println();
}

void loop()
{
  Vector normGyro = mpu.readNormalizeGyro();

  // Calculate total normalized gyroscope value
  float totalGyro = sqrt(normGyro.XAxis * normGyro.XAxis + 
                         normGyro.YAxis * normGyro.YAxis + 
                         normGyro.ZAxis * normGyro.ZAxis);

  // Fall detection logic
  if (!fallDetected && totalGyro > FALL_THRESHOLD) {
    fallDetected = true;
    Serial.println("ALERT: Fall detected!");
    stabilizationStartTime = millis(); // Start stabilization timer
  }

  // Stabilization logic
  if (fallDetected) {
    if (totalGyro < STABILIZATION_THRESHOLD) {
      if (millis() - stabilizationStartTime >= STABILIZATION_TIME) {
        fallDetected = false; // Reset fall state
        Serial.println("Stabilization complete. Fall state cleared.");
      }
    } else {
      stabilizationStartTime = millis(); // Reset stabilization timer if still unstable
    }
  }

  delay(100); // Delay to reduce serial monitor flooding
}
