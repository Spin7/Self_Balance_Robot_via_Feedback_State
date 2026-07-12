#ifndef PITCH_SENSOR_H
#define PITCH_SENSOR_H

#include <Wire.h>
#include <Arduino.h>

class PitchSensor {
  public:
    PitchSensor(uint8_t mpu_address = 0x68)
      : MPU_ADDR(mpu_address), angleGyro(0), angle(0), lastTime(0) {}

    void begin() {
      Wire.begin();
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x6B);  // PWR_MGMT_1
      Wire.write(0);     // Wake up
      Wire.endTransmission(true);

      lastTime = millis() / 1000.0;
    }

    void update() {
      readMPU6500();

      float currentTime = millis() / 1000.0;
      float elapsedTime = currentTime - lastTime;
      lastTime = currentTime;

      float angleAcc = atan2(accX, accZ) * 180.0 / PI;
      angleGyro += gyroY * elapsedTime;

      // Complementary filter
      angle = 0.98 * (angle + gyroY * elapsedTime) + 0.02 * angleAcc;
    }

    float getPitch() const {
      return angle;
    }

    float getPitch_Roll() const {
      return (float)gyroY / 65.5;;
    }

  private:
    const uint8_t MPU_ADDR;
    float accX, accZ, gyroY;
    float angleGyro, angle;
    float lastTime;

    void readMPU6500() {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B);  // ACCEL_XOUT_H
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 6, true);
      accX = (Wire.read() << 8 | Wire.read()) / 16384.0;
      Wire.read(); Wire.read(); // Ignore Y
      accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x43);  // GYRO_YOUT_H
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 2, true);
      gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    }
};

#endif
