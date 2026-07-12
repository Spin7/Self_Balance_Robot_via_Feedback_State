#include "Gyro.h"
#include <math.h>

void Gyro::kalman_1d(float KalmanInput, float KalmanMeasurement) {
  KalmanAngleRoll = KalmanAngleRoll + 0.004 * KalmanInput;
  KalmanUncertaintyAngleRoll = KalmanUncertaintyAngleRoll + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertaintyAngleRoll / (KalmanUncertaintyAngleRoll + 3 * 3);
  KalmanAngleRoll = KalmanAngleRoll + KalmanGain * (KalmanMeasurement - KalmanAngleRoll);
  KalmanUncertaintyAngleRoll = (1 - KalmanGain) * KalmanUncertaintyAngleRoll;
}

void Gyro::read_raw_gyro_acc() {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 14);

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Temperature
  int16_t GyroX = Wire.read() << 8 | Wire.read();

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  RateRoll = (float)GyroX / 65.5;
  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / 3.14159);
}

void Gyro::begin() {
  Wire.begin();
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  for (int i = 0; i < RateCalibrationNumber; i++) {
    read_raw_gyro_acc();
    RateCalibrationRoll += RateRoll;
    delay(3);
  }
  RateCalibrationRoll /= RateCalibrationNumber;
  LoopTimer = micros();
}

void Gyro::update() {
  read_raw_gyro_acc();
  RateRoll -= RateCalibrationRoll;
  kalman_1d(RateRoll, AngleRoll);

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}

float Gyro::get_roll() {
  return KalmanAngleRoll;
}

float Gyro::get_rate_roll() {
  return RateRoll;
}

