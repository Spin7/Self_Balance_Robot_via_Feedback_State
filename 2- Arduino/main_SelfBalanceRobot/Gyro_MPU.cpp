#include "Gyro_MPU.h"
#include <math.h>
#include <Arduino.h>
Gyro_MPU::Gyro_MPU()
  : RateRoll(0), RatePitch(0), RateYaw(0),
    RateCalibrationRoll(0), RateCalibrationPitch(0), RateCalibrationYaw(0),
    AccX(0), AccY(0), AccZ(0),
    AngleRoll(0), AnglePitch(0), AngleYaw(0),
    KalmanAngleRoll(0), KalmanUncertaintyAngleRoll(4.0),
    KalmanAnglePitch(0), KalmanUncertaintyAnglePitch(0.6),
    LoopTimer(0) {}

void Gyro_MPU::begin() {
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    for (int i = 0; i < RateCalibrationNumber; i++) {
        gyro_signals();
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        delay(3);
    }

    RateCalibrationRoll /= RateCalibrationNumber;
    RateCalibrationPitch /= RateCalibrationNumber;
    RateCalibrationYaw /= RateCalibrationNumber;

    LoopTimer = micros();
}

void Gyro_MPU::gyro_signals() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();

    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();

    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;

    AccX = (float)AccXLSB / 4096;
    AccY = (float)AccYLSB / 4096;
    AccZ = (float)AccZLSB / 4096;

    AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / M_PI);
    AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / M_PI);
}

void Gyro_MPU::kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState += 0.004 * KalmanInput;
    KalmanUncertainty += 0.004 * 0.004 * 16.0;

    float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9.0);
    KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty *= (1 - KalmanGain);

    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

void Gyro_MPU::update() {
    gyro_signals();

    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    AngleYaw += RateYaw * 0.002;

    while (micros() - LoopTimer < 4000);
    LoopTimer = micros();
}

float Gyro_MPU::get_RollAngle() {
    return KalmanAngleRoll;
}

float Gyro_MPU::get_PitchAngle() {
    return KalmanAnglePitch;
}

float Gyro_MPU::get_YawAngle() {
    return AngleYaw;
}

float Gyro_MPU::get_RollAngle_Rate() {
    return RateRoll;
}

float Gyro_MPU::get_PitchAngle_Rate() {
    return RatePitch;
}

float Gyro_MPU::get_YawAngle_Rate() {
    return RateYaw;
}
