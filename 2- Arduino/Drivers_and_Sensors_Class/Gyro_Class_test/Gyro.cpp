#include "Gyro.h"
#include <math.h>
#include <Arduino.h>


Gyro::Gyro() {
    RateCalibrationRoll = RateCalibrationPitch = RateCalibrationYaw = 0;
    KalmanAngleRoll = 0;
    KalmanUncertaintyAngleRoll = 4.0f;
    KalmanAnglePitch = 0;
    KalmanUncertaintyAnglePitch = 0.2f;
    AngleYaw = 0;
}

void Gyro::begin() {
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    for (int i = 0; i < RateCalibrationNumber; i++) {
        gyroSignals();
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

void Gyro::gyroSignals() {
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

    AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180 / 3.14159);
    AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180 / 3.14159);
}

void Gyro::kalman1D(float& state, float& uncertainty, float input, float measurement) {
    state += 0.004 * input;
    uncertainty += 0.004 * 0.004 * 16.0;

    float gain = uncertainty / (uncertainty + 9.0);
    state += gain * (measurement - state);
    uncertainty *= (1 - gain);

    Kalman1DOutput[0] = state;
    Kalman1DOutput[1] = uncertainty;
}

void Gyro::update() {
    gyroSignals();

    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    kalman1D(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];

    kalman1D(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];

    AngleYaw += RateYaw * 0.002;

    while (micros() - LoopTimer < 4000);
    LoopTimer = micros();
}

float Gyro::getRollAngle() {
    return KalmanAngleRoll;
}

float Gyro::getPitchAngle() {
    return KalmanAnglePitch-1;
}

float Gyro::getYawAngle() {
    return AngleYaw;
}


