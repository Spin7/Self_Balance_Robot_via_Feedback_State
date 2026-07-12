#ifndef GYRO_MPU_H
#define GYRO_MPU_H
#include <Arduino.h>
#include <Wire.h>

class Gyro_MPU {
public:
    Gyro_MPU();

    void begin();
    void update();

    float get_RollAngle();
    float get_PitchAngle();
    float get_YawAngle();

    float get_RollAngle_Rate();
    float get_PitchAngle_Rate();
    float get_YawAngle_Rate();

private:
    float RateRoll, RatePitch, RateYaw;
    float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
    const int RateCalibrationNumber = 2000;
    float AccX, AccY, AccZ;
    float AngleRoll, AnglePitch, AngleYaw;
    float KalmanAngleRoll, KalmanUncertaintyAngleRoll;
    float KalmanAnglePitch, KalmanUncertaintyAnglePitch;
    float Kalman1DOutput[2];
    uint32_t LoopTimer;

    void gyro_signals();
    void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
};

#endif
