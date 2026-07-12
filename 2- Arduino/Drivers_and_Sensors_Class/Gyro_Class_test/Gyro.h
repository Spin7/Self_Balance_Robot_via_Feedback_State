#ifndef GYRO_H
#define GYRO_H

#include <Wire.h>

class Gyro {
public:
    Gyro();
    void begin();
    void update();
    float getRollAngle();
    float getPitchAngle();
    float getYawAngle();

private:
    float RateRoll, RatePitch, RateYaw;
    float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
    int RateCalibrationNumber = 2000;

    float AccX, AccY, AccZ;
    float AngleRoll, AnglePitch, AngleYaw;

    uint32_t LoopTimer;

    float KalmanAngleRoll, KalmanUncertaintyAngleRoll;
    float KalmanAnglePitch, KalmanUncertaintyAnglePitch;
    float Kalman1DOutput[2];

    void gyroSignals();
    void kalman1D(float& state, float& uncertainty, float input, float measurement);
};

#endif


