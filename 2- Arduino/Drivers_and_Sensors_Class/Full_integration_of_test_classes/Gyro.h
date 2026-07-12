#ifndef GYRO_H
#define GYRO_H

#include <Wire.h>

class Gyro {
  public:
    void begin();
    void update();
    float get_roll();
    float get_rate_roll();

  private:
    float RateRoll, RateCalibrationRoll = 0;
    float AccX, AccY, AccZ;
    float AngleRoll;
    float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
    uint32_t LoopTimer;
    const int addr = 0x68;
    const int RateCalibrationNumber = 4000;

    void kalman_1d(float KalmanInput, float KalmanMeasurement);
    void read_raw_gyro_acc();
};

#endif

