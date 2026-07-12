#ifndef L298N_DRIVER_TWO_MOTOR_H
#define L298N_DRIVER_TWO_MOTOR_H

#include <Arduino.h>

class L298N_driver_two_motor {
private:
  // Motor A pins
  int ENA, IN1, IN2;
  // Motor B pins
  int ENB, IN3, IN4;

public:
  // Constructor
  L298N_driver_two_motor(int ena, int in1, int in2, int enb, int in3, int in4);

  // Initialize the pins
  void begin();

  // Forward movement
  void motor_A_forward(int v_a);
  void motor_B_forward(int v_b);

  // Backward movement
  void motor_A_backward(int v_a);
  void motor_B_backward(int v_b);

  // Stop motors
  void motor_A_stop();
  void motor_B_stop();

  // Both motors
  void stop_all();
};

#endif
