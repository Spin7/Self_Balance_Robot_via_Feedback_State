#include "L298N_driver_two_motor.h"

// Create driver instance with the pins
//L298N_driver_two_motor motorDriver(10, 9, 8, 5, 7, 6);
L298N_driver_two_motor motorDriver(5,8,9,10,11,6);

void setup() {
  Serial.begin(115200);
  motorDriver.begin();  // Initialize the pins
}

void loop() {
  motorDriver.motor_A_forward(100);
  motorDriver.motor_B_forward(100);
  //motorDriver.motor_A_backward(60)
  //motorDriver.motor_B_backward(75);
  //motorDriver.motor_A_stop();
  //motorDriver.motor_B_stop();
  //motorDriver.motor_B_stop();
  delay(1000);

  motorDriver.motor_A_backward(100);
  motorDriver.motor_B_backward(100);
  //motorDriver.motor_B_forward(1000);
  //motorDriver.motor_A_stop();
  delay(1000);

  //motorDriver.stop_all();
 // delay(2000);
}

