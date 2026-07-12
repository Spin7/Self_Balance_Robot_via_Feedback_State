#include "Gyro.h"
#include "Taco.h"
#include "L298N_driver_two_motor.h"

// Motors: ENA, IN1, IN2, ENB, IN3, IN4
L298N_driver_two_motor motorDriver(10, 9, 8, 5, 7, 6);

// Tachometers: pinA and pinB
Taco tacoIzquierdo(12,20);  // interrupt pin left wheel
Taco tacoDerecho(12,20);    // interrupt pin right wheel

// Gyroscope
Gyro mpu;

void setup() {
  Serial.begin(115200);

  motorDriver.begin();
  tacoIzquierdo.begin();
  tacoDerecho.begin();
  mpu.begin();
}

void loop() {
  // Update sensor readings
  tacoIzquierdo.update();
  tacoDerecho.update();
  mpu.update();
  

  // Read sensors
  float rpmIzq = tacoIzquierdo.get_rpm();
  float rpmDer = tacoDerecho.get_rpm();
  float yaw = mpu.get_roll();
  float yaw_rate = mpu.get_rate_roll();

  // Motor control (move forward with PWM=150)
  motorDriver.motor_A_forward(150);
  motorDriver.motor_B_forward(150);

  // Motor control (move backward with PWM=150)
  //motorDriver.motor_A_backward(150);
  //motorDriver.motor_B_backward(150);

  // Motor control (stop)
  //motorDriver.motor_A_stop();
  //motorDriver.motor_B_stop();

  // Print data
  Serial.print("RPM Izquierda: ");
  Serial.print(rpmIzq);
  Serial.print(" | RPM Derecha: ");
  Serial.print(rpmDer);
  Serial.print(" | Yaw: ");
  Serial.print(yaw, 2);
  Serial.print("° | Vel. Angular: ");
  Serial.print(yaw_rate, 2);
  Serial.println(" °/s");

  delay(500); // Adjust as needed
}
