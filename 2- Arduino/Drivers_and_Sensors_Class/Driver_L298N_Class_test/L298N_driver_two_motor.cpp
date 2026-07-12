#include "L298N_driver_two_motor.h"
#include <Arduino.h>


L298N_driver_two_motor::L298N_driver_two_motor(int ena, int in1, int in2, int in3, int in4, int enb)
  : ENA(ena), IN1(in1), IN2(in2), ENB(enb), IN3(in3), IN4(in4) {}

void L298N_driver_two_motor::begin() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void L298N_driver_two_motor::motor_A_forward(int v_a) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, v_a);
}

void L298N_driver_two_motor::motor_B_forward(int v_b) {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, v_b);
}

void L298N_driver_two_motor::motor_A_backward(int v_a) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, v_a);
}

void L298N_driver_two_motor::motor_B_backward(int v_b) {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, v_b);
}

void L298N_driver_two_motor::motor_A_stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

void L298N_driver_two_motor::motor_B_stop() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void L298N_driver_two_motor::stop_all() {
  motor_A_stop();
  motor_B_stop();
}
