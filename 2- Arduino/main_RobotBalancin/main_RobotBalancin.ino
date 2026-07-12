#include <Wire.h>
#include "L298N_driver_two_motor.h"
#include "ControlD.h"
#include "Taco_twoPin.h"
#include "VelocityIntegrator.h"
#include "Gyro_MPU.h"
#include <math.h>

// Custom classes
Taco tacoA(2, 20);
Taco tacoB(3, 20);
Gyro_MPU gyro;
L298N_driver_two_motor motorDriver(5, 8, 9, 10, 11, 6);
ControlD controller;
VelocityIntegrator integrador;
//////////////////////////////////////////////
bool flag = false;
float targetAngle = 0.0;  // The robot must be vertical
float u;
int sing =1;
float prev_Pitch = 0;
//////////////////////////////////////////////
const float Tm = 0.01; // Sampling time in seconds
unsigned long t_ant = 0.0;
unsigned long t_act;
unsigned long t_ant2 = 0.0;
bool first_time = true;

////////////////////Best K//////////////////////////

float k1 = -4.7152;
float k2 = -3.3278;
float k3 = -17.0580;
float k4 = -1.8229;

//////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  tacoA.begin();
  tacoB.begin();
  Wire.begin();
  Wire.setClock(400000);
  delay(250);
  gyro.begin();
  motorDriver.begin();
  delay(3000);
}

void loop() {
  //////////////// FEEDBACK GAIN K DEFINITION ////////////////
  float k1 = -4.7152;
  float k2 = -3.3278;
  float k3 = -17.0580;
  float k4 = -1.8229;

  //////////////// STATE VARIABLES ACQUISITION ////////////////
  gyro.update(); // Updates the gyroscope reading
  float Pitch_angle = gyro.get_PitchAngle(); // Gets the theta angle
  float Pitch_rate = gyro.get_PitchAngle_Rate(); // Gets the angular velocity

  // Complementary filter for the angle
  Pitch_rate = (0.6 * Pitch_rate+ (1 - 0.6) * prev_Pitch)-3; // Subtract 3 to correct the offset
  prev_Pitch= Pitch_rate;

  tacoA.update(); // Updates the wheel A tachometer
  tacoB.update(); // Updates the wheel B tachometer
  // Takes velocity as the average of the previous readings
  float w_ruedas = (tacoA.get_angular_rate()+tacoB.get_angular_rate())/2;
  float v_lineal = w_ruedas*0.031; // Linear wheel velocity

  integrador.setVelocity(sing*v_lineal);
  integrador.update(); 

  // Position is obtained by numerically integrating the velocity
  float x = integrador.getPosition(); 

  //////////////// //////////////// //////////////// ////////////////
  // 10 ms sampling
  if (millis() - t_ant >= 10){
    t_ant  = millis();
    // Calculation of the control effort
    u =-( x*k1 + v_lineal*k2 + Pitch_angle*k3+ Pitch_rate*k4);
    } 
  // Motor control with u
  Control_Motors(u);
}

void Control_Motors (float output) {
  int baseSpeed = constrain(abs(output), 0, 230);

  if (output > 1) {
    // Tilted forward → move backward
    motorDriver.motor_A_backward(baseSpeed);
    motorDriver.motor_B_backward(baseSpeed+1);
    sing =1;
  } else if (output < -1) {
    // Tilted backward → move forward
    motorDriver.motor_A_forward(baseSpeed);
    motorDriver.motor_B_forward(baseSpeed+1);
    sing =-1;
  } else {
    // Near equilibrium → stop
    motorDriver.motor_A_stop();
    motorDriver.motor_B_stop();
  }
}
