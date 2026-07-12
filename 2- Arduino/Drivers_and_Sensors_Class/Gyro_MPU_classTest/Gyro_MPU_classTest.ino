#include <Wire.h>
#include "Gyro_MPU.h"

Gyro_MPU gyro;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay(250);
  gyro.begin();
}

void loop() {
  gyro.update();
  Serial.print("Pitch: "); Serial.println(gyro.get_PitchAngle());
  Serial.print("Pitch Rate: "); Serial.println(gyro.get_PitchAngle_Rate());
  delay(10);
}
