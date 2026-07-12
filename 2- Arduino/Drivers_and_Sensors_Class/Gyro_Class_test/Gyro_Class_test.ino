#include "Gyro.h"

Gyro mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    delay(250);
    mpu.begin();
    
}

void loop() {
  mpu.update();
  Serial.print("PitchAngle [°]: ");
  Serial.println(mpu.getPitchAngle());
}
