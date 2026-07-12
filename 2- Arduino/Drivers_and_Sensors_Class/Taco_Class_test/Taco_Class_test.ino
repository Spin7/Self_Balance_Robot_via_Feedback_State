#include "Taco.h"

//Taco taco(12, 20);  // pin 12, 20 pulses per revolution
Taco tacoA(2, 20);

void setup() {
  Serial.begin(115200);
  tacoA.begin();
}

void loop() {
  

  tacoA.update();
  Serial.print(" | Vel. angular: ");
  Serial.print(tacoA.get_angular_rate());
  Serial.println(" rad/s");

  delay(100);
}

