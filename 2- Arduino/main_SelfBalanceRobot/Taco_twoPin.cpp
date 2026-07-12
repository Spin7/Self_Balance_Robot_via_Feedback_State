#include "Taco_twoPin.h"
#include <Arduino.h>
Taco* Taco::instance0 = nullptr;
Taco* Taco::instance1 = nullptr;

Taco::Taco(uint8_t interruptPin, unsigned int pulsesPerRevolution)
    : pin(interruptPin), ppr(pulsesPerRevolution) {}

void Taco::begin() {
  pinMode(pin, INPUT_PULLUP);
  if (pin == 2) {
    instance0 = this;
    attachInterrupt(digitalPinToInterrupt(pin), isr0, RISING);
  } else if (pin == 3) {
    instance1 = this;
    attachInterrupt(digitalPinToInterrupt(pin), isr1, RISING);
  }
}

void Taco::isr0() {
  if (instance0) instance0->countPulse();
}

void Taco::isr1() {
  if (instance1) instance1->countPulse();
}

void Taco::countPulse() {
  unsigned long now = micros();
  if (now - lastInterrupt > debounceTime) {
    pulseCounter++;
    lastInterrupt = now;
  }
}

void Taco::update() {
  unsigned long now = millis();
  if (now - lastMeasurementTime >= measurementInterval) {
    unsigned long pulses = pulseCounter;
    pulseCounter = 0;
    rpm = (pulses * 60000.0) / (ppr * measurementInterval);
    angularVelocity = (rpm * 2.0 * PI) / 60.0;
    lastMeasurementTime = now;
  }
}

float Taco::get_rpm() {
  return rpm;
}

float Taco::get_angular_rate() {
  return angularVelocity;
}
