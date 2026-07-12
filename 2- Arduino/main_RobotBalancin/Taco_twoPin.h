#ifndef TACO_H
#define TACO_H

#include <Arduino.h>

class Taco {
  public:
    Taco(uint8_t interruptPin, unsigned int pulsesPerRevolution);
    void begin();
    void update();
    float get_rpm();
    float get_angular_rate();

  private:
    uint8_t pin;
    unsigned int ppr;
    volatile unsigned long pulseCounter = 0;
    volatile unsigned long lastInterrupt = 0;
    const unsigned long debounceTime = 5000;

    unsigned long lastMeasurementTime = 0;
    unsigned long measurementInterval = 1000;
    unsigned long previousCounter = 0;

    float rpm = 0;
    float angularVelocity = 0;

    void countPulse();

    // ISR helpers
    static void isr0();
    static void isr1();
    static Taco* instance0;
    static Taco* instance1;
};

#endif
