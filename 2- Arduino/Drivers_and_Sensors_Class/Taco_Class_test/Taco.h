#ifndef TACO_H
#define TACO_H

#include <Arduino.h>

class Taco {
  public:
    Taco(uint8_t pinInterrupcion, unsigned int pulsosPorRevolucion);
    void begin();
    void update();
    float get_rpm();
    float get_angular_rate();

  private:
    uint8_t pin;
    unsigned int ppr;
    volatile unsigned long contadorPulsos;
    volatile unsigned long ultimaInterrupcion;
    const unsigned long tiempoDebounce = 5000; // in microseconds
    unsigned long tiempoUltimaMedicion = 0;
    unsigned long intervaloMedicion = 1000; // in milliseconds
    unsigned long contadorPrevio = 0;
    float rpm = 0;
    float velocidadAngular = 0;

    static void isr_wrapper();
    void contarPulso();

    static Taco* instancia;
};

#endif
