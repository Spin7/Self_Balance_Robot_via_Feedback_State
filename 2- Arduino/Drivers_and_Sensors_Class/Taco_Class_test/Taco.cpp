#include "Taco.h"

Taco* Taco::instancia = nullptr;

Taco::Taco(uint8_t pinInterrupcion, unsigned int pulsosPorRevolucion)
  : pin(pinInterrupcion), ppr(pulsosPorRevolucion), contadorPulsos(0), ultimaInterrupcion(0) {}

void Taco::begin() {
  instancia = this;
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin), isr_wrapper, RISING);
}

void Taco::isr_wrapper() {
  if (instancia) {
    instancia->contarPulso();
  }
}

void Taco::contarPulso() {
  unsigned long tiempoActual = micros();
  if (tiempoActual - ultimaInterrupcion > tiempoDebounce) {
    contadorPulsos++;
    ultimaInterrupcion = tiempoActual;
  }
}

void Taco::update() {
  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoUltimaMedicion >= intervaloMedicion) {
    noInterrupts();
    unsigned long pulsos = contadorPulsos - contadorPrevio;
    contadorPrevio = contadorPulsos;
    interrupts();

    float revoluciones = (float)pulsos / ppr;
    rpm = revoluciones * (60000.0 / intervaloMedicion);
    velocidadAngular = rpm * (2 * PI / 60.0);

    tiempoUltimaMedicion = tiempoActual;
  }
}

float Taco::get_rpm() {
  return rpm;
}

float Taco::get_angular_rate() {
  return velocidadAngular;
}
