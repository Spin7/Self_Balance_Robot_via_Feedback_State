#ifndef VELOCITY_INTEGRATOR_H
#define VELOCITY_INTEGRATOR_H

#include <Arduino.h>

class VelocityIntegrator {
private:
    float position;           // Integrated position [m]
    float lastVelocity;       // Last known velocity [m/s]
    unsigned long lastTime;   // Last update time [us]

public:
    VelocityIntegrator() {
        position = 0.0;
        lastVelocity = 0.0;
        lastTime = micros();
    }

    // Sets the current robot velocity (in m/s)
    void setVelocity(float velocity) {
        lastVelocity = velocity;
    }

    // Must be called periodically to integrate the position
    void update() {
        unsigned long currentTime = micros();
        unsigned long dtMicros = currentTime - lastTime;

        if (dtMicros >= 5000) { // Only updates if at least 5 ms have passed
            float deltaTime = dtMicros / 1e6f;
            position += lastVelocity * deltaTime;
            lastTime = currentTime;
        }
    }

    // Returns the integrated position in meters
    float getPosition() const {
        return position;
    }

    // Resets the position and timer
    void reset() {
        position = 0.0f;
        lastTime = micros();
    }
};

#endif // VELOCITY_INTEGRATOR_H
