#pragma once
#include <Arduino.h>
class FastIO
{
private:
    uint8_t mask;
    uint8_t port;
    volatile uint8_t *outputReg;
    volatile uint8_t *inputReg;
    volatile uint8_t *modeReg;

public:
    FastIO();
    virtual ~FastIO();

    void begin(uint8_t pin);
    void pinMode(uint8_t mode);
    void digitalWrite(uint8_t val);
    int digitalRead();

    void pulse(int times = 1, int pulseDuration = 30);
};
