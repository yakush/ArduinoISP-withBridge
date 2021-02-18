#include "fastIO.h"

FastIO::FastIO()
{
}

FastIO::~FastIO()
{
}

void FastIO::begin(uint8_t pin)
{
    mask = digitalPinToBitMask(pin);
    port = digitalPinToPort(pin);
    outputReg = portOutputRegister(port);
    inputReg = portInputRegister(port);
    modeReg = portModeRegister(port);
}

void FastIO::pinMode(uint8_t mode)
{
    uint8_t oldSREG = SREG;
    cli();

    switch (mode)
    {
    case INPUT:

        *modeReg &= ~mask;   // INPUT
        *outputReg &= ~mask; // PULLUP
        break;

    case INPUT_PULLUP:
        *modeReg &= ~mask;  // INPUT
        *outputReg |= mask; // NO PULLUP
        break;

    case OUTPUT:
        *modeReg |= mask; // OUTPUT
        break;
    }

    SREG = oldSREG;
}

void FastIO::digitalWrite(uint8_t val)
{
    if (val)
        *outputReg |= mask; // HI
    else
        *outputReg &= ~mask; // LOW
}

int FastIO::digitalRead()
{
    return ((*inputReg) & mask) ? HIGH : LOW;
}

void FastIO::pulse(int times, int pulseDuration)
{
    //verify pin is output (mode register has 1 as pin mask position)
    if (!(*modeReg & mask))
    {
        return;
    }

    do
    {
        digitalWrite(HIGH);
        delay(pulseDuration);
        digitalWrite(LOW);
        delay(pulseDuration);
    } while (times--);
}