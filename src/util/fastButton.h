#pragma once
#include <Arduino.h>
#include "fastIO.h"

class FastButton
{
private:
    FastIO io;
    int buttonState = LOW;
    int currentRead = -1; //uninitialized
    unsigned long lastChangeTime = 0;
    unsigned long debounceDelay;
    bool justFell = false;
    bool justRose = false;

public:
    FastButton();
    virtual ~FastButton();

    void begin(uint8_t pin, bool pullup = true, unsigned long debounceDelay = 50);

    /**
     * updates the button (call every loop)
     * @return the button changed state
     */
    bool update();

    /**
     * @return the button's current stable state (HIGH|LOW)
     */
    uint8_t state();

    /**
     * @return true if the button just rose (stabely)
     */
    bool rose();

    /**
     * @return true if the button just fell (stabely)
     */
    bool fell();

    /**
     * @return true if the button just toggled (stabely)
     */
    bool toggled();
};
