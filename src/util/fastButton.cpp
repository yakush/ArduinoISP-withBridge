#include "fastButton.h"

FastButton::FastButton()
{
}

FastButton::~FastButton()
{
}

void FastButton::begin(uint8_t _pin, bool _pullup, unsigned long _debounceDelay)
{
    io.begin(_pin);
    debounceDelay = _debounceDelay;

    //setup pin mode
    io.pinMode(_pullup ? INPUT_PULLUP : INPUT);
}

bool FastButton::update()
{
    unsigned long now = millis();

    //update current read
    int prevRead = currentRead;
    currentRead = io.digitalRead();

    if (prevRead != currentRead)
    {
        // not stable
        lastChangeTime = now;
    }
    else
    {
        if (now - lastChangeTime > debounceDelay)
        {
            // stable read
            if (currentRead != buttonState)
            {
                buttonState = currentRead;
                justRose = buttonState;
                justFell = !buttonState;
                return true;
            }
        }
    }
    justRose = false;
    justFell = false;
    return false;
}

uint8_t FastButton::state()
{
    return buttonState;
}

bool FastButton::rose()
{
    return justRose;
}

bool FastButton::fell()
{
    return justFell;
}

bool FastButton::toggled()
{
    return justRose || justFell;
}