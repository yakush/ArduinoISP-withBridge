#pragma once

#include <Arduino.h>
#include "config.h"
#include "util/fastIO.h"

/*
    TESTING serial bridge functionality    
*/

// #define BRIDGE_USE_INUPUT_AS_OUT_HI

FastIO bridge_rx;
FastIO bridge_tx;
FastIO native_rx;
FastIO native_tx;

FastIO led_on;
FastIO led_rx;
FastIO led_tx;

//-------------------------------------------------------
void bridge_heartbeat();
void bridge_run();

//-------------------------------------------------------
void setup()
{
    // - fast io pins
    led_on.begin(PIN_LED_BRIDGE);
    led_rx.begin(PIN_LED_BRIDGE_RX);
    led_tx.begin(PIN_LED_BRIDGE_TX);
    bridge_rx.begin(PIN_BRIDGE_RX);
    bridge_tx.begin(PIN_BRIDGE_TX);
    native_rx.begin(PIN_NATIVE_RX);
    native_tx.begin(PIN_NATIVE_TX);

    // - init leds
    led_on.digitalWrite(LOW);
    led_rx.digitalWrite(LOW);
    led_tx.digitalWrite(LOW);

    led_on.pinMode(OUTPUT);
    led_rx.pinMode(OUTPUT);
    led_tx.pinMode(OUTPUT);

    // - init uart
    bridge_rx.pinMode(INPUT);
    native_rx.pinMode(INPUT);

#ifdef BRIDGE_USE_INUPUT_AS_OUT_HI
    bridge_tx.pinMode(INPUT_PULLUP);
    native_tx.pinMode(INPUT_PULLUP);
#else
    bridge_tx.digitalWrite(LOW); // to be safer ?
    bridge_tx.pinMode(OUTPUT);
    native_tx.digitalWrite(LOW); // to be safer ?
    native_tx.pinMode(OUTPUT);
#endif

    // -- setup interrupts
    // enable PCINT on the rx pins:
    PCMSK2 |= (1 << (digitalPinToPCMSKbit(PIN_BRIDGE_RX)) |
               1 << (digitalPinToPCMSKbit(PIN_NATIVE_RX)));

    // reset interrupt flags
    PCIFR = B00000000;

    // enable interrupt group (never disable, maybe someone else needs it!)
    PCICR |= (1 << (digitalPinToPCICRbit(PIN_BRIDGE_RX)) |
              1 << (digitalPinToPCICRbit(PIN_NATIVE_RX)));
}

//-------------------------------------------------------
void loop()
{
    bridge_heartbeat();
}
//-------------------------------------------------------
void bridge_heartbeat()
{
    static unsigned long last_time = 0;
    static uint8_t hbval = 128;
    static int8_t hbdelta = 8;

    unsigned long now = millis();

    if ((now - last_time) < 40)
        return;

    last_time = now;
    if (hbval > 192)
        hbdelta = -hbdelta;
    if (hbval < 32)
        hbdelta = -hbdelta;
    hbval += hbdelta;
    analogWrite(PIN_LED_BRIDGE, hbval);
}
//-------------------------------------------------------
inline void bridge_run()
{
    //interrupts off
    uint8_t oldSREG = SREG;
    cli();

    //-- option 1: use input-with-pullup / output-low

    //BRIDGE_RX ---> TX
    {
        int val = bridge_rx.digitalRead();
        if (val)
        {
#ifdef BRIDGE_USE_INUPUT_AS_OUT_HI
            native_tx.pinMode(INPUT_PULLUP);
#else
            native_tx.digitalWrite(HIGH);
#endif
        }
        else
        {
#ifdef BRIDGE_USE_INUPUT_AS_OUT_HI
            native_tx.digitalWrite(LOW);
            native_tx.pinMode(OUTPUT);
#else
            native_tx.digitalWrite(LOW);
#endif
        }
        led_rx.digitalWrite(!val);
    }

    //RX ---> BRIDGE_TX
    {
        int val = native_rx.digitalRead();
        if (val)
        {
#ifdef BRIDGE_USE_INUPUT_AS_OUT_HI
            bridge_tx.pinMode(INPUT_PULLUP);
#else
            bridge_tx.digitalWrite(HIGH);
#endif
        }
        else
        {
#ifdef BRIDGE_USE_INUPUT_AS_OUT_HI
            bridge_tx.digitalWrite(LOW);
            bridge_tx.pinMode(OUTPUT);
#else
            bridge_tx.digitalWrite(LOW);
#endif
        }
        led_tx.digitalWrite(!val);
    }

    //interrupts on
    SREG = oldSREG;
}

//-------------------------------------------------------
//- ISR
ISR(PCINT2_vect)
{
    bridge_run();
}
