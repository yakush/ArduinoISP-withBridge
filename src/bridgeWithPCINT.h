#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "pins_arduino.h"
#include "config.h"
#include "util/fastIO.h"

//-------------------------------------------------------

#define BRIDGE_USE_INUPUT_AS_OUT_HI

//-------------------------------------------------------

bool bridge_running = false;

FastIO bridge_rx;
FastIO bridge_tx;
FastIO native_rx;
FastIO native_tx;

FastIO led_on;
FastIO led_rx;
FastIO led_tx;

void bridge_setup()
{
    led_on.begin(PIN_LED_BRIDGE);
    led_rx.begin(PIN_LED_BRIDGE_RX);
    led_tx.begin(PIN_LED_BRIDGE_TX);

    //leds
    led_on.pinMode(OUTPUT);
    led_rx.pinMode(OUTPUT);
    led_tx.pinMode(OUTPUT);

    led_on.digitalWrite(LOW);
    led_rx.digitalWrite(LOW);
    led_tx.digitalWrite(LOW);

    bridge_rx.begin(PIN_BRIDGE_RX);
    bridge_tx.begin(PIN_BRIDGE_TX);
    native_rx.begin(PIN_NATIVE_RX);
    native_tx.begin(PIN_NATIVE_TX);

    // -- setup interrupts
    // enable PCINT on the rx pins:
    PCMSK2 |= (1 << (digitalPinToPCMSKbit(PIN_BRIDGE_RX)) |
               1 << (digitalPinToPCMSKbit(PIN_NATIVE_RX)));

    // reset interrupt flags
    PCIFR = B00000000;

    //enable interrupt group (never disable, maybe someone else needs it!)
    PCICR |= (1 << (digitalPinToPCICRbit(PIN_BRIDGE_RX)) |
              1 << (digitalPinToPCICRbit(PIN_NATIVE_RX)));

    //bridge uart pins
#ifdef BRIDGE_USE_INUPUT_AS_OUT_HI
    bridge_rx.pinMode(INPUT);
    bridge_tx.pinMode(INPUT_PULLUP);
#else
    bridge_tx.digitalWrite(LOW); // to be safer ?
    bridge_rx.pinMode(INPUT);
    bridge_tx.pinMode(OUTPUT);
#endif
}

void bridge_start()
{
    bridge_running = true;
    //blink all lights at start
    led_rx.pulse(2);
    led_tx.pulse(2);
    led_on.pulse(2);

    //led_on on
    led_on.digitalWrite(HIGH);
    led_rx.digitalWrite(LOW);
    led_tx.digitalWrite(LOW);

#ifdef BRIDGE_USE_INUPUT_AS_OUT_HI
    bridge_rx.pinMode(INPUT);
    bridge_tx.pinMode(INPUT_PULLUP);

    native_rx.pinMode(INPUT);
    native_tx.pinMode(INPUT_PULLUP);
#else
    bridge_tx.digitalWrite(LOW); // to be safer ?
    bridge_rx.pinMode(INPUT);
    bridge_tx.pinMode(OUTPUT);

    native_tx.digitalWrite(LOW); // to be safer ?
    native_rx.pinMode(INPUT);
    native_tx.pinMode(OUTPUT);
#endif
}

void bridge_stop()
{
    bridge_running = false;
    //leds off
    led_on.digitalWrite(LOW);
    led_rx.digitalWrite(LOW);
    led_tx.digitalWrite(LOW);

    //native uart pins
    native_rx.pinMode(INPUT);
    native_tx.pinMode(INPUT);

    //bridge uart pins
#ifdef BRIDGE_USE_INUPUT_AS_OUT_HI
    bridge_rx.pinMode(INPUT);
    bridge_tx.pinMode(INPUT_PULLUP);
#else
    bridge_tx.digitalWrite(LOW); // to be safer ?
    bridge_rx.pinMode(INPUT);
    bridge_tx.pinMode(OUTPUT);
#endif
}

void bridge_run()
{
    //interrupts off
    uint8_t oldSREG = SREG;
    cli();

    /** NOTE: UART is neutrally pulled high
     *  HIGH means 0 , LOW means 1
     *  so rx/tx indication LEDS should be reversed :
     *  OFF when line is HIGH and ON when line is low !
     */

#ifdef BRIDGE_USE_INUPUT_AS_OUT_HI
    //-- option 1: use input-with-pullup / output-low

    //BRIDGE_RX ---> TX
    if (bridge_rx.digitalRead())
    {
        native_tx.pinMode(INPUT_PULLUP);
        led_rx.digitalWrite(LOW);
    }
    else
    {
        native_tx.digitalWrite(LOW);
        native_tx.pinMode(OUTPUT);
        led_rx.digitalWrite(HIGH);
    }

    //RX ---> BRIDGE_TX
    if (native_rx.digitalRead())
    {
        bridge_tx.pinMode(INPUT_PULLUP);
        led_tx.digitalWrite(LOW);
    }
    else
    {
        bridge_tx.digitalWrite(LOW);
        bridge_tx.pinMode(OUTPUT);
        led_tx.digitalWrite(HIGH);
    }
#else
    //-- option 2: use output HI / LO

    // BRIDGE_RX ---> TX
    int rx = bridge_rx.digitalRead();
    native_tx.digitalWrite(rx);
    led_rx.digitalWrite(!rx);
    // RX ---> BRIDGE_TX
    int tx = native_rx.digitalRead();
    bridge_tx.digitalWrite(tx);
    led_tx.digitalWrite(!tx);
#endif

    //interrupts on
    SREG = oldSREG;
}

ISR(PCINT2_vect)
{
    if (!bridge_running)
        return;

    bridge_run();
}

void bridge_loop()
{
    bridge_run();
}

/*

#ifdef BRIDGE_USE_SOFTWARE_SERIAL
    //using software serial
#else
    //using bitbang
#endif

*/