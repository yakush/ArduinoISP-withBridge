#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "pins_arduino.h"
#include "config.h"
#include "util/fastIO.h"

//-------------------------------------------------------
#define BRIDGE_USE_SOFTWARE_SERIAL
#define BRIDGE_USE_INUPUT_AS_OUT_HI

#define BAUD (9600)

//-------------------------------------------------------
FastIO bridge_rx;
FastIO bridge_tx;
FastIO native_rx;
FastIO native_tx;

FastIO led_on;
FastIO led_rx;
FastIO led_tx;

#ifdef BRIDGE_USE_SOFTWARE_SERIAL
SoftwareSerial bridgeSerial(PIN_BRIDGE_RX, PIN_BRIDGE_TX);
#endif

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

#ifdef BRIDGE_USE_SOFTWARE_SERIAL
    //using software serial
#else
    bridge_rx.begin(PIN_BRIDGE_RX);
    bridge_tx.begin(PIN_BRIDGE_TX);
    native_rx.begin(PIN_NATIVE_RX);
    native_tx.begin(PIN_NATIVE_TX);

    //bridge uart pins
#ifdef BRIDGE_USE_INUPUT_AS_OUT_HI
    bridge_rx.pinMode(INPUT);
    bridge_tx.pinMode(INPUT_PULLUP);
#else
    bridge_tx.digitalWrite(LOW); // to be safer ?
    bridge_rx.pinMode(INPUT);
    bridge_tx.pinMode(OUTPUT);
#endif

#endif
}

void bridge_start()
{

    //blink all lights at start
    led_rx.pulse(2);
    led_tx.pulse(2);
    led_on.pulse(2);

    //led_on on
    led_on.digitalWrite(HIGH);
    led_rx.digitalWrite(LOW);
    led_tx.digitalWrite(LOW);

#ifdef BRIDGE_USE_SOFTWARE_SERIAL
    //-- using software serial
    Serial.begin(BAUD);
    bridgeSerial.begin(BAUD);
#else
    //-- using bitbang

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
#endif
}

void bridge_stop()
{
    //leds off
    led_on.digitalWrite(LOW);
    led_rx.digitalWrite(LOW);
    led_tx.digitalWrite(LOW);

#ifdef BRIDGE_USE_SOFTWARE_SERIAL
    //using software serial
    Serial.end();
    bridgeSerial.end();
#else
    //using bitbang
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
#endif
}

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

void bridge_loop()
{
    bridge_heartbeat();

#ifdef BRIDGE_USE_SOFTWARE_SERIAL
    //using software serial
    if (Serial.available())
    {
        bridgeSerial.write(Serial.read());
    }
    if (bridgeSerial.available())
    {
        Serial.write(bridgeSerial.read());
    }

#else
    //using bitbang
    /** NOTE: UART is neutrally pulled high
     *  HIGH means 0 , LOW means 1
     *  so rx/tx indication LEDS should be reversed :
     *  OFF when line is HIGH and ON when line is low !
     */

    //interrupts off
    uint8_t oldSREG = SREG;
    cli();

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

    SREG = oldSREG; // turn interrupts back on

#endif
}


/*

#ifdef BRIDGE_USE_SOFTWARE_SERIAL
    //using software serial
#else
    //using bitbang
#endif

*/