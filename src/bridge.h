#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "pins_arduino.h"
#include "config.h"
#include "util/fastIO.h"

class Bridge_CLASS
{
private:
    bool running = false;
    FastIO led_on;
    FastIO led_rx;
    FastIO led_tx;
    FastIO pin_enable;

    FastIO pin_rx;
    FastIO pin_tx;

public:
    Bridge_CLASS() {}
    virtual ~Bridge_CLASS() {}
    //-------------------------------------------------------
    void setup()
    {
        //setup pins
        led_on.begin(PIN_LED_BRIDGE_HB);
        led_rx.begin(PIN_LED_BRIDGE_RX);
        led_tx.begin(PIN_LED_BRIDGE_TX);
        pin_enable.begin(PIN_ENABLE_BRIDGE);
        pin_rx.begin(PIN_NATIVE_RX);
        pin_tx.begin(PIN_NATIVE_TX);

        led_on.digitalWrite(LOW);
        led_on.pinMode(OUTPUT);

        led_rx.digitalWrite(LOW);
        led_rx.pinMode(OUTPUT);

        led_tx.digitalWrite(LOW);
        led_tx.pinMode(OUTPUT);

        pin_enable.digitalWrite(LOW);
        pin_enable.pinMode(OUTPUT);
    }

    void begin()
    {
        if (running)
            return;

        Serial.end();

        pin_rx.pinMode(INPUT);
        pin_tx.pinMode(INPUT);
        pin_enable.digitalWrite(HIGH);

        led_on.pulse(2);
        led_rx.pulse(2);
        led_tx.pulse(2);

        running = true;
    }
    //-------------------------------------------------------
    void end()
    {
        if (!running)
            return;

        pin_enable.digitalWrite(LOW);

        led_on.digitalWrite(LOW);
        led_rx.digitalWrite(LOW);
        led_tx.digitalWrite(LOW);

        running = false;
    }
    //-------------------------------------------------------
    void loop()
    {
        if (!running)
            return;

        heartbeat();

        // rx/tx LEDS
        // note: UART is inverted - HIGH is logical 0, LOW is logical 1
        // note: native TX is connected to bridge RX and native RX is connected to bridge TX
        led_rx.digitalWrite(!pin_tx.digitalRead());
        led_tx.digitalWrite(!pin_rx.digitalRead());
    }
    //-------------------------------------------------------
    void heartbeat()
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
        analogWrite(PIN_LED_BRIDGE_HB, hbval);
    }
};

//-----------------------------------------------------------------
// global :
Bridge_CLASS Bridge;
//extern ns_bridge::Bridge_CLASS Bridge;
//-----------------------------------------------------------------
