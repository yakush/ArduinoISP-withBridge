#pragma once

#include <Arduino.h>
#include "config.h"
#include "util/fastIO.h"

/*
    TESTING fast fast serial bridge functionality    
*/

FastIO bridge_rx;
FastIO bridge_tx;
FastIO native_rx;
FastIO native_tx;

FastIO led_on;
FastIO led_rx;
FastIO led_tx;

uint8_t bridge_RX_state = 1;
uint8_t bridge_TX_state = 1;

//-------------------------------------------------------
void bridge_heartbeat();
void bridge_run();
void ISR_BRIDGE();

//-------------------------------------------------------
void setup()
{
    // -- fast io pins
    led_on.begin(PIN_LED_BRIDGE);
    led_rx.begin(PIN_LED_BRIDGE_RX);
    led_tx.begin(PIN_LED_BRIDGE_TX);
    bridge_rx.begin(PIN_BRIDGE_RX);
    bridge_tx.begin(PIN_BRIDGE_TX);
    native_rx.begin(PIN_NATIVE_RX);
    native_tx.begin(PIN_NATIVE_TX);

    // -- init leds
    led_on.digitalWrite(LOW);
    led_rx.digitalWrite(LOW);
    led_tx.digitalWrite(LOW);

    led_on.pinMode(OUTPUT);
    led_rx.pinMode(OUTPUT);
    led_tx.pinMode(OUTPUT);

    // -- init uart
    bridge_rx.pinMode(INPUT);
    native_rx.pinMode(INPUT);

    bridge_tx.digitalWrite(LOW); // set value before changing to output
    bridge_tx.pinMode(OUTPUT);
    native_tx.digitalWrite(LOW); // set value before changing to output
    native_tx.pinMode(OUTPUT);

    // -- setup interrupts

    // PCINT on native rx pin
    PCMSK2 |= _BV(digitalPinToPCMSKbit(PIN_NATIVE_RX)); // enable PCINT on native rx pin
    PCIFR = B00000000;                                  // reset interrupt flags
    PCICR |= _BV(digitalPinToPCICRbit(PIN_NATIVE_RX));  // enable interrupt group (never disable, maybe someone else needs it!)

    // INT on bridge rx pin
    attachInterrupt(digitalPinToInterrupt(PIN_BRIDGE_RX), ISR_BRIDGE, CHANGE);    
}

//-------------------------------------------------------
void loop()
{
    bridge_heartbeat();
    // invert LEDS (UART protocol : HIGH is 0 , LOW is 1)
    led_tx.digitalWrite(!bridge_TX_state);
    led_rx.digitalWrite(!bridge_RX_state);
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
//- interrupts
//-------------------------------------------------------

// PCINT: native(RX) ---> bridge(TX)
ISR(PCINT2_vect)
{
    // read D0 (RX native) -> write D3 (TX bridge)
    bridge_TX_state = (PIND & B00000001);
    PORTD = (bridge_TX_state)
                ? PORTD | B00001000
                : PORTD & B11110111;
}

// INT :bridge(RX) ---> native(TX)
void ISR_BRIDGE()
{
    // read D2 (RX bridge) -> write D1 (TX native)
    bridge_RX_state = (PIND & B00000100);
    PORTD = (bridge_RX_state)
                ? PORTD | B00000010
                : PORTD & B11111101;
}