#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "config.h"
#include "util/fastIO.h"
#include "util/fastButton.h"

/*
    TESTING serial bridge functionality with software serial
*/

FastIO led_on;
FastIO led_rx;
FastIO led_tx;

FastButton btnUp;
FastButton btnDown;

uint8_t bridge_RX_state = 1;
uint8_t bridge_TX_state = 1;

#define SerialNative Serial
SoftwareSerial SerialBridge(PIN_BRIDGE_RX, PIN_BRIDGE_TX);

//-------------------------------------------------------
void bridge_heartbeat();
void bridge_run();

long BAUDList[] = {9600, 19200, 38400, 57600, 115200};
size_t BAUDListNum = sizeof(BAUDList) / sizeof(BAUDList[0]);
size_t BAUDListIdx = 0;
long BAUD = BAUDList[BAUDListIdx];

//-------------------------------------------------------
void startSerial()
{
    BAUD = BAUDList[BAUDListIdx];

    SerialBridge.end();
    SerialNative.end();

    delay(100);

    SerialBridge.begin(BAUD);
    SerialNative.begin(BAUD);

    delay(100);

    SerialNative.print("\n-- BRIDGE STARTED @ ");
    SerialNative.print(BAUD);
    SerialNative.println(" --");
}

void incBaud()
{
    if (BAUDListIdx == BAUDListNum - 1)
        return;
    BAUDListIdx++;
    startSerial();
}
void decBaud()
{
    if (BAUDListIdx == 0)
        return;
    BAUDListIdx--;
    startSerial();
}

//-------------------------------------------------------
void setup()
{
    btnUp.begin(PIN_BTN_PROGRAMMER);
    btnDown.begin(PIN_BTN_BRIDGE);

    // -- fast io pins
    led_on.begin(PIN_LED_BRIDGE);
    led_rx.begin(PIN_LED_BRIDGE_RX);
    led_tx.begin(PIN_LED_BRIDGE_TX);

    // -- init leds
    led_on.digitalWrite(LOW);
    led_rx.digitalWrite(LOW);
    led_tx.digitalWrite(LOW);

    led_on.pinMode(OUTPUT);
    led_rx.pinMode(OUTPUT);
    led_tx.pinMode(OUTPUT);

    startSerial();
}

//-------------------------------------------------------
void loop()
{
    bridge_heartbeat();

    //buttons for baud speeds
    btnUp.update();
    btnDown.update();

    if (btnUp.fell())
        incBaud();

    if (btnDown.fell())
        decBaud();

    //native -> bridge
    if (SerialNative.available())
    {
        bridge_TX_state = SerialNative.read();
        SerialBridge.write(bridge_TX_state);
    }

    //bridge -> native
    if (SerialBridge.available())
    {
        bridge_RX_state = SerialBridge.read();
        SerialNative.write(bridge_RX_state);
    }

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
