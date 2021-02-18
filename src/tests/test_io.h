#include <Arduino.h>
#include "config.h"
#include "util/fastIO.h"

FastIO io;
FastIO led;

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;

    io.begin(A0);
    led.begin(A2);

    io.pinMode(INPUT_PULLUP);
    led.pinMode(OUTPUT);
    // pinMode(A0,INPUT_PULLUP);
}

void loop()
{
    int v = io.digitalRead();
    Serial.print(v);
    Serial.print(" ");
    Serial.println(digitalRead(A0));

    led.digitalWrite(v ? HIGH : LOW);
}