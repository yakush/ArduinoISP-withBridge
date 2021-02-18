#include <Arduino.h>
#include "config.h"
#include "programmer.h"
#include "util/fastButton.h"

// #include "bridgeWithPCINT.h"
#include "bridge.h"

enum t_state
{
    PROGRAMMER,
    BRIDGE,
};

//-------------------------------------------------------
// vars
t_state state = PROGRAMMER;
FastButton btnProgrammer;
FastButton btnBridge;
//-------------------------------------------------------

void setup()
{
    btnProgrammer.begin(PIN_BTN_PROGRAMMER);
    btnBridge.begin(PIN_BTN_BRIDGE);

    programmer_setup();
    bridge_setup();

    //start with programmer mode
    state = PROGRAMMER;
    bridge_stop();
    programmer_start();
}

void loop()
{
    // Serial.println("hello");
    // return;

    btnProgrammer.update();
    btnBridge.update();
    
    // if (btnProgrammer.fell())
    // {
    //     Serial.println("btn programmer");
    // }
    // if (btnBridge.fell())
    // {
    //     Serial.println("btn bridge");
    // }

    if (state != PROGRAMMER && btnProgrammer.fell())
    {
        state = PROGRAMMER;
        bridge_stop();
        programmer_start();
    }
    else if (state != BRIDGE && btnBridge.fell())
    {
        state = BRIDGE;
        programmer_stop();
        bridge_start();
    }
    
    // -- loop select
    if (state == PROGRAMMER)
        programmer_loop();
    else if (state == BRIDGE)
        bridge_loop();
}
