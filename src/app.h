#include <Arduino.h>
#include "config.h"
#include "programmer.h"
#include "util/fastButton.h"

// #include "bridgeWithPCINT.h"
#include "bridge.h"

enum t_state
{
    NONE,
    PROGRAMMER,
    BRIDGE,
};

//-------------------------------------------------------
// vars
t_state state = NONE;
t_state stateRequest = NONE;

FastButton btnProgrammer;
FastButton btnBridge;
//-------------------------------------------------------

void setup()
{
    btnProgrammer.begin(PIN_BTN_PROGRAMMER);
    btnBridge.begin(PIN_BTN_BRIDGE);

    programmer_setup();
    Bridge.setup();

    //start with programmer mode
    stateRequest = PROGRAMMER;
}

void loop()
{
    // Serial.println("hello");
    // return;

    //check buttons and request state change:
    btnProgrammer.update();
    btnBridge.update();

    if (btnProgrammer.fell())
        stateRequest = PROGRAMMER;
    else if (btnBridge.fell())
        stateRequest = BRIDGE;

    //TODO: if self programming is pressed - set to none, if released - restore to previous state

    //update state if needed:
    if (stateRequest != state)
    {
        switch (stateRequest)
        {
        case PROGRAMMER:
            Bridge.end();
            programmer_begin();
            break;

        case BRIDGE:
            programmer_end();
            Bridge.begin();
            break;

        case NONE:
            programmer_end();
            Bridge.end();
            break;
        }

        state = stateRequest;
    }

    // -- loops
    programmer_loop();
    Bridge.loop();
}
