#pragma once

// -------------------------------------------------------
/* note 
 * on Nano, Pro Mini, and Mini :
 * A6 and A7 can ONLY be used as analog pins!
 */
// PINS
#define PIN_BTN_PROGRAMMER (A0)
#define PIN_BTN_BRIDGE (A1)
#define PIN_LED_BRIDGE (A2)

#define PIN_LED_BRIDGE_RX (A3)
#define PIN_LED_BRIDGE_TX (A4)

#define PIN_BRIDGE_RX (2)
#define PIN_BRIDGE_TX (3)
#define PIN_NATIVE_RX (0)
#define PIN_NATIVE_TX (1)

// -- also - programmer is using the following pins (on nano) :
// MOSI  (11)
// MISO  (12)
// SCK   (13)
// RESET (10) // Use pin 10 to reset the target rather than SS
// LED_HB (9)
// LED_ERR (8)
// LED_PMODE (7)
