     // *****************************************************************************
// * button-interface.c
// *
// * Routines for counting the number of button presses to implement a complex
// * button interface.
// *
// * Copyright 2015 Silicon Laboratories, Inc.                              *80*
// *****************************************************************************

#include PLATFORM_HEADER
#include CONFIGURATION_HEADER
#include "stack/include/ember-types.h"
#include "event_control/event.h"
#include "hal/hal.h"
#include EMBER_AF_API_BUTTON_INTERFACE
#include "hal/micro/system-timer.h"
#include "debug-printing.h"
// ------------------------------------------------------------------------------
// Plugin private macro definitions
#define BUTTON_TIMEOUT_MS     EMBER_AF_PLUGIN_BUTTON_INTERFACE_BUTTON_TIMEOUT_MS
#define BUTTON_SHORT_TIMEOUT  ( \
    EMBER_AF_PLUGIN_BUTTON_INTERFACE_BUTTON_TIMEOUT_MS / 4)

// ------------------------------------------------------------------------------
// plugin private typedefs and enums

// Button GPIO state possibilities
enum {
  BUTTON_STATE_LOW          = 0x00,
  BUTTON_STATE_HIGH         = 0x01,
  BUTTON_STATE_UNINIT       = 0xFF,
};

// Button debouncing and long/short differentiation state machine enums
enum {
  BUTTON_PRESSED_AT_STARTUP = 0x00,
  BUTTON_PRESSED_SHORT      = 0x01,
  BUTTON_PRESSED_LONG       = 0x02,
  BUTTON_IDLE               = 0x03,
};

// ------------------------------------------------------------------------------
// Plugin events
EmberEventControl emberAfPluginButtonInterfaceButton0PressedEventControl;
EmberEventControl emberAfPluginButtonInterfaceButton0ReleasedEventControl;
EmberEventControl emberAfPluginButtonInterfaceButton1PressedEventControl;
EmberEventControl emberAfPluginButtonInterfaceButton1ReleasedEventControl;
EmberEventControl emberAfPluginButtonInterfaceButtonTimeoutEventControl;
EmberEventControl emberAfPluginButtonInterfaceButton2PressedEventControl;
EmberEventControl emberAfPluginButtonInterfaceButton2ReleasedEventControl;
// ------------------------------------------------------------------------------
// plugin private function prototypes
static void clearButtonCounters(void);

// ------------------------------------------------------------------------------
// plugin private global variables

// Timers to track how long a button has been pressed
static uint16_t button0Timer = 0;
static uint16_t button1Timer = 0;
static uint16_t button2Timer = 0;

static uint16_t button0Counter = 0;
static uint16_t button1Counter = 0;
static uint16_t button2Counter = 0;

#ifdef BUTTON0
static uint8_t button0Polarity = EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO;
#endif
#ifdef BUTTON1
static uint8_t button1Polarity = EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO;
#endif

#ifdef BUTTON2
static uint8_t button2Polarity = EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO;
#endif


static uint8_t button0LastState = BUTTON_STATE_UNINIT;
static uint8_t button1LastState = BUTTON_STATE_UNINIT;
static uint8_t button2LastState = BUTTON_STATE_UNINIT;

static uint8_t button0PressedState = BUTTON_PRESSED_AT_STARTUP;
static uint8_t button1PressedState = BUTTON_PRESSED_AT_STARTUP;
static uint8_t button2PressedState = BUTTON_PRESSED_AT_STARTUP;


// ------------------------------------------------------------------------------
// Plugin consumed callback implementations

// ------------------------------------------------------------------------------
// Plugin event handlers
void emberAfPluginButtonInterfaceButtonTimeoutEventHandler(void)
{
  emberEventControlSetInactive(
    emberAfPluginButtonInterfaceButtonTimeoutEventControl);

  clearButtonCounters();
}

void emberAfPluginButtonInterfaceButton0PressedEventHandler(void)
{
  // if the button is being pressed, we must deactivate this event control.
  emberEventControlSetInactive(
    emberAfPluginButtonInterfaceButtonTimeoutEventControl);

  if (button0LastState != BUTTON_STATE_LOW) {
    button0LastState = BUTTON_STATE_LOW;
    emberAfPluginButtonInterfaceButton0LowCallback();
  }
  switch (button0PressedState) {
    case BUTTON_PRESSED_AT_STARTUP:
    case BUTTON_IDLE:
      button0PressedState = BUTTON_PRESSED_SHORT;

      emberEventControlSetDelayMS(
        emberAfPluginButtonInterfaceButton0PressedEventControl,
        BUTTON_TIMEOUT_MS);

      button0Timer = halCommonGetInt16uMillisecondTick();

      break;
    case BUTTON_PRESSED_SHORT:
      button0PressedState = BUTTON_PRESSED_LONG;

    case BUTTON_PRESSED_LONG:
      emberEventControlSetInactive(
        emberAfPluginButtonInterfaceButton0PressedEventControl);
      if( halCommonGetInt16uMillisecondTick()>=(button0Timer+BUTTON_TIMEOUT_MS))
        {
            emberAfPluginButtonInterfaceButton0PressingCallback();
        }
      break;
  }


  return;
}

void emberAfPluginButtonInterfaceButton0ReleasedEventHandler(void)
{
  uint16_t timePressed = halCommonGetInt16uMillisecondTick() - button0Timer;
  emberEventControlSetInactive(
    emberAfPluginButtonInterfaceButton0ReleasedEventControl);
  emberEventControlSetInactive(
    emberAfPluginButtonInterfaceButton0PressedEventControl);


  if (button0LastState != BUTTON_STATE_HIGH) {
    button0LastState = BUTTON_STATE_HIGH;
    emberAfPluginButtonInterfaceButton0HighCallback();
  }
  button0Counter = timePressed;
  if (timePressed >= BUTTON_TIMEOUT_MS) {
    emberAfPluginButtonInterfaceButton0PressedLongCallback(
      timePressed,
      button0PressedState == BUTTON_PRESSED_AT_STARTUP);
    clearButtonCounters();
  } else {

    emberAfPluginButtonInterfaceButton0PressedShortCallback(button0Counter);
    emberEventControlSetActive(
      emberAfPluginButtonInterfaceButtonTimeoutEventControl);
  }
  button0PressedState = BUTTON_IDLE;
}

void emberAfPluginButtonInterfaceButton1PressedEventHandler(void)
{
  // if the button is being pressed, we must deactivate this event control.
  emberEventControlSetInactive(
    emberAfPluginButtonInterfaceButtonTimeoutEventControl);

  if (button1LastState != BUTTON_STATE_LOW) {
    button1LastState = BUTTON_STATE_LOW;
    emberAfPluginButtonInterfaceButton1LowCallback();
  }
  switch (button1PressedState) {
    case BUTTON_PRESSED_AT_STARTUP:
    case BUTTON_IDLE:
      button1PressedState = BUTTON_PRESSED_SHORT;
      emberEventControlSetDelayMS(
        emberAfPluginButtonInterfaceButton1PressedEventControl,
        BUTTON_TIMEOUT_MS);
      button1Timer = halCommonGetInt16uMillisecondTick();
      break;
    case BUTTON_PRESSED_SHORT:
      button1PressedState = BUTTON_PRESSED_LONG;
    case BUTTON_PRESSED_LONG:
      emberEventControlSetDelayMS(
        emberAfPluginButtonInterfaceButton1PressedEventControl,
        BUTTON_SHORT_TIMEOUT);
      emberAfPluginButtonInterfaceButton1PressingCallback();
      break;
  }

  return;
}

void emberAfPluginButtonInterfaceButton1ReleasedEventHandler(void)
{
  uint16_t timePressed = halCommonGetInt16uMillisecondTick() - button1Timer;
  emberEventControlSetInactive(
    emberAfPluginButtonInterfaceButton1ReleasedEventControl);
  emberEventControlSetInactive(
    emberAfPluginButtonInterfaceButton1PressedEventControl);

  if (button1LastState != BUTTON_STATE_HIGH) {
    button1LastState = BUTTON_STATE_HIGH;
    emberAfPluginButtonInterfaceButton1HighCallback();
  }

  button1Counter = timePressed;

  if (timePressed >= BUTTON_TIMEOUT_MS) {
    emberAfPluginButtonInterfaceButton1PressedLongCallback(
      timePressed,
      button1PressedState == BUTTON_PRESSED_AT_STARTUP);
    clearButtonCounters();
  } else {
    emberAfPluginButtonInterfaceButton1PressedShortCallback(button1Counter);
    emberEventControlSetActive(
      emberAfPluginButtonInterfaceButtonTimeoutEventControl);
  }
  button1PressedState = BUTTON_IDLE;
}


void emberAfPluginButtonInterfaceButton2PressedEventHandler(void)
{
  // if the button is being pressed, we must deactivate this event control.
  emberEventControlSetInactive(
    emberAfPluginButtonInterfaceButtonTimeoutEventControl);

  if (button2LastState != BUTTON_STATE_LOW) {
    button2LastState = BUTTON_STATE_LOW;
    emberAfPluginButtonInterfaceButton2LowCallback();
  }
  switch (button2PressedState) {
    case BUTTON_PRESSED_AT_STARTUP:
    case BUTTON_IDLE:
      button2PressedState = BUTTON_PRESSED_SHORT;
      emberEventControlSetDelayMS(
        emberAfPluginButtonInterfaceButton2PressedEventControl,
        BUTTON_TIMEOUT_MS);
      button2Timer = halCommonGetInt16uMillisecondTick();
      break;
    case BUTTON_PRESSED_SHORT:
      button2PressedState = BUTTON_PRESSED_LONG;
    case BUTTON_PRESSED_LONG:
      emberEventControlSetDelayMS(
        emberAfPluginButtonInterfaceButton2PressedEventControl,
        BUTTON_SHORT_TIMEOUT);
    //  emberAfPluginButtonInterfaceButton1PressingCallback();
      break;
  }

  return;
}

void emberAfPluginButtonInterfaceButton2ReleasedEventHandler(void)
{
  uint16_t timePressed = halCommonGetInt16uMillisecondTick() - button2Timer;
  emberEventControlSetInactive(
    emberAfPluginButtonInterfaceButton2ReleasedEventControl);
  emberEventControlSetInactive(
    emberAfPluginButtonInterfaceButton2PressedEventControl);

  if (button2LastState != BUTTON_STATE_HIGH) {
    button2LastState = BUTTON_STATE_HIGH;
   // emberAfPluginButtonInterfaceButton1HighCallback();
  }

  button2Counter = timePressed;

  if (timePressed >= BUTTON_TIMEOUT_MS) {
    emberAfPluginButtonInterfaceButton2PressedLongCallback(
      timePressed,
      button2PressedState == BUTTON_PRESSED_AT_STARTUP);
    clearButtonCounters();
  } else {
    emberAfPluginButtonInterfaceButton2PressedShortCallback(button2Counter);
    emberEventControlSetActive(
      emberAfPluginButtonInterfaceButtonTimeoutEventControl);
  }
  button2PressedState = BUTTON_IDLE;
}

// ------------------------------------------------------------------------------
// Plugin public API function implementations
void halPluginButtonInterfaceSetButtonPolarity(
  uint8_t                    button,
  HalButtonInterfacePolarity polarity)
{
#ifdef BUTTON0
  if (button == BUTTON0) {
    button0Polarity = polarity;
  }
#endif
#ifdef BUTTON1
  if (button == BUTTON1) {
    button1Polarity = polarity;
  }
#endif

#ifdef BUTTON2
  if (button == BUTTON2) {
    button2Polarity = polarity;
  }
#endif
}

uint8_t halPluginButtonInterfaceButtonPoll(uint8_t button)
{

//  if (button == BUTTON0) {
//#ifndef BUTTON0
//    return 0xFF;
//#else
//    if (button0Polarity == EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO) {
//      return halButtonState(BUTTON0);
//    }
//    if (halButtonState(BUTTON0) == BUTTON_PRESSED) {
//      return BUTTON_RELEASED;
//    } else {
//      return BUTTON_PRESSED;
//    }
//#endif
//  } else {
//#ifndef BUTTON1
//    return 0xFF;
//#else
//    if (button1Polarity == EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO) {
//      return halButtonState(BUTTON1);
//    }
//    if (halButtonState(BUTTON1) == BUTTON_PRESSED) {
//      return BUTTON_RELEASED;
//    } else {
//      return BUTTON_PRESSED;
//    }
//#endif
//
//#ifndef BUTTON2
//    return 0xFF;
//#else
//    if (button2Polarity == EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO) {
//      return halButtonState(BUTTON2);
//    }
//    if (halButtonState(BUTTON2) == BUTTON_PRESSED) {
//      return BUTTON_RELEASED;
//    } else {
//      return BUTTON_PRESSED;
//    }
//#endif
//  }
}

uint8_t halPluginButtonInterfaceButton0Poll(void)
{
#ifdef BUTTON0
  if (button0Polarity == EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO) {
    return halButtonState(BUTTON0);
  }
  if (halButtonState(BUTTON0) == BUTTON_PRESSED) {
    return BUTTON_RELEASED;
  } else {
    return BUTTON_PRESSED;
  }
#else
  return 0xFF;
#endif
}

uint8_t halPluginButtonInterfaceButton1Poll(void)
{
#ifdef BUTTON1
  if (button1Polarity == EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO) {
    return halButtonState(BUTTON1);
  }
  if (halButtonState(BUTTON1) == BUTTON_PRESSED) {
    return BUTTON_RELEASED;
  } else {
    return BUTTON_PRESSED;
  }
#else
  return 0xFF;
#endif
}


uint8_t halPluginButtonInterfaceButton2Poll(void)
{
#ifdef BUTTON2
  if (button2Polarity == EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO) {
    return halButtonState(BUTTON2);
  }
  if (halButtonState(BUTTON2) == BUTTON_PRESSED) {
    return BUTTON_RELEASED;
  } else {
    return BUTTON_PRESSED;
  }
#else
  return 0xFF;
#endif
}

// ------------------------------------------------------------------------------
// Plugin private function implementations
static void clearButtonCounters(void)
{
  button0Counter = 0;
  button1Counter = 0;
  button2Counter = 0;
}

// ------------------------------------------------------------------------------
// Plugin ISR function implementations
// WARNING: these functions are in ISR so we must do minimal
// processing and not make any blocking calls (like printf)
// or calls that take a long time.

void emberAfHalButtonIsrCallback(uint8_t button, uint8_t state)
{

  emberAfCorePrintln("emberAfHalButtonIsrCallback");
  // ISR CONTEXT!!!
#ifdef BUTTON0
  if (button == BUTTON0) {
    if (state == BUTTON_PRESSED) {
      // button.c assumes the button is active low, so a BUTTON_PRESSED will
      // been
      // 1 when the button GPIO is low and 0 when the GPIO is high.
      if (button0Polarity == EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO) {
        emberEventControlSetActive(
          emberAfPluginButtonInterfaceButton0PressedEventControl);
      } else { // button0Polarity == ACTIVE_HI
        emberEventControlSetActive(
          emberAfPluginButtonInterfaceButton0ReleasedEventControl);
      }
    } else { // state == BUTTON_RELEASED
      if (button0Polarity == EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO) {
        emberEventControlSetActive(
          emberAfPluginButtonInterfaceButton0ReleasedEventControl);
      } else { // button0Polairty == ACTIVE_HI
        emberEventControlSetActive(
          emberAfPluginButtonInterfaceButton0PressedEventControl);
      }
    }
  }
#endif // ifdef BUTTON0
#ifdef BUTTON1
  if (button == BUTTON1) {
    if (state == BUTTON_PRESSED) {
      if (button1Polarity == EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO) {
        emberEventControlSetActive(
          emberAfPluginButtonInterfaceButton1PressedEventControl);
      } else { // button1Polarity == ACTIVE_HI
        emberEventControlSetActive(
          emberAfPluginButtonInterfaceButton1ReleasedEventControl);
      }
    } else { // state == BUTTON_RELEASED
      if (button1Polarity == EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO) {
        emberEventControlSetActive(
          emberAfPluginButtonInterfaceButton1ReleasedEventControl);
      } else { // button1Polairty == ACTIVE_HI
        emberEventControlSetActive(
          emberAfPluginButtonInterfaceButton1PressedEventControl);
      }
    }
  }
#endif // ifdef BUTTON1


#ifdef BUTTON2
  if (button == BUTTON2) {
    if (state == BUTTON_PRESSED) {
      if (button2Polarity == EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO) {

        emberEventControlSetActive(
          emberAfPluginButtonInterfaceButton2PressedEventControl);
      } else { // button2Polarity == ACTIVE_HI

        emberEventControlSetActive(
          emberAfPluginButtonInterfaceButton2ReleasedEventControl);
      }
    } else { // state == BUTTON_RELEASED
      if (button2Polarity == EMBER_AF_BUTTON_INTERFACE_POLARITY_ACTIVE_LO) {

        emberEventControlSetActive(
          emberAfPluginButtonInterfaceButton2ReleasedEventControl);
      } else { // button2Polairty == ACTIVE_HI

        emberEventControlSetActive(
          emberAfPluginButtonInterfaceButton2PressedEventControl);
      }
    }
  }
#endif // ifdef BUTTON2

}
