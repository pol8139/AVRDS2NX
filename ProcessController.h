#ifndef _PROCESSCONTROLLER_H_
#define _PROCESSCONTROLLER_H_

#include <stdint.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#include "mega32u4_dualshock2/mega32u4_uart.h"
#include "mega32u4_dualshock2/mega32u4_dualshock2.h"

#define DS2_SELECT 0x01

enum {
    SWITCH_ZERO    = 0x00,
    SWITCH_Y       = 0x01,
    SWITCH_X       = 0x02,
    SWITCH_B       = 0x04,
    SWITCH_A       = 0x08,
    SWITCH_RSL     = 0x10,
    SWITCH_RSR     = 0x20,
    SWITCH_R       = 0x40,
    SWITCH_ZR      = 0x80,
    SWITCH_MINUS   = 0x01,
    SWITCH_PLUS    = 0x02,
    SWITCH_RSTICK  = 0x04,
    SWITCH_LSTICK  = 0x08,
    SWITCH_HOME    = 0x10,
    SWITCH_CAPTURE = 0x20,
    SWITCH_NONE    = 0x40,
    SWITCH_GRIP    = 0x80,
    SWITCH_DOWN    = 0x01,
    SWITCH_UP      = 0x02,
    SWITCH_RIGHT   = 0x04,
    SWITCH_LEFT    = 0x08,
    SWITCH_LSR     = 0x10,
    SWITCH_LSL     = 0x20,
    SWITCH_L       = 0x40,
    SWITCH_ZL      = 0x80
};

enum {
    ConnecionInfo_BatteryLevel,
    ButtonRight,
    ButtonMiddle,
    ButtonLeft,
    Analog0,
    Analog1,
    Analog2,
    Analog3,
    Analog4,
    Analog5
};

void InitHardware(void);
void NormalizeRumble(uint8_t *);
void GetControllerInputData(uint8_t *);
uint8_t GetCounter(void);
void PrintRumble(void);

#endif