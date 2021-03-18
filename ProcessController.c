#include "ProcessController.h"

#define DS2INPUT

volatile uint8_t GlobalCounter = 0;
uint8_t DeviceID;
const uint8_t ButtonMap[16][2] =
{
    {ButtonMiddle, SWITCH_MINUS },
    {ButtonMiddle, SWITCH_LSTICK},
    {ButtonMiddle, SWITCH_RSTICK},
    {ButtonMiddle, SWITCH_PLUS  },
    {ButtonLeft,   SWITCH_UP    },
    {ButtonLeft,   SWITCH_RIGHT },
    {ButtonLeft,   SWITCH_DOWN  },
    {ButtonLeft,   SWITCH_LEFT  },
    {ButtonLeft,   SWITCH_ZL    },
    {ButtonRight,  SWITCH_ZR    },
    {ButtonLeft,   SWITCH_L     },
    {ButtonRight,  SWITCH_R     },
    {ButtonRight,  SWITCH_X     },
    {ButtonRight,  SWITCH_A     },
    {ButtonRight,  SWITCH_B     },
    {ButtonRight,  SWITCH_Y     }
};

ISR(TIMER0_COMPA_vect)
{
    GlobalCounter++;
}

void InitHardware(void)
{
    TCCR0A = _BV(WGM01); // CTC
    TCCR0B = _BV(CS02); // Prescale 1/256
    OCR0A = 62; // 1ms (1/16MHz * 256 * 62 ~ 1ms)
    TIMSK0 = _BV(OCIE0A); // Compare match A interrupt enable
    #ifdef UARTINPUT
        initUart();
    #elif defined DS2INPUT
        uint8_t Buffer[MAX_NUM_RECIEVE];
        initSPIMaster();
        readDataDS2(Buffer);
        delayFrame(1);
        configModeEnterDS2(Buffer);
        delayFrame(1);
        queryModelAndModeDS2(Buffer);
        delayFrame(1);
        DeviceID = Buffer[NUM_ID];
        if(DeviceID == 0xF3) {
            setModeAndLockDS2(Buffer, MODE_ANALOG, LOCK_DISABLE);
            delayFrame(1);
            vibrationEnableDS2(Buffer);
            delayFrame(1);
            configModeExitDS2(Buffer);
            delayFrame(1);
        }
    #endif
}

void GetControllerInputData(uint8_t *Data)
{
    #ifdef UARTINPUT
        char c;
        c = recieveUART();
        switch(c) {
            case 'f':
                Data[ButtonRight] = SWITCH_A;
                break;
            case 'c':
                Data[ButtonRight] = SWITCH_B;
                break;
            case 'w':
                Data[ButtonLeft] = SWITCH_UP;
                break;
            case 'a':
                Data[ButtonLeft] = SWITCH_LEFT;
                break;
            case 's':
                Data[ButtonLeft] = SWITCH_DOWN;
                break;
            case 'd':
                Data[ButtonLeft] = SWITCH_RIGHT;
                break;
            case 'z':
                Data[ButtonMiddle] = SWITCH_HOME;
                break;
            default:
                break;
    #elif defined DS2INPUT
        uint8_t Buffer[MAX_NUM_RECIEVE];
        uint16_t BuffButton = 0x0000;
        if(DeviceID == 0xF3) {
            readDataAndVibrateEXDS2(Buffer, VIBRATE_SMALL_DISABLE, VIBRATE_BIG_DISABLE);
            BuffButton = easyDechatter(~(Buffer[3] | (Buffer[4] << 8)));
            for(uint8_t i = 0; i < 16; i++) {
                if(BuffButton & _BV(i)) {
                    Data[ButtonMap[i][0]] |=  ButtonMap[i][1];
                }
            }
            Buffer[8] ^= 0xFF;
            easyDeadZone8(Buffer + 7);
            Data[Analog0] = Buffer[7] << 4;
            Data[Analog1] = Buffer[7] >> 4;
            Data[Analog2] = Buffer[8];
            Buffer[6] ^= 0xFF;
            easyDeadZone8(Buffer + 5);
            Data[Analog3] = Buffer[5] << 4;
            Data[Analog4] = Buffer[5] >> 4;
            Data[Analog5] = Buffer[6];
        } else {
            ;
        }
    #endif
}

uint8_t GetCounter(void)
{
    return GlobalCounter;
}