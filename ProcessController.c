#include "ProcessController.h"

// #define UARTINPUT
#define DS2INPUT
// #define DDRMAT
// #define ONLYSMALLVIBRATOR
#define INCREASERUMBLE
// #define UARTDEBUG_TIMING

volatile uint8_t GlobalCounter = 0;
uint8_t HDRumbleLowAmp, HDRumbleHighAmp;
// uint8_t HDRumbleLowFreq, HDRumbleHighFreq;

#ifdef DS2INPUT
uint8_t DeviceID;
const uint8_t ButtonMap[3][16][2] =
{
    {
        {ButtonMiddle, SWITCH_ZERO   },
        {ButtonMiddle, SWITCH_LSTICK },
        {ButtonMiddle, SWITCH_RSTICK },
        {ButtonMiddle, SWITCH_PLUS   },
        {ButtonLeft,   SWITCH_UP     },
        {ButtonLeft,   SWITCH_RIGHT  },
        {ButtonLeft,   SWITCH_DOWN   },
        {ButtonLeft,   SWITCH_LEFT   },
        {ButtonLeft,   SWITCH_ZL     },
        {ButtonRight,  SWITCH_ZR     },
        {ButtonLeft,   SWITCH_L      },
        {ButtonRight,  SWITCH_R      },
        {ButtonRight,  SWITCH_X      },
        {ButtonRight,  SWITCH_A      },
        {ButtonRight,  SWITCH_B      },
        {ButtonRight,  SWITCH_Y      }
    }, {
        {ButtonMiddle, SWITCH_ZERO   },
        {ButtonMiddle, SWITCH_LSTICK },
        {ButtonMiddle, SWITCH_RSTICK },
        {ButtonMiddle, SWITCH_PLUS   },
        {ButtonLeft,   SWITCH_UP     },
        {ButtonLeft,   SWITCH_RIGHT  },
        {ButtonLeft,   SWITCH_DOWN   },
        {ButtonLeft,   SWITCH_LEFT   },
        {ButtonMiddle, SWITCH_CAPTURE},
        {ButtonMiddle, SWITCH_HOME   },
        {ButtonMiddle, SWITCH_MINUS  },
        {ButtonMiddle, SWITCH_PLUS   },
        {ButtonRight,  SWITCH_X      },
        {ButtonRight,  SWITCH_A      },
        {ButtonRight,  SWITCH_B      },
        {ButtonRight,  SWITCH_Y      }
    }, {
        {ButtonMiddle, SWITCH_MINUS  },
        {ButtonMiddle, SWITCH_ZERO   },
        {ButtonMiddle, SWITCH_ZERO   },
        {ButtonMiddle, SWITCH_PLUS   },
        {ButtonLeft,   SWITCH_UP     },
        {ButtonLeft,   SWITCH_RIGHT  },
        {ButtonLeft,   SWITCH_DOWN   },
        {ButtonLeft,   SWITCH_LEFT   },
        {ButtonMiddle, SWITCH_ZERO   },
        {ButtonMiddle, SWITCH_ZERO   },
        {ButtonMiddle, SWITCH_ZERO   },
        {ButtonMiddle, SWITCH_ZERO   },
        {ButtonMiddle, SWITCH_ZERO   },
        {ButtonRight,  SWITCH_A      },
        {ButtonRight,  SWITCH_B      },
        {ButtonMiddle, SWITCH_ZERO   }
    }
};
#endif

ISR(TIMER0_COMPA_vect)
{
    GlobalCounter++;
    #if defined UARTDEBUG_TIMING
        transmitUart('.');
    #endif
}

void InitHardware(void)
{
    TCCR0A = _BV(WGM01); // CTC
    TCCR0B = _BV(CS02); // Prescale 1/256
    OCR0A = 62; // 1ms (1/16MHz * 256 * 62 ~ 1ms)
    TIMSK0 = _BV(OCIE0A); // Compare match A interrupt enable
    #if defined UARTINPUT || defined UARTDEBUG_TIMING
        initUart();
    #endif
    #if defined DS2INPUT
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

void NormalizeRumble(uint8_t *RawRumbeData)
{
    HDRumbleHighAmp = (RawRumbeData[1] >> 1) + (RawRumbeData[5] >> 1);
    HDRumbleHighAmp += HDRumbleHighAmp >> 2;
    HDRumbleLowAmp = (RawRumbeData[2] >> 7) + ((RawRumbeData[3] - 0x40) << 1) + (RawRumbeData[6] >> 7) + ((RawRumbeData[7] - 0x40) << 1);
    HDRumbleLowAmp += HDRumbleLowAmp >> 2;
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
        }
    #elif defined DS2INPUT
        uint8_t Buffer[MAX_NUM_RECIEVE];
        uint16_t BuffButton = 0x0000;
        uint8_t ButtonMappingMode = 0;
        uint8_t SizeofData;
        if(DeviceID == 0xF3) {
            #if defined ONLYSMALLVIBRATOR
                HDRumbleHighAmp = (HDRumbleHighAmp >> 1) + (HDRumbleLowAmp >> 1);
                if(HDRumbleHighAmp & 0x80) {
                    HDRumbleHighAmp = 0xFF;
                } else {
                    HDRumbleHighAmp <<= 1;
                }
                HDRumbleLowAmp = VIBRATE_BIG_DISABLE;
            #endif
        }
        #if defined INCREASERUMBLE
            if(HDRumbleLowAmp & 0x80) {
                HDRumbleLowAmp = 0xFF;
            } else {
                HDRumbleLowAmp <<= 1;
            }
            if(HDRumbleHighAmp & 0x80) {
                HDRumbleHighAmp = 0xFF;
            } else {
                HDRumbleHighAmp <<= 1;
            }
        #endif
        if(DeviceID == 0xF3) {
            SizeofData = readDataAndVibrateEXDS2(Buffer, DeltaSigmaRumbleStrength(HDRumbleHighAmp), HDRumbleLowAmp);
        } else {
            SizeofData = readDataAndVibrateDS2(Buffer, DeltaSigmaRumbleStrength(HDRumbleHighAmp));
        }
        BuffButton = easyDechatter(~(Buffer[3] | (Buffer[4] << 8)));
        #ifdef DDRMAT
            ButtonMappingMode = 2;
        #else
            if(BuffButton & DS2_SELECT) {
                ButtonMappingMode = 1;
            } else {
                ButtonMappingMode = 0;
            }
        #endif
        for(uint8_t i = 0; i < 16; i++) {
            if(BuffButton & _BV(i)) {
                Data[ButtonMap[ButtonMappingMode][i][0]] |=  ButtonMap[ButtonMappingMode][i][1];
            }
        }
        if(SizeofData > 8) {
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
            Data[Analog0] = 0x00;
            Data[Analog1] = 0x08;
            Data[Analog2] = 0x80;
            Data[Analog3] = 0x00;
            Data[Analog4] = 0x08;
            Data[Analog5] = 0x80;
        }
    #endif
}

uint8_t DeltaSigmaRumbleStrength(uint8_t AnalogInput)
{
    static int16_t Integral = 0;
    static uint8_t OneBitOut = 0;
    Integral += AnalogInput - OneBitOut * 256;
    if(Integral > 0) {
        OneBitOut = 1;
    } else {
        OneBitOut = 0;
    }
    return OneBitOut;
}

uint8_t GetCounter(void)
{
    return GlobalCounter;
}

void PrintRumble(void)
{
    if(HDRumbleHighAmp != 0 && HDRumbleLowAmp != 0) {
        char s[4] = {};
        // ByteHexDump(s, HDRumbleLowAmp);
        itoa(HDRumbleLowAmp, s, 10);
        transmitUartString(s);
        transmitUart('\t');
        // ByteHexDump(s + 3, HDRumbleHighAmp);
        itoa(HDRumbleHighAmp, s, 10);
        transmitUartStringCRLF(s);
    }
}