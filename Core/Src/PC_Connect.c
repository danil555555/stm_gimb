/*
 * PC_Connect.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Gigabyte
 */
#include "PC_Connect.h"
#include "drv8302.h"
extern float vq;
extern float vd;
extern float Kp;
extern float Ki;
extern Current current;

static uint8_t TxBuf[5];
static uint8_t RxBuf[5];

void USB_ProcessCommand(uint8_t *buf)
{
    uint8_t cmd = buf[0];
    float val = *((float*)(buf + 1));

    switch(cmd)
    {
        case 0x01:

            break;

        case 0x21:
        	Kp += 0.05f;
            break;

        case 0x22:
        	Kp -= 0.05f;
            break;

        case 0x23:
        {
        	Ki += 0.0001f;
            break;
        }
        case 0x24:
        {
        	Ki -= 0.0001f;
            break;
        }


        case 0x30:
        {
            float sendVal = vq;
            TxBuf[0] = cmd;
            memcpy(&TxBuf[1], &sendVal, 4);
            CDC_Transmit_FS(TxBuf, 5);
            break;
        }

        case 0x31:
        {
            float sendVal = vd;
            TxBuf[0] = cmd;
            memcpy(&TxBuf[1], &sendVal, 4);
            CDC_Transmit_FS(TxBuf, 5);
            break;
        }

        case 0x32:
        {

            break;
        }

        default:

            break;
    }
}
