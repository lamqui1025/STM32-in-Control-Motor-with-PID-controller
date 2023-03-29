#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f1xx_hal.h"
#include <stdlib.h>
#include <stdbool.h>

#define MAX_LEN 18

//extern void SerialInit(void);
extern uint8_t *subString(uint8_t *pBuff, int nPos, int nIndex);
extern bool StrCompare(uint8_t *pBuff, uint8_t *pSample, uint8_t nSize);
extern void SerialWriteComm(uint8_t *pStrCmd, uint8_t *pOpt, uint8_t *pData);
extern void SerialParse(uint8_t *pBuff);
//extern void SerialAcceptReceive(void);

#endif
