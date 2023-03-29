#include "serial.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"

uint8_t g_nRxBuff[MAX_LEN];
uint8_t g_strCommand[4];
uint8_t g_nOption[3];
uint8_t g_nData[8];
bool g_bDataAvailable = false;

uint8_t STX[] = {0x02U};
uint8_t ETX[] = {0x03U};
uint8_t ACK[] = {0x06U};
uint8_t SYN[] = {0x16U};

uint8_t *subString(uint8_t *pBuff, int nPos, int nIndex)
{
    uint8_t *t = &pBuff[nPos];
    pBuff[nPos - 1] = '\0';
    for (int i = nIndex; i < (strlen((char *)t) + 1); i++)
    {
        t[i] = '\0';
    }
    return t;
}

bool StrCompare(uint8_t *pBuff, uint8_t *pSample, uint8_t nSize)
{
    for (int i = 0; i < nSize; i++)
    {
        if (pBuff[i] != pSample[i])
        {
            return false;
        }
    }

    return true;
}


void SerialWriteComm(uint8_t *pStrCmd, uint8_t *pOpt, uint8_t *pData)
{
    uint8_t *pBuff;
    pBuff = (uint8_t *)malloc(18);
    uint8_t nIndex = 0;

    memcpy(pBuff + nIndex, STX, 1);
    nIndex += 1;
    memcpy(pBuff + nIndex, pStrCmd, 4);
    nIndex += 4;
    memcpy(pBuff + nIndex, pOpt, 3);
    nIndex += 3;
    memcpy(pBuff + nIndex, pData, 8);
    nIndex += 8;
    memcpy(pBuff + nIndex, ACK, 1);
    nIndex += 1;
    memcpy(pBuff + nIndex, ETX, 1);

    //HAL_UART_Transmit(&huart1, pBuff, MAX_LEN, 1000);
	  CDC_Transmit_FS(pBuff,18);
    free(pBuff);
}

void SerialParse(uint8_t *pBuff)
{
    if ((pBuff[0] == STX[0]) && (pBuff[17] == ETX[0]))
    {
        memcpy(g_strCommand, subString(pBuff, 1, 4), 4);
        memcpy(g_nOption, subString(pBuff, 5, 3), 3);
        memcpy(g_nData, subString(pBuff, 8, 8), 8);
		}
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//   if (huart->Instance == huart1.Instance)
//    {
//        g_bDataAvailable = true;
//        SerialParse(g_nRxBuff);
//    }
//}
