/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "motor.h"
#include "pid.h"
#include "serial.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern PID_CONTROL_t tPIDControl;
extern PROFILE_t tProfile;
extern PROCESS_t tProcess;

extern bool g_bDataAvailable;
extern uint8_t nTxBuff[MAX_LEN];
extern uint8_t g_strCommand[4];
extern uint8_t g_nOption[3];
extern uint8_t g_nData[8];

extern int32_t g_nActPulse;
extern int32_t g_nCmdPulse;
extern uint16_t g_nIndex;
extern float g_dPIDError;

uint8_t g_strTxCommand[4];
uint8_t g_nTxOption[3];
uint8_t g_nTxData[8];
uint16_t g_nTuningSampleCount = 0;

uint16_t g_nCntPulse = 0; // count pulse speed
uint16_t g_nPosPulse = 0; // Position Pulse
uint16_t g_nSetPoint;
float g_nSetPulse;
extern uint8_t direct_st; //forward or reverse
extern int32_t g_nDutyCycle;
extern float fDuty;
			 bool g_direct;  //forward or reverse

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	MotorInit();
  tProcess = NONE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (g_bDataAvailable == true)
    {
	  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
      if (StrCompare(g_strCommand, (uint8_t *)"SPID", 4))
      {
        tProcess = SPID;
      }
      else if (StrCompare(g_strCommand, (uint8_t *)"PTUN", 4))
      {
        tProcess = PTUN_RES;
      }
			else if (StrCompare(g_strCommand, (uint8_t *)"STUN", 4))
			{
				tProcess = STUN_RES;
			}
			else if (StrCompare(g_strCommand,(uint8_t *)"SRUN", 4))
			{
				tProcess = SRUN_RES;
			}
			else if(StrCompare(g_strCommand, (uint8_t *)"PRUN", 4))
			{
				tProcess = PRUN_RES;
			}
			else if (StrCompare(g_strCommand, (uint8_t *)"STOP", 4))
			{
				tProcess = STOP;
			}
			else tProcess = NONE;
			g_bDataAvailable = false;
    }
		//---------------------------------------------------------------
    switch (tProcess)
    {
    case NONE:
      //SerialAcceptReceive();
      break;
    case SPID:
      SerialWriteComm(g_strCommand, g_nOption, g_nData);
			g_nCmdPulse = 0;
			PIDReset(&tPIDControl);
			__HAL_TIM_SetCounter(&htim4, 32768);
			g_nIndex = 0;

      /* Get PID params */
      tPIDControl.dKp = (float)g_nData[0] + (float)g_nData[1] / 10;
      tPIDControl.dKi = (float)g_nData[2] + (float)g_nData[3] / 10;
      tPIDControl.dKd = (float)g_nData[4] + (float)g_nData[5] / (pow((float)10, (float)g_nData[6]));

      tProcess = NONE;
      break;
    case PTUN_RES:
      SerialWriteComm(g_strCommand, g_nOption, g_nData);
			g_nSetPoint = (uint16_t)g_nData[0]*256 + (uint16_t)g_nData[1];
      tProcess = PTUN;
      break;
		case STUN_RES:
			SerialWriteComm(g_strCommand, g_nOption, g_nData);
			g_nSetPoint = (uint16_t)g_nData[0]*256 + (uint16_t)g_nData[1];
      tProcess = STUN;
			break;
    case PTUN:
      break;
		case STUN:
			break;
		case PRUN_RES:
			//
			SerialWriteComm(g_strCommand, g_nOption, g_nData);
			g_nSetPoint = (uint16_t)g_nData[6]*256 + (uint16_t)g_nData[7];
			if(g_nData[5] == 'F') g_direct = 1; //FOWARD
			else if(g_nData[5] == 'R') g_direct = 0; //REVERSE
			
			//Reset variable 
      PIDReset(&tPIDControl);
      g_nActPulse = 0;
      g_nCmdPulse = 0;
		
      // Get Pmax, Vmax, Amax
			tProfile.dAccelMax = (float)((g_nData[0] >> 4) * 4096) + (float)((g_nData[0] & 0x0F) * 256) + (float)((g_nData[1] >> 4) * 16) + (float)((g_nData[1] & 0x0F) * 1);
      tProfile.dVelMax = (float)((g_nData[2] >> 4) * 4096) + (float)((g_nData[2] & 0x0F) * 256) + (float)((g_nData[3] >> 4) * 16) + (float)((g_nData[3] & 0x0F) * 1);
      tProfile.dPosMax = (float)((g_nData[6] >> 4) * 4096 ) + (float)((g_nData[6] & 0x0F) * 256) + (float)((g_nData[7] >> 4) * 16) + (float)((g_nData[7] & 0x0F) * 1);

			// Calculate params for trapezoidal speed
      tProfile.dA1 = 0.5f * tProfile.dAccelMax;
      tProfile.dA2 = tProfile.dVelMax;
      tProfile.dB2 = -0.5f * tProfile.dVelMax * tProfile.dVelMax / tProfile.dAccelMax;
      tProfile.dA3 = -0.5f * tProfile.dAccelMax;
      tProfile.dB3 = tProfile.dPosMax * tProfile.dAccelMax / tProfile.dVelMax + tProfile.dVelMax;
      tProfile.dC3 = -0.5f * tProfile.dPosMax * tProfile.dPosMax * tProfile.dAccelMax / (tProfile.dVelMax * tProfile.dVelMax) - 0.5f * tProfile.dVelMax * tProfile.dVelMax / tProfile.dAccelMax;
      tProfile.dMidStep1 = tProfile.dVelMax / tProfile.dAccelMax;
      tProfile.dMidStep2 = tProfile.dPosMax / tProfile.dVelMax;
      tProfile.dMidStep3 = tProfile.dMidStep1 + tProfile.dMidStep2;

      tProfile.nTime = 0;
			//
      tProcess = PRUN;
			break;
		case SRUN_RES:
			//
			SerialWriteComm(g_strCommand, g_nOption, g_nData);
			g_nSetPoint = (uint16_t)g_nData[6]*256 + (uint16_t)g_nData[7];
			if(g_nData[5] == 'F')
			{
				g_nSetPulse = ConvertDegToPulse(g_nSetPoint) / 100;
				g_direct = 1; //FOWARD
			}
			else if(g_nData[5] == 'R')
			{
				g_nSetPulse = -ConvertDegToPulse(g_nSetPoint) / 100;
				g_direct = 0; //REVERSE
			}
      tProcess = SRUN;
			break;
		case PRUN:
			break;
		case SRUN:
			break;
		case STOP:
			SerialWriteComm(g_strCommand, g_nOption, g_nData);
			
				MotorSetDuty(0);
				g_nActPulse = 0;
				g_nDutyCycle = 0;
				fDuty = 0;
				tProcess = NONE;
      break;

		
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Timer 2: 10ms */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim2.Instance)
  {
    switch (tProcess)
    {
    case NONE:
      break;
    case SPID:
      break;
		case PTUN_RES:
			break;
		case STUN_RES:
			break;
    case PTUN:
			if (g_nIndex <= 500)
			{
				MotorPosTuning(g_nSetPoint);   //Tuning PID position (o)
				g_nIndex ++;
				// Send data Tuning
			  memcpy(g_strTxCommand, "PTUN", 4);
				memset(g_nTxOption, '\0', 3);
				g_nTxData[0] = 0x00;
				g_nTxData[1] = 0x00;
				g_nTxData[2] = 0x00;
				g_nTxData[3] = 0x00;
				g_nTxData[4] = 0x00;
				g_nTxData[5] = direct_st;
				g_nTxData[6] = (g_nPosPulse&0xFF00)>>8;
				g_nTxData[7] = (g_nPosPulse&0xFF);
				
        SerialWriteComm(g_strTxCommand, g_nTxOption, g_nTxData);
				
        memset(g_strTxCommand, '\0', 4);
        memset(g_nTxOption, '\0', 3);
        memset(g_nTxData, '\0', 8);
			}
      else
			{
				g_nIndex = 0;
				tProcess = NONE;
				MotorSetDuty(0);
				g_nActPulse = 0;
				g_nDutyCycle = 0;
			}
      break;
		case STUN:
			if (g_nIndex <= 1000)
			{
				MotorSpeedTuning(g_nSetPoint);    // Tunuing PID Speed (RPM) 
				g_nIndex ++;
				// Send data Tuning
			  memcpy(g_strTxCommand, "STUN", 4);
				memset(g_nTxOption, '\0', 3);
				g_nTxData[0] = 0x00;
				g_nTxData[1] = 0x00;
				g_nTxData[2] = 0x00;
				g_nTxData[3] = 0x00;
				g_nTxData[4] = 0x00;
				g_nTxData[5] = direct_st;
				g_nTxData[6] = (g_nCntPulse&0xFF00)>>8;
				g_nTxData[7] = (g_nCntPulse&0xFF);
				
        SerialWriteComm(g_strTxCommand, g_nTxOption, g_nTxData);
				
        memset(g_strTxCommand, '\0', 4);
        memset(g_nTxOption, '\0', 3);
        memset(g_nTxData, '\0', 8);
			}
      else
			{
				g_nIndex = 0;
				tProcess = NONE;
				MotorSetDuty(0);
				g_nActPulse = 0;
				g_nDutyCycle = 0;
				fDuty = 0;
			}
		case PRUN_RES:
			break;
		case SRUN_RES:
			break;
		case PRUN:
			//
			MotorMovePos();
			memcpy(g_strTxCommand, "PRUN", 4);
			memset(g_nTxOption, '\0', 3);
		
			g_nTxData[0] = 0x00;
			g_nTxData[1] = 0x00;
			g_nTxData[2] = 0x00; //(g_nPosPulse&0xFF);
			g_nTxData[3] = 0x00;//(g_nPosPulse&0xFF00)>>8;
			g_nTxData[4] = 0x00;
			g_nTxData[5] = direct_st; // FOWARD(1) or REVERSE(0)
			g_nTxData[6] = (g_nPosPulse&0xFF00)>>8;
			g_nTxData[7] = (g_nPosPulse&0xFF);
			
      SerialWriteComm(g_strTxCommand, g_nTxOption, g_nTxData);
				
        //memset(g_strTxCommand, '\0', 4);
        //memset(g_nTxOption, '\0', 3);
        //memset(g_nTxData, '\0', 8);
			break;
		case SRUN:
			//
			ControlSpeed(g_nSetPulse);
			memcpy(g_strTxCommand, "SRUN", 4);
			memset(g_nTxOption, '\0', 3);
				g_nTxData[0] = 0x00;
				g_nTxData[1] = 0x00;
				g_nTxData[2] = 0x00;
				g_nTxData[3] = 0x00;
				g_nTxData[4] = 0x00;
				g_nTxData[5] = direct_st;
				g_nTxData[6] = (g_nCntPulse&0xFF00)>>8;
				g_nTxData[7] = (g_nCntPulse&0xFF);
				
      SerialWriteComm(g_strTxCommand, g_nTxOption, g_nTxData);
			break;
		case STOP:
			break;
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
