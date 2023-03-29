#include "motor.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "tim.h"
#include "pid.h"
#include "gpio.h"
#include "main.h"
// Speed count Pulse ---
extern uint16_t g_nCntPulse;
extern uint16_t g_nPosPulse;
			 uint8_t direct_st; // bien trang thai chieu quay Thuan or Nghich
/* PID */
PROCESS_t tProcess;
PID_CONTROL_t tPIDControl;
PROFILE_t tProfile;

int32_t g_nDutyCycle;
int32_t g_nActPulse; // so nguyen co dau --
int32_t g_nCmdPulse; // so nguyen co dau --
uint16_t g_nIndex = 0;
float g_dCmdVel;
float fDuty;
extern bool g_direct; // bien dieu khien chieu quay FOWARD or REVERSE

void MotorSetDir(int8_t nDir)
{
	switch (nDir)
	{
	case 0: // CW
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		break;
	case 1: // CCW
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		break;
	default:
		break;
	}
}

void MotorSetDuty(uint16_t nDuty)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nDuty);
}
//------------------
void MotorInit(void)
{
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

	PIDReset(&tPIDControl);
	PIDInit(&tPIDControl, 1., 0., 0.00);
	MotorSetDir(1);
}
//-------------------
float ConvertDegToPulse(uint16_t nDeg)
{
	float dPulse = (float)nDeg * 4 * 11 * 56 / 360;

	return dPulse;
}
//uint16_t ConvertPosToPulse(uint16_t nDeg)
//{
//	uint16_t uPulse = nDeg * 4 * 11 * 56 / 360;

//	return uPulse;
//}
//------------------
//uint16_t ConvertPulseToDeg(uint16_t nPulse)
//{
//	float dDeg = nPulse * 360 / 4 / 11 / 56;
//	return (uint16_t)dDeg;
//}
// SPEED => PULSE - - -
float ConvertSpeedToPulse(uint16_t nSpeed)
{
	float dPulse = nSpeed * 4 * 11 * 56 / 6000;
	return dPulse;
}

void MotorGetPulse(uint32_t *nPulse)
{
	*nPulse = __HAL_TIM_GetCounter(&htim4);
	__HAL_TIM_SetCounter(&htim4, 32768);
}
//TUNING POSITION --------------------
void MotorPosTuning(uint16_t nPos)
{
	uint32_t nPulse;
	MotorGetPulse(&nPulse);
	g_nActPulse += nPulse - 32768;

	float	CmdPosPulse = ConvertDegToPulse(nPos);
	g_nDutyCycle = (int16_t)PIDCompute(&tPIDControl, CmdPosPulse, g_nActPulse, 0.01f);
	if (g_nDutyCycle >= 0)
	{
		MotorSetDir(1);
		MotorSetDuty(abs(g_nDutyCycle));
	}
	else if (g_nDutyCycle < 0)
	{
		MotorSetDir(0);
		MotorSetDuty(abs(g_nDutyCycle));
	}
	// Store data
	if(g_nActPulse >= 0)
	{
		direct_st = 1; // Chieu duong - FORWARD
		g_nPosPulse = g_nActPulse;
	}
	else if(g_nActPulse < 0)
	{
		direct_st = 0; // Chieu am - REVERSE
		g_nPosPulse = abs(g_nActPulse);
	}
}
//TUNING SPEED -------------------------
void MotorSpeedTuning(uint16_t nSpeed)
{
	uint32_t nPulse;
	MotorGetPulse(&nPulse);
	g_nActPulse = nPulse - 32768;
  float fCmdPulse = ConvertSpeedToPulse(nSpeed);
	//g_nCmdPulse = (uint16_t)ConvertSpeedToPulse(nSpeed);
	fDuty += PIDCompute(&tPIDControl, fCmdPulse, g_nActPulse, 0.01f);
	g_nDutyCycle = (int16_t)fDuty;
	if (g_nDutyCycle >= 0)
	{
		MotorSetDir(1);
		MotorSetDuty(abs(g_nDutyCycle));
	}
	else if (g_nDutyCycle < 0)
	{
		MotorSetDir(0);
		MotorSetDuty(abs(g_nDutyCycle));
	}
	
	if(g_nActPulse >=0)
	{
		direct_st = 1;
		g_nCntPulse = g_nActPulse;
	}
	else
	{
		direct_st = 0;
		g_nCntPulse = abs(g_nActPulse);
	}
}
// RUN -----------------------
void MotorMovePos(void)
{
	uint32_t nPulse;
	MotorGetPulse(&nPulse); 
	g_nActPulse += nPulse - 32768; 
	float dPosTemp = 0;
	
	// Profile Trapezoidal Speed
	if (tProfile.nTime <= tProfile.dMidStep1)
	{
		dPosTemp = (int32_t)(tProfile.dA1 * tProfile.nTime * tProfile.nTime);
		g_dCmdVel = 2 * tProfile.dA1 * tProfile.nTime;
	}
	else if (tProfile.nTime <= tProfile.dMidStep2)
	{
		dPosTemp = (int32_t)(tProfile.dA2 * tProfile.nTime + tProfile.dB2);
		g_dCmdVel = tProfile.dA2;
	}
	else if (tProfile.nTime <= tProfile.dMidStep3)
	{
		dPosTemp = (int32_t)(tProfile.dA3 * tProfile.nTime * tProfile.nTime + tProfile.dB3 * tProfile.nTime + tProfile.dC3);
		g_dCmdVel = 2 * tProfile.dA3 * tProfile.nTime + tProfile.dB3;
	}
	else
	{
		dPosTemp = tProfile.dPosMax;
	}

	// Control PID
	if(g_direct)
		g_nCmdPulse = ConvertDegToPulse(dPosTemp); // FORWARD
	else if(!g_direct)
		g_nCmdPulse = -ConvertDegToPulse(dPosTemp); // REVERSE
	
	g_nDutyCycle = (int16_t)PIDCompute(&tPIDControl, g_nCmdPulse, g_nActPulse, 0.01f);
	if (g_nDutyCycle >= 0)
	{
		MotorSetDir(1);
		MotorSetDuty(abs(g_nDutyCycle));
	}
	else if (g_nDutyCycle < 0)
	{
		MotorSetDir(0);
		MotorSetDuty(abs(g_nDutyCycle));
	}

	if (tProfile.nTime > tProfile.dMidStep3)
	{
		tProcess = NONE;
		__HAL_TIM_SetCounter(&htim4, 32768);
		dPosTemp = 0;
		g_nActPulse = 0;
		g_nDutyCycle = 0;
		g_dCmdVel = 0;
		tProfile.nTime = 0;
		g_nPosPulse = 0; // them vao de xoa bien dem xung 
		
		MotorSetDuty(abs(g_nDutyCycle));
	}
	else
	{
		if(g_nActPulse >= 0)
		{
			direct_st = 1;
			g_nPosPulse = g_nActPulse;
		}
		else if(g_nActPulse < 0)
		{
			direct_st = 0;
			g_nPosPulse = abs(g_nActPulse);
		}
	}
	// Compute timer count for control PID
	tProfile.nTime += 0.01;
}
void ControlSpeed(float cmdPulse)
{
	uint32_t nPulse;
	MotorGetPulse(&nPulse);
	g_nActPulse = (int32_t)nPulse - 32768;
	
	//g_nCmdPulse = ConvertSpeedToPulse(nSpeed);
	fDuty += PIDCompute(&tPIDControl, cmdPulse, g_nActPulse, 0.01f);
	g_nDutyCycle = (int16_t)fDuty;
	if (g_nDutyCycle >= 0)
	{
		MotorSetDir(1);
		MotorSetDuty(abs(g_nDutyCycle));
	}
	else if (g_nDutyCycle < 0)
	{
		MotorSetDir(0);
		MotorSetDuty(abs(g_nDutyCycle));
	}
	
	if (g_nActPulse >= 0)
	{
		direct_st = 1;
		g_nCntPulse = g_nActPulse;
	}
	else if(g_nActPulse < 0)
	{
		direct_st = 0;
		g_nCntPulse = abs(g_nActPulse);
	}
}
