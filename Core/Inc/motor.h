#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"
#include <stdlib.h>
#include <stdbool.h>

extern void MotorSetDir(int8_t nDir);
extern void MotorSetDuty(uint16_t nDuty);
extern void MotorInit(void);
extern float ConvertDegToPulse(uint16_t nDeg);
//extern uint16_t ConvertPulseToDeg(uint16_t nPulse);
extern float ConvertSpeedToPulse(uint16_t nSpeed);
extern void MotorGetPulse(uint32_t *nPulse);
extern void MotorMovePos(void);
extern void MotorPosTuning(uint16_t nPos);
extern void MotorSpeedTuning(uint16_t nSpeed);
extern void ControlSpeed(float cmdPulse);
#endif
