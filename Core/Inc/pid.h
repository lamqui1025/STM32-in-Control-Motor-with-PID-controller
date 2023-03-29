#ifndef __PID_H
#define __PID_H

#include "stm32f1xx_hal.h"
#include <stdlib.h>
#include <stdbool.h>

typedef struct {
	float dKp, dKi, dKd;
	float dErrorTerm;
	float dIntergral;
} PID_CONTROL_t;

typedef struct 
{
  float dAccelMax;
  float dVelMax;
  float dPosMax;
  float dA1;
  float dA2, dB2;
  float dA3, dB3, dC3;
  float dMidStep1;
  float dMidStep2;
  float dMidStep3;
  float nTime;
} PROFILE_t;

typedef enum
{
  NONE = 1,
  SPID,
  PTUN,
	STUN,
  PTUN_RES,
	STUN_RES,

	SRUN,
	SRUN_RES,
	PRUN,
	PRUN_RES,
	STOP
} PROCESS_t;


extern void PIDReset(PID_CONTROL_t *PID_Ctrl);
extern void PIDInit(PID_CONTROL_t *PID_Ctrl, float dKp, float dKi, float dKd);
extern void PIDTuningSet(PID_CONTROL_t *PID_Ctrl, float dKp, float dKi, float dKd);
extern float PIDCompute(PID_CONTROL_t *PID_Ctrl, float dCmdValue, float dActValue, float dTs);


#endif
