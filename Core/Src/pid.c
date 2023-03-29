#include "pid.h"
#include "tim.h"

float g_dPIDError = 0;

void PIDReset(PID_CONTROL_t *PID_Ctrl)
{
	PID_Ctrl->dIntergral = 0.0f;
	PID_Ctrl->dErrorTerm = 0.0f;
	g_dPIDError = 0;
	
}
void PIDInit(PID_CONTROL_t *PID_Ctrl, float dKp, float dKi, float dKd)
{
	PIDReset(PID_Ctrl);
	PID_Ctrl->dKp = dKp;
	PID_Ctrl->dKi = dKi;
	PID_Ctrl->dKd = dKp;
	__HAL_TIM_SetCounter(&htim4, 32768);
}
void PIDTuningSet(PID_CONTROL_t *PID_Ctrl, float dKp, float dKi, float dKd)
{
	// Check if the parameters are valid
  if(dKp < 0.0f || dKi < 0.0f || dKp < 0.0f)
  {
      return;
  }
	// Save the parameters for displaying purposes
	PID_Ctrl->dKp = dKp;
	PID_Ctrl->dKi = dKi;
	PID_Ctrl->dKd = dKd;
}

float PIDCompute(PID_CONTROL_t *PID_Ctrl, float dCmdValue, float dActValue, float dTs)
{
	float dPIDResult;
	g_dPIDError = dCmdValue - dActValue;
	float dP = 0, dI = 0, dD = 0;
	
	dP = PID_Ctrl->dKp * g_dPIDError;
	PID_Ctrl->dIntergral += g_dPIDError;
	dI = PID_Ctrl->dKi * dTs / 2 * PID_Ctrl->dIntergral;
	dD = PID_Ctrl->dKd * (g_dPIDError - PID_Ctrl->dErrorTerm) / dTs;
	dPIDResult = dP + dI + dD;
	PID_Ctrl->dErrorTerm = g_dPIDError;
	
	return dPIDResult;
}
