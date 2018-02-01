/*****************************************************************
 *
 * HwSimuTask.cpp
 *
 * RTX Model Interface, Version 1.0
 * Shanghai Share-E tech. Co., LTD.
 *
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!! Do not edit this file. !!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 ****************************************************************/
/*#include "stdafx.h"*/
#include <stdio.h>
#include <float.h>
#include <string>
#include <hwrtxif.h>
#include <HwSimTask.h>
#include <HwSimTaskSupport.h>

int OnSetupTask();
int OnInitializeTask();
int OnStartTask();
int OnStepTask(int iTidNo, hw_int64 llTickCount, hw_int64 llTime);
int OnStopTask();
int OnUnsetupTask();

#ifdef __cplusplus
extern "C"
{
#endif

	extern const hw_uint32 g_ulParameterNum;
	extern const hw_uint32 g_ulVariableNum;
	extern const hw_uint32 g_ulInputPortNum;
	extern const hw_uint32 g_ulOutputPortNum;
	extern const USER_PORT g_Parameters[];
	extern const USER_PORT g_Variables[];
	extern const USER_PORT g_InputPorts[];
	extern const USER_PORT g_OutputPorts[];

	extern const hw_uint16 g_SampleRates[];
	extern const hw_uint16 g_usSampleRateNum;

#ifdef __cplusplus
}
#endif

HW_SIM_TASK_API HSIMTASK RTAPI HwTask_SetupTask(hw_uint16 ulTaskIdx, HW_MSG_PROC msgProc)
{
	HSIMTASK hSimTask = RtxIf_SetupTask(ulTaskIdx, msgProc);
	if (hSimTask)
	{
		TaskIf_PostSimuMessage(hSimTask, TASK_MSG_DEBUG, "HwTask_SetupTask()\n");
		OnSetupTask();
	}
	return hSimTask;
}

HW_SIM_TASK_API int RTAPI HwTask_InitializeTask(HSIMTASK hTask, double dFinalTime)
{
	TaskIf_PostSimuMessage(hTask, TASK_MSG_DEBUG, "HwTask_InitializeTask()\n");
	int iError = RtxIf_InitializeTask(hTask, dFinalTime);
	if (HWERR_OK!=iError)
	{
		return iError;
	}
	return OnInitializeTask();
}


HW_SIM_TASK_API int RTAPI HwTask_StartTask(HSIMTASK hTask)
{
	TaskIf_PostSimuMessage(hTask, TASK_MSG_DEBUG, "HwTask_StartTask()\n"); 
	int iError = RtxIf_StartTask(hTask);
	if (HWERR_OK != iError)
	{
		return iError;
	}
	return OnStartTask();
}

HW_SIM_TASK_API int RTAPI HwTask_StepOutput(HSIMTASK hTask, int iTidNo, hw_int64 llTickCount, hw_int64 llTime)
{
#if 0
	TaskIf_PostSimuMessage(hTask, TASK_MSG_DEBUG, "HwTask_StepOutput()\n");
#endif

	int iError = RtxIf_StepOutput(hTask, iTidNo, llTickCount, llTime);
	if (HWERR_OK != iError)
	{
		return iError;
	}

	return OnStepTask(iTidNo, llTickCount, llTime);
}

HW_SIM_TASK_API int RTAPI HwTask_StepUpdate(HSIMTASK hTask, int iTidNo, hw_int64 llTickCount, hw_int64 llTime)
{
	return HWERR_OK;
}

HW_SIM_TASK_API int RTAPI HwTask_StopTask(HSIMTASK hTask)
{
	TaskIf_PostSimuMessage(hTask, TASK_MSG_DEBUG, "HwTask_StopTask()\n");
	int iError = RtxIf_StopTask(hTask);
	if (HWERR_OK != iError)
	{
		return iError;
	}
	return OnStopTask();
}


HW_SIM_TASK_API int RTAPI HwTask_UnsetupTask(HSIMTASK hTask)
{
	TaskIf_PostSimuMessage(hTask, TASK_MSG_DEBUG, "HwTask_UnsetupTask()\n");
	int iError = RtxIf_UnsetupTask(hTask);
	if (HWERR_OK != iError)
	{
		return iError;
	}
	return OnUnsetupTask();
}

HW_SIM_TASK_API void RTAPI HwTask_LockUserData(HSIMTASK hTask)
{
	TaskIf_LockUserData(hTask);
}

HW_SIM_TASK_API void RTAPI HwTask_UnlockUserData(HSIMTASK hTask)
{
	TaskIf_UnlockUserData(hTask);
}

HW_SIM_TASK_API const char* HwTask_GetErrorStatus(HSIMTASK hTask)
{
	return TaskIf_GetErrorStatus(hTask);
}
HW_SIM_TASK_API void HwTask_SetOverrun(HSIMTASK hTask, hw_bool bOverrun)
{
	TaskIf_SetOverrun(hTask, bOverrun);
}

HW_SIM_TASK_API void HwTask_SetOverrunEnable(HSIMTASK hTask, hw_bool bEnable)
{
	TaskIf_SetOverrunEnable(hTask, bEnable);
}

HW_SIM_TASK_API hw_bool HwTask_GetOverrunEnable(HSIMTASK hTask)
{
	return TaskIf_GetOverrunEnable(hTask);
}

HW_SIM_TASK_API hw_bool HwTask_GetStopRequested(HSIMTASK hTask)
{
	return TaskIf_GetStopRequested(hTask);
}

HW_SIM_TASK_API void* RTAPI HwTask_GetParameterTransAddress(HSIMTASK hTask, hw_uint32 ulParamIdx, hw_uint32 ulBaseIdx, hw_uint32 ulOffsetIdx, hw_uint32 ulDataSize)
{
	if (ulParamIdx >= g_ulParameterNum) return NULL;
	if (ulDataSize != g_Parameters[ulParamIdx].usSize) return NULL;

	return g_Parameters[ulParamIdx].lpAddr;
}

HW_SIM_TASK_API void* RTAPI HwTask_GetVariableTransAddress(HSIMTASK hTask, hw_uint32 ulVariableIdx, hw_uint32 ulBaseIdx, hw_uint32 ulOffsetIdx, hw_uint32 ulDataSize)
{
	if (ulVariableIdx >= g_ulVariableNum) return NULL;
	if (ulDataSize < g_Variables[ulVariableIdx].usSize) return NULL;

	return g_Variables[ulVariableIdx].lpAddr;
}

HW_SIM_TASK_API void* RTAPI HwTask_GetInputPortTransAddress(HSIMTASK hTask, hw_uint32 ulInputPortIdx, hw_uint32 ulBaseIdx, hw_uint32 ulOffsetIdx, hw_uint32 ulDataSize)
{
	if (ulInputPortIdx >= g_ulInputPortNum) return NULL;
	if (ulDataSize < g_InputPorts[ulInputPortIdx].usSize) return NULL;

	return g_InputPorts[ulInputPortIdx].lpAddr;
}

HW_SIM_TASK_API void* RTAPI HwTask_GetOutputPortTransAddress(HSIMTASK hTask, hw_uint32 ulOutputPortIdx, hw_uint32 ulBaseIdx, hw_uint32 ulOffsetIdx, hw_uint32 ulDataSize)
{
	if (ulOutputPortIdx >= g_ulOutputPortNum) return NULL;
	if (ulDataSize < g_OutputPorts[ulOutputPortIdx].usSize) return NULL;

	return g_OutputPorts[ulOutputPortIdx].lpAddr;
}