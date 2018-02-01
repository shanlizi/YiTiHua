/*****************************************************************
 *
 * HwDataItem.h
 *
 * Shanghai Share-E tech. Co., LTD.
 *
 ****************************************************************/
#include "rtx_inc.h"
#include "PXI2205Util.h"
#include "CPCI6216Util.h"
#include "CPCI7432Util.h"
#include "CPCI7841Util.h"
#include "MIC3612Util.h"
#include "ComFrame.h"

#ifndef __HWDATAITEM_20170908__
#define __HWDATAITEM_20170908__


/* {{JETLAB_PARAMETER */
extern double g_dDA[16];
extern hw_uint8 g_ucDO[32];
extern hw_uint32 g_ulTestCaseNo;	/* 测试项编号 */
extern double g_dFreq;
extern bool DI_DO;
extern hw_uint8 g_IMUTickFlag;   ////1-12，6位置或12位置
extern hw_uint8 g_RunTimes;
/* }}JETLAB_PARAMETER */


/* {{JETLAB_VARIABLE */
extern double g_TimeKO7;
extern double g_TimeTurnOn;
extern double g_dSine;
extern double ADValue[2][MAX_2205_AD_NUM];
extern hw_uint8 DOValue[MAX_7432_DO_NUM];
extern hw_uint8 DIValue[MAX_7432_DI_NUM];
extern double DAValue[MAX_6216_DA_NUM];

extern bool bTimerState[5];
extern bool bRegisterWrong;

extern bool DI_KI[2][8];  //分别为DDK1-1,DDK1-2，KI3A，KI4，KI5，KI6，KI7，KI3B， 对应字节数为0,1,2,15,3,4,5,6
extern bool DI_DDK2_YC[2];
extern double DI_DDK2_JC[2];
extern double DI_DDK11_YC[2];
extern double DI_DDK12_YC[2];
extern double DI_KO7[2];

extern hw_uint32 g_Count_Wrong;
extern hw_uint32 g_Count_Recv;

extern hw_uint32 g_Count;

extern hw_uint32 lFlag;//作为单个测试程序结束标志

extern float dDAInputValue;
extern float dDAInputTest;

extern hw_uint32 CPU_ID;
extern hw_uint32 CPU_Rev_ID;

extern CIMUComm g_YTH;

extern hw_uint8 g_IMUTickFlag0;  //用于监视一个位置是否测试结束
extern hw_uint32 g_IMUTickBegin;  //用于IMU接收数据开始时 的Tick
extern hw_uint32 g_IMUTickEnd;    //用于IMU接收数据结束时 的Tick
extern hw_uint32 g_IMUCountLoss;
extern hw_uint32 g_IMUCountRecv;
extern hw_uint16 g_IMUCount;
extern double    g_COMErrorRate;

extern double g_DAValue[10][8];  //采集8个值，有10组
extern double DADA11;

extern double g_27VA_up;
extern double g_27VA_Down;
extern double g_27VB_up;
extern double g_27VB_Down;
extern double g_60VC_up;
extern double g_60VC_Down;
extern double g_JHH_ValueA;
extern double g_JHH_Value1;
extern double g_JHH_Value2;

extern double g_DDK2_GND;        //常闭点，接GND1
extern double g_DDK2_XuanKong;   //悬空
extern double g_ReadValue[9];

//FlasH   RAM
extern hw_uint32 g_FlagRAM[2];
extern hw_uint32 g_RAMAdress[2];
extern hw_uint32 g_FlashNum[2][2];  //一维有程序FLASH和数据FLASH，二维有使能与非使能
extern hw_uint32 g_FlashClear[2][2];
extern hw_uint32 g_FlashWrite[2][2];
extern hw_uint32 g_FailAdress[2][2];

extern hw_uint8 g_SRAM;         //传给报表SBSRAM
extern hw_uint8 g_FLASH;        //传给报表FLASH
extern hw_uint8 g_FLASHClose;   //传给报表FLASH封锁

extern double g_IMUPowerStart[3];    //填写IMU启动状态时的瞬间电流  分别对应18V，27V，36V
extern double g_IMUPowerRunState[3];

/* }}JETLAB_VARIABLE */


/* {{JETLAB_INPORT */

/* }}JETLAB_INPORT */


/* {{JETLAB_OUTPORT */

/* }}JETLAB_OUTPORT */


#endif /* __HWDATAITEM_20170908__ */


