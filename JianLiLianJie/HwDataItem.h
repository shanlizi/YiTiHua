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
extern hw_uint32 g_ulTestCaseNo;	/* �������� */
extern double g_dFreq;
extern bool DI_DO;
extern hw_uint8 g_IMUTickFlag;   ////1-12��6λ�û�12λ��
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

extern bool DI_KI[2][8];  //�ֱ�ΪDDK1-1,DDK1-2��KI3A��KI4��KI5��KI6��KI7��KI3B�� ��Ӧ�ֽ���Ϊ0,1,2,15,3,4,5,6
extern bool DI_DDK2_YC[2];
extern double DI_DDK2_JC[2];
extern double DI_DDK11_YC[2];
extern double DI_DDK12_YC[2];
extern double DI_KO7[2];

extern hw_uint32 g_Count_Wrong;
extern hw_uint32 g_Count_Recv;

extern hw_uint32 g_Count;

extern hw_uint32 lFlag;//��Ϊ�������Գ��������־

extern float dDAInputValue;
extern float dDAInputTest;

extern hw_uint32 CPU_ID;
extern hw_uint32 CPU_Rev_ID;

extern CIMUComm g_YTH;

extern hw_uint8 g_IMUTickFlag0;  //���ڼ���һ��λ���Ƿ���Խ���
extern hw_uint32 g_IMUTickBegin;  //����IMU�������ݿ�ʼʱ ��Tick
extern hw_uint32 g_IMUTickEnd;    //����IMU�������ݽ���ʱ ��Tick
extern hw_uint32 g_IMUCountLoss;
extern hw_uint32 g_IMUCountRecv;
extern hw_uint16 g_IMUCount;
extern double    g_COMErrorRate;

extern double g_DAValue[10][8];  //�ɼ�8��ֵ����10��
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

extern double g_DDK2_GND;        //���յ㣬��GND1
extern double g_DDK2_XuanKong;   //����
extern double g_ReadValue[9];

//FlasH   RAM
extern hw_uint32 g_FlagRAM[2];
extern hw_uint32 g_RAMAdress[2];
extern hw_uint32 g_FlashNum[2][2];  //һά�г���FLASH������FLASH����ά��ʹ�����ʹ��
extern hw_uint32 g_FlashClear[2][2];
extern hw_uint32 g_FlashWrite[2][2];
extern hw_uint32 g_FailAdress[2][2];

extern hw_uint8 g_SRAM;         //��������SBSRAM
extern hw_uint8 g_FLASH;        //��������FLASH
extern hw_uint8 g_FLASHClose;   //��������FLASH����

extern double g_IMUPowerStart[3];    //��дIMU����״̬ʱ��˲�����  �ֱ��Ӧ18V��27V��36V
extern double g_IMUPowerRunState[3];

/* }}JETLAB_VARIABLE */


/* {{JETLAB_INPORT */

/* }}JETLAB_INPORT */


/* {{JETLAB_OUTPORT */

/* }}JETLAB_OUTPORT */


#endif /* __HWDATAITEM_20170908__ */


