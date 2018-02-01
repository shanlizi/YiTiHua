/*****************************************************************
 *
 * HwUserApp.cpp
 *
 * Shanghai Share-E tech. Co., LTD.
 *
 串口通道信息如下：

0卡0通道 TX    DY口
         RX   DY口

0卡1通道 TX    FK口
 		 RX   FK口

0卡2通道 TX    GC口 74ZKN
		 RX   YC口

0卡3通道 TX    转台控制机
		 RX    GC口 74ZKN

1卡0通道 TX     模拟惯组RGC3信号
		 RX    惯组15ZKN-GC口
 ****************************************************************/
    
#include <rtx_inc.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <HwSimTaskSupport.h>
#include "HwDataItem.h"
#include "ComFrame.h"

hw_uint8 g_RunTimes = 0;  //参数，需用户输入，默认为0，报表结果填写第几次，共有3次分别对应0，1，2

double g_dDA[16] ={ 0 };
hw_uint8 g_ucDO[32] ={ 0 };

CPXI2205Util g_ad;
CCPCI6216Util g_da;
CCPCI7432Util g_dio;
//CYiTiHuaTestComm g_YTH;
CIMUComm g_YTH;
CMIC3612Util g_Com;
hw_uint8* lpSendBuf;
hw_uint8* lpReceBuf;

#define D_SEND_COUNT 5000
#define D_SEND_COUNT_IMU 7210   //IMU计数,180s/2.5ms = 72000,外加10帧，免除首部的糟粕帧，因为在测试用例中参数与变量的同步有延迟
#define D_COUNTTEST 3   //每个测试用例重复执行3次忙吧3此结果均写入测试报告中
hw_uint8 g_CountTest = -1;  //测试用例重复执行3次中的第几次，可取0，1，2
/****************上传给报表的数据*******************/
double g_ReadValue[9] = {0.0}; //保存9个电压值，对应报表中的电源遥测
//开机时间
double g_TimeKO7 = 0.0;     //约等于300ms
double g_TimeTurnOn = 0.0;  //约等于2.3秒

//建立连接测试：
hw_uint32 CPU_ID = 0;
hw_uint32 CPU_Rev_ID = 0;

//CPU和寄存器测试
double g_dFreq = 200.0;
bool bTimerState[5] = {0};
bool bRegisterWrong = 1;

//FlasH   RAM
hw_int8 g_CountFlashRam = -1;
hw_uint32 g_FlagRAM[2] = { 0 };  
hw_uint32 g_RAMAdress[2] = { 0 };
hw_uint32 g_FlashNum[2][2] = {0};  //一维有程序FLASH和数据FLASH，二维有使能与非使能
hw_uint32 g_FlashClear[2][2] = { 0 };
hw_uint32 g_FlashWrite[2][2] = { 0 };
hw_uint32 g_FailAdress[2][2] = { 0 };

hw_uint8 g_SRAM = 0;         //传给报表SBSRAM
hw_uint8 g_FLASH = 0;        //传给报表FLASH
hw_uint8 g_FLASHClose = 0;   //传给报表FLASH封锁




//IMU
hw_uint32 g_IMUCountLoss = 0;
hw_uint32 g_IMUCountRecv = 0;
double g_IMUErrorRate = 0.0;
hw_uint32 g_IMUTickBegin = 0;  //用于IMU接收数据开始时 的Tick
hw_uint32 g_IMUTickEnd = 0;    //用于IMU接收数据结束时 的Tick
hw_uint8 g_IMUTickFlag = 0xFF;   //1-12，6位置或12位置,为参数
hw_uint8 g_IMUTickFlag0 = 0xFF;    //g_IMUTickFlag0为变量，其值保持与g_IMUTickFlag一致，因为上位机软件while()循环只能判断变量，但是我们改变的是参数
//以及以下数据
//float		m_fdWx;		/* X1a轴角增量 dWx*/
//float		m_fdWy;		/* Y1a轴角增量 dWy*/
//float		m_fdWz;		/* Z1a轴角增量 dWz*/
//
//float		m_fdVx;		/* X1a速度增量 dVx*/
//float		m_fdVy;		/* Y1a速度增量 dVy*/
//float		m_fdVz;		/* Z1a速度增量 dVz*/
//
//float		m_fWx;		/* X1a轴角速度 Wx*/
//float		m_fWy;		/* Y1a轴角速度 Wy*/
//float		m_fWz;		/* Z1a轴角速度 Wz*/
//
//float		m_fNx;		/* X1a轴加速度 Nx*/
//float		m_fNy;		/* Y1a轴加速度 Ny*/
//float		m_fNz;		/* Z1a轴加速度 Nz*/


//串口  包括FK， DY ,YC ,GC
hw_uint8  g_DataCOMSend = 0;
hw_uint32 g_Count_Wrong = 0;
hw_uint32 g_Count_Recv = 0;  //测试接收5000帧
double    g_COMErrorRate = 0.0; 
bool bFlagFirstCom = hw_true;


//DA测试
double g_DAValue[10][8] = { 0 };  //采集8个值，有10组,1组初始值，9组（-10.0, -7.5, -5.0, -2.5, 0.0, 2.5, 5.0, 7.5, 9.999 ）
double DADA11 = 0.0;

//激活好  用于保存激活好变化时的阈值电压
double g_27VA_up = 0.0;
double g_27VA_Down = 0.0;
double g_27VB_up = 0.0;
double g_27VB_Down = 0.0;
double g_60VC_up = 0.0;
double g_60VC_Down = 0.0;
double g_JHH_ValueA = 0.0;  //输出电流
double g_JHH_Value1 = 0.0;  //输出+3.1
double g_JHH_Value2 = 0.0;  //输出-3.1

//DDK2
double g_DDK2_GND = 0.0;        //常闭点，接GND1
double g_DDK2_XuanKong = 0.0;   //悬空

/*************************************************/

hw_uint32 g_ulTestCaseNoOld = 0;

//DA测试
float dDAInput[9] = { -10.0, -7.5, -5.0, -2.5, 0.0, 2.5, 5.0, 7.5, 9.999 };
float dDAInputValue = 0.0;
float dDAInputTest = 0.0;
hw_int32 DACount = 0;

//开关量测试
bool DI_DO = 0 ;
hw_int8 DI_Count = -1;
hw_uint8 DI_Input[2] = { 0, 1 };
bool DI_KI[2][8] = { 0 };

bool DI_DDK2_YC[2] = {0};
double DI_DDK2_JC[2] = {0.0};
double DI_DDK11_YC[2] = { 0.0 };
double DI_DDK12_YC[2] = { 0.0 };
double DI_KO7[2] = { 0.0 };
//IMU
hw_uint16 g_IMUCount = 0;
hw_uint16 g_IMULastCount = 0;
hw_uint8 g_IMUFirstTime = 3;  //接收Buf有128个字节，缓存2帧数据
double g_IMUPowerStart[3] = { 0 };    //填写IMU启动状态时的瞬间电流  分别对应18V，27V，36V
double g_IMUPowerRunState[3] = { 0 };  //填写IMU运行状态时的电流  约0.86A
hw_int8 g_IMUPowerIndex = -1;  //0,1,2 分别对应18V，27V，36V
double g_IMUPowerTemp = 0;


//串口
hw_uint32 g_Count = 0;

//开机时间
bool bFlagFirst = hw_true;
bool bFlagKO7 = hw_true;
bool bFlagTurnOn = hw_true;



#define CHANNEL_POWER_YITIHUA_27VA 1
#define CHANNEL_POWER_YITIHUA_27VB 2
#define CHANNEL_POWER_YITIHUA_60VC 1

#define D_MATCH_CONNECT   10002
#define D_MATCH_CPUREG    10003
#define D_MATCH_RAMFLASH  10004
#define D_MATCH_IMU       10005
#define D_MATCH_IMU_START 10055    //用于测试IMU  SinglePosition状态，填写报表的启动状态，拉偏状态
#define D_MATCH_WATCHDOG  10006
#define D_MATCH_FK        10007
#define D_MATCH_DY        10008
#define D_MATCH_YC        10009
#define D_MATCH_GC        10010
#define D_MATCH_DA        10011
#define D_MATCH_DI        10012
#define D_MATCH_ENGINE    10013
#define D_MATCH_HELP      10014
#define D_MATCH_JIHUOHAO  10015

#define D_MATCH_YC_GC     10016  //整机测试时，YC和GC不能直接测试，可通过IMU惯测间接测试，此状态直接执行D_MATCH_IMU的代码
#define D_MATCH_DDK2      10017  //弹动信号2

#define D_MATCH_POWER_ON       10101   //电源开操作
#define D_MATCH_POWER_OFF      10102   //电源关操作
#define D_MATCH_POWER_ON_18V   10103   //18V电源  IMU拉偏状态
#define D_MATCH_POWER_ON_36V   10104   //36V电源  IMU拉偏状态
#define D_MATCH_TEST           10201   //仅供测试


hw_uint32 g_ulTestCaseNo = 0xFFFFFFFF;	/* 测试项编号 */
enum
{
	S_MATCH_WAIT    = 0xFF,
	S_MATCH_SEND0   = 0,
	S_MATCH_SEND1   = 1,
	S_MATCH_SEND2   = 2,
	S_MATCH_SEND3   = 3,
	S_MATCH_RECV1   = 4,
	S_MATCH_RECV2   = 5,
	S_MATCH_RECV3   = 6,
};
hw_uint32 lFlag = S_MATCH_SEND0;


int OnSetupTask()
{
	WSADATA		wsd;
	if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0)
		return false;

	if (wsd.wVersion != 2)
	{
		/* Tell the user that we could not find a usable */
		/* WinSock DLL.                                  */
		WSACleanup();
		return -1;
	}
	return 0;
}

int OnInitializeTask()
{
	hw_uint8 ch = 0;

	g_da.Start(0);
	g_ad.LoadCoefficientFile();
	
	for (ch = 0; ch < MAX_2205_AD_NUM; ch++)
	{
		g_ad.SetChannelMode(0, ch, PXI2205_REFV_10_V, hw_false, hw_false);
	}
	g_ad.Start(0);
	

	for (ch = 0; ch < (MAX_2205_AD_NUM / 2); ch++)
	{
		g_ad.SetChannelMode(1, ch, PXI2205_REFV_10_V, hw_true, hw_false);
	}
	g_ad.Start(1);

	g_dio.Start(0);
	//g_dio.SetFeiKongWenYaDianYuan(hw_true);
	g_dio.WriteDOValue(0);
	g_Com.Open(0, 1, 153600, hw_true);
	g_Com.Open(0, 0, 614400, hw_true);
	g_Com.Open(0, 2, 614400, hw_true);
	g_Com.Open(0, 3, 614400, hw_true);
	g_Com.Open(1, 0, 614400, hw_true);

	return 0;
}


int OnStartTask()
{
	return 0;
}


int OnStepTask(int iTidNo, hw_int64 llTickCount, hw_int64 llTime)
{
	hw_uint8 ch = 0;
	hw_uint8 lpRecedata[128] = {0};
	hw_int32 lLengthRece = 0;
	
	if (0==iTidNo)
	{
		
		
		if (/*D_MATCH_POWER_ON != g_ulTestCaseNo && */g_ulTestCaseNoOld != g_ulTestCaseNo)
		{
			g_IMULastCount = 0;
			g_CountTest = -1;
			g_Count = 0;
			g_Count_Wrong = 0;
			g_Count_Recv = 0;			
			lFlag = S_MATCH_SEND0;
			bFlagFirst = hw_true;
			bFlagFirstCom = hw_true;
			printd("Start a new TEST,TesT_ID = %d\n", g_ulTestCaseNo);
		}
		g_ulTestCaseNoOld = g_ulTestCaseNo;

		switch (g_ulTestCaseNo)  //总测试项
		{
		case D_MATCH_TEST:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				g_dio.SetDOValue(0, DO0::DDXH2弹动信号2_继电器控制, 0);
				lFlag = S_MATCH_SEND1;
				printd("D_MATCH_TEST %d \n",0);
				break;
			default:
				break;
			}
			break;
		case D_MATCH_POWER_ON:  //上电操作，同时计算产品开机时间
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				lFlag = S_MATCH_SEND1;
				g_dio.SetFeiKong27VBDianYuan(hw_false);

				HwPower_OpenDevice(POWER_AGILENT5749, POWER_TYPE_AGILENT5749, "192.168.138.164", POWERPORT, hw_false);
				HwPower_TurnOn(POWER_AGILENT5749, 1);
				HwPower_SetCurrent(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 4.0, 0);
				HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 0, 0);

				HwPower_OpenDevice(POWER_AGILENT6702, POWER_TYPE_AGILENT6702, "192.168.138.163", POWERPORT, hw_false);
				HwPower_TurnOn(POWER_AGILENT6702, POWER_FEIKONG_27VB);
				HwPower_SetCurrent(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, 4.0, 0);
				HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, 0, 0);
				Sleep(1000);
				break;
			case S_MATCH_SEND1:
				lFlag = S_MATCH_SEND2;
				HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, 27, 0);
				Sleep(1000);  //延时，等待程控电源电压稳定
				//g_dio.SetFeiKong27VADianYuan(hw_true);
				g_dio.SetFeiKong27VBDianYuan(hw_true);
				//g_dio.SetFeiKong60VCDianYuan(hw_true);
				for (int i=0;i<10;i++)
				{
					HwPower_GetCurrent(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, &g_IMUPowerTemp);
					if (g_IMUPowerTemp > g_IMUPowerStart[1])
					{
						g_IMUPowerStart[1] = g_IMUPowerTemp;
					}					
				}
				g_dio.SetFeiKong27VBDianYuan(hw_false);
				Sleep(1000);
				g_dio.SetFeiKong27VBDianYuan(hw_true);
				g_IMUPowerIndex = 1;
				g_TimeKO7 = 0.0;
				g_TimeTurnOn = 0.0;
				break;
			case S_MATCH_SEND2:
				if (bFlagKO7)
				{
					g_TimeKO7 += 2.5;
				}
				if (bFlagTurnOn)
				{
					g_TimeTurnOn += 2.5;

					lLengthRece = g_Com.Receive(0, 2, g_YTH.RecvBuf());
					g_YTH.AppendData(lLengthRece);
					if (bFlagFirst) //清除之前上电是遗留在buf中的数据，仅执行一次
					{
						bFlagFirst = hw_false;
						g_YTH.ClearBuf();
					}
					
					if (g_YTH.ParseFrame())
					{
						bFlagTurnOn = hw_false;
						lFlag = S_MATCH_RECV1;
					}
				}
				break;
			case S_MATCH_RECV1:
				for (int i=0;i<10;i++)
				{
					g_ad.ReadData(0);
					g_ReadValue[0] += ADValue[0][AD0::GCZ15惯测正15V遥测];
					g_ReadValue[1] += ADValue[0][AD0::GCF15惯测负15V遥测];
					g_ReadValue[2] += ADValue[0][AD0::GCZ5惯测正5V遥测];
					g_ReadValue[3] += ADValue[0][AD0::GCF5惯测负5V遥测];
					g_ReadValue[4] += ADValue[0][AD0::Z5VYC主5V遥测];
					g_ReadValue[5] += ADValue[0][AD0::Z15VYC正15V遥测];
					g_ReadValue[6] += ADValue[0][AD0::F15VYC负15V遥测];
					g_ReadValue[7] += ADValue[0][AD0::DQKZ电气控制正5V1遥测];
					g_ReadValue[8] += ADValue[0][AD0::DQKZ电气控制正5V2遥测];
				}
				g_ReadValue[0] /= 10;
				g_ReadValue[1] /= 10;
				g_ReadValue[2] /= 10;
				g_ReadValue[3] /= 10;
				g_ReadValue[4] /= 10;
				g_ReadValue[5] /= 10;
				g_ReadValue[6] /= 10;
				g_ReadValue[7] /= 10;
				g_ReadValue[8] /= 10;
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;
		case D_MATCH_POWER_ON_18V:  //18V电源  IMU拉偏状态
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				lFlag = S_MATCH_SEND1;
				g_dio.SetFeiKong27VBDianYuan(hw_false);

				HwPower_OpenDevice(POWER_AGILENT5749, POWER_TYPE_AGILENT5749, "192.168.138.164", POWERPORT, hw_false);
				HwPower_TurnOn(POWER_AGILENT5749, 1);
				HwPower_SetCurrent(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 4.0, 0);
				HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 0, 0);

				HwPower_OpenDevice(POWER_AGILENT6702, POWER_TYPE_AGILENT6702, "192.168.138.163", POWERPORT, hw_false);
				HwPower_TurnOn(POWER_AGILENT6702, POWER_FEIKONG_27VB);
				HwPower_SetCurrent(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, 4.0, 0);
				HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, 0, 0);
				Sleep(1000);
				break;
			case S_MATCH_SEND1:
				lFlag = S_MATCH_SEND2;
				HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, 18, 0);
				Sleep(1000);  //延时，等待程控电源电压稳定
				//g_dio.SetFeiKong27VADianYuan(hw_true);
				g_dio.SetFeiKong27VBDianYuan(hw_true);
				//g_dio.SetFeiKong60VCDianYuan(hw_true);
				for (int i = 0; i < 10; i++)
				{
					HwPower_GetCurrent(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, &g_IMUPowerTemp);
					if (g_IMUPowerTemp > g_IMUPowerStart[0])
					{
						g_IMUPowerStart[0] = g_IMUPowerTemp;
					}
				}
				g_IMUPowerIndex = 0;
				break;
			case S_MATCH_SEND2:
				lLengthRece = g_Com.Receive(0, 2, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				if (bFlagFirst) //清除之前上电是遗留在buf中的数据，仅执行一次
				{
					bFlagFirst = hw_false;
					g_YTH.ClearBuf();
				}
				if (g_YTH.ParseFrame())
				{
					lFlag = S_MATCH_WAIT;
				}
				break;
			default:
				break;
			}
			break;
		case D_MATCH_POWER_ON_36V:  //36V电源  IMU拉偏状态
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				lFlag = S_MATCH_SEND1;
				g_dio.SetFeiKong27VBDianYuan(hw_false);

				HwPower_OpenDevice(POWER_AGILENT5749, POWER_TYPE_AGILENT5749, "192.168.138.164", POWERPORT, hw_false);
				HwPower_TurnOn(POWER_AGILENT5749, 1);
				HwPower_SetCurrent(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 4.0, 0);
				HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 0, 0);

				HwPower_OpenDevice(POWER_AGILENT6702, POWER_TYPE_AGILENT6702, "192.168.138.163", POWERPORT, hw_false);
				HwPower_TurnOn(POWER_AGILENT6702, POWER_FEIKONG_27VB);
				HwPower_SetCurrent(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, 4.0, 0);
				HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, 0, 0);
				Sleep(1000);
				break;
			case S_MATCH_SEND1:
				lFlag = S_MATCH_SEND2;
				HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, 36, 0);
				Sleep(1000);  //延时，等待程控电源电压稳定
							  //g_dio.SetFeiKong27VADianYuan(hw_true);
				g_dio.SetFeiKong27VBDianYuan(hw_true);
				//g_dio.SetFeiKong60VCDianYuan(hw_true);
				for (int i = 0; i < 10; i++)
				{
					HwPower_GetCurrent(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, &g_IMUPowerTemp);
					if (g_IMUPowerTemp > g_IMUPowerStart[2])
					{
						g_IMUPowerStart[2] = g_IMUPowerTemp;
					}
				}				
				g_IMUPowerIndex = 2;
				break;
			case S_MATCH_SEND2:
				lLengthRece = g_Com.Receive(0, 2, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				if (bFlagFirst) //清除之前上电是遗留在buf中的数据，仅执行一次
				{
					bFlagFirst = hw_false;
					g_YTH.ClearBuf();
				}
				if (g_YTH.ParseFrame())
				{
					lFlag = S_MATCH_WAIT;
				}
				break;
			default:
				break;
			}
			break;
		case D_MATCH_POWER_OFF:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				//g_dio.SetFeiKong60VCDianYuan(hw_false);
				//g_dio.SetFeiKong27VADianYuan(hw_false);
				g_dio.SetFeiKong27VBDianYuan(hw_false);
				HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, 0, 0);
				HwPower_TurnOff(POWER_AGILENT5749, 1);
				HwPower_TurnOff(POWER_AGILENT6702, POWER_FEIKONG_27VB);
				HwPower_CloseDevice(POWER_AGILENT5749);
				HwPower_CloseDevice(POWER_AGILENT6702);

				//g_dio.Stop(0);
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;
		case D_MATCH_CONNECT:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND2;
				break;
			case S_MATCH_SEND2:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_EQUIP_CONNECT;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				lLengthRece = g_Com.Receive(0, 1, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					if (0 == *(hw_uint16*)(g_YTH.RecvData()))
					{
						CPU_ID = *(hw_uint32*)(g_YTH.RecvData() + 2);
						CPU_Rev_ID = *(hw_uint32*)(g_YTH.RecvData() + 6);
						lFlag = S_MATCH_SEND3;
						printd("CONNECT_TEST succeed!\n");
					}
				}
				break;
			case S_MATCH_SEND3:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;

		case D_MATCH_CPUREG:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			/*case S_MATCH_SEND1:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_CPU_TEST;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;*/
			case S_MATCH_SEND1:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_CPU_TEST;
				*(hw_uint32*)(g_YTH.SendData() + 2) = 0;
				*(hw_uint32*)(g_YTH.SendData() + 6) = g_dFreq * 10000;;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				lLengthRece = g_Com.Receive(0, 1, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					if ((0x01 == *(hw_uint16*)(g_YTH.RecvData())))
					{
						int state = *(hw_uint8*)(g_YTH.RecvData() + 2);

						for (int i=0;i<5;i++)
						{
							bTimerState[i] = 0;
							if (state>>i && 0x01)
							{
								bTimerState[1] = 1;
							}

						}
						/* 寄存器校验状态（前5位）*(hw_uint8*)(g_YTH.RecvData() + 2) */
						lFlag = S_MATCH_SEND2;
					}
				}
				break;
			case S_MATCH_SEND2:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_CPU_TEST;
				*(hw_uint32*)(g_YTH.SendData() + 2) = 0;
				*(hw_uint32*)(g_YTH.SendData() + 6) = g_dFreq * 10000;;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV2;
				break;
			case S_MATCH_RECV2:
				lLengthRece = g_Com.Receive(0, 1, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					if ((0x01 == *(hw_uint16*)(g_YTH.RecvData())))
					{
						/* 寄存器校验状态（前1位）*(hw_uint32*)(g_YTH.RecvData() + 2) */
						bRegisterWrong = *(hw_uint32*)(g_YTH.RecvData() + 2);
						printd("CPU_TEST succeed!\n");
						
						lFlag = S_MATCH_SEND3;
					}
				}
				break;
			case S_MATCH_SEND3:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;
		case D_MATCH_RAMFLASH:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				g_CountFlashRam++;
				g_dio.SetDOValue(0, DO0::CXFXKZ程序Flash写控制_继电器控制, g_CountFlashRam);
				g_dio.SetDOValue(0, DO0::BCSJ补偿数据Flash写控制_继电器控制, g_CountFlashRam);
				g_dio.WriteDOValue(0);
				Sleep(2000);
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			case S_MATCH_SEND1:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_RAM_FLASH_TEST;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				lLengthRece = g_Com.Receive(0, 1, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					if ((0x0D == *(hw_uint16*)(g_YTH.RecvData())))
					{
						g_FlagRAM[g_CountFlashRam] = *(hw_uint32*)(g_YTH.RecvData() + 2);
						g_RAMAdress[g_CountFlashRam] = *(hw_uint32*)(g_YTH.RecvData() + 6);
						lFlag = S_MATCH_RECV2;
					}
				}
				break;
			case S_MATCH_RECV2:
				lLengthRece = g_Com.Receive(0, 1, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					if ((0x0D == *(hw_uint16*)(g_YTH.RecvData())))
					{
						g_FlashNum[0][g_CountFlashRam] = *(hw_uint32*)(g_YTH.RecvData() + 2);
						g_FlashClear[0][g_CountFlashRam] = *(hw_uint32*)(g_YTH.RecvData() + 6);
						g_FlashWrite[0][g_CountFlashRam] = *(hw_uint32*)(g_YTH.RecvData() + 10);
						g_FailAdress[0][g_CountFlashRam] = *(hw_uint32*)(g_YTH.RecvData() + 14);
						lFlag = S_MATCH_RECV3;
					}
				}
				break;
			case S_MATCH_RECV3:
				lLengthRece = g_Com.Receive(0, 1, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					if ((0x0D == *(hw_uint16*)(g_YTH.RecvData())))
					{
						g_FlashNum[1][g_CountFlashRam] = *(hw_uint32*)(g_YTH.RecvData() + 2);
						g_FlashClear[1][g_CountFlashRam] = *(hw_uint32*)(g_YTH.RecvData() + 6);
						g_FlashWrite[1][g_CountFlashRam] = *(hw_uint32*)(g_YTH.RecvData() + 10);
						g_FailAdress[1][g_CountFlashRam] = *(hw_uint32*)(g_YTH.RecvData() + 14);
						if (1 == g_CountFlashRam)
						{
							g_SRAM = g_FlagRAM[0];
							//使能时，程序FLASH与数据FLASH均 擦除成功  写入成功
							if (g_FlashClear[0][1] && g_FlashClear[1][1] && g_FlashWrite[0][1] && g_FlashWrite[1][1])
							{
								g_FLASH = 1;
							}
							else
							{
								g_FLASH = 0;
							}
							//非使能时，程序FLASH与数据FLASH均 擦除失败  写入失败
							if (!g_FlashClear[0][0] && !g_FlashClear[1][0] && !g_FlashWrite[0][0] && !g_FlashWrite[1][0])
							{
								g_FLASHClose = 1;
							}
							else
							{
								g_FLASHClose = 0;
							}
							lFlag = S_MATCH_SEND2;
						}
						else
						{
							lFlag = S_MATCH_SEND0;
						}
						Sleep(1000);
					}
				}
				break;
			case S_MATCH_SEND2:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;
		case D_MATCH_IMU:
			g_IMUTickFlag0 = g_IMUTickFlag;
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				if (0 == g_IMUTickFlag)
				{
					lFlag = S_MATCH_SEND0;
					g_IMUFirstTime = 3;
					g_IMUCountLoss = 0;
					//bFlagFirstCom = 1;
					break;
				}
				g_IMUCountRecv = 0;
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			case S_MATCH_SEND1:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_IMU_TEST;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				if (bFlagFirstCom)  //此处只运行一次,清除缓存中遗留的数据
				{
					g_YTH.ClearBuf();
					bFlagFirstCom = hw_false;
					g_IMUTickBegin = llTickCount + 1;
					printd("IMU tick count start %d \n", llTickCount+1);
				} 		
				lLengthRece = g_Com.Receive(0, 2, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.ParseFrame())
				{
					//g_IMUTickFlag = 1;
					g_IMUCount = *(hw_uint16*)(g_YTH.RecvData());
					g_IMUCountRecv++;
					if ((1 != g_IMUCount - g_IMULastCount) && (!g_IMUFirstTime) && (0!= g_IMUCount))
					{
						g_IMUCountLoss++;
						printd("IMU LOSE g_IMUCount:%d,  g_IMULastCount:%d \n", g_IMUCount, g_IMULastCount);
					}
					if (g_IMUFirstTime)
					{
						g_IMUFirstTime--; //接收Buf有128个字节，缓存2帧数据
					}
					
					if (D_SEND_COUNT_IMU == g_IMUCountRecv)
					{
						g_IMUTickFlag = 0;
						g_IMUTickFlag0 = g_IMUTickFlag;
						g_IMUTickEnd = llTickCount;
						printd("IMU tick count start %d \n", llTickCount);
						g_IMUFirstTime = 3;
						g_IMUErrorRate = (double)g_IMUCountLoss / g_IMUCountRecv;
						lFlag = S_MATCH_SEND0;
						printd("g_IMUCount %d  --g_IMULastCount:%d   --g_IMUCountRecv:%d  g_IMUCountLoss:%d \n", g_IMUCount, g_IMULastCount, g_IMUCountRecv, g_IMUCountLoss);
					}
					g_IMULastCount = g_IMUCount;
				}
				break;
			case S_MATCH_SEND2:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;
		case D_MATCH_IMU_START:
			g_IMUTickFlag0 = g_IMUTickFlag;
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				if (0 == g_IMUTickFlag)
				{
					lFlag = S_MATCH_SEND0;
					g_IMUFirstTime = 3;
					g_IMUCountLoss = 0;
					//bFlagFirstCom = 1;
					break;
				}
				g_IMUCountRecv = 0;
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			case S_MATCH_SEND1:
				HwPower_GetCurrent(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, &g_IMUPowerRunState[g_IMUPowerIndex]);
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_IMU_TEST;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				if (bFlagFirstCom)  //此处只运行一次,清除缓存中遗留的数据
				{
					g_YTH.ClearBuf();
					bFlagFirstCom = hw_false;
					g_IMUTickBegin = llTickCount + 1;
					printd("IMU tick count start %d \n", llTickCount + 1);
				}
				lLengthRece = g_Com.Receive(0, 2, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.ParseFrame())
				{
					//g_IMUTickFlag = 1;
					g_IMUCount = *(hw_uint16*)(g_YTH.RecvData());
					g_IMUCountRecv++;
					if ((1 != g_IMUCount - g_IMULastCount) && (!g_IMUFirstTime) && (0 != g_IMUCount))
					{
						g_IMUCountLoss++;
						printd("IMU LOSE g_IMUCount:%d,  g_IMULastCount:%d \n", g_IMUCount, g_IMULastCount);
					}
					if (g_IMUFirstTime)
					{
						g_IMUFirstTime--; //接收Buf有128个字节，缓存2帧数据
					}

					if (D_SEND_COUNT_IMU == g_IMUCountRecv)
					{
						g_IMUTickFlag = 0;
						g_IMUTickFlag0 = g_IMUTickFlag;
						g_IMUTickEnd = llTickCount;
						printd("IMU tick count start %d \n", llTickCount);
						g_IMUFirstTime = 3;
						g_IMUErrorRate = (double)g_IMUCountLoss / g_IMUCountRecv;
						lFlag = S_MATCH_SEND0;
						printd("g_IMUCount %d  --g_IMULastCount:%d   --g_IMUCountRecv:%d  g_IMUCountLoss:%d \n", g_IMUCount, g_IMULastCount, g_IMUCountRecv, g_IMUCountLoss);
					}
					g_IMULastCount = g_IMUCount;
				}
				break;
			case S_MATCH_SEND2:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;
		case D_MATCH_WATCHDOG:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			case S_MATCH_SEND1:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_WATCH_DOG_TEST;
				*(hw_uint32*)(g_YTH.SendData() + 2) = 0;
				*(hw_uint32*)(g_YTH.SendData() + 6) = g_dFreq * 10000;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_WAIT;
				printd("WATCH_DOG Test succeed!\n");
				break;
			default:
				break;
			}
			break;
		case D_MATCH_FK:	
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			case S_MATCH_SEND1:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_COMM_FK;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				lLengthRece = g_Com.Receive(0, 1, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					if (0x04 == *(hw_uint16*)(g_YTH.RecvData()))
					{
						lFlag = S_MATCH_SEND2;
					}
				}
				break;
			case S_MATCH_SEND2:
				lFlag = S_MATCH_RECV2;
				if (D_SEND_COUNT == g_Count++)
				{
					g_COMErrorRate = (double)g_Count_Wrong / g_Count_Recv;
					g_Count = 0;
					lFlag = S_MATCH_SEND3;
					break;
				}
				g_DataCOMSend = g_Count % 10;
				for (int i = 0; i < 0x36; i++)
				{
					*(hw_uint8*)(g_YTH.SendData() + i) = g_DataCOMSend;
				}
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				break;
			case S_MATCH_RECV2:
				lLengthRece = g_Com.Receive(0, 1, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					g_Count_Recv++;
					for (int i = 0; i < 0x36; i++)
					{
						if (*(hw_uint8*)(g_YTH.RecvData() + i) != g_DataCOMSend)
						{
							g_Count_Wrong++;
							break;
						}
						lFlag = S_MATCH_SEND2;
					}
				}
				break;
			case S_MATCH_SEND3:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_WAIT;
				printd("FK Test succeed!\n");
				break;
			default:
				break;
			}
			break;
		case D_MATCH_DY:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			case S_MATCH_SEND1:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_COMM_DY;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				lLengthRece = g_Com.Receive(0, 0, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					if (0x06 == *(hw_uint16*)(g_YTH.RecvData()))
					{
						lFlag = S_MATCH_SEND2;
					}
				}
				break;
			case S_MATCH_SEND2:
				lFlag = S_MATCH_RECV2;
				if (D_SEND_COUNT == g_Count++)
				{
					g_COMErrorRate = (double)g_Count_Wrong / g_Count_Recv;
					g_Count = 0;
					lFlag = S_MATCH_SEND3;
					break;
				}
				g_DataCOMSend = g_Count % 10;
				for (int i = 0; i < 0x36; i++)
				{
					*(hw_uint8*)(g_YTH.SendData() + i) = g_DataCOMSend;
				}
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 0, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				break;
			case S_MATCH_RECV2:
				lLengthRece = g_Com.Receive(0, 0, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					lFlag = S_MATCH_SEND2;
					g_Count_Recv++;
					for (int i = 0; i < 0x36; i++)
					{
						if (*(hw_uint8*)(g_YTH.RecvData() + i) != g_DataCOMSend)
						{
							g_Count_Wrong++;
							break;
						}
						
					}
				}
				break;
			case S_MATCH_SEND3:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 0, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				printd("DY Test succeed!\n");
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;
		case D_MATCH_YC:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			case S_MATCH_SEND1:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_COMM_DY;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				lLengthRece = g_Com.Receive(0, 2, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					if (0x06 == *(hw_uint16*)(g_YTH.RecvData()))
					{
						lFlag = S_MATCH_SEND2;
					}
				}
				break;
			case S_MATCH_SEND2:
				lFlag = S_MATCH_RECV2;
				if (D_SEND_COUNT == g_Count++)
				{
					g_COMErrorRate = (double)g_Count_Wrong / g_Count_Recv;
					g_Count = 0;
					lFlag = S_MATCH_SEND3;
					break;
				}
				g_DataCOMSend = g_Count % 10;
				for (int i = 0; i < 0x36; i++)
				{
					*(hw_uint8*)(g_YTH.SendData() + i) = g_DataCOMSend;
				}
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 0, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				break;
			case S_MATCH_RECV2:
				lLengthRece = g_Com.Receive(0, 2, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					lFlag = S_MATCH_SEND2;
					g_Count_Recv++;
					for (int i = 0; i < 0x36; i++)
					{
						if (*(hw_uint8*)(g_YTH.RecvData() + i) != g_DataCOMSend)
						{
							g_Count_Wrong++;
							break;
						}

					}
				}
				break;
			case S_MATCH_SEND3:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 0, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				printd("DY Test succeed!\n");
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;
		case D_MATCH_GC:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			case S_MATCH_SEND1:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_COMM_GC;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				lLengthRece = g_Com.Receive(0, 2, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					if (0x05 == *(hw_uint16*)(g_YTH.RecvData()))
					{
						lFlag = S_MATCH_SEND2;
					}
				}
				break;
			case S_MATCH_SEND2:
				lFlag = S_MATCH_RECV2;
				if (D_SEND_COUNT == g_Count++)
				{
					g_COMErrorRate = (double)g_Count_Wrong / g_Count_Recv;
					g_Count = 0;
					lFlag = S_MATCH_SEND3;
					break;
				}
				g_DataCOMSend = g_Count % 10;
				for (int i = 0; i < 0x36; i++)
				{
					*(hw_uint8*)(g_YTH.SendData() + i) = g_DataCOMSend;
				}
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 2, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				break;
			case S_MATCH_RECV2:
				lLengthRece = g_Com.Receive(0, 2, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					g_Count_Recv++;
					for (int i = 0; i < 0x36; i++)
					{
						if (*(hw_uint8*)(g_YTH.RecvData() + i) != g_DataCOMSend)
						{
							g_Count_Wrong++;
							break;
						}
						lFlag = S_MATCH_SEND2;
					}
				}
				break;
			case S_MATCH_SEND3:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 2, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				printd("GC Test succeed!\n");
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;
		case D_MATCH_YC_GC:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			case S_MATCH_SEND1:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_IMU_TEST;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				if (bFlagFirstCom)  //此处只运行一次,清除缓存中遗留的数据
				{
					g_YTH.ClearBuf();
					bFlagFirstCom = hw_false;
				}
				lLengthRece = g_Com.Receive(0, 2, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.ParseFrame())
				{
					g_IMUCount = *(hw_uint16*)(g_YTH.RecvData());
					g_Count_Recv++;
					if ((1 != g_IMUCount - g_IMULastCount) && (!g_IMUFirstTime) && (0!= g_IMUCount))
					{
						g_Count_Wrong++;
						printd("YC_GC LOSE CurrentCountNo:%d,  LastCountNo:%d \n", g_IMUCount, g_IMULastCount);
					}
					if (g_IMUFirstTime)  //接收Buf有128个字节，缓存2帧数据
					{
						g_IMUFirstTime--; //	区别是否是第一次接收数据
					}
					
					if (D_SEND_COUNT == g_Count_Recv)
					{
						g_COMErrorRate = (double)g_Count_Wrong / g_Count_Recv;
						lFlag = S_MATCH_SEND2;
						//printd("g_IMUCount %d  --g_IMULastCount:%d   --g_IMUCountRecv:%d  g_IMUCountLoss:%d \n", g_IMUCount, g_IMULastCount, g_IMUCountRecv, g_IMUCountLoss);
					}
					g_IMULastCount = g_IMUCount;
				}
				break;
			case S_MATCH_SEND2:
				g_IMUFirstTime = 3;
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;
		case D_MATCH_DA:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				g_ad.ReadData(1);
				for (int i = 0; i < 8; i++)
				{
					g_DAValue[9][i] = ADValue[1][i + 2];  //初始值放在10组中的最后一组
				}
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			case S_MATCH_SEND1:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_DA_TEST_SINGLE;
				*(float*)(g_YTH.SendData() + 2) =  dDAInput[DACount];
				*(float*)(g_YTH.SendData() + 6) = dDAInput[DACount];
				*(float*)(g_YTH.SendData() + 10) = dDAInput[DACount];
				*(float*)(g_YTH.SendData() + 14) = dDAInput[DACount];
				dDAInputValue = dDAInput[DACount];			
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				lLengthRece = g_Com.Receive(0, 1, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				if (9 == DACount)
				{
					lFlag = S_MATCH_WAIT;
					DACount = 0;
					break;
				}
				while (g_YTH.VerifyFrame())
				{
					if (0x0A == *(hw_uint16*)(g_YTH.RecvData()))
					{
						Sleep(200);
						g_ad.ReadData(1);
						for (int i = 0; i < 8; i++)
						{
							g_DAValue[DACount][i] = ADValue[1][i + 2];
							/*if (1 == DACount && 1 == i)
							{
								DADA11 = ADValue[1][3];
							}*/
						}
						lFlag = S_MATCH_SEND2;
						DACount++;
					}
				}
				break;
			case S_MATCH_SEND2:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;
		case D_MATCH_DI:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				DI_Count++;
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			case S_MATCH_SEND1:
				g_dio.SetDOValue(0, DO0::DDXH2弹动信号2_继电器控制, DI_Input[DI_Count]);

				g_dio.SetDOValue(0, DO0::N14舵系统BIT信号, DI_Input[DI_Count]);                     //KI3A
				g_dio.SetDOValue(0, DO0::N23舵系统BIT信号, DI_Input[DI_Count]);                     //KI3B
				g_dio.SetDOValue(0, DO0::DYTBIT导引头BIT信号, DI_Input[DI_Count]);                  //KI4
				g_dio.SetDOValue(0, DO0::YXBIT引信BIT信号, DI_Input[DI_Count]);                     //KI5
				g_dio.SetDOValue(0, DO0::CXFXKZ程序Flash写控制_继电器控制, DI_Input[DI_Count]);      //KI6
				g_dio.SetDOValue(0, DO0::BCSJ补偿数据Flash写控制_继电器控制, DI_Input[DI_Count]);    //KI7
				g_dio.SetDOValue(0, DO0::DDXH1弹动信号1_1_继电器控制, DI_Input[DI_Count]);           //DDK1-1
				g_dio.SetDOValue(0, DO0::DDXH2弹动信号1_2_继电器控制, DI_Input[DI_Count]);           //DDK1-2
				g_dio.WriteDOValue(0);
				Sleep(100);
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_DI_TEST;
				*(hw_uint8*)(g_YTH.SendData() + 6) = DI_Input[DI_Count];// DI_DO;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				lLengthRece = g_Com.Receive(0, 1, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					if (0x02 == *(hw_uint16*)(g_YTH.RecvData()))
					{
						hw_uint32 KI = *(hw_uint32*)(g_YTH.RecvData() + 2);
						//printd("KI:%d", KI);
						for (int i = 0; i < 7; i++)
						{
							DI_KI[DI_Count][i] = (KI >> i) & 0x01;
						}
						DI_KI[DI_Count][7] = (KI >> 15) & 0x01;  //KI3B
						//printd("DI TEST succeed!--%d\n", DI_Count);
						lFlag = S_MATCH_RECV2;					
					}
				}
				break;
			case S_MATCH_RECV2:
				g_ad.ReadData(0);
				g_ad.ReadData(1);
				g_dio.ReadDIValue(0);
				DI_DDK2_YC[DI_Count] = DIValue[DI0::DDXHY弹动信号2遥测];
				DI_DDK2_JC[DI_Count] = ADValue[1][AD1::DDXH弹动信号2检测];
				DI_DDK11_YC[DI_Count] = ADValue[0][AD0::DDXH弹动信号1_1遥测];
				DI_DDK12_YC[DI_Count] = ADValue[0][AD0::DDXH弹动信号1_2遥测];
				DI_KO7[DI_Count] = ADValue[1][AD1::KO7];

				if (1 == DI_Count)  //重复2次，分别测试全0，全1
				{
					lFlag = S_MATCH_SEND2;
				} 
				else
				{
					lFlag = S_MATCH_SEND0;
				}
				Sleep(500);
				break;
			case S_MATCH_SEND2:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;
		case D_MATCH_ENGINE:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			case S_MATCH_SEND1:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_ENGINE_TEST;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				lLengthRece = g_Com.Receive(0, 1, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					if (0x0B == *(hw_uint16*)(g_YTH.RecvData()))
					{
						printd("Engine TEST succeed!");
						lFlag = S_MATCH_WAIT;
					}
				}
				break;
			default:
				break;
			}
			break;
		case D_MATCH_HELP:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITHUA_TEST_CMD_TEST_END;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_SEND1;
				break;
			case S_MATCH_SEND1:
				*(hw_uint16*)(g_YTH.SendData() + 0) = YITIHUA_TEST_CMD_HELM_TEST;
				g_YTH.MakeFrame(0x36);
				g_Com.Send(0, 1, g_YTH.SendFrame(), g_YTH.SendFrameLength());
				lFlag = S_MATCH_RECV1;
				break;
			case S_MATCH_RECV1:
				lLengthRece = g_Com.Receive(0, 1, g_YTH.RecvBuf());
				g_YTH.AppendData(lLengthRece);
				while (g_YTH.VerifyFrame())
				{
					if (0x0C == *(hw_uint16*)(g_YTH.RecvData()))
					{
						printd("Help_TEST succeed!");
						lFlag = S_MATCH_WAIT;
					}
				}
				break;
			default:
				break;
			}
			break;
		case D_MATCH_JIHUOHAO:
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				HwPower_TurnOn(POWER_AGILENT6702, POWER_FEIKONG_27VA);
				HwPower_SetCurrent(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VA, 4.0, 0);
				HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VA, 27, 0);

				HwPower_SetCurrent(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 4.0, 0);
				HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 60, 0);
				Sleep(2000);
				g_dio.SetFeiKong27VADianYuan(hw_true);
				g_dio.SetFeiKong60VCDianYuan(hw_true);
				Sleep(1000);
				g_ad.ReadData(1);
				if (ADValue[1][AD1::JHH激活好] > 3)
				{
					g_JHH_Value1 = ADValue[1][AD1::JHH激活好];
					g_JHH_ValueA = ADValue[1][AD1::JHH激活好] / 200; //输出电流，激活好内阻为200Ω
					lFlag = S_MATCH_SEND1;
				}
				else
				{
					g_dio.SetFeiKong60VCDianYuan(hw_false);
					g_dio.SetFeiKong27VADianYuan(hw_false);
					HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VA, 0, 0);
					HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 0, 0);
					HwPower_TurnOff(POWER_AGILENT6702, POWER_FEIKONG_27VA);
					lFlag = S_MATCH_WAIT;
					printf("Wrong:The JiHuoHao state is wrong.The Value = %lf \n", ADValue[1][AD1::JHH激活好]);
				}
				break;
			case S_MATCH_SEND1:  //27VA下降阈值
				for (double i=27.0;i>=20.0;i-=0.05)
				{					
					HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VA, i, 0);
					Sleep(10);
					g_ad.ReadData(1);
					if (ADValue[1][AD1::JHH激活好] < -3)
					{
						g_JHH_Value2 = ADValue[1][AD1::JHH激活好];
						g_27VA_Down = i;
						//ShareE_PowerSupply_Set_Voltage(POWER_AGILENT6702_TYPE, CHANNEL_POWER_YITIHUA_27VA, 27, 0);
						break;
						lFlag = S_MATCH_RECV1;
					}
					if (20.0 == i)
					{
						g_dio.SetFeiKong60VCDianYuan(hw_false);
						g_dio.SetFeiKong27VADianYuan(hw_false);
						HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 0, 0);
						HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VA, 0, 0);
						HwPower_TurnOff(POWER_AGILENT6702, POWER_FEIKONG_27VA);
						lFlag = S_MATCH_WAIT;
						printf("Wrong:The JiHuoHao 27VA=20.0V，but the state not change\n");
					}

				}
				break;
			case S_MATCH_RECV1:  //27VA上升阈值
				for (double i = g_27VA_Down-2; i <=27; i += 0.05)
				{
					HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VA, i, 0);
					Sleep(10);
					g_ad.ReadData(1);
					if (ADValue[1][AD1::JHH激活好] > 3)
					{
						g_27VA_up = i;
						HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VA, 27, 0);
						break;
						lFlag = S_MATCH_SEND2;
					}
					if (27 == i)
					{
						g_dio.SetFeiKong60VCDianYuan(hw_false);
						g_dio.SetFeiKong27VADianYuan(hw_false);
						HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 0, 0);
						HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VA, 0, 0);
						HwPower_TurnOff(POWER_AGILENT6702, POWER_FEIKONG_27VA);

						lFlag = S_MATCH_WAIT;
						printf("Wrong:The JiHuoHao 27VA=27.0V，but the state not change\n");
					}

				}
				break;
			case S_MATCH_SEND2:  //27VB下降阈值
				for (double i = 27.0; i >= 20.0; i -= 0.05)
				{
					HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, i, 0);
					Sleep(10);
					g_ad.ReadData(1);
					if (ADValue[1][AD1::JHH激活好] < -3)
					{
						g_27VB_Down = i;
						//ShareE_PowerSupply_Set_Voltage(POWER_AGILENT6702_TYPE, CHANNEL_POWER_YITIHUA_27VA, 27, 0);
						break;
						lFlag = S_MATCH_RECV2;
					}
					if (20.0 == i)
					{
						g_dio.SetFeiKong60VCDianYuan(hw_false);
						g_dio.SetFeiKong27VADianYuan(hw_false);
						HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 0, 0);
						HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VA, 0, 0);
						HwPower_TurnOff(POWER_AGILENT6702, POWER_FEIKONG_27VA);
						lFlag = S_MATCH_WAIT;
						printf("Wrong:The JiHuoHao 27VB=20.0V，but the state not change\n");
					}

				}
				break;
			case S_MATCH_RECV2:  //27VB上升阈值
				for (double i = g_27VB_Down - 2; i <= 27; i += 0.05)
				{
					HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, i, 0);
					Sleep(10);
					g_ad.ReadData(1);
					if (ADValue[1][AD1::JHH激活好] > 3)
					{
						g_27VB_up = i;
						HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VB, 27, 0);
						break;
						lFlag = S_MATCH_SEND3;
					}
					if (27 == i)
					{
						g_dio.SetFeiKong60VCDianYuan(hw_false);
						g_dio.SetFeiKong27VADianYuan(hw_false);
						HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 0, 0);
						HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VA, 0, 0);
						HwPower_TurnOff(POWER_AGILENT6702, POWER_FEIKONG_27VA);
						lFlag = S_MATCH_WAIT;
						printf("Wrong:The JiHuoHao 27VB=27.0V，but the state not change\n");
					}

				}
				break;
			case S_MATCH_SEND3:  //60VC下降阈值
				for (double i = 60.0; i >= 20.0; i -= 0.1)
				{
					HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, i, 0);
					Sleep(10);
					g_ad.ReadData(1);
					if (ADValue[1][AD1::JHH激活好] < -3)
					{
						g_60VC_Down = i;
						//ShareE_PowerSupply_Set_Voltage(POWER_AGILENT6702_TYPE, CHANNEL_POWER_YITIHUA_27VA, 27, 0);
						break;
						lFlag = S_MATCH_RECV3;
					}
					if (20.0 == i)
					{
						g_dio.SetFeiKong60VCDianYuan(hw_false);
						g_dio.SetFeiKong27VADianYuan(hw_false);
						HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 0, 0);
						HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VA, 0, 0);
						HwPower_TurnOff(POWER_AGILENT6702, POWER_FEIKONG_27VA);
						lFlag = S_MATCH_WAIT;
						printf("Wrong:The JiHuoHao 60VC=20.0V，but the state not change\n");
					}

				}
				break;
			case S_MATCH_RECV3:  //60VC上升阈值
				for (double i = g_60VC_Down - 5; i <= 60; i += 0.1)
				{
					HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, i, 0);
					Sleep(10);
					g_ad.ReadData(1);
					if (ADValue[1][AD1::JHH激活好] > 3)
					{
						g_60VC_up = i;
						g_dio.SetFeiKong60VCDianYuan(hw_false);
						g_dio.SetFeiKong27VADianYuan(hw_false);
						HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 0, 0);
						HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VA, 0, 0);
						HwPower_TurnOff(POWER_AGILENT6702, POWER_FEIKONG_27VA);
						lFlag = S_MATCH_WAIT;
						printf("JiHuoHao Test succeed \n");
						break;
					}
					if (27 == i)
					{
						g_dio.SetFeiKong60VCDianYuan(hw_false);
						g_dio.SetFeiKong27VADianYuan(hw_false);
						HwPower_SetVoltage(POWER_AGILENT5749, CHANNEL_POWER_YITIHUA_60VC, 0, 0);
						HwPower_SetVoltage(POWER_AGILENT6702, CHANNEL_POWER_YITIHUA_27VA, 0, 0);
						HwPower_TurnOff(POWER_AGILENT6702, POWER_FEIKONG_27VA);
						lFlag = S_MATCH_WAIT;
						printf("Wrong:The JiHuoHao 27VB=27.0V，but the state not change\n");
					}

				}
				break;
			default:
				break;
			}
			break;
		case D_MATCH_DDK2:
			g_ad.ReadData(1);
			switch (lFlag)
			{
			case S_MATCH_SEND0:
				lFlag = S_MATCH_SEND1;
				g_DDK2_GND = ADValue[1][AD1::DDXH弹动信号2检测];	
				g_dio.SetDOValue(0, DO0::DDXH2弹动信号2_继电器控制, 1);
				Sleep(100);
				break;
			case S_MATCH_SEND1:
				g_DDK2_XuanKong = ADValue[1][AD1::DDXH弹动信号2检测];
				lFlag = S_MATCH_WAIT;
				break;
			default:
				break;
			}
			break;
				break;
		default:
			break;
		}

		/*for (ch=0; ch < MAX_7432_DO_NUM; ch++)
		{
			g_dio.SetDOValue(0, ch, g_ucDO[ch]);
		}
		g_dio.WriteDOValue(0);
		

		for (ch=0; ch < MAX_6216_DA_NUM; ch++)
		{
			g_da.Write(0, ch, g_dDA[ch]);
		}

		g_dio.ReadDIValue(0);
		g_ad.ReadData(0);*/
		if (D_MATCH_POWER_ON == g_ulTestCaseNo)
		{
			g_ad.ReadData(1);
			if (bFlagKO7 && ADValue[1][AD1::KO7] > 3.5)
			{
				bFlagKO7 = hw_false;
			}
		}
		
	}
	return 0;
}


int OnStopTask()
{
	/*
	g_dio.SetFeiKong60VCDianYuan(hw_false);
	g_dio.SetFeiKong27VBDianYuan(hw_false);
	g_dio.SetGuanZu27VBDianYuan(hw_false);
	g_dio.SetFeiKongWenYaDianYuan(hw_false);
	g_dio.WriteDOValue(0);
	*/
	g_Com.Close(0, 1);
	g_Com.Close(0, 0);
	g_Com.Close(0, 2);
	g_Com.Close(0, 3);
	g_Com.Close(1, 0);
	g_da.Stop(0);
	//g_ad.Stop(0);
	g_ad.Stop(1);
	g_dio.Stop(0);
	return 0;
}

int OnUnsetupTask()
{
	WSACleanup();
	return 0;
}


