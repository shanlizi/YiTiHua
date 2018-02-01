#ifndef __PXI2205UTIL_H__
#define __PXI2205UTIL_H__

#include <rtx_ad_2205.h>
#include <hw_osdef.h>

#define PXI2205_BOARD_NUM		2

namespace AD0
{
	enum AD0_CH_DEF : hw_uint8
	{
		FDJDH发动机点火指令1遥测,
		FDJDH发动机点火指令2遥测,
		NLDJS内联动解锁曳光管点火指令1遥测,
		YJKZDH硬件控制点火指令1遥测,
		DDXH弹动信号1_1遥测,
		DDXH弹动信号1_2遥测,
		YJKZDH硬件控制点火指令2遥测,
		GCZ15惯测正15V遥测,
		GCZ5惯测正5V遥测,
		GCF15惯测负15V遥测,
		GCF5惯测负5V遥测,
		RQDNLD燃气舵内联动解锁曳光管点火指令2遥测,
		Z5VYC主5V遥测,
		Z15VYC正15V遥测,
		F15VYC负15V遥测,
		RQDNLD燃气舵内联动解锁曳光管点火1,
		RQDNLD燃气舵内联动解锁曳光管点火1遥测,
		FDJDH发动机点火1,
		FDJDH发动机点火1遥测,
		RQDNLD燃气舵内联动解锁曳光管点火2,
		RQDNLD燃气舵内联动解锁曳光管点火2遥测,
		FDJDH发动机点火2,
		FDJDH发动机点火2遥测,
		DQKZ电气控制正5V1遥测,
		DQKZ电气控制正5V2遥测,
		ECDY二次电源正15V遥测,
		ECDY二次电源负15V遥测,
		ECDY二次电源正5V遥测,
		ECDY二次电源负5V遥测,
		N14舵BIT信号,
		N23舵BIT信号,
		DJCX舵机舱X1向振动,
		DJCY舵机舱Y1向振动,
		DJCZ舵机舱Z1向振动,

		N14舵系统BIT信号_舵系统预留,
		N23舵系统BIT信号_舵系统预留,
		DXT舵系统27VA控制电压遥测_舵系统预留,
		DXT舵系统27VB控制电压遥测_舵系统预留,
		GLDY功率电源1电压遥测_舵系统预留,
		GLDY功率电源2电压遥测_舵系统预留,
		GLDY功率电源3电压遥测_舵系统预留,
		GLDY功率电源4电压遥测_舵系统预留,
		WDCGQ温度传感器_舵系统预留,

		CH43,
		CH44,
		CH45,
		CH46,

		DGLDY大功率电源输出电流监视,
		DY27电源27V0输出电压监视,
		DY27电源27V1输出电压监视,
		DY27电源27V2输出电压监视,
		DY27电源27V3输出电压监视,
		DGLDY大功率电源电压输出电压监视,
		XGLDY小功率60V电源输出电压监视,
		DY电源正15V输出电压监视,
		DY电源负15V电源输出电压监视,
		DY电源5V输出电压监视,
		DY电源5V1输出电压监视,
		DY电源5V2输出电压监视,
		DY电源正15V输出电流监视,
		DY电源负15V输出电流监视,
		DY电源5V输出电流监视,
		DY电源5V1输出电流监视,
		DY电源5V2输出电流监视
	};
}

namespace AD1
{
	enum AD1_CH_DEF : hw_uint8
	{
		DDXH弹动信号2检测,
		KO7,
		DZL舵指令U1遥测,
		DZL舵指令U2遥测,
		DZL舵指令U3遥测,
		DZL舵指令U4遥测,
		DZL舵指令U1,
		DZL舵指令U2,
		DZL舵指令U3,
		DZL舵指令U4,
		N1舵反馈电压,
		N2舵反馈电压,
		N3舵反馈电压,
		N4舵反馈电压,
		FZMN负载模拟铰链力矩信号1,
		FZMN负载模拟铰链力矩信号2,
		FZMN负载模拟铰链力矩信号3,
		FZMN负载模拟铰链力矩信号4,
		N1舵反馈_舵系统备份,
		N2舵反馈_舵系统备份,
		N3舵反馈_舵系统备份,
		N4舵反馈_舵系统备份,
		DXY舵1电机电流_遥测信号,
		DXY舵2电机电流_遥测信号,
		DXY舵3电机电流_遥测信号,
		DXY舵4电机电流_遥测信号,
		DXY舵1电机电流_遥测信号_舵系统备份,
		DXY舵2电机电流_遥测信号_舵系统备份,
		DXY舵3电机电流_遥测信号_舵系统备份,
		DXY舵4电机电流_遥测信号_舵系统备份,
		JHH激活好遥测,
		JHH激活好,
	};
}

typedef struct
{
	hw_uint8	ucPos;
	hw_uint8	ucRefVolRage;
	hw_bool		bDiff;
	hw_bool		bUnipolar;
} PXI_2205_CH_MODE;

#ifndef _AD_CH_COEF_DEFINED
#define _AD_CH_COEF_DEFINED
typedef struct
{	/* y= ax+b */
	double	a;
	double	b;
} AD_CH_COEF;
#endif /* _AD_CH_COEF_DEFINED */


#define AVG_POINT_NUM			1

#define DO_CALIBRATION			0


class CPXI2205Util
{
public:
	CPXI2205Util();
	~CPXI2205Util();

public:
	hw_bool LoadCoefficientFile();

	hw_bool SetChannelMode(hw_uint8 ucBoardNo, hw_uint8 uchChanno, hw_uint8 uchRefVoltage, hw_bool bDiff, hw_bool bUnipolar);
	hw_bool Start(hw_uint8 ucBoardNo);
	hw_bool Stop(hw_uint8 ucBoardNo);
	hw_bool ReadData(hw_uint8 ucBoardNo);

	double GetChannelValue(hw_uint8 ucBoardNo, hw_uint8 ch);

protected:
	hw_uint8			m_ucChannelNum[PXI2205_BOARD_NUM];	/* 设置过工作模式的通道数 */

	PXI_2205_CH_MODE	m_stChMode[PXI2205_BOARD_NUM][MAX_2205_AD_NUM];
	AD_CH_COEF			m_stCoefficient[PXI2205_BOARD_NUM][MAX_2205_AD_NUM];

	double				m_dAvgPointBuf[PXI2205_BOARD_NUM][AVG_POINT_NUM][MAX_2205_AD_NUM];
	hw_uint16			m_usCurAvgBufIdx[PXI2205_BOARD_NUM];	/* m_dAvgPointBuf当前缓冲区索引 */
	hw_uint16			m_usAvgBufDataNum[PXI2205_BOARD_NUM];	/* m_dAvgPointBuf有效数据点数 */
};

#endif /* __PXI2205UTIL_H__ */
