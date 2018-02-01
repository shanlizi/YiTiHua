#ifndef __PXI2205UTIL_H__
#define __PXI2205UTIL_H__

#include <rtx_ad_2205.h>
#include <hw_osdef.h>

#define PXI2205_BOARD_NUM		2

namespace AD0
{
	enum AD0_CH_DEF : hw_uint8
	{
		FDJDH���������ָ��1ң��,
		FDJDH���������ָ��2ң��,
		NLDJS����������ҷ��ܵ��ָ��1ң��,
		YJKZDHӲ�����Ƶ��ָ��1ң��,
		DDXH�����ź�1_1ң��,
		DDXH�����ź�1_2ң��,
		YJKZDHӲ�����Ƶ��ָ��2ң��,
		GCZ15�߲���15Vң��,
		GCZ5�߲���5Vң��,
		GCF15�߲⸺15Vң��,
		GCF5�߲⸺5Vң��,
		RQDNLDȼ��������������ҷ��ܵ��ָ��2ң��,
		Z5VYC��5Vң��,
		Z15VYC��15Vң��,
		F15VYC��15Vң��,
		RQDNLDȼ��������������ҷ��ܵ��1,
		RQDNLDȼ��������������ҷ��ܵ��1ң��,
		FDJDH���������1,
		FDJDH���������1ң��,
		RQDNLDȼ��������������ҷ��ܵ��2,
		RQDNLDȼ��������������ҷ��ܵ��2ң��,
		FDJDH���������2,
		FDJDH���������2ң��,
		DQKZ����������5V1ң��,
		DQKZ����������5V2ң��,
		ECDY���ε�Դ��15Vң��,
		ECDY���ε�Դ��15Vң��,
		ECDY���ε�Դ��5Vң��,
		ECDY���ε�Դ��5Vң��,
		N14��BIT�ź�,
		N23��BIT�ź�,
		DJCX�����X1����,
		DJCY�����Y1����,
		DJCZ�����Z1����,

		N14��ϵͳBIT�ź�_��ϵͳԤ��,
		N23��ϵͳBIT�ź�_��ϵͳԤ��,
		DXT��ϵͳ27VA���Ƶ�ѹң��_��ϵͳԤ��,
		DXT��ϵͳ27VB���Ƶ�ѹң��_��ϵͳԤ��,
		GLDY���ʵ�Դ1��ѹң��_��ϵͳԤ��,
		GLDY���ʵ�Դ2��ѹң��_��ϵͳԤ��,
		GLDY���ʵ�Դ3��ѹң��_��ϵͳԤ��,
		GLDY���ʵ�Դ4��ѹң��_��ϵͳԤ��,
		WDCGQ�¶ȴ�����_��ϵͳԤ��,

		CH43,
		CH44,
		CH45,
		CH46,

		DGLDY���ʵ�Դ�����������,
		DY27��Դ27V0�����ѹ����,
		DY27��Դ27V1�����ѹ����,
		DY27��Դ27V2�����ѹ����,
		DY27��Դ27V3�����ѹ����,
		DGLDY���ʵ�Դ��ѹ�����ѹ����,
		XGLDYС����60V��Դ�����ѹ����,
		DY��Դ��15V�����ѹ����,
		DY��Դ��15V��Դ�����ѹ����,
		DY��Դ5V�����ѹ����,
		DY��Դ5V1�����ѹ����,
		DY��Դ5V2�����ѹ����,
		DY��Դ��15V�����������,
		DY��Դ��15V�����������,
		DY��Դ5V�����������,
		DY��Դ5V1�����������,
		DY��Դ5V2�����������
	};
}

namespace AD1
{
	enum AD1_CH_DEF : hw_uint8
	{
		DDXH�����ź�2���,
		KO7,
		DZL��ָ��U1ң��,
		DZL��ָ��U2ң��,
		DZL��ָ��U3ң��,
		DZL��ָ��U4ң��,
		DZL��ָ��U1,
		DZL��ָ��U2,
		DZL��ָ��U3,
		DZL��ָ��U4,
		N1�淴����ѹ,
		N2�淴����ѹ,
		N3�淴����ѹ,
		N4�淴����ѹ,
		FZMN����ģ����������ź�1,
		FZMN����ģ����������ź�2,
		FZMN����ģ����������ź�3,
		FZMN����ģ����������ź�4,
		N1�淴��_��ϵͳ����,
		N2�淴��_��ϵͳ����,
		N3�淴��_��ϵͳ����,
		N4�淴��_��ϵͳ����,
		DXY��1�������_ң���ź�,
		DXY��2�������_ң���ź�,
		DXY��3�������_ң���ź�,
		DXY��4�������_ң���ź�,
		DXY��1�������_ң���ź�_��ϵͳ����,
		DXY��2�������_ң���ź�_��ϵͳ����,
		DXY��3�������_ң���ź�_��ϵͳ����,
		DXY��4�������_ң���ź�_��ϵͳ����,
		JHH�����ң��,
		JHH�����,
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
	hw_uint8			m_ucChannelNum[PXI2205_BOARD_NUM];	/* ���ù�����ģʽ��ͨ���� */

	PXI_2205_CH_MODE	m_stChMode[PXI2205_BOARD_NUM][MAX_2205_AD_NUM];
	AD_CH_COEF			m_stCoefficient[PXI2205_BOARD_NUM][MAX_2205_AD_NUM];

	double				m_dAvgPointBuf[PXI2205_BOARD_NUM][AVG_POINT_NUM][MAX_2205_AD_NUM];
	hw_uint16			m_usCurAvgBufIdx[PXI2205_BOARD_NUM];	/* m_dAvgPointBuf��ǰ���������� */
	hw_uint16			m_usAvgBufDataNum[PXI2205_BOARD_NUM];	/* m_dAvgPointBuf��Ч���ݵ��� */
};

#endif /* __PXI2205UTIL_H__ */
