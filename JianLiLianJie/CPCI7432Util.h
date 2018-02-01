#ifndef __CPCI7432UTIL_H__
#define __CPCI7432UTIL_H__

#include <rtx_dio_7432.h>
#include <hw_osdef.h>

namespace DI0
{
	enum DI0_CH_DEF : hw_uint8
	{
		DDXHY�����ź�2ң��,					
		FDJDH���������1DSPָ��,				
		FDJDH���������2DSPָ��,				
		NLDJS����������ҷ��ܵ��1DSPָ��,	
		NLDJS����������ҷ��ܵ��2DSPָ��,
		FDJDH���������ָ��1KO1,				
		FDJDH���������ָ��2KO2,				
		YJKZDHӲ�����Ƶ�ָ���1KO5,				
		YJKZDHӲ�����Ƶ�ָ���2KO6,
		NLDJS����������ҷ��ܵ��ָ��1KO3,	
		NLDJS����������ҷ��ܵ��ָ��2KO4,	

		CH11,
		CH12,
		CH13,
		CH14,
		CH15,
		CH16,
		CH17,
		CH18,
		CH19,
		CH20,
		CH21,
		CH22,
		CH23,
		CH24,
		CH25,
		CH26,
		CH27,
		CH28,
		CH29,
		CH30,
		CH31
	};
}

namespace DO0
{
	enum DO0_CH_DEF : hw_uint8
	{
		BCSJ��������Flashд����=0,	

		CH1,
		CH2,

		N14��ϵͳBIT�ź�,			
		N23��ϵͳBIT�ź�,			
		DYTBIT����ͷBIT�ź�,
		YXBIT����BIT�ź�,
		CXFXKZ����Flashд����,
		DDXH2�����ź�2_�̵�������,
		CSBH1���Ա���1, 
		CSBH2���Ա���2,
		CXFXKZ����Flashд����_�̵�������,
		BCSJ��������Flashд����_�̵�������,
		DDXH1�����ź�1_1_�̵�������,
		DDXH2�����ź�1_2_�̵�������,

		CH15,
		CH16,
		CH17,
		CH18,
		CH19,
		CH20,

		WYDYS��ѹ��Դ������ƿ���K11,
		FKWYD�ɿ���ѹ��Դ����K1,
		FK60VC�ɿ�60VC��Դ����K2,
		FK27VA�ɿ�27VA��Դ����K3,
		FK27VB�ɿ�27VB��Դ����K4,
		MGYJZ����Ԫ�����27VB����K5,
		D15V��15V��Դ����K6,
		D27VA��27VA��Դ����K7,
		D27VB��27VB��Դ����K8,
		δʹ�ö�60VC��Դ����K9,
		D60VC��60VC��Դ����K10
	};
}


class CCPCI7432Util
{
public:
	CCPCI7432Util();
	~CCPCI7432Util();

public:
	hw_bool Start(hw_uint8 ucBoardNo);
	hw_bool Stop(hw_uint8 ucBoardNo);

	hw_bool ReadDIValue(hw_uint8 ucBoardNo);
	hw_uint8 GetDIValue(hw_uint8 ucBoardNo, hw_uint8 ch);

	void SetDOValue(hw_uint8 ucBoardNo, hw_uint8 ch, hw_uint8 ucValue);
	void SetFeiKongWenYaDianYuan(hw_bool bOn);
	void SetFeiKong60VCDianYuan(hw_bool bOn);
	void SetFeiKong27VADianYuan(hw_bool bOn);
	void SetFeiKong27VBDianYuan(hw_bool bOn);
	void SetGuanZu27VBDianYuan(hw_bool bOn);
	void SetDuo15VDianYuan(hw_bool bOn);
	void SetDuo27VADianYuan(hw_bool bOn);
	void SetDuo27VBDianYuan(hw_bool bOn);
	void SetDuo60VCDianYuan(hw_bool bOn);
	void CloseAllPowerSupply();

	hw_bool WriteDOValue(hw_uint8 ucBoardNo);

protected:
	hw_uint32		m_ulDOValue, m_ulOldDoValue, m_ulDIValue;
};

#endif /* __CPCI7432UTIL_H__ */


