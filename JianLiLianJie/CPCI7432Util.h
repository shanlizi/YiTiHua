#ifndef __CPCI7432UTIL_H__
#define __CPCI7432UTIL_H__

#include <rtx_dio_7432.h>
#include <hw_osdef.h>

namespace DI0
{
	enum DI0_CH_DEF : hw_uint8
	{
		DDXHY弹动信号2遥测,					
		FDJDH发动机点火1DSP指令,				
		FDJDH发动机点火2DSP指令,				
		NLDJS内联动解锁曳光管点火1DSP指令,	
		NLDJS内联动解锁曳光管点火2DSP指令,
		FDJDH发动机点火指令1KO1,				
		FDJDH发动机点火指令2KO2,				
		YJKZDH硬件控制点指令火1KO5,				
		YJKZDH硬件控制点指令火2KO6,
		NLDJS内联动解锁曳光管点火指令1KO3,	
		NLDJS内联动解锁曳光管点火指令2KO4,	

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
		BCSJ补偿数据Flash写控制=0,	

		CH1,
		CH2,

		N14舵系统BIT信号,			
		N23舵系统BIT信号,			
		DYTBIT导引头BIT信号,
		YXBIT引信BIT信号,
		CXFXKZ程序Flash写控制,
		DDXH2弹动信号2_继电器控制,
		CSBH1测试保护1, 
		CSBH2测试保护2,
		CXFXKZ程序Flash写控制_继电器控制,
		BCSJ补偿数据Flash写控制_继电器控制,
		DDXH1弹动信号1_1_继电器控制,
		DDXH2弹动信号1_2_继电器控制,

		CH15,
		CH16,
		CH17,
		CH18,
		CH19,
		CH20,

		WYDYS稳压电源输入控制开关K11,
		FKWYD飞控稳压电源开关K1,
		FK60VC飞控60VC电源开关K2,
		FK27VA飞控27VA电源开关K3,
		FK27VB飞控27VB电源开关K4,
		MGYJZ敏感元件组合27VB开关K5,
		D15V舵15V电源开关K6,
		D27VA舵27VA电源开关K7,
		D27VB舵27VB电源开关K8,
		未使用舵60VC电源开关K9,
		D60VC舵60VC电源开关K10
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


