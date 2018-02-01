#ifndef __CPCI6216_H__
#define __CPCI6216_H__
#include <rtx_da_6216.h>
#include <hw_osdef.h>


namespace DA0
{
	enum DA_CH_DEF : hw_uint8
	{
		DZL1��ָ��U1,
		DZL2��ָ��U2,
		DZL3��ָ��U3,
		DZL4��ָ��U4,
		DZL1��ָ��U1_��ϵͳԤ��,
		DZL2��ָ��U2_��ϵͳԤ��,
		DZL3��ָ��U3_��ϵͳԤ��,
		DZL4��ָ��U4_��ϵͳԤ��,
		DFK1�淴��1_�ɿ�Ԥ��,
		DFK2�淴��2_�ɿ�Ԥ��,
		DFK3�淴��3_�ɿ�Ԥ��,
		DFK4�淴��4_�ɿ�Ԥ��,
		DKZL���ָ��_��ת��,
		CH13,
		CH14,
		CH15
	};
}


class CCPCI6216Util
{
public:
	CCPCI6216Util();
	~CCPCI6216Util();

public:
	hw_bool Start(hw_uint8 ucBoardNo);
	hw_bool Stop(hw_uint8 ucBoardNo);

	hw_bool Write(hw_uint8 ucBoardNo, hw_uint8 ch, double dValue);
};


#endif /* __CPCI6216_H__ */
