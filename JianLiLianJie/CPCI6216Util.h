#ifndef __CPCI6216_H__
#define __CPCI6216_H__
#include <rtx_da_6216.h>
#include <hw_osdef.h>


namespace DA0
{
	enum DA_CH_DEF : hw_uint8
	{
		DZL1∂Ê÷∏¡ÓU1,
		DZL2∂Ê÷∏¡ÓU2,
		DZL3∂Ê÷∏¡ÓU3,
		DZL4∂Ê÷∏¡ÓU4,
		DZL1∂Ê÷∏¡ÓU1_∂ÊœµÕ≥‘§¡Ù,
		DZL2∂Ê÷∏¡ÓU2_∂ÊœµÕ≥‘§¡Ù,
		DZL3∂Ê÷∏¡ÓU3_∂ÊœµÕ≥‘§¡Ù,
		DZL4∂Ê÷∏¡ÓU4_∂ÊœµÕ≥‘§¡Ù,
		DFK1∂Ê∑¥¿°1_∑…øÿ‘§¡Ù,
		DFK2∂Ê∑¥¿°2_∑…øÿ‘§¡Ù,
		DFK3∂Ê∑¥¿°3_∑…øÿ‘§¡Ù,
		DFK4∂Ê∑¥¿°4_∑…øÿ‘§¡Ù,
		DKZL∂Êøÿ÷∏¡Ó_–˝◊™µØ,
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
