#include "CPCI6216Util.h"

double DAValue[MAX_6216_DA_NUM]={ 0 };

CCPCI6216Util::CCPCI6216Util()
{
}


CCPCI6216Util::~CCPCI6216Util()
{
}


hw_bool CCPCI6216Util::Start(hw_uint8 ucBoardNo)
{
	for (int ch=0; ch < MAX_6216_DA_NUM; ch++)
	{
		DAValue[ch] = 0;
	}

	if (!JetLab_CPCI6216_Init(0))
	{
		printf("JetLab_CPCI6216_Init(), failed.\n");
		return hw_false;
	}

	return hw_true;
}

hw_bool CCPCI6216Util::Stop(hw_uint8 ucBoardNo)
{
	for (int ch=0; ch < MAX_6216_DA_NUM; ch++)
	{
		DAValue[ch] = 0;
		JetLab_CPCI6216_DAOutput(ucBoardNo, ch, 0);
	}

	JetLab_CPCI6216_Release(ucBoardNo);
	return hw_true;
}

hw_bool CCPCI6216Util::Write(hw_uint8 ucBoardNo, hw_uint8 ch, double dValue)
{
	if (JetLab_CPCI6216_DAOutput(ucBoardNo, ch, dValue))
	{
		DAValue[ch]	= dValue;
		return hw_true;
	}

	return hw_false;
}
