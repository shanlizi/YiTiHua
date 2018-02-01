#include "CPCI7841Util.h"
#include <memory.h>


CCPCI7841Util::CCPCI7841Util()
{
}


CCPCI7841Util::~CCPCI7841Util()
{
}

hw_bool CCPCI7841Util::Open(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo, hw_uint8 ucBaudRate, hw_bool bExtendedMode)
{
	hw_bool bResult = hw_false;
	hw_int32 accMask = 0x7ff;

	if (bExtendedMode)
	{
		accMask = 0x1fffffff;
		m_ucMode = CPCI7841_MODE_EXTENDED;
	}
	else
	{
		accMask = 0x7ff;
		m_ucMode = CPCI7841_MODE_STANDARD;
	}

#ifdef CAN_BAUDRATE_SELFDEFINE
	bResult = JetLab_CPCI7841_InitGlobal(ucBoardNo, ucChannelNo, ucBaudRate, 0, accMask, m_ucMode, 0, 0, 0, 0, 0);
#else
	bResult = JetLab_CPCI7841_InitGlobal(ucBoardNo, ucChannelNo, ucBaudRate, 0, accMask, m_ucMode);
#endif
	bResult = JetLab_CPCI7841_Open(ucBoardNo, ucChannelNo);

	return bResult;
}

hw_bool CCPCI7841Util::Close(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo)
{
	return JetLab_CPCI7841_Close(ucBoardNo, ucChannelNo);
}

hw_int32 CCPCI7841Util::Send(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo, hw_uint32 ulCanID, const hw_uint8 *lpData, hw_uint8 ucBytes)
{
	CPCI7841_CAN_PACKET t;

	if (ucBytes > 8) ucBytes = 8;

	t.CAN_ID		= ulCanID;
	t.rtr			= 0;
	t.mode			= m_ucMode;
	t.len			= ucBytes;
	memcpy(t.data, lpData, ucBytes);
	t.reserved		= 0;

	if (JetLab_CPCI7841_SendMsg(ucBoardNo, ucChannelNo, &t)) return ucBytes;
	return 0;
}

hw_int32 CCPCI7841Util::Receive(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo, hw_uint32 ulCanID, hw_uint8 *lpBuffer, hw_uint32 *lpCanID, hw_uint8 ucBufferSize)
{
	CPCI7841_CAN_PACKET r;

	if (JetLab_CPCI7841_RcvMsg(ucBoardNo, ucChannelNo, &r, ulCanID))
	{
		memcpy(lpBuffer, r.data, r.len);
		*lpCanID = r.CAN_ID;
		return r.len;
	}
	
	return 0;
}

void CCPCI7841Util::SetLedStatus(hw_uint8 ucBoardNo, hw_uint8 ucLedNo, hw_bool bOn)
{
	JetLab_CPCI7841_SetLedStatus(ucBoardNo, ucLedNo, bOn ? 1 : 0);
}
