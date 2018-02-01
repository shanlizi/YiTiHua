#include "MIC3612Util.h"



CMIC3612Util::CMIC3612Util()
{
}


CMIC3612Util::~CMIC3612Util()
{
}

hw_bool CMIC3612Util::Open(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo, hw_uint32 ulBaudRate, hw_bool bInterruptMode)
{
	int iArg;

	if (!JetLab_MIC3612_Init(ucBoardNo, ucChannelNo))
	{
		printf("JetLab_MIC3612_Init(), failed.\n");
		return hw_false;
	}

	if (bInterruptMode)
	{
		JetLab_MIC3612_IntConfig(ucBoardNo, ucChannelNo, 10);
		iArg = MIC3612_CHANNELMODE_INT;
		JetLab_MIC3612_Ioctl(ucBoardNo, ucChannelNo, SIOMODE_SET, &iArg);
	}
	else
	{
		iArg = MIC3612_CHANNELMODE_POLL;
		JetLab_MIC3612_Ioctl(ucBoardNo, ucChannelNo, SIOMODE_SET, &iArg);
	}

	DATAFORMAT_ST df;
	df.dataBits = 8;
	df.parityBits = 2;
	df.stopBits = 1;
	JetLab_MIC3612_Ioctl(ucBoardNo, ucChannelNo, DATAFORMAT_SET, &df);

	iArg = ulBaudRate;
	JetLab_MIC3612_Ioctl(ucBoardNo, ucChannelNo, BAUDRATE_SET, &iArg);

	return hw_true;
}

hw_int32 CMIC3612Util::Receive(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo, hw_uint8* lpBuf)
{
	unsigned long ulDataSize=0;

	if (JetLab_MIC3612_Recv(ucBoardNo, ucChannelNo, lpBuf, &ulDataSize) && ulDataSize > 0) return ulDataSize;
	return 0;
}

hw_bool CMIC3612Util::Send(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo, const hw_uint8* lpBuf, hw_uint32 ulDataSize)
{
	return JetLab_MIC3612_Send(ucBoardNo, ucChannelNo, lpBuf, ulDataSize);
}


hw_bool CMIC3612Util::Close(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo)
{
	return JetLab_MIC3612_Exit(ucBoardNo, ucChannelNo);
}
