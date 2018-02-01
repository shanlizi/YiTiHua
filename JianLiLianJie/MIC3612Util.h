#ifndef __MIC3612UTIL_H__
#define __MIC3612UTIL_H__

#include <rtx_sio_3612.h>
#include <hw_osdef.h>




class CMIC3612Util
{
public:
	CMIC3612Util();
	~CMIC3612Util();

public:
	hw_bool Open(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo, hw_uint32 ulBaudRate, hw_bool bInterruptMode);
	hw_bool Close(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo);

	hw_int32 Receive(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo, hw_uint8* lpBuf);
	hw_bool Send(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo, const hw_uint8* lpBuf, hw_uint32 ulDataSize);

protected:
};

#endif /* __MIC3612UTIL_H__ */


