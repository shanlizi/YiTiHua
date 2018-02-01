#include <rtx_can_7841.h>
#include <hw_osdef.h>

#ifndef CPCI7841_BAUDRATE_25K
#define CPCI7841_BAUDRATE_25K			0	/* 25KBps */
#define CPCI7841_BAUDRATE_50K			1	/* 50KBps */
#define CPCI7841_BAUDRATE_100K			2	/* 100KBps */
#define CPCI7841_BAUDRATE_125K			3	/* 125KBps */
#define CPCI7841_BAUDRATE_200K			4	/* 200KBps */
#define CPCI7841_BAUDRATE_250K			5	/* 250KBps */
#define CPCI7841_BAUDRATE_400K			6	/* 400KBps */
#define CPCI7841_BAUDRATE_500K			7	/* 500KBps */
#define CPCI7841_BAUDRATE_800K			8	/* 800KBps */
#define CPCI7841_BAUDRATE_1M			9	/* 1MBps */
#endif /* CPCI7841_BAUDRATE_25K */

class CCPCI7841Util
{
public:
	CCPCI7841Util();
	~CCPCI7841Util();

public:
	hw_bool Open(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo, hw_uint8 ucBaudRate, hw_bool bExtendedMode);
	hw_bool Close(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo);

	hw_int32 Send(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo, hw_uint32 ulCanID, const hw_uint8 *lpData, hw_uint8 ucBytes);
	/* ulCanID, 要接收的canId, 如果为0，接收全部 */
	hw_int32 Receive(hw_uint8 ucBoardNo, hw_uint8 ucChannelNo, hw_uint32 ulCanID, hw_uint8 *lpBuffer, hw_uint32 *lpCanID, hw_uint8 ucBufferSize);

	/* ucLedNo: 0~5 */
	void SetLedStatus(hw_uint8 ucBoardNo, hw_uint8 ucLedNo, hw_bool bOn);

protected:
	hw_uint8 m_ucMode;
};

