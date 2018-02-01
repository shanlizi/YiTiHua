#include "ComFrame.h"
#include <memory.h>
#include <rtx_inc.h>



CYiTiHuaTestComm::CYiTiHuaTestComm()
{
	m_sSendBufCount = 0;
	m_sRecvBufCount = 0;
	m_usFirstFrameOffset = 0xFFFF;
}


CYiTiHuaTestComm::~CYiTiHuaTestComm()
{
}

void  CYiTiHuaTestComm::AppendData(hw_int32 lBytesReceived)
{
	if (lBytesReceived>0)
		m_sRecvBufCount += lBytesReceived;
}

hw_bool CYiTiHuaTestComm::VerifyFrame()
{
	hw_uint16	usHead;
	hw_uint8	ucDataLen;
	hw_uint16	usSum1, usSum2;

	if (m_usFirstFrameOffset != 0xFFFF)	/* m_usFirstFrameOffset指向的数据已经处理过，丢弃 */
	{
		usHead = *(hw_uint16*)&m_ucRecvBuf[m_usFirstFrameOffset];
		ucDataLen = *(hw_uint8*)&m_ucRecvBuf[m_usFirstFrameOffset + 2];

		hw_uint16	usNextPos = m_usFirstFrameOffset + 5 + ucDataLen;
		if (usNextPos < m_sRecvBufCount)
		{
			memmove(m_ucRecvBuf, &m_ucRecvBuf[usNextPos], m_sRecvBufCount - usNextPos);
			m_sRecvBufCount -= usNextPos;
		}
		else
		{
			m_sRecvBufCount = 0;
		}
	}

	hw_uint16 usOffset=0; 
	while (usOffset < (m_sRecvBufCount - 1))
	{
		usHead = *(hw_uint16*)&m_ucRecvBuf[usOffset];
		if (usHead != YITIHUA_TEST_COMM_HEAD)	/* 帧头不匹配 */
		{
			usOffset++;
			continue;		
		}

		ucDataLen = *(hw_uint8*)&m_ucRecvBuf[usOffset + 2];
		if ((m_sRecvBufCount-usOffset) < (5 + ucDataLen))	/* 帧数据不完整 */
		{	
			break;
		}

		/* 校验 */
		usSum2 = *(hw_uint16*)&m_ucRecvBuf[usOffset+3 + ucDataLen];
		//usSum2 = CHwUtility::SwapInt16(usSum2);
		usSum1 = 0;
		for (int i=0; i < (1 + ucDataLen); i++)
		{
			usSum1 += m_ucRecvBuf[usOffset + 2 + i];
		}

		if (usSum1 != usSum2)	/* 校验失败 */
		{
			usOffset ++;
			continue; 
		}

		m_usFirstFrameOffset = usOffset;
		return true;
	}

	if (usOffset<m_sRecvBufCount)	/* 丢弃无效数据 */
	{
		if (usOffset > 0)
		{
			memmove(m_ucRecvBuf, &m_ucRecvBuf[usOffset], m_sRecvBufCount - usOffset);
			m_sRecvBufCount -= usOffset;
		}
	}
	else
	{
		m_sRecvBufCount = 0;
	}
	
	m_usFirstFrameOffset = 0xFFFF;
	return hw_false;
}

void  CYiTiHuaTestComm::MakeFrame(hw_uint8 ucDataLength)
{
	*(hw_uint16*)&m_ucSendBuf[0]	= YITIHUA_TEST_COMM_HEAD;
	*(hw_uint8*)&m_ucSendBuf[2]		= ucDataLength;

	hw_uint16	usSum1 = 0;
	for (int i=2; i < (3 + ucDataLength); i++)
	{
		usSum1 += m_ucSendBuf[i];
	}
	//usSum1 = CHwUtility::SwapInt16(usSum1);
	*(hw_uint16*)&m_ucSendBuf[3 + ucDataLength] = usSum1;
	m_sSendBufCount = ucDataLength + 5;
}


/*****************************************************
 * CIMUComm                                          *
 *****************************************************/
struct ST_TEMPERATUR
{
	hw_int32	lTemperature : 24;
	hw_int32	lDmy : 8;
};
hw_bool CIMUComm::ParseFrame()
{
	if (!VerifyFrame()) return hw_false;

	int nOffset = 2;

	m_fdWx		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* X1a轴角增量 dWx*/
	m_fdWy		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Y1a轴角增量 dWy*/
	m_fdWz		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Z1a轴角增量 dWz*/

	m_fdVx		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* X1a速度增量 dVx*/
	m_fdVy		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Y1a速度增量 dVy*/
	m_fdVz		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Z1a速度增量 dVz*/

	m_fWx		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* X1a轴角速度 Wx*/
	m_fWy		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Y1a轴角速度 Wy*/
	m_fWz		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Z1a轴角速度 Wz*/

	m_fNx		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* X1a轴加速度 Nx*/
	m_fNy		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Y1a轴加速度 Ny*/
	m_fNz		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Z1a轴加速度 Nz*/

	ST_TEMPERATUR T;
	T.lTemperature = 0;
#if 1
	hw_int32 lTemp=0;
	memcpy(&lTemp, RecvData() + nOffset, 3);
	T.lTemperature = lTemp&0xFFFFFF;
#else
	T.lTemperature | = *(hw_uint8*)(RecvData() + nOffset+2);
	T.lTemperature = (T.lTemperature << 8) | (*(hw_uint8*)(RecvData() + nOffset) + 1);
	T.lTemperature = (T.lTemperature << 8) | (*(hw_uint8*)(RecvData() + nOffset) + 0);
#endif
	nOffset += 3;

	m_fTemperature = T.lTemperature*0.1;

	/* 自检状态, 1:正常, 0:故障 */
	hw_uint8 ucStatus = *(hw_uint8*)(RecvData() + nOffset);
	m_ucStatus_XTuoLuo		= ucStatus & 0x01;			/* X陀螺 */
	m_ucStatus_YTuoLuo		= ucStatus>>1 & 0x01;	/* Y陀螺 */
	m_ucStatus_ZTuoLuo		= ucStatus>>2 & 0x01;	/* Z陀螺 */

	m_ucStatus_XJiaBiao		= ucStatus>>3 & 0x01;	/* X加表 */
	m_ucStatus_YJiaBiao		= ucStatus>>4 & 0x01;	/* Y加表 */
	m_ucStatus_ZJiaBiao		= ucStatus>>5 & 0x01;	/* Z加表 */

	m_ucStatus_Temperature	= ucStatus>>6 & 0x01;	/* 工作温度检查 */
	m_ucStatus_ParamCheck	= ucStatus>>7 & 0x01;	/* 加载参数检查 */

	return hw_true;
}
