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

	if (m_usFirstFrameOffset != 0xFFFF)	/* m_usFirstFrameOffsetָ��������Ѿ������������ */
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
		if (usHead != YITIHUA_TEST_COMM_HEAD)	/* ֡ͷ��ƥ�� */
		{
			usOffset++;
			continue;		
		}

		ucDataLen = *(hw_uint8*)&m_ucRecvBuf[usOffset + 2];
		if ((m_sRecvBufCount-usOffset) < (5 + ucDataLen))	/* ֡���ݲ����� */
		{	
			break;
		}

		/* У�� */
		usSum2 = *(hw_uint16*)&m_ucRecvBuf[usOffset+3 + ucDataLen];
		//usSum2 = CHwUtility::SwapInt16(usSum2);
		usSum1 = 0;
		for (int i=0; i < (1 + ucDataLen); i++)
		{
			usSum1 += m_ucRecvBuf[usOffset + 2 + i];
		}

		if (usSum1 != usSum2)	/* У��ʧ�� */
		{
			usOffset ++;
			continue; 
		}

		m_usFirstFrameOffset = usOffset;
		return true;
	}

	if (usOffset<m_sRecvBufCount)	/* ������Ч���� */
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

	m_fdWx		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* X1a������� dWx*/
	m_fdWy		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Y1a������� dWy*/
	m_fdWz		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Z1a������� dWz*/

	m_fdVx		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* X1a�ٶ����� dVx*/
	m_fdVy		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Y1a�ٶ����� dVy*/
	m_fdVz		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Z1a�ٶ����� dVz*/

	m_fWx		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* X1a����ٶ� Wx*/
	m_fWy		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Y1a����ٶ� Wy*/
	m_fWz		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Z1a����ٶ� Wz*/

	m_fNx		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* X1a����ٶ� Nx*/
	m_fNy		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Y1a����ٶ� Ny*/
	m_fNz		= *(float*)(RecvData() + nOffset);	nOffset += 4;	/* Z1a����ٶ� Nz*/

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

	/* �Լ�״̬, 1:����, 0:���� */
	hw_uint8 ucStatus = *(hw_uint8*)(RecvData() + nOffset);
	m_ucStatus_XTuoLuo		= ucStatus & 0x01;			/* X���� */
	m_ucStatus_YTuoLuo		= ucStatus>>1 & 0x01;	/* Y���� */
	m_ucStatus_ZTuoLuo		= ucStatus>>2 & 0x01;	/* Z���� */

	m_ucStatus_XJiaBiao		= ucStatus>>3 & 0x01;	/* X�ӱ� */
	m_ucStatus_YJiaBiao		= ucStatus>>4 & 0x01;	/* Y�ӱ� */
	m_ucStatus_ZJiaBiao		= ucStatus>>5 & 0x01;	/* Z�ӱ� */

	m_ucStatus_Temperature	= ucStatus>>6 & 0x01;	/* �����¶ȼ�� */
	m_ucStatus_ParamCheck	= ucStatus>>7 & 0x01;	/* ���ز������ */

	return hw_true;
}
