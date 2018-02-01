#include <hw_osdef.h>
#include <memory.h>

#pragma once

#define POWERPORT 5025
#define POWER_AGILENT6702			0	/*�����׵�Դ*/
#define POWER_AGILENT5749			1	/*�����׵�Դ*/
#define POWER_SORENSEN_SGA100X150	2	/*SG���ʵ�Դ*/

#define POWER_FEIKONG_27VA				1
#define POWER_FEIKONG_27VB				2
#define POWER_GUANZU_27VB				2
#define POWER_DUOJI_27VA				3
#define POWER_DUOJI_27VB				4


/* һ�廯�����豸����֡��ʽ */
#define YITIHUA_TEST_COMM_HEAD				0XA355
#define YITIHUA_TEST_COMM_SEND_BUF_SIZE		128
#define YITIHUA_TEST_COMM_RECV_BUF_SIZE		1024

#define YITHUA_TEST_CMD_TEST_END			0XFFFF	/* ֹͣ���� */
#define YITIHUA_TEST_CMD_EQUIP_CONNECT		0X0000	/* �豸���� */
#define YITIHUA_TEST_CMD_CPU_TEST			0X0001	/* ���������� */
#define YITIHUA_TEST_CMD_DI_TEST			0x0002	/* ���������� */
#define YITIHUA_TEST_CMD_WATCH_DOG_TEST		0X0003	/* ���Ź����� */
#define YITIHUA_TEST_CMD_COMM_FK			0X0004	/* ���ش��ڲ��� */
#define YITIHUA_TEST_CMD_COMM_GC			0X0005	/* �߲⴮�ڲ��� */
#define YITIHUA_TEST_CMD_COMM_DY			0X0006	/* ����ͷ���ڲ��� */
#define YITIHUA_TEST_CMD_COMM_YC			0X0007	/* ң�⴮�ڲ��� */
#define YITIHUA_TEST_CMD_IMU_TEST			0X0008	/* �߲���� */
#define YITIHUA_TEST_CMD_IMU_SELF_TEST		0X0009	/* ?? */
#define YITIHUA_TEST_CMD_DA_TEST_SINGLE		0X000A	/* ��ģת������ */
#define YITIHUA_TEST_CMD_ENGINE_TEST		0X000B	/* ������ */
#define YITIHUA_TEST_CMD_HELM_TEST			0X000C	/* �������� */
#define YITIHUA_TEST_CMD_RAM_FLASH_TEST		0X000D	/* �洢������ */

/*1. Send
	*(hw_uint16*)(obj.SendData()+0) = 0xFFFF;
	*(hw_uint8*)(obj.SendData()+2) = 0xAA;
	obj.MakeFrame(3);
	rss422.Send(0, 0, obj.SendFrame(), obj.SendFrameLength());

   2. Recv
    hw_int32 lBytes = rss422.Receive(0, 0, obj.RecvBuf());
	obj.AppendData(lBytes);
	while (obj.VerifyFrame())
	{
		// ����֡
		CPU_ID = *(hw_uint32*)(obj.RecvData()+2);
		CPU_Rev_ID = *(hw_uint32*)(obj.RecvData()+6);
	}
*/


//SendBuf[0][64] = {0x55, 0xA3, 0x3B};


class CYiTiHuaTestComm
{
public:
	CYiTiHuaTestComm();
	~CYiTiHuaTestComm();

public:
	const hw_uint8* SendFrame()
	{
		return m_ucSendBuf;
	}
	hw_uint8* SendData()
	{
		return &m_ucSendBuf[3];
	}
	hw_uint16 SendFrameLength()
	{
		return m_sSendBufCount;
	}
	void MakeFrame(hw_uint8 ucDataLength);


	void ClearBuf()
	{
		m_sRecvBufCount = 0;
		//memset(m_ucRecvBuf, 0, YITIHUA_TEST_COMM_RECV_BUF_SIZE);
	}
	hw_uint8* RecvBuf()
	{
		return &m_ucRecvBuf[m_sRecvBufCount];
	}
	const hw_uint8* RecvData()
	{
		if (m_usFirstFrameOffset!=0xFFFF)
			return &m_ucRecvBuf[m_usFirstFrameOffset+3];
		return NULL;
	}
	hw_uint16 RecvBufLength()
	{
		return YITIHUA_TEST_COMM_RECV_BUF_SIZE-m_sRecvBufCount;
	}
	void AppendData(hw_int32 lBytesReceived);
	hw_bool VerifyFrame();

protected:
	hw_uint8	m_ucSendBuf[YITIHUA_TEST_COMM_SEND_BUF_SIZE];
	hw_uint8	m_ucRecvBuf[YITIHUA_TEST_COMM_RECV_BUF_SIZE];
	hw_int16	m_sSendBufCount;
	hw_int16	m_sRecvBufCount;
	hw_uint16	m_usFirstFrameOffset;	/* ���ջ������ڵ�һ����Ч֡���׵�ַ */
};

class CIMUComm : public CYiTiHuaTestComm
{
public:

	hw_bool ParseFrame();

public:
	float		m_fdWx;		/* X1a������� dWx*/
	float		m_fdWy;		/* Y1a������� dWy*/
	float		m_fdWz;		/* Z1a������� dWz*/

	float		m_fdVx;		/* X1a�ٶ����� dVx*/
	float		m_fdVy;		/* Y1a�ٶ����� dVy*/
	float		m_fdVz;		/* Z1a�ٶ����� dVz*/

	float		m_fWx;		/* X1a����ٶ� Wx*/
	float		m_fWy;		/* Y1a����ٶ� Wy*/
	float		m_fWz;		/* Z1a����ٶ� Wz*/

	float		m_fNx;		/* X1a����ٶ� Nx*/
	float		m_fNy;		/* Y1a����ٶ� Ny*/
	float		m_fNz;		/* Z1a����ٶ� Nz*/

	float		m_fTemperature;		/* �������¶� */

	/* �Լ�״̬, 1:����, 0:���� */
	hw_uint8	m_ucStatus_XTuoLuo;		/* X���� */
	hw_uint8	m_ucStatus_YTuoLuo;		/* Y���� */
	hw_uint8	m_ucStatus_ZTuoLuo;		/* Z���� */

	hw_uint8	m_ucStatus_XJiaBiao;	/* X�ӱ� */
	hw_uint8	m_ucStatus_YJiaBiao;	/* Y�ӱ� */
	hw_uint8	m_ucStatus_ZJiaBiao;	/* Z�ӱ� */

	hw_uint8	m_ucStatus_Temperature;	/* �����¶ȼ�� */
	hw_uint8	m_ucStatus_ParamCheck;	/* ���ز������ */
};
