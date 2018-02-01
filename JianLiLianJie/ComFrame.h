#include <hw_osdef.h>
#include <memory.h>

#pragma once

#define POWERPORT 5025
#define POWER_AGILENT6702			0	/*安捷伦电源*/
#define POWER_AGILENT5749			1	/*安捷伦电源*/
#define POWER_SORENSEN_SGA100X150	2	/*SG大功率电源*/

#define POWER_FEIKONG_27VA				1
#define POWER_FEIKONG_27VB				2
#define POWER_GUANZU_27VB				2
#define POWER_DUOJI_27VA				3
#define POWER_DUOJI_27VB				4


/* 一体化测试设备串口帧格式 */
#define YITIHUA_TEST_COMM_HEAD				0XA355
#define YITIHUA_TEST_COMM_SEND_BUF_SIZE		128
#define YITIHUA_TEST_COMM_RECV_BUF_SIZE		1024

#define YITHUA_TEST_CMD_TEST_END			0XFFFF	/* 停止测试 */
#define YITIHUA_TEST_CMD_EQUIP_CONNECT		0X0000	/* 设备连接 */
#define YITIHUA_TEST_CMD_CPU_TEST			0X0001	/* 处理器测试 */
#define YITIHUA_TEST_CMD_DI_TEST			0x0002	/* 开关量测试 */
#define YITIHUA_TEST_CMD_WATCH_DOG_TEST		0X0003	/* 看门狗测试 */
#define YITIHUA_TEST_CMD_COMM_FK			0X0004	/* 发控串口测试 */
#define YITIHUA_TEST_CMD_COMM_GC			0X0005	/* 惯测串口测试 */
#define YITIHUA_TEST_CMD_COMM_DY			0X0006	/* 导引头串口测试 */
#define YITIHUA_TEST_CMD_COMM_YC			0X0007	/* 遥测串口测试 */
#define YITIHUA_TEST_CMD_IMU_TEST			0X0008	/* 惯测测试 */
#define YITIHUA_TEST_CMD_IMU_SELF_TEST		0X0009	/* ?? */
#define YITIHUA_TEST_CMD_DA_TEST_SINGLE		0X000A	/* 数模转换测试 */
#define YITIHUA_TEST_CMD_ENGINE_TEST		0X000B	/* 点火测试 */
#define YITIHUA_TEST_CMD_HELM_TEST			0X000C	/* 解锁测试 */
#define YITIHUA_TEST_CMD_RAM_FLASH_TEST		0X000D	/* 存储器测试 */

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
		// 处理帧
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
	hw_uint16	m_usFirstFrameOffset;	/* 接收缓冲区内第一个有效帧的首地址 */
};

class CIMUComm : public CYiTiHuaTestComm
{
public:

	hw_bool ParseFrame();

public:
	float		m_fdWx;		/* X1a轴角增量 dWx*/
	float		m_fdWy;		/* Y1a轴角增量 dWy*/
	float		m_fdWz;		/* Z1a轴角增量 dWz*/

	float		m_fdVx;		/* X1a速度增量 dVx*/
	float		m_fdVy;		/* Y1a速度增量 dVy*/
	float		m_fdVz;		/* Z1a速度增量 dVz*/

	float		m_fWx;		/* X1a轴角速度 Wx*/
	float		m_fWy;		/* Y1a轴角速度 Wy*/
	float		m_fWz;		/* Z1a轴角速度 Wz*/

	float		m_fNx;		/* X1a轴加速度 Nx*/
	float		m_fNy;		/* Y1a轴加速度 Ny*/
	float		m_fNz;		/* Z1a轴加速度 Nz*/

	float		m_fTemperature;		/* 惯组内温度 */

	/* 自检状态, 1:正常, 0:故障 */
	hw_uint8	m_ucStatus_XTuoLuo;		/* X陀螺 */
	hw_uint8	m_ucStatus_YTuoLuo;		/* Y陀螺 */
	hw_uint8	m_ucStatus_ZTuoLuo;		/* Z陀螺 */

	hw_uint8	m_ucStatus_XJiaBiao;	/* X加表 */
	hw_uint8	m_ucStatus_YJiaBiao;	/* Y加表 */
	hw_uint8	m_ucStatus_ZJiaBiao;	/* Z加表 */

	hw_uint8	m_ucStatus_Temperature;	/* 工作温度检查 */
	hw_uint8	m_ucStatus_ParamCheck;	/* 加载参数检查 */
};
