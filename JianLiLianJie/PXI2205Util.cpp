#include "PXI2205Util.h"
#include <HwUtilityRt.h>
#include <HwIniUtil.h>

double ADValue[2][MAX_2205_AD_NUM]={ 0 };

CPXI2205Util::CPXI2205Util()
{
	for (int bd=0; bd < PXI2205_BOARD_NUM; bd++)
	{
		for (int ch = 0; ch < MAX_2205_AD_NUM; ch++)
		{
			m_stChMode[bd][ch].ucPos = 0xFF;
			m_stChMode[bd][ch].ucRefVolRage = 0xFF;
			m_stCoefficient[bd][ch].a = 1;
			m_stCoefficient[bd][ch].b = 0;
		}
	}

//	LoadCoefficientFile();
}


CPXI2205Util::~CPXI2205Util()
{
}

hw_bool CPXI2205Util::LoadCoefficientFile()
{
	// Initialize PXI-2205
	char szCoefFile[512];
	if (CHwUtility::GetEnvironmentVariableEx("JETLAB_CONFIG_DIR", szCoefFile, 512) == 0)
	{
		printf("Missing environment variable \"JETLAB_CONFIG_DIR\"!\n");
		return hw_false;
	}
	strcat(szCoefFile, "\\ADCoef.ini");

	int nCabinetNo = IniUtil_GetProfileInt("Cabinet", "CabinetNo", 0, szCoefFile);
	char szAppName[128], szKeyName[128];

	for (int bd=0; bd < PXI2205_BOARD_NUM; bd++)
	{
		sprintf(szAppName, "AI%i_%i", nCabinetNo, bd);

		for (int ch = 0; ch < MAX_2205_AD_NUM; ch++)
		{
			sprintf(szKeyName, "ch%02ia", ch);
			const char* lpsz = IniUtil_GetProfileString(szAppName, szKeyName, "", szCoefFile);
			m_stCoefficient[bd][ch].a = IniUtil_GetProfileDouble(szAppName, szKeyName, 1, szCoefFile);
			sprintf(szKeyName, "ch%02ib", ch);
			m_stCoefficient[bd][ch].b = IniUtil_GetProfileDouble(szAppName, szKeyName, 0, szCoefFile);

			//printf("Coef[%i][%i].a=%f, Coef[%i][%i].b=%f\n", bd, ch, m_stCoefficient[bd][ch].a, bd, ch, m_stCoefficient[bd][ch].b);
		}
	}

	return hw_true;
}

hw_bool CPXI2205Util::SetChannelMode(hw_uint8 ucBoardNo, hw_uint8 uchChanno, hw_uint8 uchRefVoltage, hw_bool bDiff, hw_bool bUnipolar)
{
	if (ucBoardNo >= PXI2205_BOARD_NUM
		|| uchChanno >= MAX_2205_AD_NUM
		|| (bDiff && uchChanno >= (MAX_2205_AD_NUM / 2))
		)
	{
		return hw_false;
	}

	m_stChMode[ucBoardNo][uchChanno].ucPos			= 0xFF;
	m_stChMode[ucBoardNo][uchChanno].ucRefVolRage	= uchRefVoltage;
	m_stChMode[ucBoardNo][uchChanno].bDiff			= bDiff;
	m_stChMode[ucBoardNo][uchChanno].bUnipolar		= bUnipolar;
	return hw_true;
}

hw_bool CPXI2205Util::Start(hw_uint8 ucBoardNo)
{
	if (ucBoardNo >= PXI2205_BOARD_NUM)
	{
		return hw_false;
	}

	// Initialize PXI-2205
	char szFpgaPath[512];
	if (CHwUtility::GetEnvironmentVariableEx("JETLAB_HOME_DIR", szFpgaPath, 512) == 0)
	{
		printf("Missing environment variable \"JETLAB_HOME_DIR\"!\n");
		return hw_false;
	}
	strcat(szFpgaPath, "\\IntegrativeTest\\rtx\\rtdll\\daq2205_B2.rbf");

	if (!JetLab_PXI2205_Init(ucBoardNo, szFpgaPath))
	{
		printf("JetLab_PXI2205_Init(%i, %s), failed.\n", ucBoardNo, szFpgaPath);
		return hw_false;
	}

	JetLab_PXI2205_ResetAD(ucBoardNo);
	JetLab_PXI2205_CFIFO_Clear(ucBoardNo);

	m_ucChannelNum[ucBoardNo] = 0;
	for (int ch=0; ch < MAX_2205_AD_NUM; ch++)
	{
		if (m_stChMode[ucBoardNo][ch].ucRefVolRage == 0xFF)
		{
			m_stChMode[ucBoardNo][ch].ucPos = 0xFF;
			continue;
		}

		JetLab_PXI2205_CFIFO_Set(ucBoardNo, ch, m_stChMode[ucBoardNo][ch].ucRefVolRage, m_stChMode[ucBoardNo][ch].bDiff, m_stChMode[ucBoardNo][ch].bUnipolar);
		m_stChMode[ucBoardNo][ch].ucPos = m_ucChannelNum[ucBoardNo]++;
	}

	for (int pt=0; pt < AVG_POINT_NUM; pt++)
	{
		for (int ch=0; ch < MAX_2205_AD_NUM; ch++)
		{
			m_dAvgPointBuf[ucBoardNo][pt][ch] = 0.0;
		}
	}
	m_usCurAvgBufIdx[ucBoardNo] = 0;
	m_usAvgBufDataNum[ucBoardNo] = 0;

	JetLab_PXI2205_CFIFO_SetDone(ucBoardNo);

	return hw_true;
}

hw_bool CPXI2205Util::Stop(hw_uint8 ucBoardNo)
{
	JetLab_PXI2205_Close(ucBoardNo);

	for (int ch = 0; ch < MAX_2205_AD_NUM; ch++)
	{
		m_stChMode[ucBoardNo][ch].ucPos = 0xFF;
		m_stChMode[ucBoardNo][ch].ucRefVolRage = 0xFF;
	}

	return hw_true;
}


hw_bool CPXI2205Util::ReadData(hw_uint8 ucBoardNo)
{
	double dTemp;

	if (m_usCurAvgBufIdx[ucBoardNo] >= (AVG_POINT_NUM - 1))
		m_usCurAvgBufIdx[ucBoardNo] = 0;
	else
		m_usCurAvgBufIdx[ucBoardNo]++;

	if (m_usAvgBufDataNum[ucBoardNo]<AVG_POINT_NUM) m_usAvgBufDataNum[ucBoardNo]++;

	if (!JetLab_PXI2205_ReadData(ucBoardNo, m_dAvgPointBuf[ucBoardNo][m_usCurAvgBufIdx[ucBoardNo]], m_ucChannelNum[ucBoardNo])) return hw_false;

	hw_uint8 ucNum = (ucBoardNo == 0) ? 64 : 32;
	for (hw_uint8 ch = 0; ch < ucNum; ch++)
	{
		hw_uint8 ucPos = m_stChMode[ucBoardNo][ch].ucPos;
		if (ucPos == 0xFF) continue;

		dTemp = 0;
		for (hw_uint16 pt = 0; pt < AVG_POINT_NUM; pt++)
		{
			dTemp += m_dAvgPointBuf[ucBoardNo][pt][ch];

		}
		dTemp /=  m_usAvgBufDataNum[ucBoardNo];

#if DO_CALIBRATION
		ADValue[ucBoardNo][ch] =dTemp;
#else
		ADValue[ucBoardNo][ch] = m_stCoefficient[ucBoardNo][ch].a*dTemp + m_stCoefficient[ucBoardNo][ch].b;
#endif

	}

	return hw_true;
}


double CPXI2205Util::GetChannelValue(hw_uint8 ucBoardNo, hw_uint8 ch)
{
	return ADValue[ucBoardNo][ch];
}
