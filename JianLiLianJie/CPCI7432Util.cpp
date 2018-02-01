#include "CPCI7432Util.h"


hw_uint8 DOValue[MAX_7432_DO_NUM]={ 0 };
hw_uint8 DIValue[MAX_7432_DI_NUM]={ 0 };

CCPCI7432Util::CCPCI7432Util()
{
	m_ulDOValue = m_ulDIValue = 0;
	m_ulOldDoValue = 0xFFFFFFFF;
}


CCPCI7432Util::~CCPCI7432Util()
{
}

hw_bool CCPCI7432Util::Start(hw_uint8 ucBoardNo)
{
	for (int ch=0; ch < MAX_7432_DI_NUM; ch++)
	{
		DOValue[ch] = 0;
		DIValue[ch] = 0;
	}

	if (!JetLab_CPCI7432_Init(ucBoardNo))
	{
		printf("JetLab_CPCI7432_Init(), failed.\n");
		return hw_false;
	}

	return hw_true;
}

hw_bool CCPCI7432Util::Stop(hw_uint8 ucBoardNo)
{
	for (int ch=0; ch < MAX_7432_DI_NUM; ch++)
	{
		DOValue[ch] = 0;
		DIValue[ch] = 0;
	}
	m_ulDOValue = m_ulDIValue = 0;
	m_ulOldDoValue = 0xFFFFFFFF;

	JetLab_CPCI7432_DO_WriteAllChannel(ucBoardNo, 0);
	JetLab_CPCI7432_Release(ucBoardNo);
	return hw_true;
}

hw_bool CCPCI7432Util::ReadDIValue(hw_uint8 ucBoardNo)
{
	if (JetLab_CPCI7432_DI_ReadAllChannel(ucBoardNo, &m_ulDIValue))
	{
		for (int ch=0; ch < MAX_7432_DI_NUM; ch++)
		{
			DIValue[ch]	= (m_ulDIValue >> ch) & 1;
		}

		return hw_true;
	}

	return hw_false;
}

hw_uint8 CCPCI7432Util::GetDIValue(hw_uint8 ucBoardNo, hw_uint8 ch)
{
	return DIValue[ch];
}

void CCPCI7432Util::SetDOValue(hw_uint8 ucBoardNo, hw_uint8 ch, hw_uint8 ucValue)
{
	if (ucValue)
		m_ulDOValue |= (1 << ch);
	else
		m_ulDOValue &= ~(1 << ch);

#ifdef _DEBUG1
	{
		static hw_uint8 ucOldDO[32] = { 0 };

		if (ucOldDO[ch] != ucValue)
		{
			printf("DO[%i]: %i, %08X\n", ch, ucValue, m_ulDOValue);
		}
		ucOldDO[ch] = ucValue;
	}
#endif
}

void CCPCI7432Util::SetFeiKongWenYaDianYuan(hw_bool bOn)
{
	if (bOn)
	{
		m_ulDOValue |= 1 << DO0::WYDYS��ѹ��Դ������ƿ���K11;
		m_ulDOValue |= 1 << DO0::FKWYD�ɿ���ѹ��Դ����K1;
	}
	else
	{
		if ((m_ulDOValue & (1 << DO0::D15V��15V��Դ����K6)) == 0)
		{
			m_ulDOValue &= ~(1 << DO0::WYDYS��ѹ��Դ������ƿ���K11);
		}
		m_ulDOValue &= ~(1 << DO0::FKWYD�ɿ���ѹ��Դ����K1);
	}
	WriteDOValue(0);
}

void CCPCI7432Util::SetFeiKong60VCDianYuan(hw_bool bOn)
{
	if (bOn)
		m_ulDOValue |= 1 << DO0::FK60VC�ɿ�60VC��Դ����K2;
	else
		m_ulDOValue &= ~(1 << DO0::FK60VC�ɿ�60VC��Դ����K2);
	WriteDOValue(0);
}

void CCPCI7432Util::SetFeiKong27VADianYuan(hw_bool bOn)
{
	if (bOn)
		m_ulDOValue |= 1 << DO0::FK27VA�ɿ�27VA��Դ����K3;
	else
		m_ulDOValue &= ~(1 << DO0::FK27VA�ɿ�27VA��Դ����K3);
	WriteDOValue(0);
}

void CCPCI7432Util::SetFeiKong27VBDianYuan(hw_bool bOn)
{
	if (bOn)
		m_ulDOValue |= 1 << DO0::FK27VB�ɿ�27VB��Դ����K4;
	else
		m_ulDOValue &= ~(1 << DO0::FK27VB�ɿ�27VB��Դ����K4);
	WriteDOValue(0);
}

void CCPCI7432Util::SetGuanZu27VBDianYuan(hw_bool bOn)
{
	if (bOn)
		m_ulDOValue |= 1 << DO0::MGYJZ����Ԫ�����27VB����K5;
	else
		m_ulDOValue &= ~(1 << DO0::MGYJZ����Ԫ�����27VB����K5);
	WriteDOValue(0);
}

void CCPCI7432Util::SetDuo15VDianYuan(hw_bool bOn)
{
	if (bOn)
	{
		m_ulDOValue |= 1 << DO0::WYDYS��ѹ��Դ������ƿ���K11;
		m_ulDOValue |= 1 << DO0::D15V��15V��Դ����K6;
	}
	else
	{
		if ((m_ulDOValue & (1 << DO0::FKWYD�ɿ���ѹ��Դ����K1)) == 0)
		{
			m_ulDOValue &= ~(1 << DO0::WYDYS��ѹ��Դ������ƿ���K11);
		}
		m_ulDOValue &= ~(1 << DO0::D15V��15V��Դ����K6);
	}
	WriteDOValue(0);
}

void CCPCI7432Util::SetDuo27VADianYuan(hw_bool bOn)
{
	if (bOn)
		m_ulDOValue |= 1 << DO0::D27VA��27VA��Դ����K7;
	else
		m_ulDOValue &= ~(1 << DO0::D27VA��27VA��Դ����K7);
	WriteDOValue(0);
}

void CCPCI7432Util::SetDuo27VBDianYuan(hw_bool bOn)
{
	if (bOn)
		m_ulDOValue |= 1 << DO0::D27VB��27VB��Դ����K8;
	else
		m_ulDOValue &= ~(1 << DO0::D27VB��27VB��Դ����K8);
	WriteDOValue(0);
}

void CCPCI7432Util::SetDuo60VCDianYuan(hw_bool bOn)
{
	if (bOn)
		m_ulDOValue |= 1 << DO0::D60VC��60VC��Դ����K10;
	else
		m_ulDOValue &= ~(1 << DO0::D60VC��60VC��Դ����K10);

	WriteDOValue(0);
}

hw_bool CCPCI7432Util::WriteDOValue(hw_uint8 ucBoardNo)
{
	if (m_ulDOValue == m_ulOldDoValue) return hw_true;
	m_ulOldDoValue = m_ulDOValue;

	if (JetLab_CPCI7432_DO_WriteAllChannel(ucBoardNo, m_ulDOValue))
	{
		for (int ch=0; ch < MAX_7432_DO_NUM; ch++)
		{
			DOValue[ch] = (m_ulDOValue >> ch) & 1;
			/*printf("DO_V[%i]: %i\n", ch, DOValue[ch]);*/
		}

		return hw_true;
	}

	return hw_false;
}

void CCPCI7432Util::CloseAllPowerSupply()
{
	for (hw_uint8 ch=DO0::WYDYS��ѹ��Դ������ƿ���K11; ch <= DO0::D60VC��60VC��Դ����K10; ch++)
	{
		SetDOValue(0, ch, 0);
	}
	WriteDOValue(0);
}
