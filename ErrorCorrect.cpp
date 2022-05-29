#include "ErrorCorrect.h"


using namespace std;

/*************************************************************************
Hopfield ������������
���룺RAWDATA�е�������������ǵ�ϵͳ�Լ�Prn��
��������������ֵ
**************************************************************************/
double Hopfield(RAWDATA* raw, NAVSYS sys, int prn)
{
	//EΪ��������ڲ�վ�ĸ߶Ƚǣ���λ�ǽǶ�
	double E;

	if (sys == 0)
	{
		calNEU_AE(raw, prn, GPS);
		E = raw->sat_gps[prn - 1].eleAngle * 180 / PI;
	}
	else if (sys == 1)
	{
		calNEU_AE(raw, prn, BDS);
		E = raw->sat_bds[prn - 1].eleAngle * 180 / PI;
	}

	double H = raw->receiver.blh.h;                                           // HΪ��վ�߶�

	////������ײ�50km�����߶ȳ��ޣ��򷵻�-1
	//if (pos->blh.h > 50000)
	//	return -1;

	//��׼����Ԫ��
	double hw = 11000;
	double H0 = 0;                 // H0Ϊ�ο���ĸ߶�
	double T0 = 15 + 273.16;       // T0Ϊ�ο����ϵĸ���
	double p0 = 1013.25;           // p0�ǲο����ϵ���ѹ
	double RH0 = 0.5;              // RH0�ǲο����ϵ����ʪ��



	double T = T0 - 0.0065 * (H - H0);                                       // TΪ��վ�ϵĸ���
	double p = p0 * pow((1 - 0.0000226 * (H - H0)), 5.225);                  // pΪ��վ�ϵ���ѹ
	double RH = RH0 * exp(-0.0006396 * (H - H0));                            // RHΪ��վ�ϵ����ʪ��
	double ee = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
	double hd = 40136 + 148.72 * (T0 - 273.16);
	double Kw = 155.2e-7 * 4810 / T / T * ee * (hw - H);
	double Kd = 155.2e-7 * p / T * (hd - H);

	double angled = (E * E + 6.25);
	double anglew = (E * E + 2.25);

	double angleD = sqrt(angled) * PI / 180;
	double angleW = sqrt(anglew) * PI / 180;

	double deltad = Kd / sin(angleD);
	double deltaw = Kw / sin(angleW);
	double dTrop = deltad + deltaw;

	return dTrop;
}

/*************************************************************************
IF_GPS GPS�����������Ϲ۲�ֵ
���룺α��P1��P2
�����IF��Ϲ۲�ֵ
**************************************************************************/
double IF_GPS(double P1, double P2)
{
	return FL1_GPS * FL1_GPS / (FL1_GPS * FL1_GPS - FL2_GPS * FL2_GPS) * P1 - FL2_GPS * FL2_GPS / (FL1_GPS * FL1_GPS - FL2_GPS * FL2_GPS) * P2;
}

/*************************************************************************
IF_BDS BDS�����������Ϲ۲�ֵ
���룺α��B1��B3
�����IF��Ϲ۲�ֵ
**************************************************************************/
double IF_BDS(double B1, double B3)
{
	return   FB1_BDS * FB1_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS) * B1 - FB3_BDS * FB3_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS) * B3;
}

/*************************************************************************
DetectOutlier �ֲ�̽�⺯��
���룺RAWDATAԭʼ�۲�����
**************************************************************************/
void DetectOutlier(RAWDATA* raw)
{
	int i, j, TheNum = 0;
	bool FindFlag;
	double dGF, dMW;
	COMBINE Com_Cur[MAXCHANNUM];

	// ����memset���������ڵ���Ϲ۲�ֵ�Ľṹ�帳ֵΪ0
	memset(Com_Cur, 0, MAXCHANNUM * sizeof(COMBINE));

	//���ÿ�����ǵ�˫Ƶα�����λ�����Ƿ���Ч������
	//ValidCom��ר��Ϊ�ֲ�̽����ƵĹ۲�ֵ����Ƿ���õ�ָ��
	for (i = 0; i < raw->obs.SATNUMS; i++)
	{
		if (fabs(raw->obs.range[i].L1) < 1e-5 || fabs(raw->obs.range[i].L2) < 1e-5 || fabs(raw->obs.range[i].P1) < 1e-5 ||
			fabs(raw->obs.range[i].P2) < 1e-5)
		{
			raw->obs.range[i].ValidCom = false;
			continue;
		}
		Com_Cur[i].Prn = raw->obs.range[i].Prn;
		Com_Cur[i].Sys = raw->obs.range[i].Sys;
		Com_Cur[i].n = 1;

		//�������ڵ���Ϲ۲�ֵ�Ľṹ���е�MW��Ϲ۲�ֵ��GF��Ϲ۲�ֵ
		if (raw->obs.range[i].Sys == GPS)
		{
			Com_Cur[i].MW = (77 * raw->obs.range[i].P1 - 60 * raw->obs.range[i].P2) / 17.0 - (77 * raw->obs.range[i].L1 * WL1_GPS + 60 * raw->obs.range[i].L2 * WL2_GPS) / 137.0;
			Com_Cur[i].GF = raw->obs.range[i].P1 - raw->obs.range[i].P2;
		}
		else if (raw->obs.range[i].Sys == BDS)
		{
			Com_Cur[i].MW = (FB1_BDS * raw->obs.range[i].P1 - FB3_BDS * raw->obs.range[i].P2) / (FB1_BDS - FB3_BDS) - (FB1_BDS * raw->obs.range[i].L1 * WB1_BDS + FB3_BDS * raw->obs.range[i].L2 * WB3_BDS) / (FB1_BDS + FB3_BDS);
			Com_Cur[i].GF = raw->obs.range[i].P1 - raw->obs.range[i].P2;
		}

		// �ж����¸���Ԫ���Ƿ��ҵ���ͬһ��ϵͳͬһ��PRN��range����
		// ����raw->obs.com�ṹ���д�������ϸ���Ԫ������
		FindFlag = false;
		for (j = 0; j < MAXCHANNUM; j++)
		{
			if (raw->obs.range[i].Prn == raw->obs.com[j].Prn && raw->obs.range[i].Sys == raw->obs.com[j].Sys)
			{
				FindFlag = true;
				break;
			}
		}

		// �����ݶ���ȫ������£�
		// ���㵱ǰ��Ԫÿ������GF����һ��Ԫ��ӦGF�Ĳ�ֵ
		if (FindFlag)
		{
			dGF = Com_Cur[i].GF - raw->obs.com[j].GF;
			dMW = Com_Cur[i].MW - raw->obs.com[j].MW;

			if (fabs(dGF) < 0.2 && fabs(dMW) < 1)
			{
				raw->obs.range[i].ValidCom = true;
				Com_Cur[i].MW = (raw->obs.com[j].n * raw->obs.com[j].MW + Com_Cur[i].MW) / (raw->obs.com[j].n + 1);
				Com_Cur[i].n = raw->obs.com[j].n + 1;
			}
			else
			{
				raw->obs.range[i].ValidCom = false;
			}
			// ����������������Ԫ����Ϲ۲�ֵ��ֵ����raw->obs.com��
            // ������ѭ����ʱ��raw->obs.com������һ����Ԫ����Ϲ۲�ֵ��
			memcpy(raw->obs.com, Com_Cur, MAXCHANNUM * sizeof(COMBINE));
		}
	}
}

