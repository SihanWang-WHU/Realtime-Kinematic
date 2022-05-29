#include "ErrorCorrect.h"


using namespace std;

/*************************************************************************
Hopfield 对流层误差改正
输入：RAWDATA中的气象参数，卫星的系统以及Prn号
输出：对流层改正值
**************************************************************************/
double Hopfield(RAWDATA* raw, NAVSYS sys, int prn)
{
	//E为卫星相对于测站的高度角，单位是角度
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

	double H = raw->receiver.blh.h;                                           // H为测站高度

	////对流层底部50km，若高度超限，则返回-1
	//if (pos->blh.h > 50000)
	//	return -1;

	//标准气象元素
	double hw = 11000;
	double H0 = 0;                 // H0为参考面的高度
	double T0 = 15 + 273.16;       // T0为参考面上的干温
	double p0 = 1013.25;           // p0是参考面上的气压
	double RH0 = 0.5;              // RH0是参考面上的相对湿度



	double T = T0 - 0.0065 * (H - H0);                                       // T为测站上的干温
	double p = p0 * pow((1 - 0.0000226 * (H - H0)), 5.225);                  // p为测站上的气压
	double RH = RH0 * exp(-0.0006396 * (H - H0));                            // RH为测站上的相对湿度
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
IF_GPS GPS的消点离层组合观测值
输入：伪距P1和P2
输出：IF组合观测值
**************************************************************************/
double IF_GPS(double P1, double P2)
{
	return FL1_GPS * FL1_GPS / (FL1_GPS * FL1_GPS - FL2_GPS * FL2_GPS) * P1 - FL2_GPS * FL2_GPS / (FL1_GPS * FL1_GPS - FL2_GPS * FL2_GPS) * P2;
}

/*************************************************************************
IF_BDS BDS的消点离层组合观测值
输入：伪距B1和B3
输出：IF组合观测值
**************************************************************************/
double IF_BDS(double B1, double B3)
{
	return   FB1_BDS * FB1_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS) * B1 - FB3_BDS * FB3_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS) * B3;
}

/*************************************************************************
DetectOutlier 粗差探测函数
输入：RAWDATA原始观测数据
**************************************************************************/
void DetectOutlier(RAWDATA* raw)
{
	int i, j, TheNum = 0;
	bool FindFlag;
	double dGF, dMW;
	COMBINE Com_Cur[MAXCHANNUM];

	// 先用memset函数讲现在的组合观测值的结构体赋值为0
	memset(Com_Cur, 0, MAXCHANNUM * sizeof(COMBINE));

	//检查每颗卫星的双频伪距和相位数据是否有效和完整
	//ValidCom是专门为粗差探测设计的观测值组合是否可用的指标
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

		//计算现在的组合观测值的结构体中的MW组合观测值和GF组合观测值
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

		// 判断在下个历元中是否找到了同一个系统同一个PRN的range数据
		// 其中raw->obs.com结构体中储存的是上个历元的数据
		FindFlag = false;
		for (j = 0; j < MAXCHANNUM; j++)
		{
			if (raw->obs.range[i].Prn == raw->obs.com[j].Prn && raw->obs.range[i].Sys == raw->obs.com[j].Sys)
			{
				FindFlag = true;
				break;
			}
		}

		// 在数据都齐全的情况下：
		// 计算当前历元每颗卫星GF与上一历元对应GF的差值
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
			// 将计算出来的这个历元的组合观测值的值赋给raw->obs.com，
            // 接下来循环的时候raw->obs.com就是上一个历元的组合观测值了
			memcpy(raw->obs.com, Com_Cur, MAXCHANNUM * sizeof(COMBINE));
		}
	}
}

