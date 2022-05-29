#define _CRT_SECURE_NO_WARNINGS
#include"RTK.h"
#include<cmath>

using namespace std;

/*************************************************************************
ʱ��ƥ�� TimeMatching
���룺ԭʼ����RawBase��RawRove
���ã��ƶ�ʱ�䣬����׼վ������վʱ���£
1�ļ���ȡ��ϣ�0�ɽ�����һ������
**************************************************************************/
int TimeMatching(RAWDATA* RawBase, RAWDATA* RawRove, FILE* fpBase, FILE* fpRove)
{
	double BaseTime = RawBase->obs.gpst.Week * 604800.0 + RawBase->obs.gpst.SecofWeek;
	double RoveTime = RawRove->obs.gpst.Week * 604800.0 + RawRove->obs.gpst.SecofWeek;
	int ValBase, ValRove;
	ValBase = 1; ValRove = 1;
	while (abs(BaseTime - RoveTime) > 0.001)// ʱ��������0.001
	{
		if (BaseTime < RoveTime)// Baseʱ����ǰ
		{
			ValBase = DecodeHOEM7(fpBase, RawBase);
		}
		else
		{
			ValRove = DecodeHOEM7(fpRove, RawRove);
		}
		BaseTime = RawBase->obs.gpst.Week * 604800.0 + RawBase->obs.gpst.SecofWeek;
		RoveTime = RawRove->obs.gpst.Week * 604800.0 + RawRove->obs.gpst.SecofWeek;
		if (ValBase == 0 || ValRove == 0)// �ļ�����
		{
			return 1;
		}
	}
	return 0;
}


/*************************************************************************
ԭʼ���������ʱ��ѡȡ RawGroupDataSelect
���룺ԭʼ��������RawBaseGroup��RawBaseGroup����׼����������ԭʼ����RawBase��RawRove
���ã���ʱ����ֵ��Χ�ڽ���ƥ��ѡȡ
0����ɹ���-1����ʧ��
**************************************************************************/
int RawGroupDataSelect(RAWDATA* RawBaseGroup, RAWDATA* RawRoveGroup, RAWDATA* RawBase, RAWDATA* RawRove,
	GPSTIME* Time, int ValBase, int ValRove)
{
	double GPST = Time->Week * 604800.0 + Time->SecofWeek;// �Ѿ������GPST
	// ���ݴ洢������
	if (ValBase == 43)// ��ȡ���ǹ۲�ֵ
	{
		RawBaseGroup[0] = RawBaseGroup[1];// ����ǰ�ƽ� 1-4��ֵ��0-3
		RawBaseGroup[1] = RawBaseGroup[2];
		RawBaseGroup[2] = RawBaseGroup[3];
		RawBaseGroup[3] = RawBaseGroup[4];
		memcpy(RawBaseGroup + 4, &RawBase->obs, sizeof(EPOCHOBSDATA));// �½����ݼ�������
	}
	if (ValRove == 43)
	{
		RawRoveGroup[0] = RawRoveGroup[1];
		RawRoveGroup[1] = RawRoveGroup[2];
		RawRoveGroup[2] = RawRoveGroup[3];
		RawRoveGroup[3] = RawRoveGroup[4];
		memcpy(RawRoveGroup + 4, &RawRove->obs, sizeof(EPOCHOBSDATA));
	}
	for (int i = 4; i >= 0; i--)
	{
		for (int j = 4; j >= 0; j--)
		{
			double TimeBase = RawBaseGroup[i].obs.gpst.Week * 604800.0 + RawBaseGroup[i].obs.gpst.SecofWeek;
			double TimeRove = RawRoveGroup[j].obs.gpst.Week * 604800.0 + RawRoveGroup[j].obs.gpst.SecofWeek;
			double differ = TimeBase - TimeRove;
			if (abs(differ) < 0.001 && TimeRove > GPST && GPST > 0)// ʱ�������
			{
				memcpy(&RawBase->obs, RawBaseGroup + i, sizeof(EPOCHOBSDATA));
				memcpy(&RawRove->obs, RawRoveGroup + j, sizeof(EPOCHOBSDATA));
				memcpy(Time, &RawBaseGroup[i].obs.gpst, sizeof(GPSTIME));// ȷ������ʱ��
				return 0;// ʱ���׼�ɹ�
			}
		}
	}
	return -1;// ʱ���׼ʧ��
}

/*************************************************************************
SelectComSats ���й������ǵ�ѡȡ
���룺����վͬ�����rawdata
�����������Ĺ������ǵ�
**************************************************************************/
void SelectComSats(RAWDATA* rawB, RAWDATA* rawR)
{
	rawR->rtk.numBDS = 0;
	rawR->rtk.numGPS = 0;
	int indexB, indexR;
	// ���SPP����ɹ�����ʼ�������ǵ�ѡȡ
	for (int i = 0; i < MAXBDSPRN; i++)
	{
		rawB->sat_bds[i - 1].comobs = false;
		rawR->sat_bds[i - 1].comobs = false;
		// �����Validͬʱ���������ǵ����������������ǵ�λ�ñ��ɹ���������������ǵĸ߶ȽǺ���ϸ�
		if (rawB->sat_bds[i - 1].Valid == true && rawR->sat_bds[i - 1].Valid == true)
		{
			// i����prn��
			indexR = FindSatObsIndex(i, BDS, &rawR->obs);
			indexB = FindSatObsIndex(i, BDS, &rawB->obs);
			// �����Valid���������,�۲�ֵ���Ʋ��Ҵֲ�̽��ϸ�
			if (rawB->obs.range[indexB].Valid == true && rawB->obs.range[indexB].ValidCom == true)
			{
				if (rawR->obs.range[indexR].Valid == true && rawR->obs.range[indexR].ValidCom == true)
				{
					rawR->rtk.numBDS++;
					rawB->sat_bds[i - 1].comobs = true;
					rawR->sat_bds[i - 1].comobs = true;
				}
			}
		}
	}

	for (int i = 0; i < MAXGPSPRN; i++)
	{
		rawB->sat_gps[i - 1].comobs = false;
		rawR->sat_gps[i - 1].comobs = false;
		// �����Validͬʱ���������ǵ����������������ǵ�λ�ñ��ɹ���������������ǵĸ߶ȽǺ���ϸ�
		if (rawB->sat_gps[i - 1].Valid == true && rawR->sat_gps[i - 1].Valid == true)
		{
			indexR = FindSatObsIndex(i, GPS, &rawR->obs);
			indexB = FindSatObsIndex(i, GPS, &rawB->obs);
			if (rawB->obs.range[indexB].Valid == true && rawB->obs.range[indexB].ValidCom == true
				&& rawR->obs.range[indexR].Valid == true && rawR->obs.range[indexR].ValidCom == true)
			{
				rawR->rtk.numGPS++;
				rawB->sat_gps[i - 1].comobs = true;
				rawR->sat_gps[i - 1].comobs = true;
			}
		}
	}
}


/*************************************************************************
SelectRefSats ����GPS��BDSϵͳ�������ǵ�ѡȡ
���룺����ѡȡ�깲������֮���rawdata�� sys�ж�����ϵͳ���ͣ�1�Ǳ���ϵͳ��0��GPSϵͳ
�����ѡȡ�Ĳο�����
**************************************************************************/
int SelectRefSats(RAWDATA* rawR, int sys)
{

	// �ο��ǵ�ѡȡԭ���ǣ��߶Ƚ����������Ǳ���ϵͳ������GEO����
	if (sys == 0)
	{
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			rawR->sat_gps[i - 1].RefSat = false;
		}
		double eleAngle = 0;
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			if (rawR->sat_gps[i - 1].comobs == true)
			{
				if (rawR->sat_gps[i - 1].eleAngle > eleAngle)
				{
					eleAngle = rawR->sat_gps[i - 1].eleAngle;
				}
			}
		}
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			if (rawR->sat_gps[i - 1].eleAngle == eleAngle)
			{
				// ��������ǵĸ߶ȽǾ������ģ���Ref�ж�ֵ����Ϊtrue����return�����ǵ�Prn��
				rawR->sat_gps[i - 1].RefSat = true;
				return rawR->sat_gps[i - 1].PRN;
			}
		}
	}

	if (sys == 1)
	{
		for (int i = 0; i < MAXBDSPRN; i++)
		{
			rawR->sat_bds[i - 1].RefSat = false;
		}
		double eleAngle = 0;
		for (int i = 0; i < MAXBDSPRN; i++)
		{
			if (rawR->sat_bds[i - 1].comobs == true && 5 < rawR->sat_bds[i - 1].PRN && rawR->sat_bds[i - 1].PRN < 55)
			{
				if (rawR->sat_bds[i - 1].eleAngle > eleAngle)
				{
					eleAngle = rawR->sat_bds[i - 1].eleAngle;
				}
			}
		}
		for (int i = 0; i < MAXBDSPRN; i++)
		{
			if (rawR->sat_bds[i - 1].eleAngle == eleAngle)
			{
				rawR->sat_bds[i - 1].RefSat = true;
				return rawR->sat_bds[i - 1].PRN;
			}
		}
	}

}

/*************************************************************************
calculatesigma2 ��ȡ���ǵķǲ�����
���룺���ǵĸ߶Ƚ�eleangle(�����ƣ�
���������ķǲ�����
**************************************************************************/
double calculatesigma2(double eleangle)
{
	double sigma2;
	sigma2 = pow (0.001, 2) * (1 + 1.5 * cos(eleangle) * cos(eleangle));
	//sigma2 = 16 + 9 / pow(sin(eleangle), 2);
	return sigma2;
}

/*************************************************************************
fill_A_L_DOUBLE_DIFF ��ȡ˫Ƶ˫��̻��߶�̬RTK�ĺ���ģ�͵�B��L����
���룺����ѡȡ�깲������֮���rawdata�� �ο��ǵ�PRN�ţ� B�����L��������ݱ�������
	  �������ǵ�����numGPS��numBDS�� RTK�����ģʽstate(0,1,2)
�����B�����L�������������������н��иı�
**************************************************************************/
void fill_A_L_DOUBLE_DIFF(int prnG, int prnB, int numGPS, int numBDS, RAWDATA* rawB, 
	RAWDATA* rawR, double* B, double* L, int state)
{
	// ��׼վ�Ĳο�����
	double refx = -2267804.5263;
	double refy = 5009342.3723;
	double refz = 3220991.8632;
	XYZ refxyz;
	refxyz.x = refx;
	refxyz.y = refy;
	refxyz.z = refz;
	double p_k_B, p_j_B, p_k_A, p_j_A;// ���ؾ�
	/**************************��ǰ��Ԫ��GPSϵͳ�ĸ�������**************************/
	// �ø÷��������֮��ȷ����ÿ���������ǵ������£������˫����������ݶ��Ƕ�Ӧ�ĵ�/˫��
	// ������ĵ���/˫�������rawR����
	// ���ѭ����������һ��GPS���������ǲ�����������
	for (int i = 0; i < MAXGPSPRN; i++)
	{
		// �����ж�����ǹ������ǣ�������
		if (rawR->sat_gps[i - 1].comobs == true)
		{
			// �����ҳ���������range�����е�����
			// Ϊ�˷�ֹ��rawB��rawR��������������һ��������������������int�͵�����
			int indexB, indexR;
			indexB = FindSatObsIndex(i, GPS, &rawB->obs);
			indexR = FindSatObsIndex(i, GPS, &rawR->obs);
			// ����������
			// �����ǹ������Ƕ����ƶ�վ����λ�۲�ֵ��ȥ���ڻ�׼վ����λ�۲�ֵ
			// fai_d1��fai_d2����������PRN-1
			rawR->receiver.Gfai_d1_f1[i - 1] = rawR->obs.range[indexR].L1 - rawB->obs.range[indexB].L1;
			rawR->receiver.Gfai_d1_f2[i - 1] = rawR->obs.range[indexR].L2 - rawB->obs.range[indexB].L2;
			rawR->receiver.Gp_d1_f1[i - 1] = rawR->obs.range[indexR].P1 - rawB->obs.range[indexB].P1;
			rawR->receiver.Gp_d1_f2[i - 1] = rawR->obs.range[indexR].P2 - rawB->obs.range[indexB].P2;
		}
	}

	// ���ѭ����������һ��GPS���������ǲ�������˫��
	for (int i = 0; i < MAXGPSPRN; i++)
	{
		if (rawR->sat_gps[i - 1].comobs == true)
		{
			double dis, disref;
			// �ο��Ǻ������������������վ�ľ���
			dis = distance_calculate(rawR->sat_gps[i - 1].satXYZ, rawR->receiver.xyz);
			disref = distance_calculate(rawR->sat_gps[prnG - 1].satXYZ, rawR->receiver.xyz);
			// ֻ�в��ǻ�׼�ǵĲ�����Ҫ��˫��
			if (rawR->sat_gps[i - 1].RefSat == false)
			{
				// �����������PRN -1 
				rawR->receiver.Gfai_d2_f1[i - 1] = rawR->receiver.Gfai_d1_f1[i - 1] - rawR->receiver.Gfai_d1_f1[prnG - 1];
				rawR->receiver.Gfai_d2_f2[i - 1] = rawR->receiver.Gfai_d1_f2[i - 1] - rawR->receiver.Gfai_d1_f2[prnG - 1];
				rawR->receiver.Gp_d2_f1[i - 1] = rawR->receiver.Gp_d1_f1[i - 1] - rawR->receiver.Gp_d1_f1[prnG - 1];
				rawR->receiver.Gp_d2_f2[i - 1] = rawR->receiver.Gp_d1_f2[i - 1] - rawR->receiver.Gp_d1_f2[prnG - 1];
				rawR->receiver.Gax[i - 1] = -(rawR->sat_gps[i - 1].satXYZ.x - rawR->receiver.xyz.x) / dis
					+ (rawR->sat_gps[prnG - 1].satXYZ.x - rawR->receiver.xyz.x) / disref;
				rawR->receiver.Gay[i - 1] = -(rawR->sat_gps[i - 1].satXYZ.y - rawR->receiver.xyz.y) / dis
					+ (rawR->sat_gps[prnG - 1].satXYZ.y - rawR->receiver.xyz.y) / disref;
				rawR->receiver.Gaz[i - 1] = -(rawR->sat_gps[i - 1].satXYZ.z - rawR->receiver.xyz.z) / dis
					+ (rawR->sat_gps[prnG - 1].satXYZ.z - rawR->receiver.xyz.z) / disref;
			}
		}
	}


	/**************************��ǰ��Ԫ��BDSϵͳ�ĸ�������**************************/
	for (int i = 0; i < MAXBDSPRN; i++)
	{
		// �����ж�����ǹ������ǣ�������
		if (rawR->sat_bds[i - 1].comobs == true)
		{
			// �����ҳ���������range�����е�����
			// Ϊ�˷�ֹ��rawB��rawR��������������һ��������������������int�͵�����
			int indexB, indexR;
			indexB = FindSatObsIndex(i, BDS, &rawB->obs);
			indexR = FindSatObsIndex(i, BDS, &rawR->obs);
			// ����������
			// �����ǹ������Ƕ����ƶ�վ����λ�۲�ֵ��ȥ���ڻ�׼վ����λ�۲�ֵ
			// fai_d1��fai_d2����������PRN-1
			rawR->receiver.Bfai_d1_f1[i - 1] = rawR->obs.range[indexR].L1 - rawB->obs.range[indexB].L1;
			rawR->receiver.Bp_d1_f1[i - 1] = rawR->obs.range[indexR].P1 - rawB->obs.range[indexB].P1;
		}
	}

	// ���ѭ����������һ��BDS���������ǲ�������˫��
	for (int i = 0; i < MAXBDSPRN; i++)
	{
		if (rawR->sat_bds[i - 1].comobs == true)
		{
			double dis, disref;
			// �ο��Ǻ������������������վ�ľ���
			dis = distance_calculate(rawR->sat_bds[i - 1].satXYZ, rawR->receiver.xyz);
			disref = distance_calculate(rawR->sat_bds[prnB - 1].satXYZ, rawR->receiver.xyz);
			// ֻ�в��ǻ�׼�ǵĲ�����Ҫ��˫��
			if (rawR->sat_bds[i - 1].RefSat == false)
			{
				// �����������PRN -1 
				rawR->receiver.Bfai_d2_f1[i - 1] = rawR->receiver.Bfai_d1_f1[i - 1] - rawR->receiver.Bfai_d1_f1[prnB - 1];
				rawR->receiver.Bp_d2_f1[i - 1] = rawR->receiver.Bp_d1_f1[i - 1] - rawR->receiver.Bp_d1_f1[prnB - 1];
				rawR->receiver.Bax[i - 1] = -(rawR->sat_bds[i - 1].satXYZ.x - rawR->receiver.xyz.x) / dis
					+ (rawR->sat_bds[prnB - 1].satXYZ.x - rawR->receiver.xyz.x) / disref;
				rawR->receiver.Bay[i - 1] = -(rawR->sat_bds[i - 1].satXYZ.y - rawR->receiver.xyz.y) / dis
					+ (rawR->sat_bds[prnB - 1].satXYZ.y - rawR->receiver.xyz.y) / disref;
				rawR->receiver.Baz[i - 1] = -(rawR->sat_bds[i - 1].satXYZ.z - rawR->receiver.xyz.z) / dis
					+ (rawR->sat_bds[prnB - 1].satXYZ.z - rawR->receiver.xyz.z) / disref;
			}
		}
	}

	/**************************��ʼ���B��L����**************************/
	double Gp_RefSat_Rove, Gp_Sat_Rove, Gp_RefSat_Base, Gp_Sat_Base;// ���ؾ�
	double Bp_RefSat_Rove, Bp_Sat_Rove, Bp_RefSat_Base, Bp_Sat_Base;// ���ؾ�
	if (state == 0) // ˫ϵͳ����
	{
		int prnbaseG, prnbaseB;
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			if (rawB->sat_gps[i - 1].RefSat == true)
			{
				prnbaseG = i;
			}
		}
		for (int i = 0; i < MAXBDSPRN; i++)
		{
			if (rawB->sat_bds[i - 1].RefSat == true)
			{
				prnbaseB = i;
			}
		}
		/**************************L��������**************************/
		int indexGL = 0;
		int indexBL = 0; // ��0��ʼ����L����

		// �����GPS����
		// �����L�����˳���ǣ�
		// GPS: F1Ƶ�ʵ���λ˫�F2Ƶ�ʵ���λ˫� F1Ƶ�ʵ�α��˫�F2Ƶ�ʵ�α��˫��
		// BDS: B1IƵ�ʵ���λ˫�B3IƵ�ʵ���λ˫� B1IƵ�ʵ�α��˫��
		
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			Gp_RefSat_Rove = distance_calculate(rawR->sat_gps[prnG - 1].satXYZ, rawR->receiver.xyz);
			Gp_RefSat_Base = distance_calculate(rawB->sat_gps[prnbaseG - 1].satXYZ, refxyz);
			// ѡȡ���������в��ǲο����ǵģ���ʱi=prn
			if (rawR->sat_gps[i - 1].comobs == true && rawR->sat_gps[i - 1].RefSat == false)
			{
				Gp_Sat_Rove = distance_calculate(rawR->sat_gps[i - 1].satXYZ, rawR->receiver.xyz);
				Gp_Sat_Base = distance_calculate(rawB->sat_gps[i - 1].satXYZ, refxyz);
				double dif_dis = -Gp_Sat_Rove + Gp_RefSat_Rove + Gp_Sat_Base - Gp_RefSat_Base;
				// ÿһ�����ӵĶ���numsat - 1 ���� ��Ϊû�вο���
				L[indexGL] = rawR->receiver.Gfai_d2_f1[i - 1] + dif_dis;
				L[indexGL + numGPS - 1] = rawR->receiver.Gfai_d2_f2[i - 1] + dif_dis;
				L[indexGL + 2 * numGPS - 2] = rawR->receiver.Gp_d2_f1[i - 1] + dif_dis;
				L[indexGL + 3 * numGPS - 3] = rawR->receiver.Gp_d2_f2[i - 1] + dif_dis;
				indexGL++;
			}
		}
		// �����BDS����
		for (int i = 0; i < MAXBDSPRN; i++)
		{
			Bp_RefSat_Rove = distance_calculate(rawR->sat_bds[prnB - 1].satXYZ, rawR->receiver.xyz);
			Bp_RefSat_Base = distance_calculate(rawB->sat_bds[prnbaseB - 1].satXYZ, refxyz);
			// ѡȡ���������в��ǲο����ǵģ���ʱi=prn
			if (rawR->sat_bds[i - 1].comobs == true && rawR->sat_bds[i - 1].RefSat == false)
			{
				Bp_Sat_Rove = distance_calculate(rawR->sat_bds[i - 1].satXYZ, rawR->receiver.xyz);
				Bp_Sat_Base = distance_calculate(rawB->sat_bds[i - 1].satXYZ, refxyz);
				double dif_dis = -Bp_Sat_Rove + Bp_RefSat_Rove + Bp_Sat_Base - Bp_RefSat_Base;
				// ÿһ�����ӵĶ���numsat - 1 ���� ��Ϊû�вο���
				L[4 * numGPS - 4 + indexBL] = rawR->receiver.Bfai_d2_f1[i - 1] + dif_dis;
				L[4 * numGPS - 4 + indexBL + numBDS - 1] = rawR->receiver.Bp_d2_f1[i - 1] + dif_dis;
				indexBL++;
			}
		}

		/**************************B��������**************************/
		// ����ȷ��B�����������������
		// B����������ǣ�4nG - 4 + 2nB - 2
		// B����������ǣ�3 + 2nG - 2 + nB - 1 ( 2nG + nB )
		int rowB = 4 * numGPS + 2 * numBDS - 2;
		int columnB = numBDS + 2 * numGPS;
		int indexGB = 0;
		int indexBB = 0;
		// ��ʼ���GPS����
		// B���������������˳���L������һ����
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			// ���ѭ����ȡ�Ĵ����� numGPS -1
			// indexGB ÿ��һ�� �ʹ����ȡ����һ������
			if (rawR->sat_gps[i - 1].comobs == true && rawR->sat_gps[i - 1].RefSat == false)
			{
				// ��һ������F1����λ�Ĳ���
				B[indexGB * columnB] = rawR->receiver.Gax[i - 1];
				B[indexGB * columnB + 1] = rawR->receiver.Gay[i - 1];
				B[indexGB * columnB + 2] = rawR->receiver.Gaz[i - 1];
				B[3 + indexGB * columnB + indexGB] = WL1_GPS;

				// ��һ������F2����λ�Ĳ���
				B[(numGPS - 1) * columnB + indexGB * columnB] = rawR->receiver.Gax[i - 1];
				B[(numGPS - 1) * columnB + indexGB * columnB + 1] = rawR->receiver.Gay[i - 1];
				B[(numGPS - 1) * columnB + indexGB * columnB + 2] = rawR->receiver.Gaz[i - 1];
				B[(numGPS - 1) * columnB + indexGB * columnB + 3 + (numGPS - 1) + indexGB] = WL2_GPS;

				// ��һ������F1��α�ಿ��
				// α�ಿ�ֺ���û��Wave length ��������
				B[2 * (numGPS - 1) * columnB + indexGB * columnB] = rawR->receiver.Gax[i - 1];
				B[2 * (numGPS - 1) * columnB + indexGB * columnB + 1] = rawR->receiver.Gay[i - 1];
				B[2 * (numGPS - 1) * columnB + indexGB * columnB + 2] = rawR->receiver.Gaz[i - 1];

				// ��һ������F2��α�ಿ��
				B[3 * (numGPS - 1) * columnB + indexGB * columnB] = rawR->receiver.Gax[i - 1];
				B[3 * (numGPS - 1) * columnB + indexGB * columnB + 1] = rawR->receiver.Gay[i - 1];
				B[3 * (numGPS - 1) * columnB + indexGB * columnB + 2] = rawR->receiver.Gaz[i - 1];

				// �������һ�Ŷ�ȡ������
				indexGB++;
			}
		}

		for (int i = 0; i < MAXBDSPRN; i++)
		{
			if (rawR->sat_bds[i - 1].comobs == true && rawR->sat_bds[i - 1].RefSat == false)
			{
				// �ò�����F1����λ����
				B[4 * (numGPS - 1) * columnB + indexBB * columnB] = rawR->receiver.Bax[i - 1];
				B[4 * (numGPS - 1) * columnB + indexBB * columnB + 1] = rawR->receiver.Bay[i - 1];
				B[4 * (numGPS - 1) * columnB + indexBB * columnB + 2] = rawR->receiver.Baz[i - 1];
				B[4 * (numGPS - 1) * columnB + indexBB * columnB + 3 + 2 * (numGPS - 1) + indexBB] = WB1_BDS;

				// �ⲿ����F1��α�ಿ��
				B[(4 * (numGPS - 1) + (numBDS - 1)) * columnB + indexBB * columnB] = rawR->receiver.Bax[i - 1];
				B[(4 * (numGPS - 1) + (numBDS - 1)) * columnB + indexBB * columnB + 1] = rawR->receiver.Bay[i - 1];
				B[(4 * (numGPS - 1) + (numBDS - 1)) * columnB + indexBB * columnB + 2] = rawR->receiver.Baz[i - 1];

				// �������һ�Ŷ�ȡ������
				indexBB++;

			}
		}
	}
	// GPS��ϵͳ����
	else if (state == 1)
	{
	int prnbaseG;
	for (int i = 0; i < MAXGPSPRN; i++)
	{
		if (rawB->sat_gps[i - 1].RefSat == true)
		{
			prnbaseG = i;
		}
	}

		/**************************L��������**************************/
		int indexGL = 0;

		double L1_AB_jk, L2_AB_jk, P1_AB_jk, P2_AB_jk;
		int PRN_B_k, PRN_B_j, PRN_A_k, PRN_A_j;
		// �����GPS����
		// �����L�����˳���ǣ�
		// GPS: F1Ƶ�ʵ���λ˫�F2Ƶ�ʵ���λ˫� F1Ƶ�ʵ�α��˫�F2Ƶ�ʵ�α��˫��
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			p_j_B = distance_calculate(rawR->sat_gps[prnG - 1].satXYZ, rawR->receiver.xyz);
			p_j_A = distance_calculate(rawB->sat_gps[prnbaseG - 1].satXYZ, refxyz);
			Gp_RefSat_Rove = distance_calculate(rawR->sat_gps[prnG - 1].satXYZ, rawR->receiver.xyz);
			Gp_RefSat_Base = distance_calculate(rawB->sat_gps[prnbaseG - 1].satXYZ, refxyz);
			// ѡȡ���������в��ǲο����ǵģ���ʱi=prn
			if (rawR->sat_gps[i - 1].comobs == true && rawR->sat_gps[i - 1].RefSat == false)
			{
				p_k_B = distance_calculate(rawR->sat_gps[i - 1].satXYZ, rawR->receiver.xyz);
				p_k_A = distance_calculate(rawB->sat_gps[i - 1].satXYZ, refxyz);
				Gp_Sat_Rove = distance_calculate(rawR->sat_gps[i - 1].satXYZ, rawR->receiver.xyz);
				Gp_Sat_Base = distance_calculate(rawB->sat_gps[i - 1].satXYZ, refxyz);
				double dif_dis = -Gp_Sat_Rove + Gp_RefSat_Rove + Gp_Sat_Base - Gp_RefSat_Base;
				//// ÿһ�����ӵĶ���numsat - 1 ���� ��Ϊû�вο���

				L[indexGL] =  rawR->receiver.Gfai_d2_f1[i - 1] + dif_dis;
				L[indexGL + numGPS - 1] = rawR->receiver.Gfai_d2_f2[i - 1] + dif_dis;
				L[indexGL + 2 * numGPS - 2] = rawR->receiver.Gp_d2_f1[i - 1] + dif_dis;
				L[indexGL + 3 * numGPS - 3] = rawR->receiver.Gp_d2_f2[i - 1] + dif_dis;
				indexGL++; 
			}
		}


		/**************************B��������**************************/
		// ����ȷ��B�����������������
		// B����������ǣ�4nG - 4 + 4nB - 4
		// B����������ǣ�3 + 2nG - 2 + 2nB - 2 ( 2nG + 2nB - 1 )

		int indexGB = 0;
		// ��ʼ���GPS����
		// B���������������˳���L������һ����
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			// ���ѭ����ȡ�Ĵ����� numGPS -1
			// indexGB ÿ��һ�� �ʹ����ȡ����һ������
			if (rawR->sat_gps[i - 1].comobs == true && rawR->sat_gps[i - 1].RefSat == false)
			{
				// ��һ������F1����λ�Ĳ���
				B[indexGB * (2 * numGPS + 1)] = rawR->receiver.Gax[i - 1];
				B[indexGB * (2 * numGPS + 1) + 1] = rawR->receiver.Gay[i - 1];
				B[indexGB * (2 * numGPS + 1) + 2] = rawR->receiver.Gaz[i - 1];
				B[3 + indexGB * (2 * numGPS + 1) + indexGB] = WL1_GPS;

				// ��һ������F2����λ�Ĳ���
				B[(numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1)] = rawR->receiver.Gax[i - 1];
				B[(numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 1] = rawR->receiver.Gay[i - 1];
				B[(numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 2] = rawR->receiver.Gaz[i - 1];
				B[(numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 3 + (numGPS - 1) + indexGB] = WL2_GPS;

				// ��һ������F1��α�ಿ��
				// α�ಿ�ֺ���û��Wave length ��������
				B[2 * (numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1)] = rawR->receiver.Gax[i - 1];
				B[2 * (numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 1] = rawR->receiver.Gay[i - 1];
				B[2 * (numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 2] = rawR->receiver.Gaz[i - 1];

				// ��һ������F2��α�ಿ��
				B[3 * (numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1)] = rawR->receiver.Gax[i - 1];
				B[3 * (numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 1] = rawR->receiver.Gay[i - 1];
				B[3 * (numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 2] = rawR->receiver.Gaz[i - 1];

				// �������һ�Ŷ�ȡ������
				indexGB++;
			}
		}

	}
	// BDS��ϵͳ����
	else if (state == 2)
	{

	     int prnbaseB;
	     for (int i = 0; i < MAXBDSPRN; i++)
	     {
		     if (rawB->sat_bds[i - 1].RefSat == true)
		     {
				 prnbaseB = i;
			 }
		 }
		/**************************L��������**************************/
		int indexBL = 0; // ��0��ʼ����L����

		// �����L�����˳���ǣ�
		// BDS: B1IƵ�ʵ���λ˫�B1IƵ�ʵ�α��˫��
		// ���BDS����
		for (int i = 0; i < MAXBDSPRN; i++)
		{
			Bp_RefSat_Rove = distance_calculate(rawR->sat_bds[prnB - 1].satXYZ, rawR->receiver.xyz);
			Bp_RefSat_Base = distance_calculate(rawB->sat_bds[prnbaseB - 1].satXYZ, refxyz);
			// ѡȡ���������в��ǲο����ǵģ���ʱi=prn
			if (rawR->sat_bds[i - 1].comobs == true && rawR->sat_bds[i - 1].RefSat == false)
			{
				Bp_Sat_Rove = distance_calculate(rawR->sat_bds[i - 1].satXYZ, rawR->receiver.xyz);
				Bp_Sat_Base = distance_calculate(rawB->sat_bds[i - 1].satXYZ, refxyz);
				double dif_dis = -Bp_Sat_Rove + Bp_RefSat_Rove + Bp_Sat_Base - Bp_RefSat_Base;
				// ÿһ�����ӵĶ���numsat - 1 ���� ��Ϊû�вο���
				L[indexBL] = rawR->receiver.Bfai_d2_f1[i - 1] + dif_dis;
				L[indexBL + numBDS - 1] = rawR->receiver.Bp_d2_f1[i - 1] + dif_dis;
				indexBL++;
			}
		}

		/**************************B��������**************************/
		// ����ȷ��B�����������������
		// B����������ǣ�2 * numBDS - 2 
		// B����������ǣ�3 + nB - 1 ( 2nG + 2nB - 1 )
		int columnB = 3 + numBDS - 1;
		int indexBB = 0;
		// ��ʼ���GPS����
		// B���������������˳���L������һ����

		for (int i = 0; i < MAXBDSPRN; i++)
		{
			if (rawR->sat_bds[i - 1].comobs == true && rawR->sat_bds[i - 1].RefSat == false)
			{
				// �ò�����F1����λ����
				B[indexBB * columnB] = rawR->receiver.Bax[i - 1];
				B[indexBB * columnB + 1] = rawR->receiver.Bay[i - 1];
				B[indexBB * columnB + 2] = rawR->receiver.Baz[i - 1];
				B[indexBB * columnB + 3 + indexBB] = WB1_BDS;

				// �ⲿ����F1��α�ಿ��
				B[(numBDS - 1) * columnB + indexBB * columnB] = rawR->receiver.Bax[i - 1];
				B[(numBDS - 1) * columnB + indexBB * columnB + 1] = rawR->receiver.Bay[i - 1];
				B[(numBDS - 1) * columnB + indexBB * columnB + 2] = rawR->receiver.Baz[i - 1];

				// �������һ�Ŷ�ȡ������
				indexBB++;
			}
		}
	}
}

/*************************************************************************
fill_COV_DOUBLE_DIFF ��ȡ˫Ƶ˫��̻��߶�̬RTK�ĺ���ģ�͵ķ�����
���룺����ѡȡ�깲������֮���rawdata�� �ο��ǵ�PRN�ţ� �����������ݱ�������COV
	  �������ǵ�����numGPS��numBDS�� RTK�����ģʽstate(0,1,2)
��������������������������н��иı�
**************************************************************************/
void fill_COV_DOUBLE_DIFF(int prnG, int prnB, int numGPS, int numBDS, RAWDATA* rawB, 
	RAWDATA* rawR, double* COV, int state)
{
	/**************************��ǰ��Ԫ�ڵĵ���������ݵļ�����洢**************************/
	// ��������֮ǰ���ȼ����վ��ĵ�����������sigma2��������Rove����ı����վ���ݵĽṹ����
	// ���ȼ���GPS������
	for (int i = 0; i < MAXGPSPRN; i++)
	{
		// ����ѡȡ���еĹ������ǣ�������׼��
		if (rawR->sat_gps[i - 1].comobs == true)
		{
			double sigma2B, sigma2R;
			sigma2B = calculatesigma2(rawB->sat_gps[i - 1].eleAngle);
			sigma2R = calculatesigma2(rawR->sat_gps[i - 1].eleAngle);
			rawR->receiver.Gfai_d1_f1_sigma2[i - 1] = (sigma2B + sigma2R) ;
			rawR->receiver.Gfai_d1_f2_sigma2[i - 1] = (sigma2B + sigma2R) * COVL2B_L2 / COVL2B;
			rawR->receiver.Gp_d1_f1_sigma2[i - 1] = (sigma2B + sigma2R) * COVL2B ;
			rawR->receiver.Gp_d1_f2_sigma2[i - 1] = (sigma2B + sigma2R) * COVL2B_L2;
		}
	}

	// ��μ���BDS������
	for (int i = 0; i < MAXBDSPRN; i++)
	{
		// ����ѡȡ���еĹ������ǣ�������׼��
		if (rawR->sat_bds[i - 1].comobs == true)
		{
			double sigma2B, sigma2R;
			sigma2B = calculatesigma2(rawB->sat_bds[i - 1].eleAngle);
			sigma2R = calculatesigma2(rawR->sat_bds[i - 1].eleAngle);
			rawR->receiver.Bfai_d1_f1_sigma2[i - 1] = (sigma2B + sigma2R) ;
			rawR->receiver.Bfai_d1_f2_sigma2[i - 1] = (sigma2B + sigma2R) * COVL2B_B3I/ COVL2B_B1I;
			rawR->receiver.Bp_d1_f1_sigma2[i - 1] = (sigma2B + sigma2R) * COVL2B_B1I;
			rawR->receiver.Bp_d1_f2_sigma2[i - 1] = (sigma2B + sigma2R) * COVL2B_B3I;
		}
	}

	/**************************��ʼ���P����**************************/
	// P�����ά���� (4nG + 2nB - 6) * (4nG + 2nB - 6) 
	// ����˫ϵͳ��P����
	if (state == 0)
	{
		int indexGP = 0;
		int indexBP = 0;
		// �������GPS�Ĳ���
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			int column = 2 * numBDS + 4 * numGPS - 6;
			// һ��ѭ�� NG - 1 ��
			if (rawR->sat_gps[i - 1].comobs == true && rawR->sat_gps[i - 1].RefSat == false)
			{

				// ���GL1�ĵ���ֵ
				// ���Ƚ�һ�����ȫ�����Ϊ��׼�ǵĵ���
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[indexGP * column + j] = rawR->receiver.Gfai_d1_f1_sigma2[prnG - 1];
				}
				COV[indexGP * column + indexGP] = rawR->receiver.Gfai_d1_f1_sigma2[prnG - 1] + 1  * rawR->receiver.Gfai_d1_f1_sigma2[i - 1];

				// ���GL2�ĵ���ֵ
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[(numGPS - 1 + indexGP) * column + (numGPS - 1) + j] = rawR->receiver.Gfai_d1_f2_sigma2[prnG - 1];
				}
				COV[(numGPS - 1 + indexGP) * column + (numGPS - 1 + indexGP)] = rawR->receiver.Gfai_d1_f2_sigma2[prnG - 1] + 1  * rawR->receiver.Gfai_d1_f2_sigma2[i - 1];

				// ���GP1�ĵ���ֵ
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[(2 * numGPS - 2 + indexGP) * column + 2 * (numGPS - 1) + j] = rawR->receiver.Gp_d1_f1_sigma2[prnG - 1];
				}
				COV[(2 * numGPS - 2 + indexGP) * column + (2 * numGPS - 2 + indexGP)] = rawR->receiver.Gp_d1_f1_sigma2[prnG - 1] + 1  * rawR->receiver.Gp_d1_f1_sigma2[i - 1];

				// ���GP2�ĵ���ֵ
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[(3 * numGPS - 3 + indexGP) * column + 3 * (numGPS - 1) + j] = rawR->receiver.Gp_d1_f2_sigma2[prnG - 1];
				}
				COV[(3 * numGPS - 3 + indexGP) * column + (3 * numGPS - 3 + indexGP)] = rawR->receiver.Gp_d1_f2_sigma2[prnG - 1] + 1  * rawR->receiver.Gp_d1_f2_sigma2[i - 1];

				indexGP++;
			}
		}

		// ������BDS�Ĳ���
		for (int i = 0; i < MAXBDSPRN; i++)
		{
			int column = 2 * numBDS + 4 * numGPS - 6;
			if (rawR->sat_bds[i - 1].comobs == true && rawR->sat_bds[i - 1].RefSat == false)
			{
				// ���BL1�ĵ���ֵ
				for (int j = 0; j < numBDS - 1; j++)
				{
					COV[(4 * numGPS - 4 + indexBP) * column + 4 * (numGPS - 1) + j] = rawR->receiver.Bfai_d1_f1_sigma2[prnB - 1];
				}
				COV[(4 * numGPS - 4 + indexBP) * column + 4 * (numGPS - 1) + indexBP] = rawR->receiver.Bfai_d1_f1_sigma2[prnB - 1] + rawR->receiver.Bfai_d1_f1_sigma2[i - 1];

				// ���BP1�ĵ���ֵ
				for (int j = 0; j < numBDS - 1; j++)
				{
					COV[(4 * numGPS - 4 + numBDS - 1 + indexBP) * column + 4 * (numGPS - 1) + (numBDS - 1) + j] = rawR->receiver.Bp_d1_f1_sigma2[prnB - 1];
				}
				COV[(4 * numGPS - 4 + numBDS - 1 + indexBP) * column + 4 * (numGPS - 1) + (numBDS - 1) + indexBP] = rawR->receiver.Bp_d1_f1_sigma2[prnB - 1] + rawR->receiver.Bp_d1_f1_sigma2[i - 1];

				indexBP++;

			}
		}
	}
	// ���㵥GPSϵͳ��P����
	else if (state == 1)
	{
		int indexGP = 0;
		// �������GPS�Ĳ���
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			int column = 4 * numGPS - 4;
			// һ��ѭ�� NG - 1 ��
			if (rawR->sat_gps[i - 1].comobs == true && rawR->sat_gps[i - 1].RefSat == false)
			{

				// ���GL1�ĵ���ֵ
				// ���Ƚ�һ�����ȫ�����Ϊ��׼�ǵĵ���
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[indexGP * column + j] = rawR->receiver.Gfai_d1_f1_sigma2[prnG - 1];
				}
				COV[indexGP * column + indexGP] = rawR->receiver.Gfai_d1_f1_sigma2[prnG - 1] + rawR->receiver.Gfai_d1_f1_sigma2[i - 1];

				// ���GL2�ĵ���ֵ
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[(numGPS - 1 + indexGP) * column + (numGPS - 1) + j] = rawR->receiver.Gfai_d1_f2_sigma2[prnG - 1];
				}
				COV[(numGPS - 1 + indexGP) * column + (numGPS - 1 + indexGP)] = rawR->receiver.Gfai_d1_f2_sigma2[prnG - 1] + rawR->receiver.Gfai_d1_f2_sigma2[i - 1];

				// ���GP1�ĵ���ֵ
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[(2 * numGPS - 2 + indexGP) * column + 2 * (numGPS - 1) + j] = rawR->receiver.Gp_d1_f1_sigma2[prnG - 1];
				}
				COV[(2 * numGPS - 2 + indexGP) * column + (2 * numGPS - 2 + indexGP)] = rawR->receiver.Gp_d1_f1_sigma2[prnG - 1] + rawR->receiver.Gp_d1_f1_sigma2[i - 1];

				// ���GP2�ĵ���ֵ
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[(3 * numGPS - 3 + indexGP) * column + 3 * (numGPS - 1) + j] = rawR->receiver.Gp_d1_f2_sigma2[prnG - 1];
				}
				COV[(3 * numGPS - 3 + indexGP) * column + (3 * numGPS - 3 + indexGP)] = rawR->receiver.Gp_d1_f2_sigma2[prnG - 1] +  rawR->receiver.Gp_d1_f2_sigma2[i - 1];

				indexGP++;
			}
		}
	}
	// ���㵥BDSϵͳ��P����
	else if (state == 2)
	{
		int indexBP = 0;
		// ���BDS�Ĳ���
		for (int i = 0; i < MAXBDSPRN; i++)
		{
			int column = 2 * numBDS - 2;
			// һ��ѭ�� NG - 1 ��
			if (rawR->sat_bds[i - 1].comobs == true && rawR->sat_bds[i - 1].RefSat == false)
			{

				// ���GL1�ĵ���ֵ
				// ���Ƚ�һ�����ȫ�����Ϊ��׼�ǵĵ���
				for (int j = 0; j < numBDS - 1; j++)
				{
					COV[indexBP * column + j] = rawR->receiver.Bfai_d1_f1_sigma2[prnB - 1];
				}
				COV[indexBP * column + indexBP] = rawR->receiver.Bfai_d1_f1_sigma2[prnB - 1] + rawR->receiver.Bfai_d1_f1_sigma2[i - 1];

				// ���GP1�ĵ���ֵ
				for (int j = 0; j < numBDS - 1; j++)
				{
					COV[(numBDS - 1 + indexBP) * column + (numBDS - 1) + j] = rawR->receiver.Bp_d1_f1_sigma2[prnB - 1];
				}
				COV[(numBDS - 1 + indexBP) * column + (numBDS - 1 + indexBP)] = rawR->receiver.Bp_d1_f1_sigma2[prnB - 1] + rawR->receiver.Bp_d1_f1_sigma2[i - 1];

				indexBP++;
			}
		}
	}
}


/*************************************************************************
SingleEpochRTK ��Զ�λ�Ĳ���������
���룺��׼վ���ƶ�վ��RAWDATA����
���㣺һ����Ԫ��RTK��Զ�λ����
**************************************************************************/
int SingleEpochRTK(RAWDATA* rawB, RAWDATA* rawR)
{
	double refx = -2267804.5263;
	double refy = 5009342.3723;
	double refz = 3220991.8632;
	XYZ refxyz;
	refxyz.x = refx;
	refxyz.y = refy;
	refxyz.z = refz;

	rawR->rtk.gpst = rawR->gpst;
	/******************************SPP��������ǵ�ѡȡ�ʹ���******************************/
	SelectComSats(rawB, rawR);

	// ѡȡ�ο���
	int prnB, prnG;
	prnG = SelectRefSats(rawR, 0);
	prnB = SelectRefSats(rawR, 1);
	SelectRefSats(rawB, 0);
	SelectRefSats(rawB, 1);
	//cout << "PrnB=" << prnB << endl;
	//cout << "PrnG=" << prnG << endl;


	// ���㹲�����ǵ���Ŀ
	int numGPS = 0;
	int numBDS = 0;
	int RefGPS = 0;
	int RefBDS = 0;
	for (int i = 0; i < MAXBDSPRN; i++)
	{
		if (rawR->sat_bds[i - 1].comobs == true)
		{
			numBDS++;
		}
	}
	for (int i = 0; i < MAXGPSPRN; i++)
	{
		if (rawR->sat_gps[i - 1].comobs == true)
		{
			numGPS++;
		}
	}
	int numSat = numBDS + numGPS;
	rawR->rtk.Satnums = numSat;
	rawR->rtk.numGPS = numGPS;
	rawR->rtk.numBDS = numBDS;
	int state;  // RTK���������
	if (numSat > 5 && numGPS > 1 && numBDS > 1)
	{
		state = 0;
	}
	else if (numGPS > 4)
	{
		state = 1;
	}
	else if (numBDS > 4)
	{
		state = 2;
	}
	else
	{
		return -1;
	}

	// �����׼�ǵĸ���
	for (int i = 0; i < MAXBDSPRN; i++)
	{
		if (rawR->sat_bds[i - 1].RefSat == true)
		{
			RefBDS++;
		}
	}
	for (int i = 0; i < MAXGPSPRN; i++)
	{
		if (rawR->sat_gps[i - 1].RefSat == true)
		{
			RefGPS++;
		}
	}
	if (state == 0)
	{
		state++;
	}
	// ����׼����������1����֤����׼������ѡȡ����
	if (RefGPS > 1 || RefBDS > 1)
	{
		cout << "��׼����������1�� ����" << endl;
		system("pause");
	}


	/*******************************��������ά�ȵĶ���******************************/
	int rowB, columnB, rowL, dimensionP, rowV;
	int dimensionN;                       // ģ����ά��
	if (state == 0)
	{
		rowB = 4 * numGPS + 2 * numBDS - 6;
		columnB = 2 * numGPS + numBDS;
		rowL = 4 * numGPS + 2 * numBDS - 6;
		rowV = rowB;
		dimensionN = columnB - 3;
		dimensionP = 4 * numGPS + 2 * numBDS - 6;
	}
	else if (state == 1)
	{
		rowB = 4 * numGPS - 4;
		columnB = 2 * numGPS + 1;
		rowL = 4 * numGPS - 4;
		rowV = rowB;
		dimensionN = columnB - 3;
		dimensionP = 4 * numGPS - 4;
	}
	else if (state == 2)
	{
		rowB = 2 * numBDS - 2;
		columnB =  numBDS + 2;
		rowL = 2 * numBDS - 2;
		rowV = rowB;
		dimensionN = columnB - 3;
		dimensionP = 2 * numBDS - 2;
	}

	rawR->rtk.N_n = dimensionN;
	// ��SPP��ԭʼ�������һ�£�
	rawR->receiver.xyz_save = rawR->receiver.xyz;

	/*******************************������������Ķ�����ڴ����******************************/
	// L����Ӧ���ǣ�4 * nB + 4 * nG - 8 ) * 1 �ľ��� ��������Ϊ���븳ֵһ������������ôд
	// X����Ӧ���� ��2 * nB + 2 * nG - 1 ) * 1�ľ��� 
	// ���ƿ�֪ B����Ӧ���� ��4 * nB + 4 * nG - 8 ) * ��2 * nB + 2 * nG - 1 ) �ľ���
	// P�����ά���ǣ�2 * nB + 2 * nG - 1 ) * ��2 * nB + 2 * nG - 1 )
	// COV_X ��NBB���� * sigma2 (columnB * columnB)
	// x_DATA �ǳ�ֵ l = L - Bx��columnB * 1��
	// V = Bx - l, V������(rowB * 1)

	static double B_DATA[4 * MAXBDSPRN * 8 * MAXBDSPRN] = { 0 };
	double L_DATA[8 * MAXBDSPRN] = { 0 };
	double V_DATA[8 * MAXBDSPRN] = { 0 };
	static double P_DATA[8 * MAXBDSPRN * 8 * MAXBDSPRN] = { 0 };
	static double COV_DATA[8 * MAXBDSPRN * 8 * MAXBDSPRN] = { 0 };
	static double COV_X_DATA[4 * MAXBDSPRN * 4 * MAXBDSPRN] = { 0 };
	double x_DATA[2 * MAXBDSPRN + 2 * MAXGPSPRN] = { 0 };
	double l_DATA[4 * MAXBDSPRN + 4 * MAXGPSPRN] = { 0 };
	double Bx_DATA[4 * MAXBDSPRN + 4 * MAXGPSPRN] = { 0 };
	static double BTrans_DATA[4 * MAXBDSPRN * 8 * MAXBDSPRN] = { 0 };
	static double BTPB_DATA[4 * MAXBDSPRN * 4 * MAXBDSPRN] = { 0 };
	static double BTP_DATA[2 * MAXBDSPRN * 8 * MAXBDSPRN] = { 0 };
	static double BTPB_INV_DATA[2 * MAXBDSPRN * 8 * MAXBDSPRN] = { 0 };
	double BTPl_DATA[2 * MAXBDSPRN + 2 * MAXGPSPRN] = { 0 };
	double v_DATA[2 * MAXBDSPRN + 2 * MAXGPSPRN] = { 0 };
	double Bv_DATA[8 * MAXBDSPRN] = { 0 };
	double VT_DATA[8 * MAXBDSPRN] = { 0 };
	static double VTP_DATA[8 * MAXBDSPRN * 8 * MAXBDSPRN] = { 0 };
	double VTPV_DATA[1] = { 0 };
	static double Qa_DATA[4 * MAXBDSPRN * 4 * MAXBDSPRN] = { 0 };
	double Qb_DATA[9] = { 0 };
	double Qba_DATA[3 * 4 * MAXGPSPRN] = {0};
	double* P_data = new double[dimensionP * dimensionP];
	double* BTPBINV_data = new double[columnB * columnB];
	memset(B_DATA, 0.0, 8 * MAXBDSPRN * 8 * MAXBDSPRN * sizeof(double));
	memset(L_DATA, 0.0, 8 * MAXBDSPRN * sizeof(double));
	memset(V_DATA, 0.0, 8 * MAXBDSPRN * sizeof(double));
	memset(P_DATA, 0.0, 8 * MAXBDSPRN * 8 * MAXBDSPRN * sizeof(double));
	memset(COV_DATA, 0.0, 8 * MAXBDSPRN * 8 * MAXBDSPRN * sizeof(double));
	memset(COV_X_DATA, 0.0, 4 * MAXBDSPRN * 4 * MAXBDSPRN * sizeof(double));
	memset(x_DATA, 0.0, (2 * MAXBDSPRN + 2 * MAXGPSPRN) * sizeof(double));
	memset(l_DATA, 0.0, (4 * MAXBDSPRN + 4 * MAXGPSPRN) * sizeof(double));
	memset(Bx_DATA, 0.0, (4 * MAXBDSPRN + 4 * MAXGPSPRN) * sizeof(double));
	memset(BTrans_DATA, 0.0, 4 * MAXBDSPRN * 8 * MAXBDSPRN * sizeof(double));
	memset(BTP_DATA, 0.0, 2 * MAXBDSPRN * 8 * MAXBDSPRN * sizeof(double));
	memset(BTPB_DATA, 0.0, 4 * MAXBDSPRN * 4 * MAXBDSPRN * sizeof(double));
	memset(BTPB_INV_DATA, 0.0, 2 * MAXBDSPRN * 8 * MAXBDSPRN * sizeof(double));
	memset(VTP_DATA, 0.0, 8 * MAXBDSPRN * 8 * MAXBDSPRN * sizeof(double));
	memset(BTPl_DATA, 0.0, (2 * MAXBDSPRN + 2 * MAXGPSPRN) * sizeof(double));
	memset(v_DATA, 0.0, (2 * MAXBDSPRN + 2 * MAXGPSPRN) * sizeof(double));
	memset(Bv_DATA, 0.0, 8 * MAXBDSPRN * sizeof(double));
	memset(VT_DATA, 0.0, 8 * MAXBDSPRN * sizeof(double));
	memset(Qa_DATA, 0.0, 4 * MAXBDSPRN * 4 * MAXBDSPRN * sizeof(double));
	memset(Qb_DATA, 0.0, 9 * sizeof(double));
	memset(Qba_DATA, 0.0, 12 * MAXGPSPRN * sizeof(double));
	int satnums[MAXBDSPRN] = { 0 };

	/*******************************��������С���˼���******************************/
	double coverage = 1E-20;
	int iterator = 0;
	// ���������ļ���λ��
	//FILE* fp;
	//fp = fopen("Matrix.txt", "at");
	memset(&rawR->rtk.xyzfloat, 0.0, sizeof(XYZ));
	do {
		iterator++;
		/**********     �������     **********/
		// ����B�����L����������
		fill_A_L_DOUBLE_DIFF(prnG, prnB, numGPS, numBDS, rawB, rawR, B_DATA, L_DATA, state);
		matrix* B = new matrix;
		MatrixInit(B, rowB, columnB, B_DATA);
		matrix* L = new matrix;
		MatrixInit(L, rowL, 1, L_DATA);

		// ����P����������
		fill_COV_DOUBLE_DIFF(prnG, prnB, numGPS, numBDS, rawB, rawR, COV_DATA, state);
		matrix* COV = new matrix;
		matrix* P = new matrix;
		MatrixInit(COV, dimensionP, dimensionP, COV_DATA);
		memcpy(P_data, &COV_DATA, dimensionP * dimensionP * sizeof(double));
		matinv(P_data, dimensionP);
		MatrixInit(P, dimensionP, dimensionP, P_data);

		// X �����ά���� (2 * numGPS + 1) ,1 
		int rowx = columnB;
		// Сl����ʹ�L�����ά����ͬ
		int rowl = rowL;
		x_DATA[0] = rawR->receiver.xyz.x - refx;
		x_DATA[1] = rawR->receiver.xyz.y - refy;
		x_DATA[2] = rawR->receiver.xyz.z - refz;

		matrix* x = new matrix;
		MatrixInit(x, rowx, 1, x_DATA);

		/**********     ��С����     **********/
		matrix* B_trans = new matrix;
		MatrixTrans(B_trans, B, BTrans_DATA);

		// ����BTPB
		// BTPB�ľ����С��columnB * columnB
		// columnB = 2 * numBDS - 1;
		matrix* BTPB = new matrix;
		// BTP�ľ���Ĵ�С��columnB * dimensionP
		matrix* BTP = new matrix;
		MatrixMulti(B_trans, P, BTP, BTP_DATA);
		MatrixMulti(BTP, B, BTPB, BTPB_DATA);

		// ����BTPB�������
		matrix* BTPB_INV = new matrix;
		memcpy(BTPBINV_data, BTPB_DATA, columnB * columnB * sizeof(double));
		matinv(BTPBINV_data, columnB);
		MatrixInit(BTPB_INV, columnB, columnB, BTPBINV_data);
		//MatrixInv(BTPB_INV, BTPB, BTPB_INV_DATA);

		// ����BTPL
		// BTPl��ά���� columnB * 1
		matrix* BTPL = new matrix;
		MatrixMulti(BTP, L, BTPL, BTPl_DATA);

		// ������в����
		matrix* v = new matrix;
		MatrixMulti(BTPB_INV, BTPL, v, v_DATA);


		// ������
		rawR->receiver.xyz.x += v_DATA[0];
		rawR->receiver.xyz.y += v_DATA[1];
		rawR->receiver.xyz.z += v_DATA[2];
		rawR->rtk.xyzfloat = rawR->receiver.xyz;

		// ������������ģ���ȸ����
		for (int n = 0; n < dimensionN; n++)
		{
			rawR->rtk.N_FLOAT[n] = v_DATA[n + 3];
		}
		rawR->rtk.rtkstate = FLOATSOLUTION;

		/**********     ��������     **********/
		matrix* Bx = new matrix;
		matrix* V = new matrix;
		matrix* V_trans = new matrix;
		matrix* VTP = new matrix;
		matrix* VTPV = new matrix;
		matrix* D = new matrix;

		MatrixMulti(B, v, Bx, Bx_DATA);
		// V = Bx - L
		MatrixMinus(V, Bx, L, V_DATA);
		//����VtPv
		MatrixTrans(V_trans, V, VT_DATA);
		MatrixMulti(V_trans, P, VTP, VTP_DATA);
		MatrixMulti(VTP, V, VTPV, VTPV_DATA);
		double sigma0, sigma2;
		sigma0 = sqrt(VTPV_DATA[0] / (rowB - columnB));
		sigma2 = pow(sigma0, 2);
		rawR->rtk.sigma0 = sigma0;
		MatrixTimes(D, BTPB_INV, sigma2, COV_X_DATA);
		coverage = v_DATA[0] * v_DATA[0] + v_DATA[1] * v_DATA[1] + v_DATA[2] * v_DATA[2];

		//// ���������
		//fprintf(fp, "Sec = %lf\n", rawR->obs.gpst.SecofWeek);
		//fprintf(fp, "RefGPS = %d", prnG);
		//fprintf(fp, "  RefBDS = %d\n", prnB);
		//fprintf(fp, "A Matrix = \n");
		//MatrixPrinttoFile(B, fp);
		//fprintf(fp, "L Matrix = \n");
		//MatrixPrinttoFile(L, fp);
		//fprintf(fp, "P Matrix = \n");
		//MatrixPrinttoFile(P, fp);
		//fprintf(fp, "xhat Matrix = \n");
		//MatrixPrinttoFile(v, fp);
		//fprintf(fp, "Q Matrix = \n");
		//MatrixPrinttoFile(BTPB_INV, fp);

	} while (coverage > 1E-07 && iterator < 15);
	//fclose(fp);
	rawR->rtk.xyz.x = rawR->rtk.xyzfloat.x;
	rawR->rtk.xyz.y = rawR->rtk.xyzfloat.y;
	rawR->rtk.xyz.z = rawR->rtk.xyzfloat.z;


	/*******************************����ģ���ȵĹ̶�******************************/
	// ��ȡģ���ȷ���Э�������
	//COV��column��B��column��һ����
	int Qa_index = 0; int Qba_index = 0;
	int COV_index = 3 * columnB + 3; // �ӵ����еĵ����п�ʼ����COV
	int COV_indexb = 3;              // �ӵ�һ�еĵ����п�ʼ����COV for Qba
	// ��ʼ���������˵�һ����3��3�������Qa���������
	for (int row = 0; row < (columnB - 3); row++)
	{
		for (int column = 0; column < (columnB - 3); column++)
		{
			Qa_DATA[Qa_index] = COV_X_DATA[COV_index];
			Qa_index++;
			COV_index++;
		}
		COV_index += 3;
	}
	// ��ʼ������ǰ���г���(3,3)�����Qba
	for (int row = 0; row < 3; row++)
	{
		for (int column = 0; column < (columnB - 3); column++)
		{
			Qba_DATA[Qba_index] = COV_X_DATA[COV_indexb];
			Qba_index++;
			COV_indexb++;
		}
		COV_indexb += 3;
	}

	//// �����������Ƿ���ȷ
	//FILE* fpout = fopen("Matrix.txt", "at");
	//matrix* COV = new matrix;
	//MatrixInit(COV, columnB, columnB, COV_X_DATA);
	//fprintf(fpout, "D Matrix\n");
	//MatrixPrinttoFile(COV, fpout);
	//matrix* QaMat = new matrix;
	//MatrixInit(QaMat, dimensionN, dimensionN, Qa_DATA);
	//fprintf(fpout, "Qa Matrix\n");
	//MatrixPrinttoFile(QaMat, fpout);
	//matrix* QbaMat = new matrix;
	//MatrixInit(QbaMat , 3, dimensionN, Qba_DATA);
	//fprintf(fpout, "Qba Matrix\n");
	//MatrixPrinttoFile(QbaMat, fpout);
	//fclose(fpout);
	
	// ����Qa����ģ���ȷ���Э�������
	// ����fa����ģ���ȸ�������
	// ����F����ģ���ȹ̶�������
	// ����s����ģ���Ȳв������
	double* fa = NULL, * Qa = NULL;
	double* F = NULL, * s = NULL;
	Qa = mat(rawR->rtk.N_n, rawR->rtk.N_n);
	memcpy(Qa, Qa_DATA, rawR->rtk.N_n* rawR->rtk.N_n * sizeof(double));
	fa = mat(rawR->rtk.N_n, 1);
	for (int i = 0; i < rawR->rtk.N_n; i++)
	{
		fa[i] = rawR->rtk.N_FLOAT[i];
	}
	F = mat(rawR->rtk.N_n, rawR->rtk.N_m);
	s = mat(1, rawR->rtk.N_m);

	int fixstate = 1;
	// ����Lambda���м��� 
	// n�� ģ����ά��
	// m�� ģ���Ⱥ�ѡ���������Ϊ2
	// fa��ģ���ȸ����������n*1
	// Qa��ģ���ȷ���Э�������n*n
	// F�� ģ���ȹ̶���������n*m
	// s�� ģ���Ȳв�����ͣ�1*m
	fixstate = lambda(rawR->rtk.N_n, rawR->rtk.N_m, fa, Qa, F, s);

	if (fixstate == 0)
		// ����0��ʾģ���ȹ̶��������سɹ�
	{
		double ratio = s[1] / s[0];
		rawR->rtk.ratio = ratio;
		// �̶���֮���ģ���ȵĴ洢
		// cout << rawR->rtk.gpst.SecofWeek << " " << rawR->rtk.ratio << endl;
		for (int i = 0; i < rawR->rtk.N_n; i++)
		{
			double* FIXED2 = new double[rawR->rtk.N_n];
			// ��i�е�j�е�������i + n * j
			rawR->rtk.N_FIXED[i] = F[i];
			FIXED2[i] = F[i + rawR->rtk.N_n];
			// cout << "G" << satnums[i] <<" "<< rawR->rtk.N_FLOAT[i] << " " << rawR->rtk.N_FIXED[i] << endl;
			delete[] FIXED2;
		}
		if (rawR->rtk.ratio > AMBIGUITY)
		{
			rawR->rtk.rtkstate = FIXEDSOLUTION;
		}
	}

	/*******************************ģ���ȹ̶�����л��������ĸ���******************************/
	if (rawR->rtk.rtkstate == FIXEDSOLUTION)
	{
		double FLOATXYZ_DATA[3] = { 0 };
		double FIXEDXYZ_DATA[3] = { 0 };
		double FIXEDXYZV_DATA[3] = { 0 };
		double N_FLOAT_DATA[MAXBDSPRN] = { 0 };
		double N_FIXED_DATA[MAXBDSPRN] = { 0 };
		double aminusa_DATA[MAXBDSPRN] = { 0 };
		static double Qainv_DATA[4 * MAXBDSPRN * 4 * MAXBDSPRN] = { 0 };
		double QbaQainv_DATA[12 * MAXGPSPRN] = { 0 };
		double* Qainv_data = new double[rawR->rtk.N_n * rawR->rtk.N_n];
		memcpy(Qainv_data, Qa, rawR->rtk.N_n* rawR->rtk.N_n * sizeof(double));
		memset(N_FLOAT_DATA, 0.0, MAXBDSPRN * sizeof(double));
		memset(N_FIXED_DATA, 0.0, MAXBDSPRN * sizeof(double));
		memset(aminusa_DATA, 0.0, MAXBDSPRN * sizeof(double));
		memset(Qainv_DATA, 0.0, 4 * MAXBDSPRN * 4 * MAXBDSPRN * sizeof(double));
		memset(QbaQainv_DATA, 0.0, 12 * MAXGPSPRN * sizeof(double));
		// ������Ҫ�ĸ�������
		for (int i = 0; i < rawR->rtk.N_n; i++)
		{
			N_FLOAT_DATA[i] = rawR->rtk.N_FLOAT[i];
			N_FIXED_DATA[i] = rawR->rtk.N_FIXED[i];
		}
		FLOATXYZ_DATA[0] = rawR->rtk.xyzfloat.x;
		FLOATXYZ_DATA[1] = rawR->rtk.xyzfloat.y;
		FLOATXYZ_DATA[2] = rawR->rtk.xyzfloat.z;
		matrix* N_Float = new matrix;
		matrix* N_Fixed = new matrix;
		matrix* COV_X = new matrix;
		matrix* Qa_mat = new matrix;
		matrix* Qba_mat = new matrix;
		matrix* float_xyz = new matrix;
		matrix* fixed_xyzV = new matrix;
		matrix* Qa_inv = new matrix;
		matrix* QbaQainv = new matrix;
		matrix* aminusa = new matrix;
		matinv(Qainv_data, rawR->rtk.N_n);
		MatrixInit(Qa_inv, rawR->rtk.N_n, rawR->rtk.N_n, Qainv_data);
		MatrixInit(COV_X, columnB, columnB, COV_X_DATA);
		MatrixInit(Qa_mat, rawR->rtk.N_n, rawR->rtk.N_n, Qa);
		MatrixInit(Qba_mat, 3, rawR->rtk.N_n, Qba_DATA);
		MatrixInit(N_Float, rawR->rtk.N_n, 1, N_FLOAT_DATA);
		MatrixInit(N_Fixed, rawR->rtk.N_n, 1, N_FIXED_DATA);
		MatrixInit(float_xyz, 3, 1, FLOATXYZ_DATA);

		// ������
		//FILE* fpout;
		//fpout = fopen("Matrix.txt", "at");
		//double sec = rawR->obs.gpst.SecofWeek;
		//fprintf(fpout, "Sec = %lf\n", sec);
		//fprintf(fpout, "Qaa Matrix\n");
		////MatrixPrinttoFile(COV_X, fpout);
		//MatrixPrinttoFile(Qa_mat, fpout);
		////MatrixPrinttoFile(Qba_mat, fpout);
		////MatrixPrinttoFile(N_Float, fpout);
		//fclose(fpout);

		// ���л��߹̶���ĸ���
		// MatrixInv(Qa_inv, Qa_mat, Qainv_DATA);
		MatrixMulti(Qba_mat, Qa_inv, QbaQainv, QbaQainv_DATA);
		MatrixMinus(aminusa, N_Float, N_Fixed, aminusa_DATA);
		MatrixMulti(QbaQainv, aminusa, fixed_xyzV, FIXEDXYZV_DATA);
		rawR->rtk.xyzfixed.x = rawR->rtk.xyzfloat.x - FIXEDXYZV_DATA[0];
		rawR->rtk.xyzfixed.y = rawR->rtk.xyzfloat.y - FIXEDXYZV_DATA[1];
		rawR->rtk.xyzfixed.z = rawR->rtk.xyzfloat.z - FIXEDXYZV_DATA[2];
		rawR->rtk.xyz.x = rawR->rtk.xyzfixed.x;
		rawR->rtk.xyz.y = rawR->rtk.xyzfixed.y;
		rawR->rtk.xyz.z = rawR->rtk.xyzfixed.z;

		delete[] Qainv_data;
	}
	 


	/*******************************����ı���;���ָ��ļ���******************************/
	  XYZ2NEU(&refxyz, &rawR->rtk.xyz, &rawR->rtk.neu);

	  // ���й̶��ʵĴ洢�ͼ���
	  if (rawR->rtk.rtkstate == FLOATSOLUTION)
	  {
		  rawR->rtk.allepochs++;
	  }
	  else if (rawR->rtk.rtkstate == FIXEDSOLUTION)
	  {
		  rawR->rtk.allepochs++;
		  rawR->rtk.fixedepochs++;
	  }
	 
	  rawR->rtk.fixrate = 100 * double(rawR->rtk.fixedepochs) / double(rawR->rtk.allepochs);

	  // ����PDOPֵ
      // PDOP������ֵ�������ֱ���0��columnB + 1��2 * columnB + 2
	  double PDOP;
	  PDOP = sqrt(BTPBINV_data[0] + BTPBINV_data[columnB + 1] + BTPBINV_data[2 * columnB + 2]);
	  rawR->rtk.PDOP = PDOP;
	  rawR->rtk.sigmax = rawR->rtk.sigma0 * rawR->rtk.sigma0 * BTPBINV_data[0];
	  rawR->rtk.sigmay = rawR->rtk.sigma0 * rawR->rtk.sigma0 * BTPBINV_data[columnB + 1];
	  rawR->rtk.sigmaz = rawR->rtk.sigma0 * rawR->rtk.sigma0 * BTPBINV_data[2 * columnB + 2];


	  // �����������
	  rawR->rtk.Vector[0] = rawR->rtk.xyz.x - refx;
	  rawR->rtk.Vector[1] = rawR->rtk.xyz.y - refy;
	  rawR->rtk.Vector[2] = rawR->rtk.xyz.z - refz;

	  // ����RMS
	  rawR->rtk.RMS = sqrt((pow(rawR->rtk.Vector[0], 2) + pow(rawR->rtk.Vector[1], 2) + pow(rawR->rtk.Vector[2], 2)) / 3);


	  free(fa); free(Qa);  free(F); free(s);
	  delete[] P_data;
	  delete[] BTPBINV_data;
}


int PrintResulttoFile(string filename, RTK* Rtk)
{
	ofstream outfileRTK;
	outfileRTK.open(filename, ios::app);
	if (Rtk->rtkstate == FAILEDSOLUTION)
	{
		return 0;
	}
	else if (Rtk->rtkstate == FLOATSOLUTION)
	{
		cout << setiosflags(ios::fixed) << setprecision(4) << setiosflags(ios::right);
		cout << int(Rtk->gpst.SecofWeek) << setw(4) << Rtk->numGPS << setw(4) << int (Rtk->numBDS);
		cout << setw(7) << "FLOAT";
		cout << setw(15) << Rtk->xyz.x << setw(14) << Rtk->xyz.y << setw(14) << Rtk->xyz.z;
		cout << setw(10) << Rtk->neu.N << setw(10) << Rtk->neu.E << setw(10) << Rtk->neu.U;
		cout << setw(10) << Rtk->sigma0 << setw(10) << Rtk->PDOP;
		cout << setw(10) << Rtk->ratio;
		cout << setw(7) << Rtk->fixedepochs << setw(7) << Rtk->allepochs << setw(12) << Rtk->fixrate << "%";
		cout << endl;

		outfileRTK << setiosflags(ios::fixed) << setprecision(4) << setiosflags(ios::right);
		outfileRTK << int(Rtk->gpst.SecofWeek) << setw(4) << Rtk->numGPS << setw(4) << Rtk->numBDS;
		outfileRTK << setw(3) << "0" <<" ";
		outfileRTK << setw(13) << Rtk->xyz.x << setw(14) << Rtk->xyz.y << setw(14) << Rtk->xyz.z;
		outfileRTK << setw(10) << Rtk->neu.N << setw(10) << Rtk->neu.E << setw(10) << Rtk->neu.U;
		outfileRTK << setw(10) << Rtk->sigma0 << setw(10) << Rtk->PDOP;
		outfileRTK << setw(10) << Rtk->ratio;
		outfileRTK << setw(7) << Rtk->fixedepochs << setw(7) << Rtk->allepochs << setw(12) << Rtk->fixrate  << "%";
		outfileRTK << setw(10) << Rtk->sigmax << setw(10) << Rtk->sigmay << setw(10) << Rtk->sigmaz << setw(10) << Rtk->RMS;
		outfileRTK << endl;
	}
	else if (Rtk->rtkstate == FIXEDSOLUTION && Rtk->numBDS !=0 )
	{
		cout << setiosflags(ios::fixed) << setprecision(4) << setiosflags(ios::right);
		cout << int(Rtk->gpst.SecofWeek) << setw(4) << Rtk->numGPS << setw(4) << int(Rtk->numBDS);
		cout << setw(7) << "FIXED";
		cout << setw(15) << Rtk->xyz.x << setw(14) << Rtk->xyz.y << setw(14) << Rtk->xyz.z;
		cout << setw(10) << Rtk->neu.N << setw(10) << Rtk->neu.E << setw(10) << Rtk->neu.U;
		cout << setw(10) << Rtk->sigma0 << setw(10) << Rtk->PDOP/1.6;
		cout << setw(10) << Rtk->ratio;
		cout << setw(7) << Rtk->fixedepochs << setw(7) << Rtk->allepochs << setw(12) << Rtk->fixrate << "%";
		cout << endl;

		outfileRTK << setiosflags(ios::fixed) << setprecision(4) << setiosflags(ios::right);
		outfileRTK << int(Rtk->gpst.SecofWeek) << setw(4) << Rtk->numGPS << setw(4) << Rtk->numBDS;
		outfileRTK << setw(3) << "1" << " ";
		outfileRTK << setw(13) << Rtk->xyz.x << setw(14) << Rtk->xyz.y << setw(14) << Rtk->xyz.z;
		outfileRTK << setw(10) << Rtk->neu.N << setw(10) << Rtk->neu.E << setw(10) << Rtk->neu.U;
		outfileRTK << setw(10) << Rtk->sigma0 << setw(10) << Rtk->PDOP;
		outfileRTK << setw(10) << Rtk->ratio;
		outfileRTK << setw(7) << Rtk->fixedepochs << setw(7) << Rtk->allepochs << setw(12) << Rtk->fixrate  << "%";
		outfileRTK << setw(10) << Rtk->sigmax << setw(10) << Rtk->sigmay << setw(10) << Rtk->sigmaz << setw(10) << Rtk->RMS;
		outfileRTK << endl;
	}
	outfileRTK.close();
}

void PrintHeadRTK(string filename)
{
	
	cout << " Sec " << "  " << "nG" << "  " << "nB" << "  " << " MODE" << "  ";
	cout << "     X/m     " << "  " << "     Y/m    " << "  " << "     Z/m     " << "  ";
	cout << "   N/m   " << "  " << "  E/m   " << "  " << "  U/m   " << "  ";
	cout << " Sigma0 " << "  " << "  PDOP" << "  " << "   Ratio  " << "  ";
	cout << "FIXED " << "  " << "ALL " << "  " << "  FIXRATE " << endl;
	
	ofstream outfileRTK;
	outfileRTK.open(filename, ios::out);

	//outfileRTK << " Sec " << "  " << "nG" << "  " << "nB" << "  " << " MODE" << "  ";
	//outfileRTK << "     X/m     " << "  " << "     Y/m    " << "  " << "     Z/m     " << "  ";
	//outfileRTK << "   N/m   " << "  " << "  E/m   " << "  " << "  U/m   " << "  ";
	//outfileRTK << "Sigma0 " << "  " << "  PDOP" << "  " << "   Ratio  " << "  ";
	//outfileRTK << "FIXED " << "  " << "ALL " << "  " << "  FIXRATE " <<  endl;
	outfileRTK.close();
}
