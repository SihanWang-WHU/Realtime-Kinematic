#include"SPP_SPV.h";

using namespace std;

/*************************************************************************
distance_calculate ��������XYZ����ĵ�֮��ľ���
���룺xyz1��xyz2����ṹ��
���������֮��ľ���
**************************************************************************/
double distance_calculate(XYZ xyz1, XYZ xyz2)
{
	double distance;
	distance = sqrt(pow(xyz1.x - xyz2.x, 2) + pow(xyz1.y - xyz2.y, 2) + pow(xyz1.z - xyz2.z, 2));
	return distance;
}

/*************************************************************************
fillBl_SPP ����SPP������B��l����ļ���
���룺ԭʼ����RAWDATA��ָ��raw��SPP����״̬state��
����� B��l�����飩,���ع۲�ֵ���õĸ���
�Ѳ��� 2021.12.29
**************************************************************************/
double fillBl_SPP(RAWDATA* raw, SPPSTATE state, double* B, double* l)
{
	//���Ե�ʱ������ѡ��˫ϵͳ�����㷨����
	//state = DOUBLESYS;
	//������˫ϵͳ���������ʱ��
	if (state == DOUBLESYS)
	{
		int num = 0;
		//����ÿ���۲�ֵ���������Ǹ�����ʼ��ѭ��
		for (int i = 0; i < raw->obs.SATNUMS; i++)
		{
			if (raw->obs.range[i].Valid == true && raw->obs.range[i].ValidCom == true)
			{
				if (raw->obs.range[i].Sys == GPS)
				{
					//����Range��IF��Ϲ۲�ֵ
					raw->obs.com[i].IF = IF_GPS(raw->obs.range[i].P1, raw->obs.range[i].P2);
					int prn = raw->obs.range[i].Prn;
					double row; //row�����ǺͲ�վ֮��ľ���
					row = distance_calculate(raw->sat_gps[prn - 1].satXYZ, raw->receiver.xyz);
					//double temp;
					//temp = (raw->receiver.xyz.x - raw->sat_gps[prn - 1].satXYZ.x) / row;
					B[0 + 5 * num] = (raw->receiver.xyz.x - raw->sat_gps[prn - 1].satXYZ.x) / row;
					B[1 + 5 * num] = (raw->receiver.xyz.y - raw->sat_gps[prn - 1].satXYZ.y) / row;
					B[2 + 5 * num] = (raw->receiver.xyz.z - raw->sat_gps[prn - 1].satXYZ.z) / row;
					B[3 + 5 * num] = 1;
					B[4 + 5 * num] = 0;
					l[num] = raw->obs.com[i].IF - (row + raw->receiver.dt_GPS - raw->sat_gps[prn - 1].satClk * C_SPEED + raw->obs.com[i].tropdelay);
					num++;
				}
				else if (raw->obs.range[i].Sys == BDS)
				{
					double tgd1, tgd2;
					int prn = raw->obs.range[i].Prn;
					tgd1 = raw->bde[prn - 1].tgd1;
					tgd2 = raw->bde[prn - 1].tgd2;
					//���ȼ����IF���������Ϲ۲�ֵ
					raw->obs.com[i].IF = IF_BDS(raw->obs.range[i].P1, raw->obs.range[i].P2);
					double TGD_IF; //IF��ϵ�tgd����
					TGD_IF = tgd1 * FB1_BDS * FB1_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS);
					double row; //row�����ǺͲ�վ֮��ľ���
					row = distance_calculate(raw->sat_bds[prn - 1].satXYZ, raw->receiver.xyz);
					//double temp;
					//temp = (raw->receiver.xyz.x - raw->sat_bds[prn - 1].satXYZ.x) / row;
					B[0 + 5 * num] = (raw->receiver.xyz.x - raw->sat_bds[prn - 1].satXYZ.x) / row;
					B[1 + 5 * num] = (raw->receiver.xyz.y - raw->sat_bds[prn - 1].satXYZ.y) / row;
					B[2 + 5 * num] = (raw->receiver.xyz.z - raw->sat_bds[prn - 1].satXYZ.z) / row;
					B[3 + 5 * num] = 0;
					B[4 + 5 * num] = 1;
					l[num] = raw->obs.com[i].IF - (row + raw->receiver.dt_BDS - (raw->sat_bds[prn - 1].satClk - TGD_IF) * C_SPEED + raw->obs.com[i].tropdelay);
					num++;
				}
				else continue;
			}
			else continue;
		}
		return num;
	}

	if (state == SINGLEBDS)
	{
		int num = 0;
		//����ÿ���۲�ֵ���������Ǹ�����ʼ��ѭ��
		for (int i = 0; i < raw->obs.SATNUMS; i++)
		{
			if (raw->obs.range[i].Valid == true && raw->obs.range[i].ValidCom == true)
			{
				if (raw->obs.range[i].Sys == BDS)
				{
					double tgd1, tgd2;
					int prn = raw->obs.range[i].Prn;
					tgd1 = raw->bde[prn - 1].tgd1;
					tgd2 = raw->bde[prn - 1].tgd2;
					//���ȼ����IF���������Ϲ۲�ֵ
					raw->obs.com[i].IF = IF_BDS(raw->obs.range[i].P1, raw->obs.range[i].P2);
					double TGD_IF; //IF��ϵ�tgd����
					TGD_IF = tgd1 * FB1_BDS * FB1_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS);
					double row; //row�����ǺͲ�վ֮��ľ���
					row = distance_calculate(raw->sat_bds[prn - 1].satXYZ, raw->receiver.xyz);
					//double temp;
					//temp = (raw->receiver.xyz.x - raw->sat_bds[prn - 1].satXYZ.x) / row;
					B[0 + 4 * num] = (raw->receiver.xyz.x - raw->sat_bds[prn - 1].satXYZ.x) / row;
					B[1 + 4 * num] = (raw->receiver.xyz.y - raw->sat_bds[prn - 1].satXYZ.y) / row;
					B[2 + 4 * num] = (raw->receiver.xyz.z - raw->sat_bds[prn - 1].satXYZ.z) / row;
					B[3 + 4 * num] = 1;
					l[num] = raw->obs.com[i].IF - (row + raw->receiver.dt_BDS - (raw->sat_bds[prn - 1].satClk - TGD_IF) * C_SPEED + raw->obs.com[i].tropdelay);
					num++;
				}
				else continue;
			}
			else continue;
		}
		return num;
	}

	if (state == SINGLEGPS)
	{
		int num = 0;
		//����ÿ���۲�ֵ���������Ǹ�����ʼ��ѭ��
		for (int i = 0; i < raw->obs.SATNUMS; i++)
		{
			if (raw->obs.range[i].Valid == true && raw->obs.range[i].ValidCom == true)
			{
				if (raw->obs.range[i].Sys == GPS)
				{
					//����Range��IF��Ϲ۲�ֵ
					raw->obs.com[i].IF = IF_GPS(raw->obs.range[i].P1, raw->obs.range[i].P2);
					int prn = raw->obs.range[i].Prn;
					double row; //row�����ǺͲ�վ֮��ľ���
					row = distance_calculate(raw->sat_gps[prn - 1].satXYZ, raw->receiver.xyz);
					//double temp;
					//temp = (raw->receiver.xyz.x - raw->sat_gps[prn - 1].satXYZ.x) / row;
					B[0 + 4 * num] = (raw->receiver.xyz.x - raw->sat_gps[prn - 1].satXYZ.x) / row;
					B[1 + 4 * num] = (raw->receiver.xyz.y - raw->sat_gps[prn - 1].satXYZ.y) / row;
					B[2 + 4 * num] = (raw->receiver.xyz.z - raw->sat_gps[prn - 1].satXYZ.z) / row;
					B[3 + 4 * num] = 1;
					//B[4 + 5 * num] = 0;
					l[num] = raw->obs.com[i].IF - (row + raw->receiver.dt_GPS - raw->sat_gps[prn - 1].satClk * C_SPEED + raw->obs.com[i].tropdelay);
					num++;
				}
				else continue;
			}
			else continue;
		}
		return num;
	}
}

/*************************************************************************
fillP_SPP ����SPP������P����ļ��� ����Ȩ����λ��
���룺�۲�ֵ���õĸ���
����� P�����飩
�Ѳ��� 2021.12.29
**************************************************************************/
double fillP_spp_plain(int size, double* P)
{
	if (size <= 0 || size > MAXCHANNUM)
	{
		printf("Error in size of P!\n");
		return 0;
	}
	for (int j = 0; j < size * size; j += size + 1)
	{
		P[j] = 1;
	}
	return 1;
}

double fillP_spp(int size, double* P, RAWDATA* raw)
{
	if (size <= 0 || size > MAXCHANNUM)
	{
		printf("Error in size of P!\n");
		return 0;
	}
	for (int j = 0; j < size * size; j += size + 1)
	{
		for (int i = 0; i < MAXCHANNUM; i++)
		{
			if (raw->obs.range[i].Valid == true && raw->obs.range[i].ValidCom == true)
			{
				if (raw->obs.range[i].Sys == GPS)
				{
					int prn = raw->obs.range[i].Prn;
					P[j] = 1 / raw->sat_gps[prn - 1].eleAngle;
				}
				if (raw->obs.range[i].Sys == BDS)
				{
					int prn = raw->obs.range[i].Prn;
					P[j] = 1 / raw->sat_bds[prn - 1].eleAngle;
				}
			}
		}
	}
	return 1;
}
/*************************************************************************
fillBl_SPV ����SPV������B��l����ļ���
���룺ԭʼ����RAWDATA��ָ��raw��
����� B��l�����飩,���ع۲�ֵ���õĸ���
δ���� 2021.12.29
**************************************************************************/
double fillBl_SPV(RAWDATA* raw, double* B, double* l)
{
	int num = 0;
	for (int i = 0; i < raw->obs.SATNUMS; i++)
	{
		// ����ÿһ�������õ�range�������ǰ���Ѿ��жϹ���
		// ����Ϊ��range�Ѿ�����������������Ӧ������λ���Ѿ���������� ���Ҹ߶ȽǴ���15��
		if (raw->obs.range[i].Valid && raw->obs.range[i].D1 != 0 && raw->obs.range[i].ValidCom == true)
		{
			if (raw->obs.range[i].Sys == GPS)
			{
				int prn;
				prn = raw->obs.range[i].Prn;
				double row;
				row = distance_calculate(raw->sat_gps[prn - 1].satXYZ, raw->receiver.xyz);
				double dopp = -WL1_GPS * raw->obs.range[i].D1;
				B[4 * num + 0] = (raw->receiver.xyz.x - raw->sat_gps[prn - 1].satXYZ.x) / row;
				B[4 * num + 1] = (raw->receiver.xyz.y - raw->sat_gps[prn - 1].satXYZ.y) / row;
				B[4 * num + 2] = (raw->receiver.xyz.z - raw->sat_gps[prn - 1].satXYZ.z) / row;
				B[4 * num + 3] = 1;
				double rowdot;
				rowdot = ((raw->sat_gps[prn - 1].satXYZ.x - raw->receiver.xyz.x) * raw->sat_gps[prn - 1].satVelocity[0]
					+ (raw->sat_gps[prn - 1].satXYZ.y - raw->receiver.xyz.y) * raw->sat_gps[prn - 1].satVelocity[1]
					+ (raw->sat_gps[prn - 1].satXYZ.z - raw->receiver.xyz.z) * raw->sat_gps[prn - 1].satVelocity[2]) / row;
				l[num] = dopp - (rowdot - C_SPEED * raw->sat_gps[prn - 1].satClkDot);
				num++;
			}
			else if (raw->obs.range[i].Sys == BDS)
			{
				int prn;
				prn = raw->obs.range[i].Prn;
				double row;
				row = distance_calculate(raw->sat_bds[prn - 1].satXYZ, raw->receiver.xyz);
				double dopp;
				dopp = -WB1_BDS * raw->obs.range[i].D1;
				B[4 * num + 0] = (raw->receiver.xyz.x - raw->sat_bds[prn - 1].satXYZ.x) / row;
				B[4 * num + 1] = (raw->receiver.xyz.y - raw->sat_bds[prn - 1].satXYZ.y) / row;
				B[4 * num + 2] = (raw->receiver.xyz.z - raw->sat_bds[prn - 1].satXYZ.z) / row;
				B[4 * num + 3] = 1;
				double rowdot;
				rowdot = ((raw->sat_bds[prn - 1].satXYZ.x - raw->receiver.xyz.x) * raw->sat_bds[prn - 1].satVelocity[0]
					+ (raw->sat_bds[prn - 1].satXYZ.y - raw->receiver.xyz.y) * raw->sat_bds[prn - 1].satVelocity[1]
					+ (raw->sat_bds[prn - 1].satXYZ.z - raw->receiver.xyz.z) * raw->sat_bds[prn - 1].satVelocity[2]) / row;
				l[num] = dopp - (rowdot - C_SPEED * raw->sat_bds[prn - 1].satClkDot);
				num++;
			}
		}
	}
	if (num < 4)
	{
		return 0;
	}
	else
	{
		return num;
	}
}

/*************************************************************************
fillP_SPV ����SPV������P����ļ��� ����Ȩ����λ��
���룺�۲�ֵ���õĸ���
����� P�����飩
�Ѳ��� 2021.12.29
**************************************************************************/
double fillP_spv_plain(int size, double* P)
{
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size; j++)
		{
			P[i * size + j] = (i == j ? 1 : 0);
		}
	}
	return 1;
}
double fillP_spv(int size, double* P, RAWDATA* raw)
{
	if (size <= 0 || size > MAXCHANNUM)
	{
		printf("Error in size of P!\n");
		return 0;
	}
	for (int j = 0; j < size * size; j += size + 1)
	{
		for (int i = 0; i < MAXCHANNUM; i++)
		{
			if (raw->obs.range[i].Valid == true && raw->obs.range[i].ValidCom == true)
			{
				if (raw->obs.range[i].Sys == GPS)
				{
					int prn = raw->obs.range[i].Prn;
					P[j] = 1 / raw->sat_gps[prn - 1].eleAngle;
				}
				if (raw->obs.range[i].Sys == BDS)
				{
					int prn = raw->obs.range[i].Prn;
					P[j] = 1 / raw->sat_bds[prn - 1].eleAngle;
				}
			}
		}
	}
	return 1;
}

/*************************************************************************
SPP ��׼���㶨λλ�ü��㺯��
���룺ԭʼ����RAWDATA��ָ��raw
�����
**************************************************************************/
int SPP(RAWDATA* raw)
{
	int invalidObs = 0;
	GPSTIME* gTime = new GPSTIME;
	BLH blh = raw->RefPos.Blh;
	XYZ xyz;
	//blh.lat = 30;
	//blh.lng = 114;
	//blh.h = 0;
	BLH2XYZ(&xyz, &blh);

	XYZ* refxyz = new XYZ;
	refxyz->x = -2267804.5263;
	refxyz->y = 5009342.3723;
	refxyz->z = 3220991.8632;

	if (abs(xyz.x) > 1e10)
	{
		raw->receiver.xyz.x = refxyz->x;
		raw->receiver.xyz.y = refxyz->y;
		raw->receiver.xyz.z = refxyz->z;
	}
	else
	{
		raw->receiver.xyz.x = xyz.x;
		raw->receiver.xyz.y = xyz.y;
		raw->receiver.xyz.z = xyz.z;
	}
	raw->receiver.dt_BDS = 0;
	raw->receiver.dt_GPS = 0;

	int flag; //�����ж�����ϵͳ��GPS����BDS
	double ele = 0; //���ǵĸ߶Ƚ�(�Ƕȣ�
	double ele_rad;//���ǵĸ߶Ƚǣ����ȣ�

	//��������λ��
	for (int i = 0; i < raw->obs.SATNUMS; i++)
	{
		raw->obs.range[i].Valid = true;
		raw->obs.range[i].ValidCom = true;
		int prn = raw->obs.range[i].Prn;
		gTime = &raw->obs.range[i].gpst;
		raw->gpst = raw->obs.range[i].gpst;

		if (raw->obs.range[i].Sys == GPS)
		{
			if (raw->obs.range[i].Prn == 0)
			{
				// �����ʱ���Prn��0 ��Ѹ�range��valid��Ϊfalse
				raw->obs.range[i].Valid = false;

				invalidObs++;
				continue;
			}
			GPSPOSVEL(raw, gTime);
		}

		if (raw->obs.range[i].Sys == BDS)
		{
			if (raw->obs.range[i].Prn == 0)
			{
				raw->obs.range[i].Valid = false;

				invalidObs++;
				continue;
			}
			BDSPOSVEL(raw, gTime);
		}

	}

	//���дֲ�̽��
	DetectOutlier(raw);

	//�ж������������Ƿ��㹻
	if (invalidObs == raw->obs.SATNUMS)
	{
		printf("Ephemeris not enough!\n");
		return 0;
	}

	//�����Ӳ�Ͷ������ӳ�
	for (int i = 0; i < raw->obs.SATNUMS; i++)
	{
		if (raw->obs.range[i].Valid)
		{
			int prn = raw->obs.range[i].Prn;
			//int flag; //�����ж�����ϵͳ��GPS����BDS
			//double ele; //���ǵĸ߶Ƚ�(�Ƕȣ�
			//double ele_rad;//���ǵĸ߶Ƚǣ����ȣ�
			if (raw->obs.range[i].Sys == GPS)
			{
				//������Щ���ǵĹ۲�ֵ������λ��ͬʱ���������
				if (raw->sat_gps[prn - 1].Valid == true)
				{
					flag = 0;
					calNEU_AE(raw, prn, flag);                        //�������վ��NEU�����ǵĸ߶ȽǺͷ�λ��
					ele_rad = raw->sat_gps[prn - 1].eleAngle;
					ele = raw->sat_gps[prn - 1].eleAngle * 180 / PI;  //����ĸ߶Ƚ�ת��Ϊ����
					//�޳��߶Ƚ�С��15������
					if (ele <= 10)
					{
						/*raw->obs.range[i].Valid = false;*/
						raw->sat_gps[prn - 1].Valid = false;
						continue;
					}
					else if (ele >= 10)
					{
						raw->obs.range[i].Valid = true;
						raw->sat_gps[prn - 1].Valid = true;
					}
				}
				else
				{
					raw->obs.range[i].Valid = false;
					raw->sat_gps[prn - 1].Valid = false;
				}


			}
			else if (raw->obs.range[i].Sys == BDS)
			{
				if (raw->sat_bds[prn - 1].Valid == true)
				{
					flag = 1;
					calNEU_AE(raw, prn, flag);                        //�������վ��NEU�����ǵĸ߶ȽǺͷ�λ��
					ele_rad = raw->sat_bds[prn - 1].eleAngle;
					ele = raw->sat_bds[prn - 1].eleAngle * 180 / PI;  //����ĸ߶Ƚ�ת��Ϊ����
					//�޳��߶Ƚ�С��15������
					if (ele <= 10)
					{
						raw->sat_bds[prn - 1].Valid = false;
						/*	raw->obs.range[i].Valid = false;*/
						continue;
					}
					else if (ele >= 10)
					{
						raw->sat_bds[prn - 1].Valid = true;
						raw->obs.range[i].Valid = true;
					}
				}
				else
				{
					raw->obs.range[i].Valid = false;
					raw->sat_bds[prn - 1].Valid = false;
				}
			}
			raw->obs.com[i].tropdelay = Hopfield(raw, raw->obs.range[i].Sys, prn);
		}
		//else continue;
	}

	//���õ�������
	int iterator = 0;
	double converage = 0;

	int system_choose;                      // ����ģʽ 0Ϊ˫ϵͳ 1Ϊ��GPSϵͳ 2Ϊ������ϵͳ
	int numSat = 0, numBDS = 0, numGPS = 0; //�ֱ��������õ����ǣ����õ�GPS�������Լ����õ�BDS������

	/*
	 RefPos[3] = { -2267804.5263, 5009342.3723 , 3220991.8632 };
	*/

	//����ȵ�ѡ��
	for (int i = 0; i < raw->obs.SATNUMS; i++)
	{
		if (raw->obs.range[i].Snr1 < SnrLim && raw->obs.range[i].Snr2 < SnrLim)
		{
			raw->obs.range[i].Valid = false;
		}
		if (raw->obs.range[i].Sys == BDS)
		{
			if (raw->obs.range[i].Snr1 < SnrBDS && raw->obs.range[i].Snr2 < SnrBDS)
			{
				raw->obs.range[i].Valid = false;
			}
		}
	}

	// ѡ�����ǹ۲�����������rangedata
	for (int i = 0; i < raw->obs.SATNUMS; i++)
	{
		if (raw->obs.range[i].L1 - raw->obs.range[i].L2 > 10000)
		{
			raw->obs.range[i].Valid = false;
		}
	}

	for (int i = 0; i < raw->obs.SATNUMS; i++)
	{
		if (raw->obs.range[i].Valid && raw->obs.range[i].ValidCom)
		{
			numSat++;
			if (raw->obs.range[i].Sys == GPS)
			{
				numGPS++;
			}
			else if (raw->obs.range[i].Sys == BDS)
			{
				numBDS++;
			}
		}
		else continue;
	}

	// ���п��Խ���ʲô���㷽ʽ���ж�
	if (numSat > 5 && numBDS > 0 && numGPS > 0)
	{
		system_choose = 0;
	}
	else if (numGPS > 3)
	{
		system_choose = 1;
	}
	else if (numBDS > 3)
	{
		system_choose = 2;
	}
	else
	{
		return -1;
	}
	//��ʼSPP����
	do
	{
		//����������˫ϵͳ����ʱ��
		if (system_choose == 0)
		{
			//����С���˼���ĳ�ֵ
			double X_DATA[5] = { raw->receiver.xyz.x ,raw->receiver.xyz.y,raw->receiver.xyz.z,0, 0 };
			//double X_DATA[5] = { raw->receiver.xyz.x ,raw->receiver.xyz.y,raw->receiver.xyz.z,raw->receiver.dt_GPS,raw->receiver.dt_BDS };
			//����ֵ�ĸ��־��󸳳�ֵ
			//��Ҫע����ǣ����ھ�������һά���鶨��ģ�����������ֻҪ������ܴﵽ�����ֵ�Ϳ�����
			double B_DATA[5 * MAXCHANNUM] = { 0 };
			double B_Transpose_DATA[5 * MAXCHANNUM] = { 0 };
			double l_DATA[MAXCHANNUM] = { 0 };
			double P_DATA[MAXCHANNUM * MAXCHANNUM] = { 0 };
			SPPSTATE state = DOUBLESYS;
			int size = fillBl_SPP(raw, state, B_DATA, l_DATA);
			fillP_spp(size, P_DATA,raw);

			//B����ĳ�ʼ��
			matrix* B = new matrix;
			MatrixInit(B, size, 5, B_DATA);

			//l����ĳ�ʼ��
			matrix* l = new matrix;
			MatrixInit(l, size, 1, l_DATA);

			//P����ĳ�ʼ��
			matrix* P = new matrix;
			MatrixInit(P, size, size, P_DATA);

			//����B��ת��
			matrix* B_trans = new matrix;
			MatrixTrans(B_trans, B, B_Transpose_DATA);

			//����BTPB
			double BTPB_DATA[25] = { 0 };
			matrix* BTPB = new matrix;
			double BTP_DATA[5 * MAXCHANNUM] = { 0 };
			matrix* BTP = new matrix;
			MatrixMulti(B_trans, P, BTP, BTP_DATA);
			MatrixMulti(BTP, B, BTPB, BTPB_DATA);

			//MatrixPrint(B);
			//cout << endl;
			//MatrixPrint(l);
			//cout << endl;
			//MatrixPrint(P);
			//cout << endl;
			//MatrixPrint(B_trans);
			//cout << endl;
			//MatrixPrint(BTP);
			//cout << endl;
			//MatrixPrint(BTPB);
			//cout << endl;

			//����BTPB�������
			double BTPB_INV_DATA[25] = { 0 };
			matrix* BTPB_INV = new matrix;
			int matstate;
			matstate = MatrixInv(BTPB_INV, BTPB, BTPB_INV_DATA);
			//if (matstate == 1)
			//{
			//	break;
			//}
			double PDOP;
			PDOP = sqrt(BTPB_INV_DATA[0] + BTPB_INV_DATA[6] + BTPB_INV_DATA[12]);

			//����BTPl
			double BTPl_DATA[5] = { 0 };
			matrix* BTPl = new matrix;
			MatrixMulti(BTP, l, BTPl, BTPl_DATA);

			//������в����
			double v_DATA[5] = { 0 };
			matrix* v = new matrix;
			MatrixMulti(BTPB_INV, BTPl, v, v_DATA);

			//����V����V=Bv-l��
			double V_DATA[MAXCHANNUM] = { 0 };
			matrix* V = new matrix;
			double Bv_DATA[MAXCHANNUM] = { 0 };
			matrix* Bv = new matrix;
			MatrixMulti(B, v, Bv, Bv_DATA);
			MatrixMinus(V, Bv, l, V_DATA);

			//����VtPv
			double VT_DATA[MAXCHANNUM] = { 0 };
			matrix* V_trans = new matrix;
			MatrixTrans(V_trans, V, VT_DATA);

			double VTP_DATA[MAXCHANNUM] = { 0 };
			matrix* VTP = new matrix;
			MatrixMulti(V_trans, P, VTP, VTP_DATA);

			double VTPV_DATA[1] = { 0 };
			matrix* VTPV = new matrix;
			MatrixMulti(VTP, V, VTPV, VTPV_DATA);

			double sigma0;
			sigma0 = sqrt(VTPV_DATA[0] / (numSat - 5));

			raw->receiver.xyz.x += v_DATA[0];
			raw->receiver.xyz.y += v_DATA[1];
			raw->receiver.xyz.z += v_DATA[2];
			XYZ2NEU(refxyz, &raw->receiver.xyz, &raw->receiver.neu);
			raw->receiver.dt_GPS += v_DATA[3];
			raw->receiver.dt_BDS += v_DATA[4];
			raw->receiver.PDOP = PDOP;
			raw->receiver.sigma0 = sigma0;
			iterator++;
			converage = v_DATA[0] * v_DATA[0] + v_DATA[1] * v_DATA[1] + v_DATA[2] * v_DATA[2];
		}

		//���������㵥GPS����ʱ��
		//����һ��SPPstate = SINGLEGPS ��ֵ����û��BDS���Ӳ� ���sigma0����ֵΪ3 һЩ�����ά�ȴ�5�����4
		if (system_choose == 1)
		{
			//����С���˼���ĳ�ֵ
			double X_DATA[4] = { raw->receiver.xyz.x ,raw->receiver.xyz.y,raw->receiver.xyz.z,0 };
			//double X_DATA[4] = { raw->receiver.xyz.x ,raw->receiver.xyz.y,raw->receiver.xyz.z,raw->receiver.dt_GPS };
			//����ֵ�ĸ��־��󸳳�ֵ
			//��Ҫע����ǣ����ھ�������һά���鶨��ģ�����������ֻҪ������ܴﵽ�����ֵ�Ϳ�����
			double B_DATA[4 * MAXCHANNUM] = { 0 };
			double B_Transpose_DATA[4 * MAXCHANNUM] = { 0 };
			double l_DATA[MAXCHANNUM] = { 0 };
			double P_DATA[MAXCHANNUM * MAXCHANNUM] = { 0 };
			SPPSTATE state = SINGLEGPS;
			int size = fillBl_SPP(raw, state, B_DATA, l_DATA);
			fillP_spp(size, P_DATA,raw);

			//B����ĳ�ʼ��
			matrix* B = new matrix;
			MatrixInit(B, size, 4, B_DATA);

			//l����ĳ�ʼ��
			matrix* l = new matrix;
			MatrixInit(l, size, 1, l_DATA);

			//P����ĳ�ʼ��
			matrix* P = new matrix;
			MatrixInit(P, size, size, P_DATA);

			//����B��ת��
			matrix* B_trans = new matrix;
			MatrixTrans(B_trans, B, B_Transpose_DATA);

			//����BTPB
			double BTPB_DATA[25] = { 0 };
			matrix* BTPB = new matrix;
			double BTP_DATA[4 * MAXCHANNUM] = { 0 };
			matrix* BTP = new matrix;
			MatrixMulti(B_trans, P, BTP, BTP_DATA);
			MatrixMulti(BTP, B, BTPB, BTPB_DATA);

			//MatrixPrint(B);
			//cout << endl;
			//MatrixPrint(l);
			//cout << endl;
			//MatrixPrint(P);
			//cout << endl;
			//MatrixPrint(B_trans);
			//cout << endl;
			//MatrixPrint(BTP);
			//cout << endl;
			//MatrixPrint(BTPB);
			//cout << endl;

			//����BTPB�������
			double BTPB_INV_DATA[25] = { 0 };
			matrix* BTPB_INV = new matrix;
			MatrixInv(BTPB_INV, BTPB, BTPB_INV_DATA);
			double PDOP;
			PDOP = sqrt(BTPB_INV_DATA[0] + BTPB_INV_DATA[6] + BTPB_INV_DATA[12]);

			//����BTPl
			double BTPl_DATA[5] = { 0 };
			matrix* BTPl = new matrix;
			MatrixMulti(BTP, l, BTPl, BTPl_DATA);

			//������в����
			double v_DATA[5] = { 0 };
			matrix* v = new matrix;
			MatrixMulti(BTPB_INV, BTPl, v, v_DATA);

			//����V����V=Bv-l��
			double V_DATA[MAXCHANNUM] = { 0 };
			matrix* V = new matrix;
			double Bv_DATA[MAXCHANNUM] = { 0 };
			matrix* Bv = new matrix;
			MatrixMulti(B, v, Bv, Bv_DATA);
			MatrixMinus(V, Bv, l, V_DATA);

			//����VtPv
			double VT_DATA[MAXCHANNUM] = { 0 };
			matrix* V_trans = new matrix;
			MatrixTrans(V_trans, V, VT_DATA);

			double VTP_DATA[MAXCHANNUM] = { 0 };
			matrix* VTP = new matrix;
			MatrixMulti(V_trans, P, VTP, VTP_DATA);

			double VTPV_DATA[1] = { 0 };
			matrix* VTPV = new matrix;
			MatrixMulti(VTP, V, VTPV, VTPV_DATA);

			double sigma0;
			sigma0 = sqrt(VTPV_DATA[0] / (numSat - 4));

			raw->receiver.xyz.x += v_DATA[0];
			raw->receiver.xyz.y += v_DATA[1];
			raw->receiver.xyz.z += v_DATA[2];
			XYZ2NEU(refxyz, &raw->receiver.xyz, &raw->receiver.neu);
			raw->receiver.dt_GPS += v_DATA[3];
			raw->receiver.PDOP = PDOP;
			raw->receiver.sigma0 = sigma0;
			iterator++;
			converage = v_DATA[0] * v_DATA[0] + v_DATA[1] * v_DATA[1] + v_DATA[2] * v_DATA[2];
		}

		//���������㵥BDS����ʱ��
		//����һ��SPPstate = SINGLEBDS ��ֵ����û��GPS���Ӳ� ���sigma0����ֵΪ3 һЩ�����ά�ȴ�5�����4
		if (system_choose == 2)
		{
			//����С���˼���ĳ�ֵ
			double X_DATA[4] = { raw->receiver.xyz.x ,raw->receiver.xyz.y,raw->receiver.xyz.z,0 };
			//double X_DATA[4] = { raw->receiver.xyz.x ,raw->receiver.xyz.y,raw->receiver.xyz.z,raw->receiver.dt_BDS };
			//����ֵ�ĸ��־��󸳳�ֵ
			//��Ҫע����ǣ����ھ�������һά���鶨��ģ�����������ֻҪ������ܴﵽ�����ֵ�Ϳ�����
			double B_DATA[4 * MAXCHANNUM] = { 0 };
			double B_Transpose_DATA[4 * MAXCHANNUM] = { 0 };
			double l_DATA[MAXCHANNUM] = { 0 };
			double P_DATA[MAXCHANNUM * MAXCHANNUM] = { 0 };
			SPPSTATE state = SINGLEBDS;
			int size = fillBl_SPP(raw, state, B_DATA, l_DATA);
			fillP_spp(size, P_DATA,raw);

			//B����ĳ�ʼ��
			matrix* B = new matrix;
			MatrixInit(B, size, 4, B_DATA);


			//l����ĳ�ʼ��
			matrix* l = new matrix;
			MatrixInit(l, size, 1, l_DATA);


			//P����ĳ�ʼ��
			matrix* P = new matrix;
			MatrixInit(P, size, size, P_DATA);


			//����B��ת��
			matrix* B_trans = new matrix;
			MatrixTrans(B_trans, B, B_Transpose_DATA);

			//����BTPB
			double BTPB_DATA[25] = { 0 };
			matrix* BTPB = new matrix;
			double BTP_DATA[5 * MAXCHANNUM] = { 0 };
			matrix* BTP = new matrix;
			MatrixMulti(B_trans, P, BTP, BTP_DATA);
			MatrixMulti(BTP, B, BTPB, BTPB_DATA);

			//MatrixPrint(B);
			//cout << endl;
			//MatrixPrint(l);
			//cout << endl;
			//MatrixPrint(P);
			//cout << endl;
			//MatrixPrint(B_trans);
			//cout << endl;
			//MatrixPrint(BTP);
			//cout << endl;
			//MatrixPrint(BTPB);
			//cout << endl;

			//����BTPB�������
			double BTPB_INV_DATA[25] = { 0 };
			matrix* BTPB_INV = new matrix;
			int matstate;
			matstate = MatrixInv(BTPB_INV, BTPB, BTPB_INV_DATA);
			//if (matstate == 1)
			//{
			//	break;
			//}
			double PDOP;
			PDOP = sqrt(BTPB_INV_DATA[0] + BTPB_INV_DATA[6] + BTPB_INV_DATA[12]);

			//����BTPl
			double BTPl_DATA[5] = { 0 };
			matrix* BTPl = new matrix;
			MatrixMulti(BTP, l, BTPl, BTPl_DATA);

			//������в����
			double v_DATA[5] = { 0 };
			matrix* v = new matrix;
			MatrixMulti(BTPB_INV, BTPl, v, v_DATA);

			//����V����V=Bv-l��
			double V_DATA[MAXCHANNUM] = { 0 };
			matrix* V = new matrix;
			double Bv_DATA[MAXCHANNUM] = { 0 };
			matrix* Bv = new matrix;
			MatrixMulti(B, v, Bv, Bv_DATA);
			MatrixMinus(V, Bv, l, V_DATA);

			//����VtPv
			double VT_DATA[MAXCHANNUM] = { 0 };
			matrix* V_trans = new matrix;
			MatrixTrans(V_trans, V, VT_DATA);

			double VTP_DATA[MAXCHANNUM] = { 0 };
			matrix* VTP = new matrix;
			MatrixMulti(V_trans, P, VTP, VTP_DATA);

			double VTPV_DATA[1] = { 0 };
			matrix* VTPV = new matrix;
			MatrixMulti(VTP, V, VTPV, VTPV_DATA);

			double sigma0;
			sigma0 = sqrt(VTPV_DATA[0] / (numSat - 4));

			raw->receiver.xyz.x += v_DATA[0];
			raw->receiver.xyz.y += v_DATA[1];
			raw->receiver.xyz.z += v_DATA[2];
			XYZ2NEU(refxyz, &raw->receiver.xyz, &raw->receiver.neu);
			raw->receiver.dt_BDS += v_DATA[3];
			raw->receiver.PDOP = PDOP;
			raw->receiver.sigma0 = sigma0;
			iterator++;
			converage = v_DATA[0] * v_DATA[0] + v_DATA[1] * v_DATA[1] + v_DATA[2] * v_DATA[2];
		}

	} while (iterator < 30 && converage	>1E-10);
	raw->receiver.numGPS = numGPS;
	raw->receiver.numBDS = numBDS;
	/*cout << setprecision(3) << setiosflags(ios::fixed) << "   " << system_choose << "   " << gTime->Week << " " << gTime->SecofWeek << " " << numGPS << " " << numBDS << "  ";
	cout << setprecision(8) << setiosflags(ios::fixed) << raw->receiver.xyz.x << " " << raw->receiver.xyz.y << " " << raw->receiver.xyz.z << " " << raw->receiver.PDOP << " " << raw->receiver.sigma0 << endl;;*/
	raw->SPPstate = true;
	return 0;
}

/*************************************************************************
SPV ��׼���㶨λ�ٶȼ��㺯��
���룺ԭʼ����RAWDATA��ָ��raw
�����
**************************************************************************/
int SPV(RAWDATA* raw)
{
	int flag = 0;
	// SPP���гɹ������SPV����
	if (raw->SPPstate)
	{
		double B_DATA[4 * MAXCHANNUM] = { 0 };
		double B_Transpose_DATA[4 * MAXCHANNUM] = { 0 };
		double l_DATA[MAXCHANNUM] = { 0 };
		double P_DATA[MAXCHANNUM * MAXCHANNUM] = { 0 };
		int size = fillBl_SPV(raw, B_DATA, l_DATA);
		//�����ù۲���С��4 ��size���� -1
		if (size == 0)
		{
			//printf("Error in SPV! Satellite not enough!33\n");
			return -1;
		}
		fillP_spv(size, P_DATA,raw);

		//B����ĳ�ʼ��
		matrix* B = new matrix;
		MatrixInit(B, size, 4, B_DATA);


		//l����ĳ�ʼ��
		matrix* l = new matrix;
		MatrixInit(l, size, 1, l_DATA);


		//P����ĳ�ʼ��
		matrix* P = new matrix;
		MatrixInit(P, size, size, P_DATA);


		//����B��ת��
		matrix* B_trans = new matrix;
		MatrixTrans(B_trans, B, B_Transpose_DATA);

		//����BTPB
		double BTPB_DATA[16] = { 0 };
		matrix* BTPB = new matrix;
		double BTP_DATA[4 * MAXCHANNUM] = { 0 };
		matrix* BTP = new matrix;
		MatrixMulti(B_trans, P, BTP, BTP_DATA);
		MatrixMulti(BTP, B, BTPB, BTPB_DATA);

		//MatrixPrint(B);
		//cout << endl;
		//MatrixPrint(l);
		//cout << endl;
		//MatrixPrint(P);
		//cout << endl;
		//MatrixPrint(B_trans);
		//cout << endl;
		//MatrixPrint(BTP);
		//cout << endl;
		//MatrixPrint(BTPB);
		//cout << endl;

		//����BTPB�������
		double BTPB_INV_DATA[16] = { 0 };
		matrix* BTPB_INV = new matrix;
		MatrixInv(BTPB_INV, BTPB, BTPB_INV_DATA);
		double PDOP;
		PDOP = sqrt(BTPB_INV_DATA[0] + BTPB_INV_DATA[6] + BTPB_INV_DATA[12]);

		//����BTPl
		double BTPl_DATA[4] = { 0 };
		matrix* BTPl = new matrix;
		MatrixMulti(BTP, l, BTPl, BTPl_DATA);

		//������в����
		double v_DATA[4] = { 0 };
		matrix* v = new matrix;
		MatrixMulti(BTPB_INV, BTPl, v, v_DATA);

		raw->receiver.vel[0] = v_DATA[0];
		raw->receiver.vel[1] = v_DATA[1];
		raw->receiver.vel[2] = v_DATA[2];

		//������
	/*	cout << setprecision(8) << setiosflags(ios::fixed) << " " << raw->receiver.vel[0] << " " << raw->receiver.vel[1] << " " << raw->receiver.vel[2] << " ";
		cout << setprecision(4) << setiosflags(ios::fixed) << raw->receiver.neu.N << " " << raw->receiver.neu.E << " ";
	*/	if (flag == 0)
	{
		cout << raw->receiver.neu.U / 2 << endl;
	}
	return 0;
	}
	// ��SPPû�гɹ����н�����ֱ�ӷ���-1
	else
	{
		return -1;
	}

}