#include"SPP_SPV.h";

using namespace std;

/*************************************************************************
distance_calculate 计算两个XYZ坐标的点之间的距离
输入：xyz1和xyz2坐标结构体
输出：两点之间的距离
**************************************************************************/
double distance_calculate(XYZ xyz1, XYZ xyz2)
{
	double distance;
	distance = sqrt(pow(xyz1.x - xyz2.x, 2) + pow(xyz1.y - xyz2.y, 2) + pow(xyz1.z - xyz2.z, 2));
	return distance;
}

/*************************************************************************
fillBl_SPP 用于SPP解算中B，l矩阵的计算
输入：原始数据RAWDATA的指针raw，SPP解算状态state，
输出： B和l（数组）,返回观测值可用的个数
已测试 2021.12.29
**************************************************************************/
double fillBl_SPP(RAWDATA* raw, SPPSTATE state, double* B, double* l)
{
	//测试的时候首先选用双系统进行算法调试
	//state = DOUBLESYS;
	//当满足双系统解算的条件时：
	if (state == DOUBLESYS)
	{
		int num = 0;
		//按照每个观测值读到的卫星个数开始做循环
		for (int i = 0; i < raw->obs.SATNUMS; i++)
		{
			if (raw->obs.range[i].Valid == true && raw->obs.range[i].ValidCom == true)
			{
				if (raw->obs.range[i].Sys == GPS)
				{
					//计算Range的IF组合观测值
					raw->obs.com[i].IF = IF_GPS(raw->obs.range[i].P1, raw->obs.range[i].P2);
					int prn = raw->obs.range[i].Prn;
					double row; //row是卫星和测站之间的距离
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
					//首先计算出IF消电离层组合观测值
					raw->obs.com[i].IF = IF_BDS(raw->obs.range[i].P1, raw->obs.range[i].P2);
					double TGD_IF; //IF组合的tgd改正
					TGD_IF = tgd1 * FB1_BDS * FB1_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS);
					double row; //row是卫星和测站之间的距离
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
		//按照每个观测值读到的卫星个数开始做循环
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
					//首先计算出IF消电离层组合观测值
					raw->obs.com[i].IF = IF_BDS(raw->obs.range[i].P1, raw->obs.range[i].P2);
					double TGD_IF; //IF组合的tgd改正
					TGD_IF = tgd1 * FB1_BDS * FB1_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS);
					double row; //row是卫星和测站之间的距离
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
		//按照每个观测值读到的卫星个数开始做循环
		for (int i = 0; i < raw->obs.SATNUMS; i++)
		{
			if (raw->obs.range[i].Valid == true && raw->obs.range[i].ValidCom == true)
			{
				if (raw->obs.range[i].Sys == GPS)
				{
					//计算Range的IF组合观测值
					raw->obs.com[i].IF = IF_GPS(raw->obs.range[i].P1, raw->obs.range[i].P2);
					int prn = raw->obs.range[i].Prn;
					double row; //row是卫星和测站之间的距离
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
fillP_SPP 用于SPP解算中P矩阵的计算 （等权，单位阵）
输入：观测值可用的个数
输出： P（数组）
已测试 2021.12.29
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
fillBl_SPV 用于SPV解算中B，l矩阵的计算
输入：原始数据RAWDATA的指针raw，
输出： B和l（数组）,返回观测值可用的个数
未测试 2021.12.29
**************************************************************************/
double fillBl_SPV(RAWDATA* raw, double* B, double* l)
{
	int num = 0;
	for (int i = 0; i < raw->obs.SATNUMS; i++)
	{
		// 对于每一个可以用的range（这个在前面已经判断过）
		// 意义为该range已经被读出来并且所对应的卫星位置已经被计算出来 并且高度角大于15度
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
fillP_SPV 用于SPV解算中P矩阵的计算 （等权，单位阵）
输入：观测值可用的个数
输出： P（数组）
已测试 2021.12.29
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
SPP 标准单点定位位置计算函数
输入：原始数据RAWDATA的指针raw
输出：
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

	int flag; //用来判断卫星系统是GPS还是BDS
	double ele = 0; //卫星的高度角(角度）
	double ele_rad;//卫星的高度角（弧度）

	//计算卫星位置
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
				// 如果这时候的Prn是0 则把该range的valid设为false
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

	//进行粗差探测
	DetectOutlier(raw);

	//判断星历的数量是否足够
	if (invalidObs == raw->obs.SATNUMS)
	{
		printf("Ephemeris not enough!\n");
		return 0;
	}

	//计算钟差和对流层延迟
	for (int i = 0; i < raw->obs.SATNUMS; i++)
	{
		if (raw->obs.range[i].Valid)
		{
			int prn = raw->obs.range[i].Prn;
			//int flag; //用来判断卫星系统是GPS还是BDS
			//double ele; //卫星的高度角(角度）
			//double ele_rad;//卫星的高度角（弧度）
			if (raw->obs.range[i].Sys == GPS)
			{
				//看有哪些卫星的观测值和卫星位置同时被计算出来
				if (raw->sat_gps[prn - 1].Valid == true)
				{
					flag = 0;
					calNEU_AE(raw, prn, flag);                        //计算出测站的NEU和卫星的高度角和方位角
					ele_rad = raw->sat_gps[prn - 1].eleAngle;
					ele = raw->sat_gps[prn - 1].eleAngle * 180 / PI;  //这里的高度角转化为弧度
					//剔除高度角小于15的卫星
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
					calNEU_AE(raw, prn, flag);                        //计算出测站的NEU和卫星的高度角和方位角
					ele_rad = raw->sat_bds[prn - 1].eleAngle;
					ele = raw->sat_bds[prn - 1].eleAngle * 180 / PI;  //这里的高度角转化为弧度
					//剔除高度角小于15的卫星
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

	//设置迭代次数
	int iterator = 0;
	double converage = 0;

	int system_choose;                      // 解算模式 0为双系统 1为单GPS系统 2为单北斗系统
	int numSat = 0, numBDS = 0, numGPS = 0; //分别计算出可用的卫星，可用的GPS卫星数以及可用的BDS卫星数

	/*
	 RefPos[3] = { -2267804.5263, 5009342.3723 , 3220991.8632 };
	*/

	//信噪比的选择
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

	// 选择卫星观测数据完整的rangedata
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

	// 进行可以进行什么解算方式的判断
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
	//开始SPP解算
	do
	{
		//当条件满足双系统解算时：
		if (system_choose == 0)
		{
			//设最小二乘计算的初值
			double X_DATA[5] = { raw->receiver.xyz.x ,raw->receiver.xyz.y,raw->receiver.xyz.z,0, 0 };
			//double X_DATA[5] = { raw->receiver.xyz.x ,raw->receiver.xyz.y,raw->receiver.xyz.z,raw->receiver.dt_GPS,raw->receiver.dt_BDS };
			//给初值的各种矩阵赋初值
			//需要注意的是：由于矩阵是用一维数组定义的，所以在这里只要定义可能达到的最大值就可以了
			double B_DATA[5 * MAXCHANNUM] = { 0 };
			double B_Transpose_DATA[5 * MAXCHANNUM] = { 0 };
			double l_DATA[MAXCHANNUM] = { 0 };
			double P_DATA[MAXCHANNUM * MAXCHANNUM] = { 0 };
			SPPSTATE state = DOUBLESYS;
			int size = fillBl_SPP(raw, state, B_DATA, l_DATA);
			fillP_spp(size, P_DATA,raw);

			//B矩阵的初始化
			matrix* B = new matrix;
			MatrixInit(B, size, 5, B_DATA);

			//l矩阵的初始化
			matrix* l = new matrix;
			MatrixInit(l, size, 1, l_DATA);

			//P矩阵的初始化
			matrix* P = new matrix;
			MatrixInit(P, size, size, P_DATA);

			//计算B的转置
			matrix* B_trans = new matrix;
			MatrixTrans(B_trans, B, B_Transpose_DATA);

			//计算BTPB
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

			//计算BTPB的逆矩阵
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

			//计算BTPl
			double BTPl_DATA[5] = { 0 };
			matrix* BTPl = new matrix;
			MatrixMulti(BTP, l, BTPl, BTPl_DATA);

			//计算出残差矩阵
			double v_DATA[5] = { 0 };
			matrix* v = new matrix;
			MatrixMulti(BTPB_INV, BTPl, v, v_DATA);

			//计算V矩阵（V=Bv-l）
			double V_DATA[MAXCHANNUM] = { 0 };
			matrix* V = new matrix;
			double Bv_DATA[MAXCHANNUM] = { 0 };
			matrix* Bv = new matrix;
			MatrixMulti(B, v, Bv, Bv_DATA);
			MatrixMinus(V, Bv, l, V_DATA);

			//计算VtPv
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

		//当条件满足单GPS解算时：
		//改了一个SPPstate = SINGLEGPS 初值里面没有BDS的钟差 最后sigma0除的值为3 一些矩阵的维度从5变成了4
		if (system_choose == 1)
		{
			//设最小二乘计算的初值
			double X_DATA[4] = { raw->receiver.xyz.x ,raw->receiver.xyz.y,raw->receiver.xyz.z,0 };
			//double X_DATA[4] = { raw->receiver.xyz.x ,raw->receiver.xyz.y,raw->receiver.xyz.z,raw->receiver.dt_GPS };
			//给初值的各种矩阵赋初值
			//需要注意的是：由于矩阵是用一维数组定义的，所以在这里只要定义可能达到的最大值就可以了
			double B_DATA[4 * MAXCHANNUM] = { 0 };
			double B_Transpose_DATA[4 * MAXCHANNUM] = { 0 };
			double l_DATA[MAXCHANNUM] = { 0 };
			double P_DATA[MAXCHANNUM * MAXCHANNUM] = { 0 };
			SPPSTATE state = SINGLEGPS;
			int size = fillBl_SPP(raw, state, B_DATA, l_DATA);
			fillP_spp(size, P_DATA,raw);

			//B矩阵的初始化
			matrix* B = new matrix;
			MatrixInit(B, size, 4, B_DATA);

			//l矩阵的初始化
			matrix* l = new matrix;
			MatrixInit(l, size, 1, l_DATA);

			//P矩阵的初始化
			matrix* P = new matrix;
			MatrixInit(P, size, size, P_DATA);

			//计算B的转置
			matrix* B_trans = new matrix;
			MatrixTrans(B_trans, B, B_Transpose_DATA);

			//计算BTPB
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

			//计算BTPB的逆矩阵
			double BTPB_INV_DATA[25] = { 0 };
			matrix* BTPB_INV = new matrix;
			MatrixInv(BTPB_INV, BTPB, BTPB_INV_DATA);
			double PDOP;
			PDOP = sqrt(BTPB_INV_DATA[0] + BTPB_INV_DATA[6] + BTPB_INV_DATA[12]);

			//计算BTPl
			double BTPl_DATA[5] = { 0 };
			matrix* BTPl = new matrix;
			MatrixMulti(BTP, l, BTPl, BTPl_DATA);

			//计算出残差矩阵
			double v_DATA[5] = { 0 };
			matrix* v = new matrix;
			MatrixMulti(BTPB_INV, BTPl, v, v_DATA);

			//计算V矩阵（V=Bv-l）
			double V_DATA[MAXCHANNUM] = { 0 };
			matrix* V = new matrix;
			double Bv_DATA[MAXCHANNUM] = { 0 };
			matrix* Bv = new matrix;
			MatrixMulti(B, v, Bv, Bv_DATA);
			MatrixMinus(V, Bv, l, V_DATA);

			//计算VtPv
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

		//当条件满足单BDS解算时：
		//改了一个SPPstate = SINGLEBDS 初值里面没有GPS的钟差 最后sigma0除的值为3 一些矩阵的维度从5变成了4
		if (system_choose == 2)
		{
			//设最小二乘计算的初值
			double X_DATA[4] = { raw->receiver.xyz.x ,raw->receiver.xyz.y,raw->receiver.xyz.z,0 };
			//double X_DATA[4] = { raw->receiver.xyz.x ,raw->receiver.xyz.y,raw->receiver.xyz.z,raw->receiver.dt_BDS };
			//给初值的各种矩阵赋初值
			//需要注意的是：由于矩阵是用一维数组定义的，所以在这里只要定义可能达到的最大值就可以了
			double B_DATA[4 * MAXCHANNUM] = { 0 };
			double B_Transpose_DATA[4 * MAXCHANNUM] = { 0 };
			double l_DATA[MAXCHANNUM] = { 0 };
			double P_DATA[MAXCHANNUM * MAXCHANNUM] = { 0 };
			SPPSTATE state = SINGLEBDS;
			int size = fillBl_SPP(raw, state, B_DATA, l_DATA);
			fillP_spp(size, P_DATA,raw);

			//B矩阵的初始化
			matrix* B = new matrix;
			MatrixInit(B, size, 4, B_DATA);


			//l矩阵的初始化
			matrix* l = new matrix;
			MatrixInit(l, size, 1, l_DATA);


			//P矩阵的初始化
			matrix* P = new matrix;
			MatrixInit(P, size, size, P_DATA);


			//计算B的转置
			matrix* B_trans = new matrix;
			MatrixTrans(B_trans, B, B_Transpose_DATA);

			//计算BTPB
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

			//计算BTPB的逆矩阵
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

			//计算BTPl
			double BTPl_DATA[5] = { 0 };
			matrix* BTPl = new matrix;
			MatrixMulti(BTP, l, BTPl, BTPl_DATA);

			//计算出残差矩阵
			double v_DATA[5] = { 0 };
			matrix* v = new matrix;
			MatrixMulti(BTPB_INV, BTPl, v, v_DATA);

			//计算V矩阵（V=Bv-l）
			double V_DATA[MAXCHANNUM] = { 0 };
			matrix* V = new matrix;
			double Bv_DATA[MAXCHANNUM] = { 0 };
			matrix* Bv = new matrix;
			MatrixMulti(B, v, Bv, Bv_DATA);
			MatrixMinus(V, Bv, l, V_DATA);

			//计算VtPv
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
SPV 标准单点定位速度计算函数
输入：原始数据RAWDATA的指针raw
输出：
**************************************************************************/
int SPV(RAWDATA* raw)
{
	int flag = 0;
	// SPP进行成功则进行SPV解算
	if (raw->SPPstate)
	{
		double B_DATA[4 * MAXCHANNUM] = { 0 };
		double B_Transpose_DATA[4 * MAXCHANNUM] = { 0 };
		double l_DATA[MAXCHANNUM] = { 0 };
		double P_DATA[MAXCHANNUM * MAXCHANNUM] = { 0 };
		int size = fillBl_SPV(raw, B_DATA, l_DATA);
		//若可用观测数小于4 则size返回 -1
		if (size == 0)
		{
			//printf("Error in SPV! Satellite not enough!33\n");
			return -1;
		}
		fillP_spv(size, P_DATA,raw);

		//B矩阵的初始化
		matrix* B = new matrix;
		MatrixInit(B, size, 4, B_DATA);


		//l矩阵的初始化
		matrix* l = new matrix;
		MatrixInit(l, size, 1, l_DATA);


		//P矩阵的初始化
		matrix* P = new matrix;
		MatrixInit(P, size, size, P_DATA);


		//计算B的转置
		matrix* B_trans = new matrix;
		MatrixTrans(B_trans, B, B_Transpose_DATA);

		//计算BTPB
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

		//计算BTPB的逆矩阵
		double BTPB_INV_DATA[16] = { 0 };
		matrix* BTPB_INV = new matrix;
		MatrixInv(BTPB_INV, BTPB, BTPB_INV_DATA);
		double PDOP;
		PDOP = sqrt(BTPB_INV_DATA[0] + BTPB_INV_DATA[6] + BTPB_INV_DATA[12]);

		//计算BTPl
		double BTPl_DATA[4] = { 0 };
		matrix* BTPl = new matrix;
		MatrixMulti(BTP, l, BTPl, BTPl_DATA);

		//计算出残差矩阵
		double v_DATA[4] = { 0 };
		matrix* v = new matrix;
		MatrixMulti(BTPB_INV, BTPl, v, v_DATA);

		raw->receiver.vel[0] = v_DATA[0];
		raw->receiver.vel[1] = v_DATA[1];
		raw->receiver.vel[2] = v_DATA[2];

		//结果输出
	/*	cout << setprecision(8) << setiosflags(ios::fixed) << " " << raw->receiver.vel[0] << " " << raw->receiver.vel[1] << " " << raw->receiver.vel[2] << " ";
		cout << setprecision(4) << setiosflags(ios::fixed) << raw->receiver.neu.N << " " << raw->receiver.neu.E << " ";
	*/	if (flag == 0)
	{
		cout << raw->receiver.neu.U / 2 << endl;
	}
	return 0;
	}
	// 若SPP没有成功进行解算则直接返回-1
	else
	{
		return -1;
	}

}