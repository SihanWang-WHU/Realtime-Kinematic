#define _CRT_SECURE_NO_WARNINGS
#include"RTK.h"
#include<cmath>

using namespace std;

/*************************************************************************
时间匹配 TimeMatching
输入：原始数据RawBase、RawRove
作用：移动时间，将基准站与流动站时间聚拢
1文件读取完毕；0可进行下一步操作
**************************************************************************/
int TimeMatching(RAWDATA* RawBase, RAWDATA* RawRove, FILE* fpBase, FILE* fpRove)
{
	double BaseTime = RawBase->obs.gpst.Week * 604800.0 + RawBase->obs.gpst.SecofWeek;
	double RoveTime = RawRove->obs.gpst.Week * 604800.0 + RawRove->obs.gpst.SecofWeek;
	int ValBase, ValRove;
	ValBase = 1; ValRove = 1;
	while (abs(BaseTime - RoveTime) > 0.001)// 时间间隔大于0.001
	{
		if (BaseTime < RoveTime)// Base时间在前
		{
			ValBase = DecodeHOEM7(fpBase, RawBase);
		}
		else
		{
			ValRove = DecodeHOEM7(fpRove, RawRove);
		}
		BaseTime = RawBase->obs.gpst.Week * 604800.0 + RawBase->obs.gpst.SecofWeek;
		RoveTime = RawRove->obs.gpst.Week * 604800.0 + RawRove->obs.gpst.SecofWeek;
		if (ValBase == 0 || ValRove == 0)// 文件读完
		{
			return 1;
		}
	}
	return 0;
}


/*************************************************************************
原始数据数组的时间选取 RawGroupDataSelect
输入：原始数据数组RawBaseGroup、RawBaseGroup（基准、流动）；原始数据RawBase、RawRove
作用：在时间阈值范围内进行匹配选取
0对齐成功；-1对齐失败
**************************************************************************/
int RawGroupDataSelect(RAWDATA* RawBaseGroup, RAWDATA* RawRoveGroup, RAWDATA* RawBase, RAWDATA* RawRove,
	GPSTIME* Time, int ValBase, int ValRove)
{
	double GPST = Time->Week * 604800.0 + Time->SecofWeek;// 已经解算的GPST
	// 数据存储器更新
	if (ValBase == 43)// 读取的是观测值
	{
		RawBaseGroup[0] = RawBaseGroup[1];// 后往前推进 1-4赋值给0-3
		RawBaseGroup[1] = RawBaseGroup[2];
		RawBaseGroup[2] = RawBaseGroup[3];
		RawBaseGroup[3] = RawBaseGroup[4];
		memcpy(RawBaseGroup + 4, &RawBase->obs, sizeof(EPOCHOBSDATA));// 新进数据加入数组
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
			if (abs(differ) < 0.001 && TimeRove > GPST && GPST > 0)// 时间更新了
			{
				memcpy(&RawBase->obs, RawBaseGroup + i, sizeof(EPOCHOBSDATA));
				memcpy(&RawRove->obs, RawRoveGroup + j, sizeof(EPOCHOBSDATA));
				memcpy(Time, &RawBaseGroup[i].obs.gpst, sizeof(GPSTIME));// 确定最终时间
				return 0;// 时间对准成功
			}
		}
	}
	return -1;// 时间对准失败
}

/*************************************************************************
SelectComSats 进行共视卫星的选取
输入：两个站同步后的rawdata
输出：计算出的共视卫星等
**************************************************************************/
void SelectComSats(RAWDATA* rawB, RAWDATA* rawR)
{
	rawR->rtk.numBDS = 0;
	rawR->rtk.numGPS = 0;
	int indexB, indexR;
	// 如果SPP解算成功，则开始共视卫星的选取
	for (int i = 0; i < MAXBDSPRN; i++)
	{
		rawB->sat_bds[i - 1].comobs = false;
		rawR->sat_bds[i - 1].comobs = false;
		// 这里的Valid同时代表：该卫星的星历完整；该卫星的位置被成功计算出来；该卫星的高度角核验合格
		if (rawB->sat_bds[i - 1].Valid == true && rawR->sat_bds[i - 1].Valid == true)
		{
			// i就是prn号
			indexR = FindSatObsIndex(i, BDS, &rawR->obs);
			indexB = FindSatObsIndex(i, BDS, &rawB->obs);
			// 这里的Valid代表信噪比,观测值完善并且粗差探测合格
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
		// 这里的Valid同时代表：该卫星的星历完整；该卫星的位置被成功计算出来；该卫星的高度角核验合格
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
SelectRefSats 进行GPS和BDS系统共视卫星的选取
输入：两个选取完共视卫星之后的rawdata； sys判断卫星系统类型：1是北斗系统，0是GPS系统
输出：选取的参考卫星
**************************************************************************/
int SelectRefSats(RAWDATA* rawR, int sys)
{

	// 参考星的选取原则是：高度角最高且如果是北斗系统，不是GEO卫星
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
				// 如果该卫星的高度角就是最大的，则将Ref判断值设置为true并且return该卫星的Prn号
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
calculatesigma2 求取卫星的非差方差分量
输入：卫星的高度角eleangle(弧度制）
输出：求出的非差方差分量
**************************************************************************/
double calculatesigma2(double eleangle)
{
	double sigma2;
	sigma2 = pow (0.001, 2) * (1 + 1.5 * cos(eleangle) * cos(eleangle));
	//sigma2 = 16 + 9 / pow(sin(eleangle), 2);
	return sigma2;
}

/*************************************************************************
fill_A_L_DOUBLE_DIFF 求取双频双差短基线动态RTK的函数模型的B和L矩阵
输入：两个选取完共视卫星之后的rawdata； 参考星的PRN号， B矩阵和L矩阵的数据保存数组
	  共视卫星的数量numGPS和numBDS， RTK解算的模式state(0,1,2)
输出：B矩阵和L矩阵的数据在这个函数中进行改变
**************************************************************************/
void fill_A_L_DOUBLE_DIFF(int prnG, int prnB, int numGPS, int numBDS, RAWDATA* rawB, 
	RAWDATA* rawR, double* B, double* L, int state)
{
	// 基准站的参考坐标
	double refx = -2267804.5263;
	double refy = 5009342.3723;
	double refz = 3220991.8632;
	XYZ refxyz;
	refxyz.x = refx;
	refxyz.y = refy;
	refxyz.z = refz;
	double p_k_B, p_j_B, p_k_A, p_j_A;// 卫地距
	/**************************当前历元的GPS系统的各个参数**************************/
	// 用该方法求出来之后，确保在每个共视卫星的索引下，单差和双差的数组内容都是对应的单/双差
	// 求出来的单差/双差都保存在rawR下面
	// 这个循环用来遍历一边GPS的所有卫星并且做出单差
	for (int i = 0; i < MAXGPSPRN; i++)
	{
		// 首先判断如果是共视卫星，则做差
		if (rawR->sat_gps[i - 1].comobs == true)
		{
			// 首先找出该卫星在range数据中的索引
			// 为了防止在rawB和rawR中这两个索引不一样，在这里设置了两个int型的索引
			int indexB, indexR;
			indexB = FindSatObsIndex(i, GPS, &rawB->obs);
			indexR = FindSatObsIndex(i, GPS, &rawR->obs);
			// 首先做单差
			// 单差是共视卫星对于移动站的相位观测值减去对于基准站的相位观测值
			// fai_d1和fai_d2的索引都是PRN-1
			rawR->receiver.Gfai_d1_f1[i - 1] = rawR->obs.range[indexR].L1 - rawB->obs.range[indexB].L1;
			rawR->receiver.Gfai_d1_f2[i - 1] = rawR->obs.range[indexR].L2 - rawB->obs.range[indexB].L2;
			rawR->receiver.Gp_d1_f1[i - 1] = rawR->obs.range[indexR].P1 - rawB->obs.range[indexB].P1;
			rawR->receiver.Gp_d1_f2[i - 1] = rawR->obs.range[indexR].P2 - rawB->obs.range[indexB].P2;
		}
	}

	// 这个循环用来遍历一边GPS的所有卫星并且做出双差
	for (int i = 0; i < MAXGPSPRN; i++)
	{
		if (rawR->sat_gps[i - 1].comobs == true)
		{
			double dis, disref;
			// 参考星和其他卫星相对于流动站的距离
			dis = distance_calculate(rawR->sat_gps[i - 1].satXYZ, rawR->receiver.xyz);
			disref = distance_calculate(rawR->sat_gps[prnG - 1].satXYZ, rawR->receiver.xyz);
			// 只有不是基准星的部分需要做双差
			if (rawR->sat_gps[i - 1].RefSat == false)
			{
				// 这里的索引是PRN -1 
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


	/**************************当前历元的BDS系统的各个参数**************************/
	for (int i = 0; i < MAXBDSPRN; i++)
	{
		// 首先判断如果是共视卫星，则做差
		if (rawR->sat_bds[i - 1].comobs == true)
		{
			// 首先找出该卫星在range数据中的索引
			// 为了防止在rawB和rawR中这两个索引不一样，在这里设置了两个int型的索引
			int indexB, indexR;
			indexB = FindSatObsIndex(i, BDS, &rawB->obs);
			indexR = FindSatObsIndex(i, BDS, &rawR->obs);
			// 首先做单差
			// 单差是共视卫星对于移动站的相位观测值减去对于基准站的相位观测值
			// fai_d1和fai_d2的索引都是PRN-1
			rawR->receiver.Bfai_d1_f1[i - 1] = rawR->obs.range[indexR].L1 - rawB->obs.range[indexB].L1;
			rawR->receiver.Bp_d1_f1[i - 1] = rawR->obs.range[indexR].P1 - rawB->obs.range[indexB].P1;
		}
	}

	// 这个循环用来遍历一边BDS的所有卫星并且做出双差
	for (int i = 0; i < MAXBDSPRN; i++)
	{
		if (rawR->sat_bds[i - 1].comobs == true)
		{
			double dis, disref;
			// 参考星和其他卫星相对于流动站的距离
			dis = distance_calculate(rawR->sat_bds[i - 1].satXYZ, rawR->receiver.xyz);
			disref = distance_calculate(rawR->sat_bds[prnB - 1].satXYZ, rawR->receiver.xyz);
			// 只有不是基准星的部分需要做双差
			if (rawR->sat_bds[i - 1].RefSat == false)
			{
				// 这里的索引是PRN -1 
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

	/**************************开始填充B，L矩阵**************************/
	double Gp_RefSat_Rove, Gp_Sat_Rove, Gp_RefSat_Base, Gp_Sat_Base;// 卫地距
	double Bp_RefSat_Rove, Bp_Sat_Rove, Bp_RefSat_Base, Bp_Sat_Base;// 卫地距
	if (state == 0) // 双系统解算
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
		/**************************L矩阵的填充**************************/
		int indexGL = 0;
		int indexBL = 0; // 从0开始索引L数组

		// 先填充GPS数据
		// 这里的L矩阵的顺序是：
		// GPS: F1频率的相位双差，F2频率的相位双差， F1频率的伪距双差，F2频率的伪距双差
		// BDS: B1I频率的相位双差，B3I频率的相位双差， B1I频率的伪距双差
		
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			Gp_RefSat_Rove = distance_calculate(rawR->sat_gps[prnG - 1].satXYZ, rawR->receiver.xyz);
			Gp_RefSat_Base = distance_calculate(rawB->sat_gps[prnbaseG - 1].satXYZ, refxyz);
			// 选取共视卫星中不是参考卫星的，此时i=prn
			if (rawR->sat_gps[i - 1].comobs == true && rawR->sat_gps[i - 1].RefSat == false)
			{
				Gp_Sat_Rove = distance_calculate(rawR->sat_gps[i - 1].satXYZ, rawR->receiver.xyz);
				Gp_Sat_Base = distance_calculate(rawB->sat_gps[i - 1].satXYZ, refxyz);
				double dif_dis = -Gp_Sat_Rove + Gp_RefSat_Rove + Gp_Sat_Base - Gp_RefSat_Base;
				// 每一次增加的都是numsat - 1 个， 因为没有参考星
				L[indexGL] = rawR->receiver.Gfai_d2_f1[i - 1] + dif_dis;
				L[indexGL + numGPS - 1] = rawR->receiver.Gfai_d2_f2[i - 1] + dif_dis;
				L[indexGL + 2 * numGPS - 2] = rawR->receiver.Gp_d2_f1[i - 1] + dif_dis;
				L[indexGL + 3 * numGPS - 3] = rawR->receiver.Gp_d2_f2[i - 1] + dif_dis;
				indexGL++;
			}
		}
		// 再填充BDS数据
		for (int i = 0; i < MAXBDSPRN; i++)
		{
			Bp_RefSat_Rove = distance_calculate(rawR->sat_bds[prnB - 1].satXYZ, rawR->receiver.xyz);
			Bp_RefSat_Base = distance_calculate(rawB->sat_bds[prnbaseB - 1].satXYZ, refxyz);
			// 选取共视卫星中不是参考卫星的，此时i=prn
			if (rawR->sat_bds[i - 1].comobs == true && rawR->sat_bds[i - 1].RefSat == false)
			{
				Bp_Sat_Rove = distance_calculate(rawR->sat_bds[i - 1].satXYZ, rawR->receiver.xyz);
				Bp_Sat_Base = distance_calculate(rawB->sat_bds[i - 1].satXYZ, refxyz);
				double dif_dis = -Bp_Sat_Rove + Bp_RefSat_Rove + Bp_Sat_Base - Bp_RefSat_Base;
				// 每一次增加的都是numsat - 1 个， 因为没有参考星
				L[4 * numGPS - 4 + indexBL] = rawR->receiver.Bfai_d2_f1[i - 1] + dif_dis;
				L[4 * numGPS - 4 + indexBL + numBDS - 1] = rawR->receiver.Bp_d2_f1[i - 1] + dif_dis;
				indexBL++;
			}
		}

		/**************************B矩阵的填充**************************/
		// 首先确定B矩阵的行数和列数：
		// B矩阵的行数是：4nG - 4 + 2nB - 2
		// B矩阵的列数是：3 + 2nG - 2 + nB - 1 ( 2nG + nB )
		int rowB = 4 * numGPS + 2 * numBDS - 2;
		int columnB = numBDS + 2 * numGPS;
		int indexGB = 0;
		int indexBB = 0;
		// 开始填充GPS数据
		// B矩阵对于数据填充的顺序和L矩阵是一样的
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			// 这个循环读取的次数是 numGPS -1
			// indexGB 每加一次 就代表读取进来一颗卫星
			if (rawR->sat_gps[i - 1].comobs == true && rawR->sat_gps[i - 1].RefSat == false)
			{
				// 这一部分是F1的相位的部分
				B[indexGB * columnB] = rawR->receiver.Gax[i - 1];
				B[indexGB * columnB + 1] = rawR->receiver.Gay[i - 1];
				B[indexGB * columnB + 2] = rawR->receiver.Gaz[i - 1];
				B[3 + indexGB * columnB + indexGB] = WL1_GPS;

				// 这一部分是F2的相位的部分
				B[(numGPS - 1) * columnB + indexGB * columnB] = rawR->receiver.Gax[i - 1];
				B[(numGPS - 1) * columnB + indexGB * columnB + 1] = rawR->receiver.Gay[i - 1];
				B[(numGPS - 1) * columnB + indexGB * columnB + 2] = rawR->receiver.Gaz[i - 1];
				B[(numGPS - 1) * columnB + indexGB * columnB + 3 + (numGPS - 1) + indexGB] = WL2_GPS;

				// 这一部分是F1的伪距部分
				// 伪距部分后面没有Wave length 而都是零
				B[2 * (numGPS - 1) * columnB + indexGB * columnB] = rawR->receiver.Gax[i - 1];
				B[2 * (numGPS - 1) * columnB + indexGB * columnB + 1] = rawR->receiver.Gay[i - 1];
				B[2 * (numGPS - 1) * columnB + indexGB * columnB + 2] = rawR->receiver.Gaz[i - 1];

				// 这一部分是F2的伪距部分
				B[3 * (numGPS - 1) * columnB + indexGB * columnB] = rawR->receiver.Gax[i - 1];
				B[3 * (numGPS - 1) * columnB + indexGB * columnB + 1] = rawR->receiver.Gay[i - 1];
				B[3 * (numGPS - 1) * columnB + indexGB * columnB + 2] = rawR->receiver.Gaz[i - 1];

				// 结束后加一颗读取的卫星
				indexGB++;
			}
		}

		for (int i = 0; i < MAXBDSPRN; i++)
		{
			if (rawR->sat_bds[i - 1].comobs == true && rawR->sat_bds[i - 1].RefSat == false)
			{
				// 该部分是F1的相位部分
				B[4 * (numGPS - 1) * columnB + indexBB * columnB] = rawR->receiver.Bax[i - 1];
				B[4 * (numGPS - 1) * columnB + indexBB * columnB + 1] = rawR->receiver.Bay[i - 1];
				B[4 * (numGPS - 1) * columnB + indexBB * columnB + 2] = rawR->receiver.Baz[i - 1];
				B[4 * (numGPS - 1) * columnB + indexBB * columnB + 3 + 2 * (numGPS - 1) + indexBB] = WB1_BDS;

				// 这部分是F1的伪距部分
				B[(4 * (numGPS - 1) + (numBDS - 1)) * columnB + indexBB * columnB] = rawR->receiver.Bax[i - 1];
				B[(4 * (numGPS - 1) + (numBDS - 1)) * columnB + indexBB * columnB + 1] = rawR->receiver.Bay[i - 1];
				B[(4 * (numGPS - 1) + (numBDS - 1)) * columnB + indexBB * columnB + 2] = rawR->receiver.Baz[i - 1];

				// 结束后加一颗读取的卫星
				indexBB++;

			}
		}
	}
	// GPS单系统解算
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

		/**************************L矩阵的填充**************************/
		int indexGL = 0;

		double L1_AB_jk, L2_AB_jk, P1_AB_jk, P2_AB_jk;
		int PRN_B_k, PRN_B_j, PRN_A_k, PRN_A_j;
		// 先填充GPS数据
		// 这里的L矩阵的顺序是：
		// GPS: F1频率的相位双差，F2频率的相位双差， F1频率的伪距双差，F2频率的伪距双差
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			p_j_B = distance_calculate(rawR->sat_gps[prnG - 1].satXYZ, rawR->receiver.xyz);
			p_j_A = distance_calculate(rawB->sat_gps[prnbaseG - 1].satXYZ, refxyz);
			Gp_RefSat_Rove = distance_calculate(rawR->sat_gps[prnG - 1].satXYZ, rawR->receiver.xyz);
			Gp_RefSat_Base = distance_calculate(rawB->sat_gps[prnbaseG - 1].satXYZ, refxyz);
			// 选取共视卫星中不是参考卫星的，此时i=prn
			if (rawR->sat_gps[i - 1].comobs == true && rawR->sat_gps[i - 1].RefSat == false)
			{
				p_k_B = distance_calculate(rawR->sat_gps[i - 1].satXYZ, rawR->receiver.xyz);
				p_k_A = distance_calculate(rawB->sat_gps[i - 1].satXYZ, refxyz);
				Gp_Sat_Rove = distance_calculate(rawR->sat_gps[i - 1].satXYZ, rawR->receiver.xyz);
				Gp_Sat_Base = distance_calculate(rawB->sat_gps[i - 1].satXYZ, refxyz);
				double dif_dis = -Gp_Sat_Rove + Gp_RefSat_Rove + Gp_Sat_Base - Gp_RefSat_Base;
				//// 每一次增加的都是numsat - 1 个， 因为没有参考星

				L[indexGL] =  rawR->receiver.Gfai_d2_f1[i - 1] + dif_dis;
				L[indexGL + numGPS - 1] = rawR->receiver.Gfai_d2_f2[i - 1] + dif_dis;
				L[indexGL + 2 * numGPS - 2] = rawR->receiver.Gp_d2_f1[i - 1] + dif_dis;
				L[indexGL + 3 * numGPS - 3] = rawR->receiver.Gp_d2_f2[i - 1] + dif_dis;
				indexGL++; 
			}
		}


		/**************************B矩阵的填充**************************/
		// 首先确定B矩阵的行数和列数：
		// B矩阵的行数是：4nG - 4 + 4nB - 4
		// B矩阵的列数是：3 + 2nG - 2 + 2nB - 2 ( 2nG + 2nB - 1 )

		int indexGB = 0;
		// 开始填充GPS数据
		// B矩阵对于数据填充的顺序和L矩阵是一样的
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			// 这个循环读取的次数是 numGPS -1
			// indexGB 每加一次 就代表读取进来一颗卫星
			if (rawR->sat_gps[i - 1].comobs == true && rawR->sat_gps[i - 1].RefSat == false)
			{
				// 这一部分是F1的相位的部分
				B[indexGB * (2 * numGPS + 1)] = rawR->receiver.Gax[i - 1];
				B[indexGB * (2 * numGPS + 1) + 1] = rawR->receiver.Gay[i - 1];
				B[indexGB * (2 * numGPS + 1) + 2] = rawR->receiver.Gaz[i - 1];
				B[3 + indexGB * (2 * numGPS + 1) + indexGB] = WL1_GPS;

				// 这一部分是F2的相位的部分
				B[(numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1)] = rawR->receiver.Gax[i - 1];
				B[(numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 1] = rawR->receiver.Gay[i - 1];
				B[(numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 2] = rawR->receiver.Gaz[i - 1];
				B[(numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 3 + (numGPS - 1) + indexGB] = WL2_GPS;

				// 这一部分是F1的伪距部分
				// 伪距部分后面没有Wave length 而都是零
				B[2 * (numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1)] = rawR->receiver.Gax[i - 1];
				B[2 * (numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 1] = rawR->receiver.Gay[i - 1];
				B[2 * (numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 2] = rawR->receiver.Gaz[i - 1];

				// 这一部分是F2的伪距部分
				B[3 * (numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1)] = rawR->receiver.Gax[i - 1];
				B[3 * (numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 1] = rawR->receiver.Gay[i - 1];
				B[3 * (numGPS - 1) * (2 * numGPS + 1)
					+ indexGB * (2 * numGPS + 1) + 2] = rawR->receiver.Gaz[i - 1];

				// 结束后加一颗读取的卫星
				indexGB++;
			}
		}

	}
	// BDS单系统解算
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
		/**************************L矩阵的填充**************************/
		int indexBL = 0; // 从0开始索引L数组

		// 这里的L矩阵的顺序是：
		// BDS: B1I频率的相位双差，B1I频率的伪距双差
		// 填充BDS数据
		for (int i = 0; i < MAXBDSPRN; i++)
		{
			Bp_RefSat_Rove = distance_calculate(rawR->sat_bds[prnB - 1].satXYZ, rawR->receiver.xyz);
			Bp_RefSat_Base = distance_calculate(rawB->sat_bds[prnbaseB - 1].satXYZ, refxyz);
			// 选取共视卫星中不是参考卫星的，此时i=prn
			if (rawR->sat_bds[i - 1].comobs == true && rawR->sat_bds[i - 1].RefSat == false)
			{
				Bp_Sat_Rove = distance_calculate(rawR->sat_bds[i - 1].satXYZ, rawR->receiver.xyz);
				Bp_Sat_Base = distance_calculate(rawB->sat_bds[i - 1].satXYZ, refxyz);
				double dif_dis = -Bp_Sat_Rove + Bp_RefSat_Rove + Bp_Sat_Base - Bp_RefSat_Base;
				// 每一次增加的都是numsat - 1 个， 因为没有参考星
				L[indexBL] = rawR->receiver.Bfai_d2_f1[i - 1] + dif_dis;
				L[indexBL + numBDS - 1] = rawR->receiver.Bp_d2_f1[i - 1] + dif_dis;
				indexBL++;
			}
		}

		/**************************B矩阵的填充**************************/
		// 首先确定B矩阵的行数和列数：
		// B矩阵的行数是：2 * numBDS - 2 
		// B矩阵的列数是：3 + nB - 1 ( 2nG + 2nB - 1 )
		int columnB = 3 + numBDS - 1;
		int indexBB = 0;
		// 开始填充GPS数据
		// B矩阵对于数据填充的顺序和L矩阵是一样的

		for (int i = 0; i < MAXBDSPRN; i++)
		{
			if (rawR->sat_bds[i - 1].comobs == true && rawR->sat_bds[i - 1].RefSat == false)
			{
				// 该部分是F1的相位部分
				B[indexBB * columnB] = rawR->receiver.Bax[i - 1];
				B[indexBB * columnB + 1] = rawR->receiver.Bay[i - 1];
				B[indexBB * columnB + 2] = rawR->receiver.Baz[i - 1];
				B[indexBB * columnB + 3 + indexBB] = WB1_BDS;

				// 这部分是F1的伪距部分
				B[(numBDS - 1) * columnB + indexBB * columnB] = rawR->receiver.Bax[i - 1];
				B[(numBDS - 1) * columnB + indexBB * columnB + 1] = rawR->receiver.Bay[i - 1];
				B[(numBDS - 1) * columnB + indexBB * columnB + 2] = rawR->receiver.Baz[i - 1];

				// 结束后加一颗读取的卫星
				indexBB++;
			}
		}
	}
}

/*************************************************************************
fill_COV_DOUBLE_DIFF 求取双频双差短基线动态RTK的函数模型的方差阵
输入：两个选取完共视卫星之后的rawdata； 参考星的PRN号， 方差矩阵的数据保存数组COV
	  共视卫星的数量numGPS和numBDS， RTK解算的模式state(0,1,2)
输出：方差阵的数据在这个函数中进行改变
**************************************************************************/
void fill_COV_DOUBLE_DIFF(int prnG, int prnB, int numGPS, int numBDS, RAWDATA* rawB, 
	RAWDATA* rawR, double* COV, int state)
{
	/**************************当前历元内的单差方差阵数据的计算与存储**************************/
	// 在填充矩阵之前首先计算测站间的单差（计算出来是sigma2），存在Rove对象的保存测站数据的结构体中
	// 首先计算GPS的数据
	for (int i = 0; i < MAXGPSPRN; i++)
	{
		// 这里选取所有的共视卫星，包括基准星
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

	// 其次计算BDS的数据
	for (int i = 0; i < MAXBDSPRN; i++)
	{
		// 这里选取所有的共视卫星，包括基准星
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

	/**************************开始填充P矩阵**************************/
	// P矩阵的维度是 (4nG + 2nB - 6) * (4nG + 2nB - 6) 
	// 计算双系统的P矩阵
	if (state == 0)
	{
		int indexGP = 0;
		int indexBP = 0;
		// 首先填充GPS的部分
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			int column = 2 * numBDS + 4 * numGPS - 6;
			// 一共循环 NG - 1 次
			if (rawR->sat_gps[i - 1].comobs == true && rawR->sat_gps[i - 1].RefSat == false)
			{

				// 填充GL1的单差值
				// 首先将一块矩阵全部填充为基准星的单差
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[indexGP * column + j] = rawR->receiver.Gfai_d1_f1_sigma2[prnG - 1];
				}
				COV[indexGP * column + indexGP] = rawR->receiver.Gfai_d1_f1_sigma2[prnG - 1] + 1  * rawR->receiver.Gfai_d1_f1_sigma2[i - 1];

				// 填充GL2的单差值
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[(numGPS - 1 + indexGP) * column + (numGPS - 1) + j] = rawR->receiver.Gfai_d1_f2_sigma2[prnG - 1];
				}
				COV[(numGPS - 1 + indexGP) * column + (numGPS - 1 + indexGP)] = rawR->receiver.Gfai_d1_f2_sigma2[prnG - 1] + 1  * rawR->receiver.Gfai_d1_f2_sigma2[i - 1];

				// 填充GP1的单差值
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[(2 * numGPS - 2 + indexGP) * column + 2 * (numGPS - 1) + j] = rawR->receiver.Gp_d1_f1_sigma2[prnG - 1];
				}
				COV[(2 * numGPS - 2 + indexGP) * column + (2 * numGPS - 2 + indexGP)] = rawR->receiver.Gp_d1_f1_sigma2[prnG - 1] + 1  * rawR->receiver.Gp_d1_f1_sigma2[i - 1];

				// 填充GP2的单差值
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[(3 * numGPS - 3 + indexGP) * column + 3 * (numGPS - 1) + j] = rawR->receiver.Gp_d1_f2_sigma2[prnG - 1];
				}
				COV[(3 * numGPS - 3 + indexGP) * column + (3 * numGPS - 3 + indexGP)] = rawR->receiver.Gp_d1_f2_sigma2[prnG - 1] + 1  * rawR->receiver.Gp_d1_f2_sigma2[i - 1];

				indexGP++;
			}
		}

		// 其次填充BDS的部分
		for (int i = 0; i < MAXBDSPRN; i++)
		{
			int column = 2 * numBDS + 4 * numGPS - 6;
			if (rawR->sat_bds[i - 1].comobs == true && rawR->sat_bds[i - 1].RefSat == false)
			{
				// 填充BL1的单差值
				for (int j = 0; j < numBDS - 1; j++)
				{
					COV[(4 * numGPS - 4 + indexBP) * column + 4 * (numGPS - 1) + j] = rawR->receiver.Bfai_d1_f1_sigma2[prnB - 1];
				}
				COV[(4 * numGPS - 4 + indexBP) * column + 4 * (numGPS - 1) + indexBP] = rawR->receiver.Bfai_d1_f1_sigma2[prnB - 1] + rawR->receiver.Bfai_d1_f1_sigma2[i - 1];

				// 填充BP1的单差值
				for (int j = 0; j < numBDS - 1; j++)
				{
					COV[(4 * numGPS - 4 + numBDS - 1 + indexBP) * column + 4 * (numGPS - 1) + (numBDS - 1) + j] = rawR->receiver.Bp_d1_f1_sigma2[prnB - 1];
				}
				COV[(4 * numGPS - 4 + numBDS - 1 + indexBP) * column + 4 * (numGPS - 1) + (numBDS - 1) + indexBP] = rawR->receiver.Bp_d1_f1_sigma2[prnB - 1] + rawR->receiver.Bp_d1_f1_sigma2[i - 1];

				indexBP++;

			}
		}
	}
	// 计算单GPS系统的P矩阵
	else if (state == 1)
	{
		int indexGP = 0;
		// 首先填充GPS的部分
		for (int i = 0; i < MAXGPSPRN; i++)
		{
			int column = 4 * numGPS - 4;
			// 一共循环 NG - 1 次
			if (rawR->sat_gps[i - 1].comobs == true && rawR->sat_gps[i - 1].RefSat == false)
			{

				// 填充GL1的单差值
				// 首先将一块矩阵全部填充为基准星的单差
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[indexGP * column + j] = rawR->receiver.Gfai_d1_f1_sigma2[prnG - 1];
				}
				COV[indexGP * column + indexGP] = rawR->receiver.Gfai_d1_f1_sigma2[prnG - 1] + rawR->receiver.Gfai_d1_f1_sigma2[i - 1];

				// 填充GL2的单差值
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[(numGPS - 1 + indexGP) * column + (numGPS - 1) + j] = rawR->receiver.Gfai_d1_f2_sigma2[prnG - 1];
				}
				COV[(numGPS - 1 + indexGP) * column + (numGPS - 1 + indexGP)] = rawR->receiver.Gfai_d1_f2_sigma2[prnG - 1] + rawR->receiver.Gfai_d1_f2_sigma2[i - 1];

				// 填充GP1的单差值
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[(2 * numGPS - 2 + indexGP) * column + 2 * (numGPS - 1) + j] = rawR->receiver.Gp_d1_f1_sigma2[prnG - 1];
				}
				COV[(2 * numGPS - 2 + indexGP) * column + (2 * numGPS - 2 + indexGP)] = rawR->receiver.Gp_d1_f1_sigma2[prnG - 1] + rawR->receiver.Gp_d1_f1_sigma2[i - 1];

				// 填充GP2的单差值
				for (int j = 0; j < numGPS - 1; j++)
				{
					COV[(3 * numGPS - 3 + indexGP) * column + 3 * (numGPS - 1) + j] = rawR->receiver.Gp_d1_f2_sigma2[prnG - 1];
				}
				COV[(3 * numGPS - 3 + indexGP) * column + (3 * numGPS - 3 + indexGP)] = rawR->receiver.Gp_d1_f2_sigma2[prnG - 1] +  rawR->receiver.Gp_d1_f2_sigma2[i - 1];

				indexGP++;
			}
		}
	}
	// 计算单BDS系统的P矩阵
	else if (state == 2)
	{
		int indexBP = 0;
		// 填充BDS的部分
		for (int i = 0; i < MAXBDSPRN; i++)
		{
			int column = 2 * numBDS - 2;
			// 一共循环 NG - 1 次
			if (rawR->sat_bds[i - 1].comobs == true && rawR->sat_bds[i - 1].RefSat == false)
			{

				// 填充GL1的单差值
				// 首先将一块矩阵全部填充为基准星的单差
				for (int j = 0; j < numBDS - 1; j++)
				{
					COV[indexBP * column + j] = rawR->receiver.Bfai_d1_f1_sigma2[prnB - 1];
				}
				COV[indexBP * column + indexBP] = rawR->receiver.Bfai_d1_f1_sigma2[prnB - 1] + rawR->receiver.Bfai_d1_f1_sigma2[i - 1];

				// 填充GP1的单差值
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
SingleEpochRTK 相对定位的参数求解过程
输入：基准站和移动站的RAWDATA数据
计算：一个历元的RTK相对定位解算
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
	/******************************SPP解算和卫星的选取和处理******************************/
	SelectComSats(rawB, rawR);

	// 选取参考星
	int prnB, prnG;
	prnG = SelectRefSats(rawR, 0);
	prnB = SelectRefSats(rawR, 1);
	SelectRefSats(rawB, 0);
	SelectRefSats(rawB, 1);
	//cout << "PrnB=" << prnB << endl;
	//cout << "PrnG=" << prnG << endl;


	// 计算共视卫星的数目
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
	int state;  // RTK解算的类型
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

	// 计算基准星的个数
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
	// 若基准星数量大于1，则证明基准星数量选取错误
	if (RefGPS > 1 || RefBDS > 1)
	{
		cout << "基准星数量大于1！ 错误！" << endl;
		system("pause");
	}


	/*******************************各个矩阵维度的定义******************************/
	int rowB, columnB, rowL, dimensionP, rowV;
	int dimensionN;                       // 模糊度维数
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
	// 把SPP的原始结果保存一下：
	rawR->receiver.xyz_save = rawR->receiver.xyz;

	/*******************************各个矩阵数组的定义和内存分配******************************/
	// L矩阵应该是（4 * nB + 4 * nG - 8 ) * 1 的矩阵 在这里因为必须赋值一个常量，故这么写
	// X矩阵应该是 （2 * nB + 2 * nG - 1 ) * 1的矩阵 
	// 逆推可知 B矩阵应该是 （4 * nB + 4 * nG - 8 ) * （2 * nB + 2 * nG - 1 ) 的矩阵
	// P矩阵的维度是（2 * nB + 2 * nG - 1 ) * （2 * nB + 2 * nG - 1 )
	// COV_X 是NBB的逆 * sigma2 (columnB * columnB)
	// x_DATA 是初值 l = L - Bx（columnB * 1）
	// V = Bx - l, V矩阵是(rowB * 1)

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

	/*******************************浮点解的最小二乘计算******************************/
	double coverage = 1E-20;
	int iterator = 0;
	// 保存矩阵的文件的位置
	//FILE* fp;
	//fp = fopen("Matrix.txt", "at");
	memset(&rawR->rtk.xyzfloat, 0.0, sizeof(XYZ));
	do {
		iterator++;
		/**********     矩阵编排     **********/
		// 对于B矩阵和L矩阵进行填充
		fill_A_L_DOUBLE_DIFF(prnG, prnB, numGPS, numBDS, rawB, rawR, B_DATA, L_DATA, state);
		matrix* B = new matrix;
		MatrixInit(B, rowB, columnB, B_DATA);
		matrix* L = new matrix;
		MatrixInit(L, rowL, 1, L_DATA);

		// 对于P矩阵进行填充
		fill_COV_DOUBLE_DIFF(prnG, prnB, numGPS, numBDS, rawB, rawR, COV_DATA, state);
		matrix* COV = new matrix;
		matrix* P = new matrix;
		MatrixInit(COV, dimensionP, dimensionP, COV_DATA);
		memcpy(P_data, &COV_DATA, dimensionP * dimensionP * sizeof(double));
		matinv(P_data, dimensionP);
		MatrixInit(P, dimensionP, dimensionP, P_data);

		// X 矩阵的维度是 (2 * numGPS + 1) ,1 
		int rowx = columnB;
		// 小l矩阵和大L矩阵的维度相同
		int rowl = rowL;
		x_DATA[0] = rawR->receiver.xyz.x - refx;
		x_DATA[1] = rawR->receiver.xyz.y - refy;
		x_DATA[2] = rawR->receiver.xyz.z - refz;

		matrix* x = new matrix;
		MatrixInit(x, rowx, 1, x_DATA);

		/**********     最小二乘     **********/
		matrix* B_trans = new matrix;
		MatrixTrans(B_trans, B, BTrans_DATA);

		// 计算BTPB
		// BTPB的矩阵大小是columnB * columnB
		// columnB = 2 * numBDS - 1;
		matrix* BTPB = new matrix;
		// BTP的矩阵的大小是columnB * dimensionP
		matrix* BTP = new matrix;
		MatrixMulti(B_trans, P, BTP, BTP_DATA);
		MatrixMulti(BTP, B, BTPB, BTPB_DATA);

		// 计算BTPB的逆矩阵
		matrix* BTPB_INV = new matrix;
		memcpy(BTPBINV_data, BTPB_DATA, columnB * columnB * sizeof(double));
		matinv(BTPBINV_data, columnB);
		MatrixInit(BTPB_INV, columnB, columnB, BTPBINV_data);
		//MatrixInv(BTPB_INV, BTPB, BTPB_INV_DATA);

		// 计算BTPL
		// BTPl的维度是 columnB * 1
		matrix* BTPL = new matrix;
		MatrixMulti(BTP, L, BTPL, BTPl_DATA);

		// 计算出残差矩阵
		matrix* v = new matrix;
		MatrixMulti(BTPB_INV, BTPL, v, v_DATA);


		// 输出结果
		rawR->receiver.xyz.x += v_DATA[0];
		rawR->receiver.xyz.y += v_DATA[1];
		rawR->receiver.xyz.z += v_DATA[2];
		rawR->rtk.xyzfloat = rawR->receiver.xyz;

		// 保存计算出来的模糊度浮点解
		for (int n = 0; n < dimensionN; n++)
		{
			rawR->rtk.N_FLOAT[n] = v_DATA[n + 3];
		}
		rawR->rtk.rtkstate = FLOATSOLUTION;

		/**********     精度评定     **********/
		matrix* Bx = new matrix;
		matrix* V = new matrix;
		matrix* V_trans = new matrix;
		matrix* VTP = new matrix;
		matrix* VTPV = new matrix;
		matrix* D = new matrix;

		MatrixMulti(B, v, Bx, Bx_DATA);
		// V = Bx - L
		MatrixMinus(V, Bx, L, V_DATA);
		//计算VtPv
		MatrixTrans(V_trans, V, VT_DATA);
		MatrixMulti(V_trans, P, VTP, VTP_DATA);
		MatrixMulti(VTP, V, VTPV, VTPV_DATA);
		double sigma0, sigma2;
		sigma0 = sqrt(VTPV_DATA[0] / (rowB - columnB));
		sigma2 = pow(sigma0, 2);
		rawR->rtk.sigma0 = sigma0;
		MatrixTimes(D, BTPB_INV, sigma2, COV_X_DATA);
		coverage = v_DATA[0] * v_DATA[0] + v_DATA[1] * v_DATA[1] + v_DATA[2] * v_DATA[2];

		//// 矩阵结果输出
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


	/*******************************进行模糊度的固定******************************/
	// 获取模糊度方差协方差矩阵
	//COV的column和B的column是一样的
	int Qa_index = 0; int Qba_index = 0;
	int COV_index = 3 * columnB + 3; // 从第四行的第四列开始索引COV
	int COV_indexb = 3;              // 从第一行的第四列开始索引COV for Qba
	// 开始索引出出了第一个（3，3）矩阵的Qa矩阵的数据
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
	// 开始索引处前三行除了(3,3)矩阵的Qba
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

	//// 用来检查矩阵是否正确
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
	
	// 构建Qa矩阵：模糊度方差协方差矩阵
	// 构建fa矩阵：模糊度浮点解矩阵
	// 构建F矩阵：模糊度固定解向量
	// 构建s矩阵：模糊度残差二次型
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
	// 调用Lambda进行计算 
	// n： 模糊度维数
	// m： 模糊度候选解个数，设为2
	// fa：模糊度浮点解向量，n*1
	// Qa：模糊度方差协方差矩阵，n*n
	// F： 模糊度固定解向量，n*m
	// s： 模糊度残差二次型，1*m
	fixstate = lambda(rawR->rtk.N_n, rawR->rtk.N_m, fa, Qa, F, s);

	if (fixstate == 0)
		// 返回0表示模糊度固定函数返回成功
	{
		double ratio = s[1] / s[0];
		rawR->rtk.ratio = ratio;
		// 固定了之后的模糊度的存储
		// cout << rawR->rtk.gpst.SecofWeek << " " << rawR->rtk.ratio << endl;
		for (int i = 0; i < rawR->rtk.N_n; i++)
		{
			double* FIXED2 = new double[rawR->rtk.N_n];
			// 第i行第j列的索引是i + n * j
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

	/*******************************模糊度固定后进行基线向量的更新******************************/
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
		// 定义需要的各个矩阵
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

		// 检查矩阵
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

		// 进行基线固定解的更新
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
	 


	/*******************************结果的保存和精度指标的计算******************************/
	  XYZ2NEU(&refxyz, &rawR->rtk.xyz, &rawR->rtk.neu);

	  // 进行固定率的存储和计算
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

	  // 计算PDOP值
      // PDOP的三个值的索引分别是0，columnB + 1，2 * columnB + 2
	  double PDOP;
	  PDOP = sqrt(BTPBINV_data[0] + BTPBINV_data[columnB + 1] + BTPBINV_data[2 * columnB + 2]);
	  rawR->rtk.PDOP = PDOP;
	  rawR->rtk.sigmax = rawR->rtk.sigma0 * rawR->rtk.sigma0 * BTPBINV_data[0];
	  rawR->rtk.sigmay = rawR->rtk.sigma0 * rawR->rtk.sigma0 * BTPBINV_data[columnB + 1];
	  rawR->rtk.sigmaz = rawR->rtk.sigma0 * rawR->rtk.sigma0 * BTPBINV_data[2 * columnB + 2];


	  // 计算基线向量
	  rawR->rtk.Vector[0] = rawR->rtk.xyz.x - refx;
	  rawR->rtk.Vector[1] = rawR->rtk.xyz.y - refy;
	  rawR->rtk.Vector[2] = rawR->rtk.xyz.z - refz;

	  // 计算RMS
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
