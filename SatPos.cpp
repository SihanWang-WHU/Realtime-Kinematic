/*************************************************************************
作者 王思翰
学号 2019302141082
修改时间 2021年12月15日
**************************************************************************/
#include"SatPos.h"


using namespace std;

/*************************************************************************
calstaNEU_AE 计算函数测站的NEU
输入：原始数据RAWDATA的指针raw, 卫星结构体sat的索引i(prn号）
输出：计算得到测站的NEU坐标和卫星的高度角以及方位角
**************************************************************************/
int calNEU_AE(RAWDATA* raw, int i, int sys)
{

	if (sys == 0)
	{
		XYZ2BLH(&raw->receiver.blh, &raw->receiver.xyz);

		double N, E, U, azimuth, eleAngle;

		N = -sin(raw->receiver.blh.lat * PI / 180) * cos(raw->receiver.blh.lng * PI / 180) * (raw->sat_gps[i - 1].satXYZ.x - raw->receiver.xyz.x) - sin(raw->receiver.blh.lat * PI / 180) * sin(raw->receiver.blh.lng * PI / 180) * (raw->sat_gps[i - 1].satXYZ.y - raw->receiver.xyz.y) + cos(raw->receiver.blh.lat * PI / 180) * (raw->sat_gps[i - 1].satXYZ.z - raw->receiver.xyz.z);
		E = -sin(raw->receiver.blh.lng * PI / 180) * (raw->sat_gps[i - 1].satXYZ.x - raw->receiver.xyz.x) + cos(raw->receiver.blh.lng * PI / 180) * (raw->sat_gps[i - 1].satXYZ.y - raw->receiver.xyz.y);
		U = cos(raw->receiver.blh.lat * PI / 180) * cos(raw->receiver.blh.lng * PI / 180) * (raw->sat_gps[i - 1].satXYZ.x - raw->receiver.xyz.x) + cos(raw->receiver.blh.lat * PI / 180) * sin(raw->receiver.blh.lng * PI / 180) * (raw->sat_gps[i - 1].satXYZ.y - raw->receiver.xyz.y) + sin(raw->receiver.blh.lat * PI / 180) * (raw->sat_gps[i - 1].satXYZ.z - raw->receiver.xyz.z);

		raw->receiver.neu.N = N;
		raw->receiver.neu.E = E;
		raw->receiver.neu.U = U;

		azimuth = atan2(E, N);//方位角，rad
		eleAngle = atan2(U, sqrt(N * N + E * E));//高度角，rad

		//eleAngle = asin(U /
		//	(sqrt(N * N + E * E + U * U)));		                        //高度角，rad
		//azimuth = atan2(E, N);											//方位角，rad
		if (azimuth < 0)
			azimuth = (azimuth)+2 * PI;

		raw->sat_gps[i - 1].azimuth = azimuth;
		raw->sat_gps[i - 1].eleAngle = eleAngle;

		return 0;
	}
	else if (sys == 1)
	{
		XYZ2BLH(&raw->receiver.blh, &raw->receiver.xyz);

		double N, E, U, azimuth, eleAngle;

		N = -sin(raw->receiver.blh.lat * PI / 180) * cos(raw->receiver.blh.lng * PI / 180) * (raw->sat_bds[i - 1].satXYZ.x - raw->receiver.xyz.x) - sin(raw->receiver.blh.lat * PI / 180) * sin(raw->receiver.blh.lng * PI / 180) * (raw->sat_bds[i - 1].satXYZ.y - raw->receiver.xyz.y) + cos(raw->receiver.blh.lat * PI / 180) * (raw->sat_bds[i - 1].satXYZ.z - raw->receiver.xyz.z);
		E = -sin(raw->receiver.blh.lng * PI / 180) * (raw->sat_bds[i - 1].satXYZ.x - raw->receiver.xyz.x) + cos(raw->receiver.blh.lng * PI / 180) * (raw->sat_bds[i - 1].satXYZ.y - raw->receiver.xyz.y);
		U = cos(raw->receiver.blh.lat * PI / 180) * cos(raw->receiver.blh.lng * PI / 180) * (raw->sat_bds[i - 1].satXYZ.x - raw->receiver.xyz.x) + cos(raw->receiver.blh.lat * PI / 180) * sin(raw->receiver.blh.lng * PI / 180) * (raw->sat_bds[i - 1].satXYZ.y - raw->receiver.xyz.y) + sin(raw->receiver.blh.lat * PI / 180) * (raw->sat_bds[i - 1].satXYZ.z - raw->receiver.xyz.z);

		raw->receiver.neu.N = N;
		raw->receiver.neu.E = E;
		raw->receiver.neu.U = U;

		azimuth = atan2(E, N);//方位角，rad
		eleAngle = atan2(U, sqrt(N * N + E * E));//高度角，rad

		//eleAngle = asin(U /
		//	(sqrt(N * N + E * E + U * U)));		                        //高度角，rad
		//azimuth = atan2(E, N);											//方位角，rad
		if (azimuth < 0)
			azimuth = (azimuth)+2 * PI;

		raw->sat_bds[i - 1].azimuth = azimuth;
		raw->sat_bds[i - 1].eleAngle = eleAngle;

		return 0;
	}

}

/*************************************************************************
GPSPOSVEL GPS卫星位置速度计算函数
输入：原始数据RAWDATA的指针raw, 卫星的PRN号prn, 信号发射时的时间gTime, 卫星的位置结构体指针sat
输出：对应Prn号的GPS卫星在相应时间时候的卫星位置
已测试 2021.12.20
**************************************************************************/
double GPSPOSVEL(RAWDATA* raw, GPSTIME* gTime)
{
	double timesave = gTime->SecofWeek;
	for (int i = 0; i < MAXGPSPRN; i++)
	{
		if (raw->gpe[i].Prn == i + 1)
		{
			if (raw->gpe[i].health != 0)
			{
				raw->sat_gps[i].Valid = false;
				//cout << "The health state of the satellite is abnormal!" << endl;
				continue;
			}
			int prn;
			prn = i + 1;
			/* 迭代的次数 */
			double iterator = 0;

			//在这里的prn是卫星的PRN号，故数组的索引都是n-1
			double n0;                       // 平均运动角速度
			double tr;                       // 信号发射时的时间
			double t;                        // 信号发射时的时间
			double tk;                       // 相对于星历参考历元的时间
			double n;                        // 改正过后的平均角速度
			double Mk;                       // 平近点角
			double Ek;                       // 偏近点角
			double vk;                       // 真近点角
			double Faik;                     // 升交角距
			double deltauk;                  // 升交角距的改正数
			double uk;                       // 升交角距（改正后）
			double deltark;                  // 向径的改正数
			double rk;                       // 向径（改正后）
			double deltaik;                  // 轨道倾角的改正数
			double ik;                       // 轨道倾角（改正后）
			double xk, yk;                   // 卫星在轨道平面上的位置
			double Omegak;                   // 改正后的升交点经度

			int q;//range的索引
			q = FindSatObsIndex(prn, GPS, &raw->obs);
			raw->sat_gps[i].PRN = prn;
			raw->sat_gps[i].gpst = *gTime;
			raw->sat_gps[i].Valid = false; //初始状态全部设置为False

			//计算卫星实际的发射时间（未经钟差改正）
			double dtime;
			//计算卫星发射时间与表面时的第一个deltat（拿伪距除以C获得）
			double IF; // 伪距的IF组合观测值
			IF = FL1_GPS * FL1_GPS / (FL1_GPS * FL1_GPS - FL2_GPS * FL2_GPS) * raw->obs.range[q].P1 - FL2_GPS * FL2_GPS / (FL1_GPS * FL1_GPS - FL2_GPS * FL2_GPS) * raw->obs.range[q].P2;
			//计算平均运动角速度
			n0 = sqrt(GPS_Miu / pow(raw->gpe[prn - 1].A, 3));

			//计算相对于星历参考历元的时间tk
			tk = gTime->SecofWeek - raw->gpe[prn - 1].toe;
			double toc = gTime->SecofWeek - raw->gpe[prn - 1].toc;

			while (tk > 302400)
			{
				tk -= 604800;
			}
			while (tk < -302400)
			{
				tk += 604800;
			}

			if (fabs(tk) > 2 * 3600)
			{

				//cout <<"PRN=" <<i+1<< "卫星的星历不可用" << endl;
				continue;
			}

			//对平均运动角速度进行改正
			n = n0 + raw->gpe[prn - 1].DeltaN;

			//计算平近点角
			Mk = raw->gpe[prn - 1].M0 + n * tk;
			//迭代计算偏近点角
			/* 赋迭代的初始值 */
			double E = 0;
			Ek = 0;
			do
			{
				E = Ek;
				Ek = Mk + raw->gpe[prn - 1].ecc * sin(E);
				iterator++;
				if (iterator > 100)
				{
					cout << "偏近点角计算失败" << endl;
					break;
				}
			} while ((fabs(Ek - E) > 1e-14));

			/*************************************************************************
						   卫星钟差计算模块
			**************************************************************************/
			double F, deltatr, tempT, deltat, dt0, deltatr_dot = 0;
			double rho;
			double Ekdot;

			Ekdot = n / (1 - raw->gpe[prn - 1].ecc * cos(Ek));
			//卫星钟差改正
			F = -2 * sqrt(GPS_Miu) / pow(C_SPEED, 2);
			deltatr = F * raw->gpe[prn - 1].ecc * (sqrt(raw->gpe[prn - 1].A)) * sin(Ek);
			double dt = raw->gpe[prn - 1].af0 + raw->gpe[prn - 1].af1 * tk + raw->gpe[prn - 1].af2 * pow(tk, 2) + deltatr;
			tk -= dt;
			/*tempT = (raw->gpe[prn - 1].GPS_G.Week - raw->gpe[prn - 1].week) * 604800 - raw->gpe[prn - 1].toc;
			deltat = raw->gpe[prn - 1].af0 + raw->gpe[prn - 1].af1 * tempT + raw->gpe[prn - 1].af2 * pow(tempT, 2) + deltatr;*/
			deltat = raw->gpe[prn - 1].af0 + raw->gpe[prn - 1].af1 * tk + raw->gpe[prn - 1].af2 * pow(tk, 2) + deltatr - raw->gpe[prn - 1].tgd;
			raw->sat_gps[i].satClk = deltat;

			//卫星钟速改正
			deltatr_dot = F * raw->gpe[prn - 1].ecc * sqrt(raw->gpe[prn - 1].A) * cos(Ek) * Ekdot;
			double dtsv = raw->gpe[prn - 1].af1 + 2 * raw->gpe[prn - 1].af2 * tk + deltatr_dot;
			/*
			deltatr = F * raw->gpe[prn - 1].ecc * sqrt(raw->gpe[prn - 1].A) * cos(Ek) * dEk;
			double dtsv = raw->gpe[prn - 1].af1 + 2 * raw->gpe[prn - 1].af2 * tk + deltatr;
			*/


			raw->sat_gps[i].satClkDot = dtsv;
			dtime = IF / C_SPEED;
			gTime->SecofWeek -= dtime;

			//计算出卫星的钟差之后重新计算gTime，之后再进行位置速度的计算以及改正
			gTime->SecofWeek -= raw->sat_gps[i].satClk;

			/*************************************************************************
									   卫星位置计算模块
			**************************************************************************/
			//计算相对于星历参考历元的时间tk

			tk = gTime->SecofWeek - raw->gpe[prn - 1].toe;
			toc = gTime->SecofWeek - raw->gpe[prn - 1].toc;

			while (tk > 302400)
			{
				tk -= 604800;
			}
			while (tk < -302400)
			{
				tk += 604800;
			}

			if (fabs(tk) > 2 * 3600)
			{

				//cout <<"PRN=" <<i+1<< "卫星的星历不可用" << endl;
				continue;
			}

			//对平均运动角速度进行改正
			n = n0 + raw->gpe[prn - 1].DeltaN;

			//计算平近点角
			Mk = raw->gpe[prn - 1].M0 + n * tk;
			//迭代计算偏近点角
			/* 赋迭代的初始值 */
			E = 0;
			Ek = 0;
			do
			{
				E = Ek;
				Ek = Mk + raw->gpe[prn - 1].ecc * sin(E);
				iterator++;
				if (iterator > 100)
				{
					cout << "偏近点角计算失败" << endl;
					break;
				}
			} while ((fabs(Ek - E) > 1e-14));

			//计算真近点角,返回为弧度制（rad）
			vk = atan2((sqrt(1 - raw->gpe[prn - 1].ecc * raw->gpe[prn - 1].ecc)) * sin(Ek), cos(Ek) - raw->gpe[prn - 1].ecc);
			//计算升交角距
			Faik = vk + raw->gpe[prn - 1].omega;
			//计算二阶调和改正数
			//计算升交角距的改正数
			deltauk = raw->gpe[prn - 1].cus * sin(2 * Faik) + raw->gpe[prn - 1].cuc * cos(2 * Faik);
			//计算向径的改正数
			deltark = raw->gpe[prn - 1].crs * sin(2 * Faik) + raw->gpe[prn - 1].crc * cos(2 * Faik);
			//计算轨道倾角的改正数
			deltaik = raw->gpe[prn - 1].cis * sin(2 * Faik) + raw->gpe[prn - 1].cic * cos(2 * Faik);
			//计算经过改正的升交角距
			uk = Faik + deltauk;
			//计算经过改正的向径
			rk = raw->gpe[prn - 1].A * (1 - raw->gpe[prn - 1].ecc * cos(Ek)) + deltark;
			//计算经过改正后的轨道倾角
			ik = raw->gpe[prn - 1].I0 + raw->gpe[prn - 1].I0Dot * tk + deltaik;
			//计算卫星在轨道平面上的位置
			xk = rk * cos(uk);
			yk = rk * sin(uk);

			Omegak = raw->gpe[prn - 1].Omega0 + (raw->gpe[prn - 1].omegaDot - GPS_OmegaDot) * tk - GPS_OmegaDot * raw->gpe[prn - 1].toe;

			//得到卫星的位置（单位是m）
			//此时还没有施加地球自转改正
			raw->sat_gps[i].satXYZ.x = xk * cos(Omegak) - yk * cos(ik) * sin(Omegak);
			raw->sat_gps[i].satXYZ.y = xk * sin(Omegak) + yk * cos(ik) * cos(Omegak);
			raw->sat_gps[i].satXYZ.z = yk * sin(ik);

			//得到经过地球自传改正之后的卫星位置
			dtime += raw->sat_gps[i].satClk;
			double rotateAngle, tempSatX, tempSatY;
			rotateAngle = Omega_WGS * (dtime);
			tempSatX = raw->sat_gps[i].satXYZ.x + raw->sat_gps[i].satXYZ.y * rotateAngle;
			tempSatY = raw->sat_gps[i].satXYZ.y - raw->sat_gps[i].satXYZ.x * rotateAngle;
			raw->sat_gps[i].satXYZ.x = tempSatX;
			raw->sat_gps[i].satXYZ.y = tempSatY;


			/*************************************************************************
									   卫星速度计算模块
			**************************************************************************/
			double Faikdot, ukdot, rkdot, ikdot, Omegakdot;
			double Rdata[12];
			double xk_dot, yk_dot;
			Ekdot = n / (1 - raw->gpe[prn - 1].ecc * cos(Ek));
			Faikdot = sqrt((1 + raw->gpe[prn - 1].ecc) / (1 - raw->gpe[prn - 1].ecc)) * pow((cos(vk / 2) / (cos(Ek / 2))), 2) * Ekdot;

			ukdot = 2 * (raw->gpe[prn - 1].cus * cos(2 * Faikdot) - raw->gpe[prn - 1].cuc * sin(2 * Faik)) * Faikdot + Faikdot;
			rkdot = (raw->gpe[prn - 1].A) * (raw->gpe[prn - 1].ecc) * sin(Ek) * Ekdot + 2 * (raw->gpe[prn - 1].crs * cos(2 * Faikdot) - raw->gpe[prn - 1].crc * sin(2 * Faik)) * Faikdot;
			ikdot = raw->gpe[prn - 1].I0Dot + 2 * (raw->gpe[prn - 1].cis * cos(2 * Faik) - raw->gpe[prn - 1].cic * sin(2 * Faik)) * Faikdot;

			Omegakdot = raw->gpe[prn - 1].omegaDot - Omega_WGS;

			//给R矩阵赋值
			//这里的xk和yk是在卫星位置计算模块中得到的卫星在轨道平面的位置
			Rdata[0] = cos(Omegak);
			Rdata[1] = -sin(Omegak) * cos(ik);
			Rdata[2] = -xk * sin(Omegak) - yk * cos(Omegak) * cos(ik);
			Rdata[3] = yk * sin(Omegak) * sin(ik);
			Rdata[4] = sin(Omegak);
			Rdata[5] = cos(Omegak) * cos(ik);
			Rdata[6] = xk * cos(Omegak) - yk * sin(Omegak) * cos(ik);
			Rdata[7] = yk * cos(Omegak) * sin(ik);
			Rdata[8] = 0;
			Rdata[9] = sin(ik);
			Rdata[10] = 0;
			Rdata[11] = yk * cos(ik);

			xk_dot = rkdot * cos(uk) - rk * ukdot * sin(uk);
			yk_dot = rkdot * sin(uk) + rk * ukdot * cos(uk);

			matrix matR;
			MatrixInit(&matR, 3, 4, Rdata);

			double rawVelocityData[4] = { xk_dot,yk_dot,Omegakdot,ikdot };
			matrix matRawVelocity;
			MatrixInit(&matRawVelocity, 4, 1, rawVelocityData);

			double velocityData[3];
			matrix matVelocity;
			MatrixMulti(&matR, &matRawVelocity, &matVelocity, velocityData);

			raw->sat_gps[i].satVelocity[0] = velocityData[0];
			raw->sat_gps[i].satVelocity[1] = velocityData[1];
			raw->sat_gps[i].satVelocity[2] = velocityData[2];


			raw->sat_gps[i].Valid = true;
			//计算完成之后把时间还原
			gTime->SecofWeek = timesave;
		}

	}

	return 0;
}

/*************************************************************************
BDSPOSVEL BDS卫星位置速度计算函数
输入：原始数据RAWDATA的指针raw, 信号发射时的时间gTime
输出：对应Prn号的BDS卫星（MEO,IGSO,GEO)在相应时间时候的卫星位置
已测试 2021.12.21
**************************************************************************/
double BDSPOSVEL(RAWDATA* raw, GPSTIME* gTime)
{
	double timesave = gTime->SecofWeek;
	for (int i = 0; i < MAXBDSPRN; i++)
	{

		if (raw->bde[i].Prn == i + 1)
		{
			int prn;
			prn = i + 1;
			if (raw->bde[i].health != 0)
			{
				// 如果卫星不健康 则该卫星的位置计算出来的不可用
				raw->sat_bds[i].Valid = false;
				//cout << "The health state of the satellite is abnormal!" << endl;
				continue;
			}
			// 北斗的GEO卫星的位置计算
			if (prn <= 5 || prn >= 59)
			{
				/* 迭代的次数 */
				double iterator = 0;

				//在这里的prn是卫星的PRN号，故数组的索引都是n-1
				double n0;                       // 平均运动角速度
				double tr;                       // 信号发射时的时刻
				double tk;                       // 相对于星历参考历元的时间
				double n;                        // 改正过后的平均角速度
				double Mk;                       // 平近点角
				double Ek;                       // 偏近点角
				double vk;                       // 真近点角
				double Faik;                     // 升交角距
				double deltauk;                  // 升交角距的改正数
				double uk;                       // 升交角距（改正后）
				double deltark;                  // 向径的改正数
				double rk;                       // 向径（改正后）
				double deltaik;                  // 轨道倾角的改正数
				double ik;                       // 轨道倾角（改正后）
				double xk, yk;                   // 卫星在轨道平面上的位置
				double Omegak;                   // 改正后的升交点经度

				//测试用的时间
				//gTime->Week = 2184;
				//gTime->SecofWeek = 26700.0;

				int q;//range的索引
				q = FindSatObsIndex(prn, BDS, &raw->obs);
				raw->sat_bds[i].PRN = prn;
				raw->sat_bds[i].gpst = *gTime;
				raw->sat_bds[i].Valid = false;

				//计算卫星实际的发射时间（未经钟差改正）
				double dtime;
				//计算卫星发射时间与表面时的第一个deltat（拿伪距除以C获得）
				double IF; // 伪距的IF组合观测值
				IF = FB1_BDS * FB1_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS) * raw->obs.range[q].P1 - FB3_BDS * FB3_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS) * raw->obs.range[q].P2;

				//计算平均运动角速度
				n0 = sqrt(BDS_Miu / pow(raw->bde[prn - 1].A, 3));

				//计算相对于星历参考历元的时间tk
				tk = 604800 * (gTime->Week - 1356 - raw->bde[prn - 1].week) + gTime->SecofWeek - raw->bde[prn - 1].toe - 14;

				while (tk > 302400)
				{
					tk -= 604800;
				}
				while (tk < -302400)
				{
					tk += 604800;
				}

				if (fabs(tk) > 3600)
				{

					//cout << "PRN=" << i + 1 << "卫星的星历不可用" << endl;
					continue;

				}

				//对平均运动角速度进行改正
				n = n0 + raw->bde[prn - 1].DeltaN;

				//计算平近点角
				Mk = raw->bde[prn - 1].M0 + n * tk;
				//迭代计算偏近点角
				/* 赋迭代的初始值 */
				double E = 0;
				Ek = 0;
				do
				{
					E = Ek;
					Ek = Mk + raw->bde[prn - 1].ecc * sin(E);
					iterator++;
					if (iterator > 100)
					{
						cout << "偏近点角计算失败" << endl;
						break;
					}
				} while ((fabs(Ek - E) > 1e-14));

				/*************************************************************************
										   卫星钟差计算模块
				**************************************************************************/
				double F, deltatr, tempT, deltat, dt0, deltatr_dot = 0;
				double rho;
				double Ekdot;

				Ekdot = n / (1 - raw->bde[prn - 1].ecc * cos(Ek));

				//卫星钟差改正
				F = -2 * sqrt(BDS_Miu) / pow(C_SPEED, 2);
				deltatr = F * raw->bde[prn - 1].ecc * (sqrt(raw->bde[prn - 1].A)) * sin(Ek);
				double dt = raw->bde[prn - 1].a0 + raw->bde[prn - 1].a1 * tk + raw->bde[prn - 1].a2 * pow(tk, 2) + deltatr;
				tk -= dt;
				/*tempT = (raw->gpe[prn - 1].GPS_G.Week - raw->gpe[prn - 1].week) * 604800 - raw->gpe[prn - 1].toc;
				deltat = raw->gpe[prn - 1].af0 + raw->gpe[prn - 1].af1 * tempT + raw->gpe[prn - 1].af2 * pow(tempT, 2) + deltatr;*/
				deltat = raw->bde[prn - 1].a0 + raw->bde[prn - 1].a1 * tk + raw->bde[prn - 1].a2 * pow(tk, 2) + deltatr - raw->bde[prn - 1].tgd1;
				raw->sat_bds[i].satClk = deltat;

				//卫星钟速改正
				deltatr_dot = F * raw->bde[prn - 1].ecc * sqrt(raw->bde[prn - 1].A) * cos(Ek) * Ekdot;
				double dtsv = raw->bde[prn - 1].a1 + 2 * raw->bde[prn - 1].a2 * tk + deltatr_dot;
				/*
				deltatr = F * raw->gpe[prn - 1].ecc * sqrt(raw->gpe[prn - 1].A) * cos(Ek) * dEk;
				double dtsv = raw->gpe[prn - 1].af1 + 2 * raw->gpe[prn - 1].af2 * tk + deltatr;
				*/
				dtime = IF / C_SPEED;
				gTime->SecofWeek -= dtime;
				raw->sat_bds[i].satClkDot = dtsv;
				gTime->SecofWeek -= raw->sat_bds[i].satClk;

				/*************************************************************************
										   卫星位置计算模块
				**************************************************************************/
				//计算平均运动角速度
				n0 = sqrt(BDS_Miu / pow(raw->bde[prn - 1].A, 3));

				//计算相对于星历参考历元的时间tk
				tk = 604800 * (gTime->Week - 1356 - raw->bde[prn - 1].week) + gTime->SecofWeek - raw->bde[prn - 1].toe - 14;

				while (tk > 302400)
				{
					tk -= 604800;
				}
				while (tk < -302400)
				{
					tk += 604800;
				}

				if (fabs(tk) > 3600)
				{

					//cout << "PRN=" << i + 1 << "卫星的星历不可用" << endl;
					continue;

				}

				//对平均运动角速度进行改正
				n = n0 + raw->bde[prn - 1].DeltaN;

				//计算平近点角
				Mk = raw->bde[prn - 1].M0 + n * tk;
				//迭代计算偏近点角
				/* 赋迭代的初始值 */
				E = 0;
				Ek = 0;
				do
				{
					E = Ek;
					Ek = Mk + raw->bde[prn - 1].ecc * sin(E);
					iterator++;
					if (iterator > 100)
					{
						cout << "偏近点角计算失败" << endl;
						break;
					}
				} while ((fabs(Ek - E) > 1e-14));

				//计算真近点角,返回为弧度制（rad）
				vk = atan2((sqrt(1 - raw->bde[prn - 1].ecc * raw->bde[prn - 1].ecc)) * sin(Ek), cos(Ek) - raw->bde[prn - 1].ecc);
				//计算升交角距
				Faik = vk + raw->bde[prn - 1].omega;
				//计算二阶调和改正数
				//计算升交角距的改正数
				deltauk = raw->bde[prn - 1].cus * sin(2 * Faik) + raw->bde[prn - 1].cuc * cos(2 * Faik);
				//计算向径的改正数
				deltark = raw->bde[prn - 1].crs * sin(2 * Faik) + raw->bde[prn - 1].crc * cos(2 * Faik);
				//计算轨道倾角的改正数
				deltaik = raw->bde[prn - 1].cis * sin(2 * Faik) + raw->bde[prn - 1].cic * cos(2 * Faik);
				//计算经过改正的升交角距
				uk = Faik + deltauk;
				//计算经过改正的向径
				rk = raw->bde[prn - 1].A * (1 - raw->bde[prn - 1].ecc * cos(Ek)) + deltark;
				//计算经过改正后的轨道倾角
				ik = raw->bde[prn - 1].i0 + raw->bde[prn - 1].IDOT * tk + deltaik;
				//计算卫星在轨道平面上的位置
				xk = rk * cos(uk);
				yk = rk * sin(uk);
				Omegak = raw->bde[prn - 1].Omega0 + (raw->bde[prn - 1].omegadot) * tk - BDS_OmegaDot * raw->bde[prn - 1].toe;
				//得到卫星的位置（单位是m）
				double x, y, z;          // 经过旋转之前的卫星位置
				double x_r, y_r, z_r;    // 经过旋转之后的卫星位置

				x = xk * cos(Omegak) - yk * cos(ik) * sin(Omegak);
				y = xk * sin(Omegak) + yk * cos(ik) * cos(Omegak);
				z = yk * sin(ik);

				//Rx旋转矩阵
				double phi = -5 * PI / 180;
				double Rxdata[9];
				Rxdata[0] = 1;
				Rxdata[1] = 0;
				Rxdata[2] = 0;
				Rxdata[3] = 0;
				Rxdata[4] = cos(phi);
				Rxdata[5] = sin(phi);
				Rxdata[6] = 0;
				Rxdata[7] = -sin(phi);
				Rxdata[8] = cos(phi);
				matrix Rx;
				MatrixInit(&Rx, 3, 3, Rxdata);

				//Rz旋转矩阵
				double phi2 = BDS_OmegaDot * tk;
				double Rzdata[9];
				Rzdata[0] = cos(phi2);
				Rzdata[1] = sin(phi2);
				Rzdata[2] = 0;
				Rzdata[3] = -sin(phi2);
				Rzdata[4] = cos(phi2);
				Rzdata[5] = 0;
				Rzdata[6] = 0;
				Rzdata[7] = 0;
				Rzdata[8] = 1;

				matrix Rz;
				MatrixInit(&Rz, 3, 3, Rzdata);
				double XZdata[9] = { 0 };
				matrix XZ;
				MatrixMulti(&Rz, &Rx, &XZ, XZdata);

				double posdata[3] = { x,y,z };
				matrix temppos;
				MatrixInit(&temppos, 3, 1, posdata);
				double POS_Rdata[3] = { 0 };
				matrix POS_R;//经过旋转改正之后的位置矩阵
				MatrixMulti(&XZ, &temppos, &POS_R, POS_Rdata);

				//得到未经地球自传改正的卫星位置
				raw->sat_bds[i].satXYZ.x = POS_Rdata[0];
				raw->sat_bds[i].satXYZ.y = POS_Rdata[1];
				raw->sat_bds[i].satXYZ.z = POS_Rdata[2];

				////得到经过地球自传改正之后的卫星位置
				dtime += raw->sat_bds[i].satClk;
				double rotateAngle, tempSatX, tempSatY;
				rotateAngle = Omega_BDS * (dtime);
				tempSatX = raw->sat_bds[i].satXYZ.x + raw->sat_bds[i].satXYZ.y * rotateAngle;
				tempSatY = raw->sat_bds[i].satXYZ.y - raw->sat_bds[i].satXYZ.x * rotateAngle;
				raw->sat_bds[i].satXYZ.x = tempSatX;
				raw->sat_bds[i].satXYZ.y = tempSatY;

				/*************************************************************************
										   卫星速度计算模块
				**************************************************************************/
				double Faikdot, ukdot, rkdot, ikdot, Omegakdot;
				double Rdata[12];
				double xkDot, ykDot;
				Ekdot = n / (1 - raw->bde[prn - 1].ecc * cos(Ek));
				Faikdot = sqrt((1 + raw->bde[prn - 1].ecc) / (1 - raw->bde[prn - 1].ecc)) * pow((cos(vk / 2) / (cos(Ek / 2))), 2) * Ekdot;

				ukdot = 2 * (raw->bde[prn - 1].cus * cos(2 * Faikdot) - raw->bde[prn - 1].cuc * sin(2 * Faik)) * Faikdot + Faikdot;
				rkdot = (raw->bde[prn - 1].A) * (raw->bde[prn - 1].ecc) * sin(Ek) * Ekdot + 2 * (raw->bde[prn - 1].crs * cos(2 * Faikdot) - raw->bde[prn - 1].crc * sin(2 * Faik)) * Faikdot;
				ikdot = raw->bde[prn - 1].IDOT + 2 * (raw->bde[prn - 1].cis * cos(2 * Faik) - raw->bde[prn - 1].cic * sin(2 * Faik)) * Faikdot;

				//升交点赤经的变化率，这里不减地球自转角速度
				Omegakdot = raw->bde[prn - 1].omegadot;

				xkDot = rkdot * cos(uk) - rk * ukdot * sin(uk);
				ykDot = rkdot * sin(uk) + rk * ukdot * cos(uk);

				//计算卫星在自定义的旋转坐标系中的速度
				double vel_GK[3] =
				{
				 xkDot * cos(Omegak) - ykDot * sin(Omegak) * cos(ik) + yk * ikdot * sin(Omegak) * sin(ik) -
				(xk * sin(Omegak) - yk * cos(ik) * cos(Omegak)) * Omegakdot,
				 xkDot * sin(Omegak) + ykDot * cos(ik) * cos(Omegak) - yk * sin(ik) * cos(Omegak) * ikdot
				+ (xk * cos(Omegak) - yk * cos(ik) * sin(Omegak)) * Omegakdot,
				 ykDot * sin(ik) + yk * ikdot * cos(ik)
				};

				//计算Rz_dot(we,tk),为转换矩阵的求导
				double rotationz = BDS_OmegaDot * tk;
				double Rzdotdata[9] =
				{
				  BDS_OmegaDot * (-sin(rotationz)), BDS_OmegaDot * cos(rotationz), 0,
				  BDS_OmegaDot * (-cos(rotationz)), BDS_OmegaDot * (-sin(rotationz)), 0,
				  0, 0, 0
				};
				matrix Rzdot;
				MatrixInit(&Rzdot, 3, 3, Rzdotdata);
				double velocitydata[3] = { 0 };
				double tempdata[9] = { 0 };
				double tempdata1[9] = { 0 };
				matrix temp1;   // 用来存放Rz和Rx相乘的临时矩阵
				matrix temp2;   // 用来存放Rzdot和Rx相乘的临时矩阵
				MatrixMulti(&Rz, &Rx, &temp1, tempdata);
				MatrixMulti(&Rzdot, &Rx, &temp2, tempdata1);

				matrix temp3;   // 用来存放式子中上面一行
				matrix temp4;   // 用来存放式子中下面一行
				matrix veltemp;
				double tempdata2[3] = { 0 };
				double tempdata3[3] = { 0 };
				MatrixInit(&veltemp, 3, 1, vel_GK);
				MatrixMulti(&temp1, &veltemp, &temp3, tempdata2);
				MatrixMulti(&temp2, &temppos, &temp4, tempdata3);
				matrix Velocity;
				MatrixAdd(&Velocity, &temp3, &temp4, velocitydata);

				raw->sat_bds[i].satVelocity[0] = velocitydata[0];
				raw->sat_bds[i].satVelocity[1] = velocitydata[1];
				raw->sat_bds[i].satVelocity[2] = velocitydata[2];

				raw->sat_bds[i].Valid = true;
				//计算完成之后把时间还原
				gTime->SecofWeek = timesave;
			}
			// 北斗其他卫星的位置计算  已测试完成
			else
			{
				/* 迭代的次数 */
				double iterator = 0;

				//在这里的prn是卫星的PRN号，故数组的索引都是n-1
				double n0;                       // 平均运动角速度
				double tr;                       // 信号发射时的时间
				double t;                        // 信号发射时的时间
				double tk;                       // 相对于星历参考历元的时间
				double n;                        // 改正过后的平均角速度
				double Mk;                       // 平近点角
				double Ek;                       // 偏近点角
				double vk;                       // 真近点角
				double Faik;                     // 升交角距
				double deltauk;                  // 升交角距的改正数
				double uk;                       // 升交角距（改正后）
				double deltark;                  // 向径的改正数
				double rk;                       // 向径（改正后）
				double deltaik;                  // 轨道倾角的改正数
				double ik;                       // 轨道倾角（改正后）
				double xk, yk;                   // 卫星在轨道平面上的位置
				double Omegak;                   // 改正后的升交点经度

				/*GPSTIME* gTime = new GPSTIME;*/
				//gTime->Week = 2184;
				//gTime->SecofWeek = 26700.0;
				int q;//range的索引
				q = FindSatObsIndex(prn, BDS, &raw->obs);
				raw->sat_bds[i].PRN = prn;
				raw->sat_bds[i].gpst = *gTime;
				raw->sat_bds[i].Valid = false;

				//计算卫星实际的发射时间（未经钟差改正）
				double dtime;
				//计算卫星发射时间与表面时的第一个deltat（拿伪距除以C获得）
				double IF; // 伪距的IF组合观测值
				IF = FB1_BDS * FB1_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS) * raw->obs.range[q].P1 - FB3_BDS * FB3_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS) * raw->obs.range[q].P2;


				//计算平均运动角速度
				n0 = sqrt(BDS_Miu / pow(raw->bde[prn - 1].A, 3));

				//计算相对于星历参考历元的时间tk

				tk = gTime->SecofWeek - raw->bde[prn - 1].toe - 14;

				double toc = gTime->SecofWeek - raw->bde[prn - 1].toc;

				while (tk > 302400)
				{
					tk -= 604800;
				}
				while (tk < -302400)
				{
					tk += 604800;
				}

				if (fabs(tk) > 3600)
				{
					raw->sat_bds[i].Valid = false;
					//cout << "PRN=" << i + 1 << "卫星的星历不可用" << endl;
					continue;
				}

				//对平均运动角速度进行改正
				n = n0 + raw->bde[prn - 1].DeltaN;

				//计算平近点角
				Mk = raw->bde[prn - 1].M0 + n * tk;
				//迭代计算偏近点角
				/* 赋迭代的初始值 */
				double E = 0;
				Ek = 0;
				do
				{
					E = Ek;
					Ek = Mk + raw->bde[prn - 1].ecc * sin(E);
					iterator++;
					if (iterator > 100)
					{
						cout << "偏近点角计算失败" << endl;
						break;
					}
				} while ((fabs(Ek - E) > 1e-14));

				/*************************************************************************
						   卫星钟差计算模块
				**************************************************************************/
				double Ekdot;
				double F, deltatr, tempT, deltat, dt0, deltatr_dot = 0;
				double rho;

				Ekdot = n / (1 - raw->bde[prn - 1].ecc * cos(Ek));
				//卫星钟差改正
				F = -2 * sqrt(BDS_Miu) / pow(C_SPEED, 2);
				deltatr = F * raw->bde[prn - 1].ecc * (sqrt(raw->bde[prn - 1].A)) * sin(Ek);
				double dt = raw->bde[prn - 1].a0 + raw->bde[prn - 1].a1 * tk + raw->bde[prn - 1].a2 * pow(tk, 2) + deltatr;
				tk -= dt;
				/*tempT = (raw->bde[prn - 1].GPS_G.Week - raw->bde[prn - 1].week) * 604800 - raw->bde[prn - 1].toc;
				deltat = raw->bde[prn - 1].af0 + raw->bde[prn - 1].af1 * tempT + raw->bde[prn - 1].af2 * pow(tempT, 2) + deltatr;*/
				deltat = raw->bde[prn - 1].a0 + raw->bde[prn - 1].a1 * tk + raw->bde[prn - 1].a2 * pow(tk, 2) + deltatr - raw->bde[prn - 1].tgd1;
				raw->sat_bds[i].satClk = deltat;

				//卫星钟速改正
				deltatr_dot = F * raw->bde[prn - 1].ecc * sqrt(raw->bde[prn - 1].A) * cos(Ek) * Ekdot;
				double dtsv = raw->bde[prn - 1].a1 + 2 * raw->bde[prn - 1].a2 * tk + deltatr_dot;
				/*
				deltatr = F * raw->bde[prn - 1].ecc * sqrt(raw->bde[prn - 1].A) * cos(Ek) * dEk;
				double dtsv = raw->bde[prn - 1].af1 + 2 * raw->bde[prn - 1].af2 * tk + deltatr;
				*/
				raw->sat_bds[i].satClkDot = dtsv;

				dtime = IF / C_SPEED;
				gTime->SecofWeek -= dtime;
				//计算出卫星的钟差之后重新计算gTime，之后再进行位置速度的计算以及改正
				gTime->SecofWeek -= raw->sat_bds[i].satClk;


				/*************************************************************************
										   卫星位置计算模块
				**************************************************************************/
				//计算平均运动角速度
				n0 = sqrt(BDS_Miu / pow(raw->bde[prn - 1].A, 3));

				//计算相对于星历参考历元的时间tk

				tk = gTime->SecofWeek - raw->bde[prn - 1].toe - 14;

				toc = gTime->SecofWeek - raw->bde[prn - 1].toc;

				while (tk > 302400)
				{
					tk -= 604800;
				}
				while (tk < -302400)
				{
					tk += 604800;
				}

				if (fabs(tk) > 3600)
				{
					raw->sat_bds[i].Valid = false;
					//cout << "PRN=" << i + 1 << "卫星的星历不可用" << endl;
					continue;
				}

				//对平均运动角速度进行改正
				n = n0 + raw->bde[prn - 1].DeltaN;

				//计算平近点角
				Mk = raw->bde[prn - 1].M0 + n * tk;
				//迭代计算偏近点角
				/* 赋迭代的初始值 */
				E = 0;
				Ek = 0;
				do
				{
					E = Ek;
					Ek = Mk + raw->bde[prn - 1].ecc * sin(E);
					iterator++;
					if (iterator > 100)
					{
						cout << "偏近点角计算失败" << endl;
						break;
					}
				} while ((fabs(Ek - E) > 1e-14));

				//计算真近点角,返回为弧度制（rad）
				vk = atan2((sqrt(1 - raw->bde[prn - 1].ecc * raw->bde[prn - 1].ecc)) * sin(Ek), cos(Ek) - raw->bde[prn - 1].ecc);
				//计算升交角距
				Faik = vk + raw->bde[prn - 1].omega;
				//计算二阶调和改正数
				//计算升交角距的改正数
				deltauk = raw->bde[prn - 1].cus * sin(2 * Faik) + raw->bde[prn - 1].cuc * cos(2 * Faik);
				//计算向径的改正数
				deltark = raw->bde[prn - 1].crs * sin(2 * Faik) + raw->bde[prn - 1].crc * cos(2 * Faik);
				//计算轨道倾角的改正数
				deltaik = raw->bde[prn - 1].cis * sin(2 * Faik) + raw->bde[prn - 1].cic * cos(2 * Faik);
				//计算经过改正的升交角距
				uk = Faik + deltauk;
				//计算经过改正的向径
				rk = raw->bde[prn - 1].A * (1 - raw->bde[prn - 1].ecc * cos(Ek)) + deltark;
				//计算经过改正后的轨道倾角
				ik = raw->bde[prn - 1].i0 + raw->bde[prn - 1].IDOT * tk + deltaik;
				//计算卫星在轨道平面上的位置
				xk = rk * cos(uk);
				yk = rk * sin(uk);
				Omegak = raw->bde[prn - 1].Omega0 + (raw->bde[prn - 1].omegadot - BDS_OmegaDot) * tk - BDS_OmegaDot * raw->bde[prn - 1].toe;
				//得到卫星的位置（单位是m）--此时还没有经过地球自传改正
				raw->sat_bds[i].satXYZ.x = xk * cos(Omegak) - yk * cos(ik) * sin(Omegak);
				raw->sat_bds[i].satXYZ.y = xk * sin(Omegak) + yk * cos(ik) * cos(Omegak);
				raw->sat_bds[i].satXYZ.z = yk * sin(ik);

				////得到经过地球自传改正之后的卫星位置
				dtime += raw->sat_bds[i].satClk;
				double rotateAngle, tempSatX, tempSatY;
				rotateAngle = Omega_BDS * (dtime);
				tempSatX = raw->sat_bds[i].satXYZ.x + raw->sat_bds[i].satXYZ.y * rotateAngle;
				tempSatY = raw->sat_bds[i].satXYZ.y - raw->sat_bds[i].satXYZ.x * rotateAngle;
				raw->sat_bds[i].satXYZ.x = tempSatX;
				raw->sat_bds[i].satXYZ.y = tempSatY;

				/*************************************************************************
										   卫星速度计算模块
				**************************************************************************/
				double Faikdot, ukdot, rkdot, ikdot, Omegakdot;
				double Rdata[12];
				double xk_dot, yk_dot;
				Ekdot = n / (1 - raw->bde[prn - 1].ecc * cos(Ek));
				Faikdot = sqrt((1 + raw->bde[prn - 1].ecc) / (1 - raw->bde[prn - 1].ecc)) * pow((cos(vk / 2) / (cos(Ek / 2))), 2) * Ekdot;

				ukdot = 2 * (raw->bde[prn - 1].cus * cos(2 * Faikdot) - raw->bde[prn - 1].cuc * sin(2 * Faik)) * Faikdot + Faikdot;
				rkdot = (raw->bde[prn - 1].A) * (raw->bde[prn - 1].ecc) * sin(Ek) * Ekdot + 2 * (raw->bde[prn - 1].crs * cos(2 * Faikdot) - raw->bde[prn - 1].crc * sin(2 * Faik)) * Faikdot;
				ikdot = raw->bde[prn - 1].IDOT + 2 * (raw->bde[prn - 1].cis * cos(2 * Faik) - raw->bde[prn - 1].cic * sin(2 * Faik)) * Faikdot;

				Omegakdot = raw->bde[prn - 1].omegadot - Omega_WGS;

				//给R矩阵赋值
				//这里的xk和yk是在卫星位置计算模块中得到的卫星在轨道平面的位置
				Rdata[0] = cos(Omegak);
				Rdata[1] = -sin(Omegak) * cos(ik);
				Rdata[2] = -xk * sin(Omegak) - yk * cos(Omegak) * cos(ik);
				Rdata[3] = yk * sin(Omegak) * sin(ik);
				Rdata[4] = sin(Omegak);
				Rdata[5] = cos(Omegak) * cos(ik);
				Rdata[6] = xk * cos(Omegak) - yk * sin(Omegak) * cos(ik);
				Rdata[7] = yk * cos(Omegak) * sin(ik);
				Rdata[8] = 0;
				Rdata[9] = sin(ik);
				Rdata[10] = 0;
				Rdata[11] = yk * cos(ik);

				xk_dot = rkdot * cos(uk) - rk * ukdot * sin(uk);
				yk_dot = rkdot * sin(uk) + rk * ukdot * cos(uk);

				matrix matR;
				MatrixInit(&matR, 3, 4, Rdata);

				double rawVelocityData[4] = { xk_dot,yk_dot,Omegakdot,ikdot };
				matrix matRawVelocity;
				MatrixInit(&matRawVelocity, 4, 1, rawVelocityData);

				double velocityData[3];
				matrix matVelocity;
				MatrixMulti(&matR, &matRawVelocity, &matVelocity, velocityData);

				raw->sat_bds[i].satVelocity[0] = velocityData[0];
				raw->sat_bds[i].satVelocity[1] = velocityData[1];
				raw->sat_bds[i].satVelocity[2] = velocityData[2];

				raw->sat_bds[i].Valid = true;
				gTime->SecofWeek = timesave;

			}
		}
	}
	return 0;
}
