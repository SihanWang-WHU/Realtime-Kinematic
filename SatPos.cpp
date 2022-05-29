/*************************************************************************
���� ��˼��
ѧ�� 2019302141082
�޸�ʱ�� 2021��12��15��
**************************************************************************/
#include"SatPos.h"


using namespace std;

/*************************************************************************
calstaNEU_AE ���㺯����վ��NEU
���룺ԭʼ����RAWDATA��ָ��raw, ���ǽṹ��sat������i(prn�ţ�
���������õ���վ��NEU��������ǵĸ߶Ƚ��Լ���λ��
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

		azimuth = atan2(E, N);//��λ�ǣ�rad
		eleAngle = atan2(U, sqrt(N * N + E * E));//�߶Ƚǣ�rad

		//eleAngle = asin(U /
		//	(sqrt(N * N + E * E + U * U)));		                        //�߶Ƚǣ�rad
		//azimuth = atan2(E, N);											//��λ�ǣ�rad
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

		azimuth = atan2(E, N);//��λ�ǣ�rad
		eleAngle = atan2(U, sqrt(N * N + E * E));//�߶Ƚǣ�rad

		//eleAngle = asin(U /
		//	(sqrt(N * N + E * E + U * U)));		                        //�߶Ƚǣ�rad
		//azimuth = atan2(E, N);											//��λ�ǣ�rad
		if (azimuth < 0)
			azimuth = (azimuth)+2 * PI;

		raw->sat_bds[i - 1].azimuth = azimuth;
		raw->sat_bds[i - 1].eleAngle = eleAngle;

		return 0;
	}

}

/*************************************************************************
GPSPOSVEL GPS����λ���ٶȼ��㺯��
���룺ԭʼ����RAWDATA��ָ��raw, ���ǵ�PRN��prn, �źŷ���ʱ��ʱ��gTime, ���ǵ�λ�ýṹ��ָ��sat
�������ӦPrn�ŵ�GPS��������Ӧʱ��ʱ�������λ��
�Ѳ��� 2021.12.20
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
			/* �����Ĵ��� */
			double iterator = 0;

			//�������prn�����ǵ�PRN�ţ����������������n-1
			double n0;                       // ƽ���˶����ٶ�
			double tr;                       // �źŷ���ʱ��ʱ��
			double t;                        // �źŷ���ʱ��ʱ��
			double tk;                       // ����������ο���Ԫ��ʱ��
			double n;                        // ���������ƽ�����ٶ�
			double Mk;                       // ƽ�����
			double Ek;                       // ƫ�����
			double vk;                       // ������
			double Faik;                     // �����Ǿ�
			double deltauk;                  // �����Ǿ�ĸ�����
			double uk;                       // �����Ǿࣨ������
			double deltark;                  // �򾶵ĸ�����
			double rk;                       // �򾶣�������
			double deltaik;                  // �����ǵĸ�����
			double ik;                       // �����ǣ�������
			double xk, yk;                   // �����ڹ��ƽ���ϵ�λ��
			double Omegak;                   // ������������㾭��

			int q;//range������
			q = FindSatObsIndex(prn, GPS, &raw->obs);
			raw->sat_gps[i].PRN = prn;
			raw->sat_gps[i].gpst = *gTime;
			raw->sat_gps[i].Valid = false; //��ʼ״̬ȫ������ΪFalse

			//��������ʵ�ʵķ���ʱ�䣨δ���Ӳ������
			double dtime;
			//�������Ƿ���ʱ�������ʱ�ĵ�һ��deltat����α�����C��ã�
			double IF; // α���IF��Ϲ۲�ֵ
			IF = FL1_GPS * FL1_GPS / (FL1_GPS * FL1_GPS - FL2_GPS * FL2_GPS) * raw->obs.range[q].P1 - FL2_GPS * FL2_GPS / (FL1_GPS * FL1_GPS - FL2_GPS * FL2_GPS) * raw->obs.range[q].P2;
			//����ƽ���˶����ٶ�
			n0 = sqrt(GPS_Miu / pow(raw->gpe[prn - 1].A, 3));

			//��������������ο���Ԫ��ʱ��tk
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

				//cout <<"PRN=" <<i+1<< "���ǵ�����������" << endl;
				continue;
			}

			//��ƽ���˶����ٶȽ��и���
			n = n0 + raw->gpe[prn - 1].DeltaN;

			//����ƽ�����
			Mk = raw->gpe[prn - 1].M0 + n * tk;
			//��������ƫ�����
			/* �������ĳ�ʼֵ */
			double E = 0;
			Ek = 0;
			do
			{
				E = Ek;
				Ek = Mk + raw->gpe[prn - 1].ecc * sin(E);
				iterator++;
				if (iterator > 100)
				{
					cout << "ƫ����Ǽ���ʧ��" << endl;
					break;
				}
			} while ((fabs(Ek - E) > 1e-14));

			/*************************************************************************
						   �����Ӳ����ģ��
			**************************************************************************/
			double F, deltatr, tempT, deltat, dt0, deltatr_dot = 0;
			double rho;
			double Ekdot;

			Ekdot = n / (1 - raw->gpe[prn - 1].ecc * cos(Ek));
			//�����Ӳ����
			F = -2 * sqrt(GPS_Miu) / pow(C_SPEED, 2);
			deltatr = F * raw->gpe[prn - 1].ecc * (sqrt(raw->gpe[prn - 1].A)) * sin(Ek);
			double dt = raw->gpe[prn - 1].af0 + raw->gpe[prn - 1].af1 * tk + raw->gpe[prn - 1].af2 * pow(tk, 2) + deltatr;
			tk -= dt;
			/*tempT = (raw->gpe[prn - 1].GPS_G.Week - raw->gpe[prn - 1].week) * 604800 - raw->gpe[prn - 1].toc;
			deltat = raw->gpe[prn - 1].af0 + raw->gpe[prn - 1].af1 * tempT + raw->gpe[prn - 1].af2 * pow(tempT, 2) + deltatr;*/
			deltat = raw->gpe[prn - 1].af0 + raw->gpe[prn - 1].af1 * tk + raw->gpe[prn - 1].af2 * pow(tk, 2) + deltatr - raw->gpe[prn - 1].tgd;
			raw->sat_gps[i].satClk = deltat;

			//�������ٸ���
			deltatr_dot = F * raw->gpe[prn - 1].ecc * sqrt(raw->gpe[prn - 1].A) * cos(Ek) * Ekdot;
			double dtsv = raw->gpe[prn - 1].af1 + 2 * raw->gpe[prn - 1].af2 * tk + deltatr_dot;
			/*
			deltatr = F * raw->gpe[prn - 1].ecc * sqrt(raw->gpe[prn - 1].A) * cos(Ek) * dEk;
			double dtsv = raw->gpe[prn - 1].af1 + 2 * raw->gpe[prn - 1].af2 * tk + deltatr;
			*/


			raw->sat_gps[i].satClkDot = dtsv;
			dtime = IF / C_SPEED;
			gTime->SecofWeek -= dtime;

			//��������ǵ��Ӳ�֮�����¼���gTime��֮���ٽ���λ���ٶȵļ����Լ�����
			gTime->SecofWeek -= raw->sat_gps[i].satClk;

			/*************************************************************************
									   ����λ�ü���ģ��
			**************************************************************************/
			//��������������ο���Ԫ��ʱ��tk

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

				//cout <<"PRN=" <<i+1<< "���ǵ�����������" << endl;
				continue;
			}

			//��ƽ���˶����ٶȽ��и���
			n = n0 + raw->gpe[prn - 1].DeltaN;

			//����ƽ�����
			Mk = raw->gpe[prn - 1].M0 + n * tk;
			//��������ƫ�����
			/* �������ĳ�ʼֵ */
			E = 0;
			Ek = 0;
			do
			{
				E = Ek;
				Ek = Mk + raw->gpe[prn - 1].ecc * sin(E);
				iterator++;
				if (iterator > 100)
				{
					cout << "ƫ����Ǽ���ʧ��" << endl;
					break;
				}
			} while ((fabs(Ek - E) > 1e-14));

			//����������,����Ϊ�����ƣ�rad��
			vk = atan2((sqrt(1 - raw->gpe[prn - 1].ecc * raw->gpe[prn - 1].ecc)) * sin(Ek), cos(Ek) - raw->gpe[prn - 1].ecc);
			//���������Ǿ�
			Faik = vk + raw->gpe[prn - 1].omega;
			//������׵��͸�����
			//���������Ǿ�ĸ�����
			deltauk = raw->gpe[prn - 1].cus * sin(2 * Faik) + raw->gpe[prn - 1].cuc * cos(2 * Faik);
			//�����򾶵ĸ�����
			deltark = raw->gpe[prn - 1].crs * sin(2 * Faik) + raw->gpe[prn - 1].crc * cos(2 * Faik);
			//��������ǵĸ�����
			deltaik = raw->gpe[prn - 1].cis * sin(2 * Faik) + raw->gpe[prn - 1].cic * cos(2 * Faik);
			//���㾭�������������Ǿ�
			uk = Faik + deltauk;
			//���㾭����������
			rk = raw->gpe[prn - 1].A * (1 - raw->gpe[prn - 1].ecc * cos(Ek)) + deltark;
			//���㾭��������Ĺ�����
			ik = raw->gpe[prn - 1].I0 + raw->gpe[prn - 1].I0Dot * tk + deltaik;
			//���������ڹ��ƽ���ϵ�λ��
			xk = rk * cos(uk);
			yk = rk * sin(uk);

			Omegak = raw->gpe[prn - 1].Omega0 + (raw->gpe[prn - 1].omegaDot - GPS_OmegaDot) * tk - GPS_OmegaDot * raw->gpe[prn - 1].toe;

			//�õ����ǵ�λ�ã���λ��m��
			//��ʱ��û��ʩ�ӵ�����ת����
			raw->sat_gps[i].satXYZ.x = xk * cos(Omegak) - yk * cos(ik) * sin(Omegak);
			raw->sat_gps[i].satXYZ.y = xk * sin(Omegak) + yk * cos(ik) * cos(Omegak);
			raw->sat_gps[i].satXYZ.z = yk * sin(ik);

			//�õ����������Դ�����֮�������λ��
			dtime += raw->sat_gps[i].satClk;
			double rotateAngle, tempSatX, tempSatY;
			rotateAngle = Omega_WGS * (dtime);
			tempSatX = raw->sat_gps[i].satXYZ.x + raw->sat_gps[i].satXYZ.y * rotateAngle;
			tempSatY = raw->sat_gps[i].satXYZ.y - raw->sat_gps[i].satXYZ.x * rotateAngle;
			raw->sat_gps[i].satXYZ.x = tempSatX;
			raw->sat_gps[i].satXYZ.y = tempSatY;


			/*************************************************************************
									   �����ٶȼ���ģ��
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

			//��R����ֵ
			//�����xk��yk��������λ�ü���ģ���еõ��������ڹ��ƽ���λ��
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
			//�������֮���ʱ�仹ԭ
			gTime->SecofWeek = timesave;
		}

	}

	return 0;
}

/*************************************************************************
BDSPOSVEL BDS����λ���ٶȼ��㺯��
���룺ԭʼ����RAWDATA��ָ��raw, �źŷ���ʱ��ʱ��gTime
�������ӦPrn�ŵ�BDS���ǣ�MEO,IGSO,GEO)����Ӧʱ��ʱ�������λ��
�Ѳ��� 2021.12.21
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
				// ������ǲ����� ������ǵ�λ�ü�������Ĳ�����
				raw->sat_bds[i].Valid = false;
				//cout << "The health state of the satellite is abnormal!" << endl;
				continue;
			}
			// ������GEO���ǵ�λ�ü���
			if (prn <= 5 || prn >= 59)
			{
				/* �����Ĵ��� */
				double iterator = 0;

				//�������prn�����ǵ�PRN�ţ����������������n-1
				double n0;                       // ƽ���˶����ٶ�
				double tr;                       // �źŷ���ʱ��ʱ��
				double tk;                       // ����������ο���Ԫ��ʱ��
				double n;                        // ���������ƽ�����ٶ�
				double Mk;                       // ƽ�����
				double Ek;                       // ƫ�����
				double vk;                       // ������
				double Faik;                     // �����Ǿ�
				double deltauk;                  // �����Ǿ�ĸ�����
				double uk;                       // �����Ǿࣨ������
				double deltark;                  // �򾶵ĸ�����
				double rk;                       // �򾶣�������
				double deltaik;                  // �����ǵĸ�����
				double ik;                       // �����ǣ�������
				double xk, yk;                   // �����ڹ��ƽ���ϵ�λ��
				double Omegak;                   // ������������㾭��

				//�����õ�ʱ��
				//gTime->Week = 2184;
				//gTime->SecofWeek = 26700.0;

				int q;//range������
				q = FindSatObsIndex(prn, BDS, &raw->obs);
				raw->sat_bds[i].PRN = prn;
				raw->sat_bds[i].gpst = *gTime;
				raw->sat_bds[i].Valid = false;

				//��������ʵ�ʵķ���ʱ�䣨δ���Ӳ������
				double dtime;
				//�������Ƿ���ʱ�������ʱ�ĵ�һ��deltat����α�����C��ã�
				double IF; // α���IF��Ϲ۲�ֵ
				IF = FB1_BDS * FB1_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS) * raw->obs.range[q].P1 - FB3_BDS * FB3_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS) * raw->obs.range[q].P2;

				//����ƽ���˶����ٶ�
				n0 = sqrt(BDS_Miu / pow(raw->bde[prn - 1].A, 3));

				//��������������ο���Ԫ��ʱ��tk
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

					//cout << "PRN=" << i + 1 << "���ǵ�����������" << endl;
					continue;

				}

				//��ƽ���˶����ٶȽ��и���
				n = n0 + raw->bde[prn - 1].DeltaN;

				//����ƽ�����
				Mk = raw->bde[prn - 1].M0 + n * tk;
				//��������ƫ�����
				/* �������ĳ�ʼֵ */
				double E = 0;
				Ek = 0;
				do
				{
					E = Ek;
					Ek = Mk + raw->bde[prn - 1].ecc * sin(E);
					iterator++;
					if (iterator > 100)
					{
						cout << "ƫ����Ǽ���ʧ��" << endl;
						break;
					}
				} while ((fabs(Ek - E) > 1e-14));

				/*************************************************************************
										   �����Ӳ����ģ��
				**************************************************************************/
				double F, deltatr, tempT, deltat, dt0, deltatr_dot = 0;
				double rho;
				double Ekdot;

				Ekdot = n / (1 - raw->bde[prn - 1].ecc * cos(Ek));

				//�����Ӳ����
				F = -2 * sqrt(BDS_Miu) / pow(C_SPEED, 2);
				deltatr = F * raw->bde[prn - 1].ecc * (sqrt(raw->bde[prn - 1].A)) * sin(Ek);
				double dt = raw->bde[prn - 1].a0 + raw->bde[prn - 1].a1 * tk + raw->bde[prn - 1].a2 * pow(tk, 2) + deltatr;
				tk -= dt;
				/*tempT = (raw->gpe[prn - 1].GPS_G.Week - raw->gpe[prn - 1].week) * 604800 - raw->gpe[prn - 1].toc;
				deltat = raw->gpe[prn - 1].af0 + raw->gpe[prn - 1].af1 * tempT + raw->gpe[prn - 1].af2 * pow(tempT, 2) + deltatr;*/
				deltat = raw->bde[prn - 1].a0 + raw->bde[prn - 1].a1 * tk + raw->bde[prn - 1].a2 * pow(tk, 2) + deltatr - raw->bde[prn - 1].tgd1;
				raw->sat_bds[i].satClk = deltat;

				//�������ٸ���
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
										   ����λ�ü���ģ��
				**************************************************************************/
				//����ƽ���˶����ٶ�
				n0 = sqrt(BDS_Miu / pow(raw->bde[prn - 1].A, 3));

				//��������������ο���Ԫ��ʱ��tk
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

					//cout << "PRN=" << i + 1 << "���ǵ�����������" << endl;
					continue;

				}

				//��ƽ���˶����ٶȽ��и���
				n = n0 + raw->bde[prn - 1].DeltaN;

				//����ƽ�����
				Mk = raw->bde[prn - 1].M0 + n * tk;
				//��������ƫ�����
				/* �������ĳ�ʼֵ */
				E = 0;
				Ek = 0;
				do
				{
					E = Ek;
					Ek = Mk + raw->bde[prn - 1].ecc * sin(E);
					iterator++;
					if (iterator > 100)
					{
						cout << "ƫ����Ǽ���ʧ��" << endl;
						break;
					}
				} while ((fabs(Ek - E) > 1e-14));

				//����������,����Ϊ�����ƣ�rad��
				vk = atan2((sqrt(1 - raw->bde[prn - 1].ecc * raw->bde[prn - 1].ecc)) * sin(Ek), cos(Ek) - raw->bde[prn - 1].ecc);
				//���������Ǿ�
				Faik = vk + raw->bde[prn - 1].omega;
				//������׵��͸�����
				//���������Ǿ�ĸ�����
				deltauk = raw->bde[prn - 1].cus * sin(2 * Faik) + raw->bde[prn - 1].cuc * cos(2 * Faik);
				//�����򾶵ĸ�����
				deltark = raw->bde[prn - 1].crs * sin(2 * Faik) + raw->bde[prn - 1].crc * cos(2 * Faik);
				//��������ǵĸ�����
				deltaik = raw->bde[prn - 1].cis * sin(2 * Faik) + raw->bde[prn - 1].cic * cos(2 * Faik);
				//���㾭�������������Ǿ�
				uk = Faik + deltauk;
				//���㾭����������
				rk = raw->bde[prn - 1].A * (1 - raw->bde[prn - 1].ecc * cos(Ek)) + deltark;
				//���㾭��������Ĺ�����
				ik = raw->bde[prn - 1].i0 + raw->bde[prn - 1].IDOT * tk + deltaik;
				//���������ڹ��ƽ���ϵ�λ��
				xk = rk * cos(uk);
				yk = rk * sin(uk);
				Omegak = raw->bde[prn - 1].Omega0 + (raw->bde[prn - 1].omegadot) * tk - BDS_OmegaDot * raw->bde[prn - 1].toe;
				//�õ����ǵ�λ�ã���λ��m��
				double x, y, z;          // ������ת֮ǰ������λ��
				double x_r, y_r, z_r;    // ������ת֮�������λ��

				x = xk * cos(Omegak) - yk * cos(ik) * sin(Omegak);
				y = xk * sin(Omegak) + yk * cos(ik) * cos(Omegak);
				z = yk * sin(ik);

				//Rx��ת����
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

				//Rz��ת����
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
				matrix POS_R;//������ת����֮���λ�þ���
				MatrixMulti(&XZ, &temppos, &POS_R, POS_Rdata);

				//�õ�δ�������Դ�����������λ��
				raw->sat_bds[i].satXYZ.x = POS_Rdata[0];
				raw->sat_bds[i].satXYZ.y = POS_Rdata[1];
				raw->sat_bds[i].satXYZ.z = POS_Rdata[2];

				////�õ����������Դ�����֮�������λ��
				dtime += raw->sat_bds[i].satClk;
				double rotateAngle, tempSatX, tempSatY;
				rotateAngle = Omega_BDS * (dtime);
				tempSatX = raw->sat_bds[i].satXYZ.x + raw->sat_bds[i].satXYZ.y * rotateAngle;
				tempSatY = raw->sat_bds[i].satXYZ.y - raw->sat_bds[i].satXYZ.x * rotateAngle;
				raw->sat_bds[i].satXYZ.x = tempSatX;
				raw->sat_bds[i].satXYZ.y = tempSatY;

				/*************************************************************************
										   �����ٶȼ���ģ��
				**************************************************************************/
				double Faikdot, ukdot, rkdot, ikdot, Omegakdot;
				double Rdata[12];
				double xkDot, ykDot;
				Ekdot = n / (1 - raw->bde[prn - 1].ecc * cos(Ek));
				Faikdot = sqrt((1 + raw->bde[prn - 1].ecc) / (1 - raw->bde[prn - 1].ecc)) * pow((cos(vk / 2) / (cos(Ek / 2))), 2) * Ekdot;

				ukdot = 2 * (raw->bde[prn - 1].cus * cos(2 * Faikdot) - raw->bde[prn - 1].cuc * sin(2 * Faik)) * Faikdot + Faikdot;
				rkdot = (raw->bde[prn - 1].A) * (raw->bde[prn - 1].ecc) * sin(Ek) * Ekdot + 2 * (raw->bde[prn - 1].crs * cos(2 * Faikdot) - raw->bde[prn - 1].crc * sin(2 * Faik)) * Faikdot;
				ikdot = raw->bde[prn - 1].IDOT + 2 * (raw->bde[prn - 1].cis * cos(2 * Faik) - raw->bde[prn - 1].cic * sin(2 * Faik)) * Faikdot;

				//������ྭ�ı仯�ʣ����ﲻ��������ת���ٶ�
				Omegakdot = raw->bde[prn - 1].omegadot;

				xkDot = rkdot * cos(uk) - rk * ukdot * sin(uk);
				ykDot = rkdot * sin(uk) + rk * ukdot * cos(uk);

				//�����������Զ������ת����ϵ�е��ٶ�
				double vel_GK[3] =
				{
				 xkDot * cos(Omegak) - ykDot * sin(Omegak) * cos(ik) + yk * ikdot * sin(Omegak) * sin(ik) -
				(xk * sin(Omegak) - yk * cos(ik) * cos(Omegak)) * Omegakdot,
				 xkDot * sin(Omegak) + ykDot * cos(ik) * cos(Omegak) - yk * sin(ik) * cos(Omegak) * ikdot
				+ (xk * cos(Omegak) - yk * cos(ik) * sin(Omegak)) * Omegakdot,
				 ykDot * sin(ik) + yk * ikdot * cos(ik)
				};

				//����Rz_dot(we,tk),Ϊת���������
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
				matrix temp1;   // �������Rz��Rx��˵���ʱ����
				matrix temp2;   // �������Rzdot��Rx��˵���ʱ����
				MatrixMulti(&Rz, &Rx, &temp1, tempdata);
				MatrixMulti(&Rzdot, &Rx, &temp2, tempdata1);

				matrix temp3;   // �������ʽ��������һ��
				matrix temp4;   // �������ʽ��������һ��
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
				//�������֮���ʱ�仹ԭ
				gTime->SecofWeek = timesave;
			}
			// �����������ǵ�λ�ü���  �Ѳ������
			else
			{
				/* �����Ĵ��� */
				double iterator = 0;

				//�������prn�����ǵ�PRN�ţ����������������n-1
				double n0;                       // ƽ���˶����ٶ�
				double tr;                       // �źŷ���ʱ��ʱ��
				double t;                        // �źŷ���ʱ��ʱ��
				double tk;                       // ����������ο���Ԫ��ʱ��
				double n;                        // ���������ƽ�����ٶ�
				double Mk;                       // ƽ�����
				double Ek;                       // ƫ�����
				double vk;                       // ������
				double Faik;                     // �����Ǿ�
				double deltauk;                  // �����Ǿ�ĸ�����
				double uk;                       // �����Ǿࣨ������
				double deltark;                  // �򾶵ĸ�����
				double rk;                       // �򾶣�������
				double deltaik;                  // �����ǵĸ�����
				double ik;                       // �����ǣ�������
				double xk, yk;                   // �����ڹ��ƽ���ϵ�λ��
				double Omegak;                   // ������������㾭��

				/*GPSTIME* gTime = new GPSTIME;*/
				//gTime->Week = 2184;
				//gTime->SecofWeek = 26700.0;
				int q;//range������
				q = FindSatObsIndex(prn, BDS, &raw->obs);
				raw->sat_bds[i].PRN = prn;
				raw->sat_bds[i].gpst = *gTime;
				raw->sat_bds[i].Valid = false;

				//��������ʵ�ʵķ���ʱ�䣨δ���Ӳ������
				double dtime;
				//�������Ƿ���ʱ�������ʱ�ĵ�һ��deltat����α�����C��ã�
				double IF; // α���IF��Ϲ۲�ֵ
				IF = FB1_BDS * FB1_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS) * raw->obs.range[q].P1 - FB3_BDS * FB3_BDS / (FB1_BDS * FB1_BDS - FB3_BDS * FB3_BDS) * raw->obs.range[q].P2;


				//����ƽ���˶����ٶ�
				n0 = sqrt(BDS_Miu / pow(raw->bde[prn - 1].A, 3));

				//��������������ο���Ԫ��ʱ��tk

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
					//cout << "PRN=" << i + 1 << "���ǵ�����������" << endl;
					continue;
				}

				//��ƽ���˶����ٶȽ��и���
				n = n0 + raw->bde[prn - 1].DeltaN;

				//����ƽ�����
				Mk = raw->bde[prn - 1].M0 + n * tk;
				//��������ƫ�����
				/* �������ĳ�ʼֵ */
				double E = 0;
				Ek = 0;
				do
				{
					E = Ek;
					Ek = Mk + raw->bde[prn - 1].ecc * sin(E);
					iterator++;
					if (iterator > 100)
					{
						cout << "ƫ����Ǽ���ʧ��" << endl;
						break;
					}
				} while ((fabs(Ek - E) > 1e-14));

				/*************************************************************************
						   �����Ӳ����ģ��
				**************************************************************************/
				double Ekdot;
				double F, deltatr, tempT, deltat, dt0, deltatr_dot = 0;
				double rho;

				Ekdot = n / (1 - raw->bde[prn - 1].ecc * cos(Ek));
				//�����Ӳ����
				F = -2 * sqrt(BDS_Miu) / pow(C_SPEED, 2);
				deltatr = F * raw->bde[prn - 1].ecc * (sqrt(raw->bde[prn - 1].A)) * sin(Ek);
				double dt = raw->bde[prn - 1].a0 + raw->bde[prn - 1].a1 * tk + raw->bde[prn - 1].a2 * pow(tk, 2) + deltatr;
				tk -= dt;
				/*tempT = (raw->bde[prn - 1].GPS_G.Week - raw->bde[prn - 1].week) * 604800 - raw->bde[prn - 1].toc;
				deltat = raw->bde[prn - 1].af0 + raw->bde[prn - 1].af1 * tempT + raw->bde[prn - 1].af2 * pow(tempT, 2) + deltatr;*/
				deltat = raw->bde[prn - 1].a0 + raw->bde[prn - 1].a1 * tk + raw->bde[prn - 1].a2 * pow(tk, 2) + deltatr - raw->bde[prn - 1].tgd1;
				raw->sat_bds[i].satClk = deltat;

				//�������ٸ���
				deltatr_dot = F * raw->bde[prn - 1].ecc * sqrt(raw->bde[prn - 1].A) * cos(Ek) * Ekdot;
				double dtsv = raw->bde[prn - 1].a1 + 2 * raw->bde[prn - 1].a2 * tk + deltatr_dot;
				/*
				deltatr = F * raw->bde[prn - 1].ecc * sqrt(raw->bde[prn - 1].A) * cos(Ek) * dEk;
				double dtsv = raw->bde[prn - 1].af1 + 2 * raw->bde[prn - 1].af2 * tk + deltatr;
				*/
				raw->sat_bds[i].satClkDot = dtsv;

				dtime = IF / C_SPEED;
				gTime->SecofWeek -= dtime;
				//��������ǵ��Ӳ�֮�����¼���gTime��֮���ٽ���λ���ٶȵļ����Լ�����
				gTime->SecofWeek -= raw->sat_bds[i].satClk;


				/*************************************************************************
										   ����λ�ü���ģ��
				**************************************************************************/
				//����ƽ���˶����ٶ�
				n0 = sqrt(BDS_Miu / pow(raw->bde[prn - 1].A, 3));

				//��������������ο���Ԫ��ʱ��tk

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
					//cout << "PRN=" << i + 1 << "���ǵ�����������" << endl;
					continue;
				}

				//��ƽ���˶����ٶȽ��и���
				n = n0 + raw->bde[prn - 1].DeltaN;

				//����ƽ�����
				Mk = raw->bde[prn - 1].M0 + n * tk;
				//��������ƫ�����
				/* �������ĳ�ʼֵ */
				E = 0;
				Ek = 0;
				do
				{
					E = Ek;
					Ek = Mk + raw->bde[prn - 1].ecc * sin(E);
					iterator++;
					if (iterator > 100)
					{
						cout << "ƫ����Ǽ���ʧ��" << endl;
						break;
					}
				} while ((fabs(Ek - E) > 1e-14));

				//����������,����Ϊ�����ƣ�rad��
				vk = atan2((sqrt(1 - raw->bde[prn - 1].ecc * raw->bde[prn - 1].ecc)) * sin(Ek), cos(Ek) - raw->bde[prn - 1].ecc);
				//���������Ǿ�
				Faik = vk + raw->bde[prn - 1].omega;
				//������׵��͸�����
				//���������Ǿ�ĸ�����
				deltauk = raw->bde[prn - 1].cus * sin(2 * Faik) + raw->bde[prn - 1].cuc * cos(2 * Faik);
				//�����򾶵ĸ�����
				deltark = raw->bde[prn - 1].crs * sin(2 * Faik) + raw->bde[prn - 1].crc * cos(2 * Faik);
				//��������ǵĸ�����
				deltaik = raw->bde[prn - 1].cis * sin(2 * Faik) + raw->bde[prn - 1].cic * cos(2 * Faik);
				//���㾭�������������Ǿ�
				uk = Faik + deltauk;
				//���㾭����������
				rk = raw->bde[prn - 1].A * (1 - raw->bde[prn - 1].ecc * cos(Ek)) + deltark;
				//���㾭��������Ĺ�����
				ik = raw->bde[prn - 1].i0 + raw->bde[prn - 1].IDOT * tk + deltaik;
				//���������ڹ��ƽ���ϵ�λ��
				xk = rk * cos(uk);
				yk = rk * sin(uk);
				Omegak = raw->bde[prn - 1].Omega0 + (raw->bde[prn - 1].omegadot - BDS_OmegaDot) * tk - BDS_OmegaDot * raw->bde[prn - 1].toe;
				//�õ����ǵ�λ�ã���λ��m��--��ʱ��û�о��������Դ�����
				raw->sat_bds[i].satXYZ.x = xk * cos(Omegak) - yk * cos(ik) * sin(Omegak);
				raw->sat_bds[i].satXYZ.y = xk * sin(Omegak) + yk * cos(ik) * cos(Omegak);
				raw->sat_bds[i].satXYZ.z = yk * sin(ik);

				////�õ����������Դ�����֮�������λ��
				dtime += raw->sat_bds[i].satClk;
				double rotateAngle, tempSatX, tempSatY;
				rotateAngle = Omega_BDS * (dtime);
				tempSatX = raw->sat_bds[i].satXYZ.x + raw->sat_bds[i].satXYZ.y * rotateAngle;
				tempSatY = raw->sat_bds[i].satXYZ.y - raw->sat_bds[i].satXYZ.x * rotateAngle;
				raw->sat_bds[i].satXYZ.x = tempSatX;
				raw->sat_bds[i].satXYZ.y = tempSatY;

				/*************************************************************************
										   �����ٶȼ���ģ��
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

				//��R����ֵ
				//�����xk��yk��������λ�ü���ģ���еõ��������ڹ��ƽ���λ��
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
