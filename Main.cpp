#define _CRT_SECURE_NO_WARNINGS
#include<iostream>
#include<iomanip>
#include<fstream>
#include<string>
#include<cmath>
#include"Matrix.h"
#include"sockets.h"
#include"Time.h"
#include"Coordinate.h"
#include"ConstNums.h"
#include"SatPos.h"
#include"RTK.h"
#include"SPP_SPV.h"

using namespace std;
double  RefPos[3] = { -2267804.5263, 5009342.3723 , 3220991.8632 };


void  PrintBdsEph(RAWDATA* Raw)
{
	for (int i = 0; i < MAXBDSPRN; i++)
	{
		if (Raw->bde[i].Prn == i + 1)
		{
			cout << "C" << setw(2) << setfill('0') << Raw->bde[i].Prn << " ";
			GPSTIME gpst;
			gpst.Week = Raw->bde->week + 1356;
			gpst.SecofWeek = Raw->bde->toe;
			COMMONTIME comm;
			GPS2CMT(&comm, &gpst);
			cout << comm.Year << " " << comm.Month << " " << comm.Day << " " << comm.Hour << " " << comm.Minute << " " << (int)comm.Second << " ";
			cout << setprecision(12) << Raw->bde[i].a0 << " " << Raw->bde[i].a1 << " " << Raw->bde[i].a2 << endl;
			cout << setprecision(12) << Raw->bde[i].AODE << " " << Raw->bde[i].crs << " " << Raw->bde[i].DeltaN << " " << Raw->bde[i].M0 << endl;
			cout << setprecision(12) << Raw->bde[i].cuc << " " << Raw->bde[i].ecc << " " << Raw->bde[i].cus << " " << Raw->bde[i].A << endl;
			cout << setprecision(12) << Raw->bde[i].toe << " " << Raw->bde[i].cic << " " << Raw->bde[i].Omega0 << " " << Raw->bde[i].cis << endl;
			cout << setprecision(12) << Raw->bde[i].i0 << " " << Raw->bde[i].crc << " " << Raw->bde[i].omega << " " << Raw->bde[i].omegadot << endl;
			cout << setprecision(12) << Raw->bde[i].IDOT << " " << "��" << " " << Raw->bde[i].week << " " << "��" << endl;
			cout << setprecision(12) << "��" << " " << Raw->bde[i].health << " " << Raw->bde[i].tgd1 << " " << "��" << endl;
			cout << setprecision(12) << "��" << " " << "��" << " " << "��" << " " << "��" << endl;
			cout << endl;
		}
	}

}

void  PrintGpsEph(RAWDATA* Raw)
{
	for (int i = 0; i < MAXGPSPRN; i++)
	{
		if (Raw->gpe[i].Prn == i + 1)
		{
			cout << "G" << setw(2) << setfill('0') << Raw->gpe[i].Prn << " ";
			COMMONTIME comm;
			GPSTIME gpst;
			gpst.Week = Raw->gpe[i].week;
			gpst.SecofWeek = Raw->gpe[i].toe;
			GPS2CMT(&comm, &gpst);
			cout << comm.Year << " " << comm.Month << " " << comm.Day << " " << comm.Hour << " " << comm.Minute << " " << (int)comm.Second << " " << setprecision(12) << Raw->gpe[i].af0 << " " << Raw->gpe[i].af1 << " " << Raw->gpe[i].af2 << endl;
			cout << setprecision(12) << Raw->gpe[i].IODE[0] << " " << Raw->gpe[i].crs << " " << Raw->gpe[i].DeltaN << " " << Raw->gpe[i].M0 << endl;
			cout << setprecision(12) << Raw->gpe[i].cuc << " " << Raw->gpe[i].ecc << " " << Raw->gpe[i].cus << " " << sqrt(Raw->gpe[i].A) << endl;
			cout << setprecision(12) << Raw->gpe[i].toe << " " << Raw->gpe[i].cic << " " << Raw->gpe[i].Omega0 << " " << Raw->gpe[i].cis << endl;
			cout << setprecision(12) << Raw->gpe[i].I0 << " " << Raw->gpe[i].crc << " " << Raw->gpe[i].omega << " " << Raw->gpe[i].omegaDot << endl;
			cout << setprecision(12) << Raw->gpe[i].I0Dot << " " << "��" << " " << Raw->gpe[i].zweek << " " << "��" << endl;
			cout << setprecision(12) << "��" << " " << Raw->gpe[i].health << " " << Raw->gpe[i].tgd << " " << "��" << endl;
			cout << setprecision(12) << "��" << " " << "��" << " " << "��" << " " << "��" << endl;
			cout << endl;

		}
	}
}

void  PrintObsdata(RAWDATA* Raw)
{
	COMMONTIME comm;
	GPS2CMT(&comm, &Raw->obs.gpst);
	cout << "> " << comm.Year << " " << comm.Month << " " << comm.Day << " " << comm.Hour << " " << comm.Minute << " " << (int)comm.Second << " " << Raw->obs.SATNUMS << endl;

	for (int i = 0; i < Raw->obs.SATNUMS; i++)
	{
		int sys = Raw->obs.range[i].Sys;
		if (sys == 0)
		{
			cout << "G" << setw(2) << setfill('0') << Raw->obs.range[i].Prn << " ";
			cout << setprecision(12) << Raw->obs.range[i].P1 << " " << Raw->obs.range[i].P2 << " " << Raw->obs.range[i].L1 << " " << Raw->obs.range[i].L2 << " " << Raw->obs.range[i].D1 << " " << Raw->obs.range[i].D2 << " " << Raw->obs.range[i].L1Noise << " " << Raw->obs.range[i].L2Noise << " " << Raw->obs.range[i].P1Noise << " " << Raw->obs.range[i].P2Noise << " " << Raw->obs.range[i].Snr1 << " " << Raw->obs.range[i].Snr2 << endl;
		}
		else if (sys == 1)
		{
			cout << "C" << setw(2) << setfill('0') << Raw->obs.range[i].Prn << " ";
			cout << setprecision(12) << Raw->obs.range[i].P1 << " " << Raw->obs.range[i].P2 << " " << Raw->obs.range[i].L1 << " " << Raw->obs.range[i].L2 << " " << Raw->obs.range[i].D1 << " " << Raw->obs.range[i].D2 << " " << Raw->obs.range[i].L1Noise << " " << Raw->obs.range[i].L2Noise << " " << Raw->obs.range[i].P1Noise << " " << Raw->obs.range[i].P2Noise << " " << Raw->obs.range[i].Snr1 << " " << Raw->obs.range[i].Snr2 << endl;

		}
	}
	cout << endl;
}

// ��������ȡ�ļ��Ƿ���ȷ�ĺ���
int testdecoder()
{
	FILE* fp_base;
	FILE* fp_rover;
	int Val;
	RAWDATA* base = new RAWDATA;
	RAWDATA* rover = new RAWDATA;
	GPSTIME* gTime = new GPSTIME;
	fp_base = fopen("Base.log", "rb");
	fp_rover = fopen("Rove.log", "rb");
	if (fp_base == NULL)
	{
		cout << "Cannot Open the Base File." << endl;
		return -1;
	}
	else if (fp_rover == NULL)
	{
		cout << "Cannot Open the Rover File" << endl;
		return -1;
	}
	while (1)
	{
		//DecodeHOEM7(fp_base, base);
		//DecodeHOEM7(fp_rover, rover);
		cout << "BDS����" << endl;
		PrintBdsEph(base);
		cout << "GPS����" << endl;
		PrintGpsEph(base);
		cout << "�۲�����" << endl;
		PrintObsdata(base);
	}
	return 0;
}

// �������SPP�Ƿ�׼ȷ�ĺ���
int testspp()
{
	//RefPos[3] = { -2267804.5263, 5009342.3723 , 3220991.8632 };
	FILE* fp;
	int Val;
	RAWDATA* Raw = new RAWDATA;
	GPSTIME* gTime = new GPSTIME;
	fp = fopen("Base.log", "rb");
	if (fp == NULL)
	{
		cout << "Cannot open GPS obs file." << endl;
		return 0;
	}
	cout << "State " << " " << "Week" << "  " << "Seconds" << "  " << "nG" << " " << "nB";
	cout << "        " << "X" << "                " << "Y" << "                " << "Z" << "            " << "PDOP" << "        " << "Sigma0";
	cout << "      " << "Vx" << "          " << "Vy" << "          " << "Vz";
	cout << "        " << "N" << "       " << "E" << "       " << "U" << endl;
	while (1)
	{
		bool flag;
		// DecodeHOEM7(fp, Raw);
		flag = NUMofSAT(Raw);
		if (flag == false)
		{
			continue;
		}
		if (feof(fp))
		{
			break;
		}
		SPP(Raw);
		SPV(Raw);
		flag = false;
	}
}

int main()
{
	string filename = "outputRTK.txt";
	PrintHeadRTK(filename);
	if (MODE == 1)
	{
		FILE* fp_base;
		FILE* fp_rover;
		//fp_base = fopen("Base2.log", "rb");
		//fp_rover = fopen("Rove2.log", "rb");
		//fp_base = fopen("oem719-202202131000-1.bin", "rb");
		//fp_rover = fopen("oem719-202202131000-2.bin", "rb");
		fp_base = fopen("base_202204031340.oem719", "rb");
		fp_rover = fopen("rover_202204031340.oem719", "rb");
		double refx = -2267804.5263;
		double refy = 5009342.3723;
		double refz = 3220991.8632;
		/*double SPPstate = -1;*/
		RAWDATA* rawB = new RAWDATA;
		RAWDATA* rawR = new RAWDATA;
		RAWDATA rawB_save[5], rawR_save[5];
		memset(rawB_save, 0, 5 * sizeof(RAWDATA));
		memset(rawR_save, 0, 5 * sizeof(RAWDATA));
		if (fp_base == NULL)
		{
			cout << "Cannot Open the Base File." << endl;
			return -1;
		}
		else if (fp_rover == NULL)
		{
			cout << "Cannot Open the Rover File" << endl;
			return -1;
		}
		/*********************��Whileѭ����������ʱ��ͬ����SPP�Ľ���********************/
		while (1)
		{

			int returnB = 0;
			int returnR = 0;
			double timeB = -1000.0;
			double timeR = -1000.0;
			bool stateB = false;
			bool stateR = false;
			int Val;
			returnB = DecodeHOEM7(fp_base, rawB);
			returnR = DecodeHOEM7(fp_rover, rawR);
			GPSTIME Time;
			Val = RawGroupDataSelect(rawB_save, rawR_save, rawB, rawR, &Time, returnB, returnR);// ƥ�䲢����RAW
			timeB = rawB->obs.gpst.SecofWeek;
			timeR = rawR->obs.gpst.SecofWeek;
			if (Val == 1 || stateB == false || stateR == false)// ����ʱ�䲻�ܽ���ƥ���SPP����ʧ��
			{
				Val = TimeMatching(rawB, rawR, fp_base, fp_rover);// �����ƶ�ʱ��
			}
			if (Val == 0 & returnB == 43)
			{
				SPP(rawB);
				SPP(rawR);
				stateB = rawB->SPPstate;
				stateR = rawR->SPPstate;
				if (stateB == true && stateR == true)
				{
					/*********************��Զ�λ����********************/
					int rtkstate = SingleEpochRTK(rawB, rawR);
					PrintResulttoFile(filename ,&rawR->rtk);
				}
			}

		}
	}

	else if (MODE == 0)
	{
		SOCKET SocksB, SocksR;

		if (OpenSockets(SocksB, "47.114.134.129", 7190) == false) //�����쳣����
		{
			cout << "Cannot open socketB" << endl;
			return 0;
		}
		if (OpenSockets(SocksR, "47.114.134.129", 7200) == false) //�����쳣����
		{
			cout << "Cannot open socketR" << endl;
			return 0;
		}
		double refx = -2267804.5263;
		double refy = 5009342.3723;
		double refz = 3220991.8632;

		unsigned char BuffB[MAXBUFLEN * 5];// ��Ž��յ������ݵĻ�����
		unsigned char BuffR[MAXBUFLEN * 5];// 5������
		// len1�������Ԫ������������ len2���ϸ���Ԫ��������������
		int len1B, len2B;// Base
		int len1R, len2R;// Rove
		len1B = len2B = len1R = len2R = 0;
		int ValB, ValR;// Val ���뷵����ֵ
		int ValTime;// ����ʱ����
		int ValSppB, ValSppR;// SPP����ֵ
		GPSTIME Time;
		double BaseTime, RoveTime;// ��׼վ������վʱ��

		RAWDATA* RawBase = new RAWDATA;
		RAWDATA* RawRove = new RAWDATA;
		RAWDATA  RawBGroup[5], RawRGroup[5];
		
		memset(&RawBGroup, 0, 5 * sizeof(RAWDATA));
		memset(&RawRGroup, 0, 5 * sizeof(RAWDATA));

		while (1)
		{
			len1B = recv(SocksB, (char*)BuffB + len2B, MAXBUFLEN * 5 - len2B, 0);// ��������ƴ��
			len1R = recv(SocksR, (char*)BuffR + len2R, MAXBUFLEN * 5 - len2R, 0);
			// ��ȡ��ǰʱ�̴����ڶ�����������
			ValB = DecodeOEM7MessageSock(BuffB, len1B, len2B, RawBase);
			ValR = DecodeOEM7MessageSock(BuffR, len1R, len2R, RawRove);
			if (ValB == 43 || ValR == 43)// ����ȡ�Ĺ۲�ֵ
			{
				ValTime = RawGroupDataSelect(RawBGroup, RawRGroup, RawBase, RawRove, &Time, ValB, ValR);// ƥ�䲢����RAW
				if (ValTime == -1)// ƥ��ʧ��
				{
					BaseTime = RawBase->obs.gpst.Week * 604800.0 + RawBase->obs.gpst.SecofWeek;
					RoveTime = RawRove->obs.gpst.Week * 604800.0 + RawRove->obs.gpst.SecofWeek;

					// ���ʱ��û�ж���
					while (abs(BaseTime - RoveTime) > 0.001 || BaseTime == 0 || RoveTime == 0 )
					{
						if (BaseTime < RoveTime)
						{
							Sleep(500);// ����0.5s�����еȴ�
							len1B = recv(SocksB, (char*)BuffB + len2B, MAXBUFLEN * 5 - len2B, 0);
							ValB = DecodeOEM7MessageSock(BuffB, len1B, len2B, RawBase);
						}
						else
						{
							Sleep(500);
							len1R = recv(SocksR, (char*)BuffR + len2R, MAXBUFLEN * 5 - len2R, 0);
							ValR = DecodeOEM7MessageSock(BuffR, len1R, len2R, RawRove);
						}
						BaseTime = RawBase->obs.gpst.Week * 604800.0 + RawBase->obs.gpst.SecofWeek;
						RoveTime = RawRove->obs.gpst.Week * 604800.0 + RawRove->obs.gpst.SecofWeek;
					}
					ValTime = 0;// ת������ƥ��״̬
				}
				if (ValTime == 0)// ����ʱ�����ƥ��
				{
					// ��7190�˿ڵ�������ֵ��7180
					memcpy(RawRove->bde, RawBase->bde, MAXBDSPRN * sizeof(BDSEPHEM));
					memcpy(RawRove->gpe, RawBase->gpe, MAXGPSPRN * sizeof(GPSEPHEM));
					memcpy(&RawRove->RefPos, &RawBase->RefPos, sizeof(REFPOS));

					ValSppB = SPP(RawBase);
					ValSppR = SPP(RawRove);

					if (ValSppB == 0 && ValSppR == 0)
					{
						/*********************��Զ�λ����********************/
						int rtkstate = SingleEpochRTK(RawBase, RawRove);
						PrintResulttoFile(filename, &RawRove->rtk);
					}
				}
			}
		}
	}
	system("pause");
	return 0;
	
}