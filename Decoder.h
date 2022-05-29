/*************************************************************************
���ƣ�����ģ��ͷ�ļ�
���� ��˼��
ѧ�� 2019302141082
�޸�ʱ�� 2021��11��24��
**************************************************************************/
#pragma once

#include <iostream>
#include<fstream>
#include<string.h>
#include<stdio.h>
#include <cmath>
#include"ConstNums.h"
#include"Coordinate.h"
#include"Time.h"

#ifndef DECODER_H
#define DECODER_H

/*************************************************************************
��Ҫ�Ľṹ��Ķ���
*************************************************************************/

//ö���Ͷ������Ƕ�λϵͳ
enum NAVSYS
{
	GPS, BDS, GLONASS, GALILEO, QZSS
};

// ö���Ͷ���RTK����״̬
enum RTKSTATE
{
	FAILEDSOLUTION, FLOATSOLUTION, FIXEDSOLUTION
};

//Range���ݵĽṹ��
struct RANGEDATA
{
	bool Valid = false;                       //ͨ��״̬
	bool flag = false;                        //�ù۲�ֵ�Ƿ񱻶�ȡ��
	bool ValidCom = true;                     //��range���ݵ���Ϲ۲�ֵ�Ƿ��Ǵֲfalse��Ϊ�Ǵֲ
	NAVSYS Sys;
	short Prn;                                //���Ǻ�
	double P1, P2;                            //˫Ƶα�࣬GPSΪl1,l2������Ϊb1,b3
	double L1, L2;                            //�ز���λ
	double D1, D2;                            //������Ƶ��
	float Snr1, Snr2;                         //�����
	float P1Noise, P2Noise;                   //α�ྫ�� standard deviation (m)
	float L1Noise, L2Noise;                   //�ز����� standard deviation (m)
	GPSTIME gpst;
};

//�۲�ֵ�ĸ������
struct COMBINE
{
	NAVSYS Sys;
	short Prn;//���Ǻ�
	double MW;
	double GF;
	double IF;
	int n;
	double tropdelay;  //��������Ķ�������ӳ�
};

//ÿ����Ԫ�ľ���۲�ֵ
struct EPOCHOBSDATA
{
	GPSTIME gpst;
	unsigned int SATNUMS;                     // �۲⵽��������             
	RANGEDATA range[MAXCHANNUM];              // ���ǹ۲����ݾ��嵽��Ӧ��ĳ����Ԫ������������	
	COMBINE com[MAXCHANNUM];                  // ������Ϲ۲����ݾ��嵽��Ӧ��ĳ����Ԫ������������
};

//GPS����
struct GPSEPHEM
{
	int Prn;
	NAVSYS Sys;
	unsigned long week, zweek;
	double tow;
	int health;
	int IODC, IODE[2];
	double toe;                               // ������������
	double toc;                               // ���������Ӳ��
	double A;
	double DeltaN;
	double M0;
	double ecc;
	double omega;
	double cuc;
	double cus;
	double crc;
	double crs;
	double cic;
	double cis;
	double I0;
	double I0Dot;
	double Omega0;
	double omegaDot;
	double tgd, tgd1, tgd2;
	double af0;
	double af1;
	double af2;
	double N;
	GPSTIME GPS_G;
	bool Valid = false;                 //�����ж���û�ж�����������Ĭ��ΪFalse
};

//BDS����
struct BDSEPHEM
{
	int Prn;
	unsigned long week;
	NAVSYS  Sys;
	int health;
	double tgd1, tgd2;
	int AODC;
	int toc;
	double a0, a1, a2;
	int  AODE;
	int toe;
	double A;
	double ecc;
	double omega;
	double DeltaN;
	double M0;
	double Omega0;
	double omegadot;
	double i0;
	double IDOT;
	double cuc, cus, crc, crs, cic, cis;
	GPSTIME GPSG;
	bool Valid = false;                 //�����ж���û�ж�����������Ĭ��ΪFalse

};

//�������ݽṹ��
struct satPos
{
	bool RefSat = false;               // �������Ƿ�Ϊ�ο�����
	bool comobs = false;               // �������Ƿ��ǹ�������
	bool Valid = false;                // ���Ǽ��������λ���Ƿ����
	GPSTIME gpst;
	unsigned short PRN;
	XYZ satXYZ;                        // ����λ��
	double satVelocity[3];             // �����ٶ�
	double satClk;                     // �Ӳ�
	double satClkDot;                  // ����
	double eleAngle;                   // �߶Ƚ�
	double azimuth;                    // ��λ��
	double ionDelay;                   // ������ӳ�
	double tropDelay;                  // �������ӳ�
	double GF;                         // GF��Ϲ۲�ֵ
};

//���ջ����ݽṹ��
struct STATION
{
	BLH blh;
	XYZ xyz;
	XYZ xyz_save;
	NEU neu;
	double vel[3];
	double vel_sigma0;
	double dt_GPS;
	double dt_BDS;
	double PDOP;
	double sigma0;
	int numGPS, numBDS;

	// ���沿�ֵ����ݶ�����Rover�������ʹ�õ�
	//  GPS ��ϵ�в���
	// �����ǵľ�������������Ի�����
	double Gax[MAXGPSPRN] = { 0 };
	double Gay[MAXGPSPRN] = { 0 };
	double Gaz[MAXGPSPRN] = { 0 };
	//  BDS ��ϵ�в���
    // �����ǵľ�������������Ի�����
	double Bax[MAXBDSPRN] = { 0 };
	double Bay[MAXBDSPRN] = { 0 };
	double Baz[MAXBDSPRN] = { 0 };

	// Base �� Rover�ĵ�����λ�۲�ֵ
	double Gfai_d1_f1[MAXGPSPRN] = { 0 };                                // GPS��L1���ε���λ����۲�ֵ
	double Gfai_d1_f2[MAXGPSPRN] = { 0 };                                // GPS��L2���ε���λ����۲�ֵ
	double Gp_d1_f1[MAXGPSPRN] = { 0 };                                  // GPS��L1���ε�α�൥��۲�ֵ
	double Gp_d1_f2[MAXGPSPRN] = { 0 };                                  // GPS��L2���ε�α�൥��۲�ֵ
	double Gfai_d1_f1_sigma2[MAXGPSPRN] = { 0 };                         // GPS��L1���ε���λ���������
	double Gfai_d1_f2_sigma2[MAXGPSPRN] = { 0 };                         // GPS��L2���ε���λ���������
	double Gp_d1_f1_sigma2[MAXGPSPRN] = { 0 };                           // GPS��L1���ε�α�൥�������
	double Gp_d1_f2_sigma2[MAXGPSPRN] = { 0 };                           // GPS��L1���ε�α�൥�������

	// ����������PRN��˫����λ�۲�ֵ
	double Gfai_d2_f1[MAXGPSPRN] = { 0 };                                // GPS��L1���ε���λ˫��۲�ֵ
	double Gfai_d2_f2[MAXGPSPRN] = { 0 };                                // GPS��L2���ε���λ˫��۲�ֵ
	double Gp_d2_f1[MAXGPSPRN] = { 0 };                                  // GPS��L1���ε�α��˫��۲�ֵ
	double Gp_d2_f2[MAXGPSPRN] = { 0 };                                  // GPS��L2���ε�α��˫��۲�ֵ

	// Base �� Rover�ĵ�����λ�۲�ֵ
	double Bfai_d1_f1[MAXBDSPRN] = { 0 };                                // BDS��B1I���ε���λ����۲�ֵ
	double Bfai_d1_f2[MAXBDSPRN] = { 0 };                                // BDS��B3I���ε���λ����۲�ֵ
	double Bp_d1_f1[MAXBDSPRN] = { 0 };                                  // BDS��B1I���ε�α�൥��۲�ֵ
	double Bp_d1_f2[MAXBDSPRN] = { 0 };                                  // BDS��B3I���ε�α�൥��۲�ֵ
	double Bfai_d1_f1_sigma2[MAXBDSPRN] = { 0 };                         // BDS��B1I���ε���λ���������
	double Bfai_d1_f2_sigma2[MAXBDSPRN] = { 0 };                         // BDS��B3I���ε���λ���������
	double Bp_d1_f1_sigma2[MAXBDSPRN] = { 0 };                           // BDS��B1I���ε�α�൥�������
	double Bp_d1_f2_sigma2[MAXBDSPRN] = { 0 };                           // BDS��B3I���ε�α�൥�������

	// ����������PRN��˫����λ�۲�ֵ
	double Bfai_d2_f1[MAXBDSPRN] = { 0 };                                // BDS��B1I���ε���λ˫��۲�ֵ
	double Bfai_d2_f2[MAXBDSPRN] = { 0 };                                // BDS��B3I���ε���λ˫��۲�ֵ
	double Bp_d2_f1[MAXBDSPRN] = { 0 };                                  // BDS��B1I���ε�α��˫��۲�ֵ
	double Bp_d2_f2[MAXBDSPRN] = { 0 };                                  // BDS��B3I���ε�α��˫��۲�ֵ
};

// renix�ļ��еĽ��ջ���������
struct REFPOS
{
	GPSTIME Time;
	BLH Blh;
	double DevB;
	double DevL;
	double DevH;
	double Accu;
};

// RTK�Ľṹ��
struct RTK
{
	// ģ������ز���
	double N_FLOAT[4 * MAXCHANNUM] = { 0 };                       // ģ���ȸ��������
	double N_FIXED[4 * MAXCHANNUM] = { 0 };                       // ģ���ȵĹ̶�������
	int N_m = 2;                                                  // ģ���Ⱥ�ѡ��ĸ���	
	int N_n;                                                      // ģ���ȵ�ά��
	double ratio = 0;
	int allepochs = 0;
	int fixedepochs = 0;
	double fixrate = 0;


	// �����ʱ�����
	BLH blh;
	XYZ xyz;
	NEU neu;
	BLH blhfloat;
	XYZ xyzfloat;
	NEU neufloat;
	BLH blhfixed;
	XYZ xyzfixed;
	NEU neufixed;
	GPSTIME gpst;

	// ����ʱÿ�ε�����
	double residual[3] = { 0 };

	// ��������
	double Vector[3] = { 0 };

	// RTK�Ľ���״̬
	RTKSTATE rtkstate = FAILEDSOLUTION;

	// ������������
	double PDOP;
	double sigma0;
	int numGPS = 0;
	int numBDS = 0;
	int RefGPS, RefBDS;
	double sigmax, sigmay, sigmaz;
	int Satnums;
	double RMS;
};


//RAWDATA��������������ԭʼ���ݣ���������������GPS������RANGE���ݵ�
struct RAWDATA
{
	GPSTIME gpst;                 // �۲�ֵ��GPST����������
	BDSEPHEM bde[MAXBDSPRN];      // BDS������
	GPSEPHEM gpe[MAXGPSPRN];      // GPS������
	satPos sat_gps[MAXGPSPRN];    // GPS�������ݵĽṹ��
	satPos sat_bds[MAXBDSPRN];    // BDS �������ݽṹ��
	STATION receiver;             // ���ջ����ݵĽṹ��
	EPOCHOBSDATA obs;             // ÿ����Ԫ�Ĺ۲�ֵ
	REFPOS RefPos;                // ���ջ���������
	RTK rtk;                      // RTK��������ĸ�������
	bool SPPstate = false;        // �ж�SPP�����Ƿ�ɹ�
};
////��������ݽṹ��
//struct IONUTC
//{
//	double a0;
//	double a1;
//	double a2;
//	double a3;
//	double b0;
//	double b1;
//	double b2;
//	double b3;
//	unsigned long UtcWn;		//UTC reference week number
//	unsigned long Tot;			//Reference time of UTC parameters
//	double A0;					//UTC constant term of polynomial
//	double A1;					//UTC 1st order term of polynomial
//	unsigned long WnLsf;		//Future week number
//	unsigned long Dn;			//Day number
//	long deltatLs;				//Delta time due to leap seconds
//	long deltatLsf;				//Future delta time due to leap seconds
//	unsigned long deltatUtc;	//Time difference
//};

/*************************************************************************
����Ľ��뺯���Ķ���
*************************************************************************/

//CRC��У�麯��
unsigned int crc32(const unsigned char* buff, int len);

//������λ�ַ�����һЩ����
double R8(unsigned char buf[]);
float F4(unsigned char buf[]);
unsigned short U2(unsigned char buf[]);
unsigned int U4(unsigned char buf[]);

//��ȡ�ļ�ͷ�ĺ���
int DecodeHOEM7(FILE* file, RAWDATA* raw);
int DecodeOEM7MessageSock(unsigned char buff[], int lenr, int& lenrem, RAWDATA* Raw);

//��ȡmessage_id=43��range�ṹ������
int DecodeObs(unsigned char buf[], EPOCHOBSDATA* obs);

//��ȡmessage_id=7��GPS��������
int DecodeGPSEph(unsigned char buf[], GPSEPHEM* gpe);

//��ȡmessage_id=1696�ı�����������
int DecodeBDSEph(unsigned char buf[], BDSEPHEM* bde);

//��ȡmessage_id=47�Ľ��ջ�������������
int DecodeREFPOS(unsigned char buff[], REFPOS* Psr);

//FindSatObsIndex Ѱ�ҵ������ǹ۲�ֵ�������ĺ���
int FindSatObsIndex(const int Prn, const NAVSYS Sys, EPOCHOBSDATA* data);

//�ж�������Range�ļ��Ƿ��ȡ�����˵ĺ���
bool NUMofSAT(RAWDATA* raw);
#endif



