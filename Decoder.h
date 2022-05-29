/*************************************************************************
名称：解码模块头文件
作者 王思翰
学号 2019302141082
修改时间 2021年11月24日
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
需要的结构体的定义
*************************************************************************/

//枚举型定义卫星定位系统
enum NAVSYS
{
	GPS, BDS, GLONASS, GALILEO, QZSS
};

// 枚举型定义RTK解算状态
enum RTKSTATE
{
	FAILEDSOLUTION, FLOATSOLUTION, FIXEDSOLUTION
};

//Range数据的结构体
struct RANGEDATA
{
	bool Valid = false;                       //通道状态
	bool flag = false;                        //该观测值是否被读取到
	bool ValidCom = true;                     //该range数据的组合观测值是否不是粗差（false即为是粗差）
	NAVSYS Sys;
	short Prn;                                //卫星号
	double P1, P2;                            //双频伪距，GPS为l1,l2，北斗为b1,b3
	double L1, L2;                            //载波相位
	double D1, D2;                            //多普勒频移
	float Snr1, Snr2;                         //载噪比
	float P1Noise, P2Noise;                   //伪距精度 standard deviation (m)
	float L1Noise, L2Noise;                   //载波精度 standard deviation (m)
	GPSTIME gpst;
};

//观测值的各种组合
struct COMBINE
{
	NAVSYS Sys;
	short Prn;//卫星号
	double MW;
	double GF;
	double IF;
	int n;
	double tropdelay;  //计算出来的对流层的延迟
};

//每个历元的距离观测值
struct EPOCHOBSDATA
{
	GPSTIME gpst;
	unsigned int SATNUMS;                     // 观测到的卫星数             
	RANGEDATA range[MAXCHANNUM];              // 卫星观测数据具体到对应的某个历元的所有卫星内	
	COMBINE com[MAXCHANNUM];                  // 卫星组合观测数据具体到对应的某个历元的所有卫星内
};

//GPS星历
struct GPSEPHEM
{
	int Prn;
	NAVSYS Sys;
	unsigned long week, zweek;
	double tow;
	int health;
	int IODC, IODE[2];
	double toe;                               // 用来计算轨道的
	double toc;                               // 用来计算钟差的
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
	bool Valid = false;                 //用来判断有没有读到该星历，默认为False
};

//BDS星历
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
	bool Valid = false;                 //用来判断有没有读到该星历，默认为False

};

//卫星数据结构体
struct satPos
{
	bool RefSat = false;               // 该卫星是否为参考卫星
	bool comobs = false;               // 该卫星是否是共视卫星
	bool Valid = false;                // 卫星计算出来的位置是否可用
	GPSTIME gpst;
	unsigned short PRN;
	XYZ satXYZ;                        // 卫星位置
	double satVelocity[3];             // 卫星速度
	double satClk;                     // 钟差
	double satClkDot;                  // 钟速
	double eleAngle;                   // 高度角
	double azimuth;                    // 方位角
	double ionDelay;                   // 电离层延迟
	double tropDelay;                  // 对流层延迟
	double GF;                         // GF组合观测值
};

//接收机数据结构体
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

	// 下面部分的内容都是再Rover里面才能使用的
	//  GPS 的系列参数
	// 与卫星的距离求出来的线性化参数
	double Gax[MAXGPSPRN] = { 0 };
	double Gay[MAXGPSPRN] = { 0 };
	double Gaz[MAXGPSPRN] = { 0 };
	//  BDS 的系列参数
    // 与卫星的距离求出来的线性化参数
	double Bax[MAXBDSPRN] = { 0 };
	double Bay[MAXBDSPRN] = { 0 };
	double Baz[MAXBDSPRN] = { 0 };

	// Base 与 Rover的单差相位观测值
	double Gfai_d1_f1[MAXGPSPRN] = { 0 };                                // GPS的L1波段的相位单差观测值
	double Gfai_d1_f2[MAXGPSPRN] = { 0 };                                // GPS的L2波段的相位单差观测值
	double Gp_d1_f1[MAXGPSPRN] = { 0 };                                  // GPS的L1波段的伪距单差观测值
	double Gp_d1_f2[MAXGPSPRN] = { 0 };                                  // GPS的L2波段的伪距单差观测值
	double Gfai_d1_f1_sigma2[MAXGPSPRN] = { 0 };                         // GPS的L1波段的相位单差中误差
	double Gfai_d1_f2_sigma2[MAXGPSPRN] = { 0 };                         // GPS的L2波段的相位单差中误差
	double Gp_d1_f1_sigma2[MAXGPSPRN] = { 0 };                           // GPS的L1波段的伪距单差中误差
	double Gp_d1_f2_sigma2[MAXGPSPRN] = { 0 };                           // GPS的L1波段的伪距单差中误差

	// 依附于卫星PRN的双差相位观测值
	double Gfai_d2_f1[MAXGPSPRN] = { 0 };                                // GPS的L1波段的相位双差观测值
	double Gfai_d2_f2[MAXGPSPRN] = { 0 };                                // GPS的L2波段的相位双差观测值
	double Gp_d2_f1[MAXGPSPRN] = { 0 };                                  // GPS的L1波段的伪距双差观测值
	double Gp_d2_f2[MAXGPSPRN] = { 0 };                                  // GPS的L2波段的伪距双差观测值

	// Base 与 Rover的单差相位观测值
	double Bfai_d1_f1[MAXBDSPRN] = { 0 };                                // BDS的B1I波段的相位单差观测值
	double Bfai_d1_f2[MAXBDSPRN] = { 0 };                                // BDS的B3I波段的相位单差观测值
	double Bp_d1_f1[MAXBDSPRN] = { 0 };                                  // BDS的B1I波段的伪距单差观测值
	double Bp_d1_f2[MAXBDSPRN] = { 0 };                                  // BDS的B3I波段的伪距单差观测值
	double Bfai_d1_f1_sigma2[MAXBDSPRN] = { 0 };                         // BDS的B1I波段的相位单差中误差
	double Bfai_d1_f2_sigma2[MAXBDSPRN] = { 0 };                         // BDS的B3I波段的相位单差中误差
	double Bp_d1_f1_sigma2[MAXBDSPRN] = { 0 };                           // BDS的B1I波段的伪距单差中误差
	double Bp_d1_f2_sigma2[MAXBDSPRN] = { 0 };                           // BDS的B3I波段的伪距单差中误差

	// 依附于卫星PRN的双差相位观测值
	double Bfai_d2_f1[MAXBDSPRN] = { 0 };                                // BDS的B1I波段的相位双差观测值
	double Bfai_d2_f2[MAXBDSPRN] = { 0 };                                // BDS的B3I波段的相位双差观测值
	double Bp_d2_f1[MAXBDSPRN] = { 0 };                                  // BDS的B1I波段的伪距双差观测值
	double Bp_d2_f2[MAXBDSPRN] = { 0 };                                  // BDS的B3I波段的伪距双差观测值
};

// renix文件中的接收机概略坐标
struct REFPOS
{
	GPSTIME Time;
	BLH Blh;
	double DevB;
	double DevL;
	double DevH;
	double Accu;
};

// RTK的结构体
struct RTK
{
	// 模糊度相关参数
	double N_FLOAT[4 * MAXCHANNUM] = { 0 };                       // 模糊度浮点解向量
	double N_FIXED[4 * MAXCHANNUM] = { 0 };                       // 模糊度的固定解向量
	int N_m = 2;                                                  // 模糊度候选解的个数	
	int N_n;                                                      // 模糊度的维度
	double ratio = 0;
	int allepochs = 0;
	int fixedepochs = 0;
	double fixrate = 0;


	// 坐标和时间参数
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

	// 计算时每次的增益
	double residual[3] = { 0 };

	// 基线向量
	double Vector[3] = { 0 };

	// RTK的解算状态
	RTKSTATE rtkstate = FAILEDSOLUTION;

	// 精度评定参数
	double PDOP;
	double sigma0;
	int numGPS = 0;
	int numBDS = 0;
	int RefGPS, RefBDS;
	double sigmax, sigmay, sigmaz;
	int Satnums;
	double RMS;
};


//RAWDATA即读进来的所有原始数据，包含北斗星历，GPS星历，RANGE数据等
struct RAWDATA
{
	GPSTIME gpst;                 // 观测值的GPST，放在这里
	BDSEPHEM bde[MAXBDSPRN];      // BDS的星历
	GPSEPHEM gpe[MAXGPSPRN];      // GPS的星历
	satPos sat_gps[MAXGPSPRN];    // GPS卫星数据的结构体
	satPos sat_bds[MAXBDSPRN];    // BDS 卫星数据结构体
	STATION receiver;             // 接收机数据的结构体
	EPOCHOBSDATA obs;             // 每个历元的观测值
	REFPOS RefPos;                // 接收机概略坐标
	RTK rtk;                      // RTK计算出来的各个参数
	bool SPPstate = false;        // 判断SPP解算是否成功
};
////电离层数据结构体
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
具体的解码函数的定义
*************************************************************************/

//CRC码校验函数
unsigned int crc32(const unsigned char* buff, int len);

//用来移位字符串的一些函数
double R8(unsigned char buf[]);
float F4(unsigned char buf[]);
unsigned short U2(unsigned char buf[]);
unsigned int U4(unsigned char buf[]);

//读取文件头的函数
int DecodeHOEM7(FILE* file, RAWDATA* raw);
int DecodeOEM7MessageSock(unsigned char buff[], int lenr, int& lenrem, RAWDATA* Raw);

//读取message_id=43的range结构体数据
int DecodeObs(unsigned char buf[], EPOCHOBSDATA* obs);

//读取message_id=7的GPS星历数据
int DecodeGPSEph(unsigned char buf[], GPSEPHEM* gpe);

//读取message_id=1696的北斗星历数据
int DecodeBDSEph(unsigned char buf[], BDSEPHEM* bde);

//读取message_id=47的接收机概略坐标数据
int DecodeREFPOS(unsigned char buff[], REFPOS* Psr);

//FindSatObsIndex 寻找单颗卫星观测值的数量的函数
int FindSatObsIndex(const int Prn, const NAVSYS Sys, EPOCHOBSDATA* data);

//判断星历和Range文件是否读取完整了的函数
bool NUMofSAT(RAWDATA* raw);
#endif



