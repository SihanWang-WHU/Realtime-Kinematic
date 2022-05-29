/*************************************************************************
���ƣ�ʱ��ת��ģ��ͷ�ļ�
���ߣ���˼��
ѧ�ţ�2019302141082
�޸�ʱ�䣺2021��11��15��
**************************************************************************/
#pragma once

#include<cmath>
#include<iostream>

#ifndef TIME_H
#define TIME_H
// ͨ��ʱ�ṹ��
struct COMMONTIME
{
	short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double Second;
};

//�������սṹ��
struct MJDTIME
{
	int Days;              //��������
	double FracDay;        //С������
};

//GPSʱ�ṹ��
struct GPSTIME
{
	unsigned short Week;   //GPS����
	double SecofWeek;      //GPS������    
};

/************************************����ʱ���ʼ���ĺ���*************************************/
//ͨ��ʱ��ʼ��
void CMT_INIT(COMMONTIME* cmt, unsigned short year, unsigned short month, unsigned short day, unsigned short hour, unsigned short min, double sec);

//�������ճ�ʼ��
void MJD_INIT(MJDTIME* mjd, int days, double fracday);

//GPSʱ��ʼ��
void GPST_INIT(GPSTIME* gpst, unsigned short week, double sec);

/************************************��ӡ����ʱ��ĺ���*************************************/
//��ӡ������
void MJD_PRINT(MJDTIME* mjd);

//��ӡGPSʱ
void GPS_PRINT(GPSTIME* gpst);

//��ӡͨ��ʱ
void CMT_PRINT(COMMONTIME* cmt);

/************************************��ʱ��֮���ת������*************************************/
//MJDת��Ϊͨ��ʱ
void MJD2CMT(COMMONTIME* cmt, MJDTIME* mjd);

//GPSת��λͨ��ʱ
void GPS2CMT(COMMONTIME* cmt, GPSTIME* gpst);

//ͨ��ʱתMJD
void CMT2MJD(MJDTIME* mjd, COMMONTIME* cmt);

//GPSʱתMJD
void GPS2MJD(MJDTIME* mjd, GPSTIME* gpst);

//MJDתGPSʱ
void MJD2GPS(GPSTIME* gpst, MJDTIME* mjd);

//ͨ��ʱתGPSʱ
void CMT2GPS(GPSTIME* gpst, COMMONTIME* cmt);

#endif

