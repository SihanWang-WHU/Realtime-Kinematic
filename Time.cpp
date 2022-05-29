/*************************************************************************
���� ��˼��
ѧ�� 2019302141082
�޸�ʱ�� 2021��11��15��
**************************************************************************/

#include"Time.h"

/*************************************************************************
CMT_INIT ͨ��ʱ��ʼ��
���룺cmtͨ��ʱ�ṹ��ָ�룬year month day hour min sec������ʱ����
�������
�Ѳ���
**************************************************************************/
void CMT_INIT(COMMONTIME* cmt, unsigned short year, unsigned short month, unsigned short day, unsigned short hour, unsigned short min, double sec)
{
	cmt->Year = year;
	cmt->Month = month;
	cmt->Day = day;
	cmt->Hour = hour;
	cmt->Minute = min;
	cmt->Second = sec;
}

/*************************************************************************
MJD_INIT �������ճ�ʼ��
���룺mjd�������սṹ��ָ�룬��������days��С������fracday
�������
�Ѳ���
**************************************************************************/
void MJD_INIT(MJDTIME* mjd, int days, double fracday)
{
	mjd->Days = days;
	mjd->FracDay = fracday;
}

/*************************************************************************
GPST_INIT GPSʱ��ʼ��
���룺GPSʱ�ṹ��ָ�룬GPS��week, GPS������sec
�������
�Ѳ���
**************************************************************************/
void GPST_INIT(GPSTIME* gpst, unsigned short week, double sec)
{
	gpst->Week = week;
	gpst->SecofWeek = sec;
}

/*************************************************************************
MJD_PRINT ���������ʱ
���룺������ʱ�ṹ��ָ��mjd
�������
�Ѳ���
**************************************************************************/
void MJD_PRINT(MJDTIME* mjd)
{
	printf("day:\t%d\n", (*mjd).Days);
	printf("fracday:\t%16.14f\n", (*mjd).FracDay);
}

/*************************************************************************
CMT_PRINT ���ͨ��ʱ
���룺ͨ��ʱ�ṹ��ָ��cmt
�������
�Ѳ���
**************************************************************************/
void CMT_PRINT(COMMONTIME* cmt)
{
	printf("year:\t%d\n", (*cmt).Year);
	printf("month:\t%d\n", (*cmt).Month);
	printf("day:\t%d\n", (*cmt).Day);
	printf("hour:\t%d\n", (*cmt).Hour);
	printf("min:\t%d\n", (*cmt).Minute);
	printf("sec:\t%16.13f\n", (*cmt).Second);
}

/*************************************************************************
GPS_PRINT ���GPSʱ
���룺GPSʱ�ṹ��ָ��gpst
�������
�Ѳ���
**************************************************************************/
void GPS_PRINT(GPSTIME* gpst)
{
	printf("week:\t%d\n", (*gpst).Week);
	printf("sow:\t%16.9f\n", (*gpst).SecofWeek);
}


/*************************************************************************
MJD2CMT ��������ת��Ϊͨ��ʱ
���룺ͨ��ʱ�ṹ��ָ��cmt���������սṹ��ָ��mjd
�������
�Ѳ���
**************************************************************************/
void MJD2CMT(COMMONTIME* cmt, MJDTIME* mjd)
{
	int a = (*mjd).Days + 2400001;
	int b = a + 1537;
	int c = int((b - 122.1) / 365.25);
	int d = int(365.25 * c);
	int e = int((b - d) / 30.6001);

	unsigned short hour = int(24 * (*mjd).FracDay);
	unsigned short min = int(24 * 60 * (*mjd).FracDay) - hour * 60;
	double sec = (*mjd).FracDay * 24 * 3600 - min * 60 - hour * 3600;
	unsigned short day = b - d - int(30.6001 * e);
	unsigned short month = e - 1 - 12 * int(e / 14);
	unsigned short year = c - 4715 - int((7 + month) / 10);
	CMT_INIT(cmt, year, month, day, hour, min, sec);
}

/*************************************************************************
MJD2CMT ��������ת��Ϊͨ��ʱ
���룺ͨ��ʱ�ṹ��ָ��cmt���������սṹ��ָ��mjd
�������
�Ѳ���
**************************************************************************/
void CMT2MJD(MJDTIME* mjd, COMMONTIME* cmt)
{
	int y = (*cmt).Year;
	int m = (*cmt).Month;

	if ((*cmt).Month <= 2)
	{
		y--;
		m += 12;
	}
	int day = int(365.25 * y) + int(30.6001 * (m + 1)) + (*cmt).Day + 1720981.5 - 2400000.5;
	double fracDay = double((*cmt).Hour) / 24 + double((*cmt).Minute) / 24 / 60 + (*cmt).Second / 24 / 3600;
	MJD_INIT(mjd, day, fracDay);
}

/*************************************************************************
GPS2MJD GPSʱת��Ϊ��������
���룺GPSʱ�ṹ��ָ��gpst���������սṹ��ָ��mjd
�������
�Ѳ���
**************************************************************************/
void GPS2MJD(MJDTIME* mjd, GPSTIME* gpst)
{
	int days;
	double fracday;
	double MJD;
	MJD = 44244 + gpst->Week * 7 + gpst->SecofWeek / 86400;
	days = int(MJD);
	fracday = MJD - days;
	MJD_INIT(mjd, days, fracday);
}

/*************************************************************************
MJD2GPS ��������ת��ΪGPSʱ
���룺GPSʱ�ṹ��ָ��gpst���������սṹ��ָ��mjd
�������
�Ѳ���
**************************************************************************/
void MJD2GPS(GPSTIME* gpst, MJDTIME* mjd)
{
	unsigned short week = int((mjd->Days + mjd->FracDay - 44244) / 7);
	double secofweek = (mjd->Days + mjd->FracDay - 44244 + (week * 7)) * 86400;
	GPST_INIT(gpst, week, secofweek);
}

/*************************************************************************
GPS2CMT GPSʱת��Ϊͨ��ʱ
���룺GPSʱ�ṹ��ָ��gpst��ͨ��ʱ�ṹ��ָ��cmt
�������
�Ѳ���
**************************************************************************/
void GPS2CMT(COMMONTIME* cmt, GPSTIME* gpst)
{
	MJDTIME* m = new MJDTIME;
	GPS2MJD(m, gpst);
	MJD2CMT(cmt, m);
}


/*************************************************************************
CMT2GPS ͨ��ʱת��ΪGPSʱ
���룺GPSʱ�ṹ��ָ��gpst��ͨ��ʱ�ṹ��ָ��cmt
�������
�Ѳ���
**************************************************************************/
void CMT2GPS(GPSTIME* gpst, COMMONTIME* cmt)
{
	MJDTIME* m = new MJDTIME;
	CMT2MJD(m, cmt);
	MJD2GPS(gpst, m);
}
