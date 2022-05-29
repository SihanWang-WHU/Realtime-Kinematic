/*************************************************************************
作者 王思翰
学号 2019302141082
修改时间 2021年11月15日
**************************************************************************/

#include"Time.h"

/*************************************************************************
CMT_INIT 通用时初始化
输入：cmt通用时结构体指针，year month day hour min sec年月日时分秒
输出：无
已测试
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
MJD_INIT 简化儒略日初始化
输入：mjd简化儒略日结构体指针，整数部分days，小数部分fracday
输出：无
已测试
**************************************************************************/
void MJD_INIT(MJDTIME* mjd, int days, double fracday)
{
	mjd->Days = days;
	mjd->FracDay = fracday;
}

/*************************************************************************
GPST_INIT GPS时初始化
输入：GPS时结构体指针，GPS周week, GPS周内秒sec
输出：无
已测试
**************************************************************************/
void GPST_INIT(GPSTIME* gpst, unsigned short week, double sec)
{
	gpst->Week = week;
	gpst->SecofWeek = sec;
}

/*************************************************************************
MJD_PRINT 输出儒略日时
输入：儒略日时结构体指针mjd
输出：无
已测试
**************************************************************************/
void MJD_PRINT(MJDTIME* mjd)
{
	printf("day:\t%d\n", (*mjd).Days);
	printf("fracday:\t%16.14f\n", (*mjd).FracDay);
}

/*************************************************************************
CMT_PRINT 输出通用时
输入：通用时结构体指针cmt
输出：无
已测试
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
GPS_PRINT 输出GPS时
输入：GPS时结构体指针gpst
输出：无
已测试
**************************************************************************/
void GPS_PRINT(GPSTIME* gpst)
{
	printf("week:\t%d\n", (*gpst).Week);
	printf("sow:\t%16.9f\n", (*gpst).SecofWeek);
}


/*************************************************************************
MJD2CMT 简化儒略日转化为通用时
输入：通用时结构体指针cmt，简化儒略日结构体指针mjd
输出：无
已测试
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
MJD2CMT 简化儒略日转化为通用时
输入：通用时结构体指针cmt，简化儒略日结构体指针mjd
输出：无
已测试
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
GPS2MJD GPS时转化为简化儒略日
输入：GPS时结构体指针gpst，简化儒略日结构体指针mjd
输出：无
已测试
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
MJD2GPS 简化儒略日转化为GPS时
输入：GPS时结构体指针gpst，简化儒略日结构体指针mjd
输出：无
已测试
**************************************************************************/
void MJD2GPS(GPSTIME* gpst, MJDTIME* mjd)
{
	unsigned short week = int((mjd->Days + mjd->FracDay - 44244) / 7);
	double secofweek = (mjd->Days + mjd->FracDay - 44244 + (week * 7)) * 86400;
	GPST_INIT(gpst, week, secofweek);
}

/*************************************************************************
GPS2CMT GPS时转化为通用时
输入：GPS时结构体指针gpst，通用时结构体指针cmt
输出：无
已测试
**************************************************************************/
void GPS2CMT(COMMONTIME* cmt, GPSTIME* gpst)
{
	MJDTIME* m = new MJDTIME;
	GPS2MJD(m, gpst);
	MJD2CMT(cmt, m);
}


/*************************************************************************
CMT2GPS 通用时转化为GPS时
输入：GPS时结构体指针gpst，通用时结构体指针cmt
输出：无
已测试
**************************************************************************/
void CMT2GPS(GPSTIME* gpst, COMMONTIME* cmt)
{
	MJDTIME* m = new MJDTIME;
	CMT2MJD(m, cmt);
	MJD2GPS(gpst, m);
}
