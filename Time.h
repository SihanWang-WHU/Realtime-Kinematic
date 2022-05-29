/*************************************************************************
名称：时间转换模块头文件
作者：王思翰
学号：2019302141082
修改时间：2021年11月15日
**************************************************************************/
#pragma once

#include<cmath>
#include<iostream>

#ifndef TIME_H
#define TIME_H
// 通用时结构体
struct COMMONTIME
{
	short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double Second;
};

//简化儒略日结构体
struct MJDTIME
{
	int Days;              //整数部分
	double FracDay;        //小数部分
};

//GPS时结构体
struct GPSTIME
{
	unsigned short Week;   //GPS周数
	double SecofWeek;      //GPS周内秒    
};

/************************************各个时间初始化的函数*************************************/
//通用时初始化
void CMT_INIT(COMMONTIME* cmt, unsigned short year, unsigned short month, unsigned short day, unsigned short hour, unsigned short min, double sec);

//简化儒略日初始化
void MJD_INIT(MJDTIME* mjd, int days, double fracday);

//GPS时初始化
void GPST_INIT(GPSTIME* gpst, unsigned short week, double sec);

/************************************打印各个时间的函数*************************************/
//打印儒略日
void MJD_PRINT(MJDTIME* mjd);

//打印GPS时
void GPS_PRINT(GPSTIME* gpst);

//打印通用时
void CMT_PRINT(COMMONTIME* cmt);

/************************************各时间之间的转换函数*************************************/
//MJD转化为通用时
void MJD2CMT(COMMONTIME* cmt, MJDTIME* mjd);

//GPS转化位通用时
void GPS2CMT(COMMONTIME* cmt, GPSTIME* gpst);

//通用时转MJD
void CMT2MJD(MJDTIME* mjd, COMMONTIME* cmt);

//GPS时转MJD
void GPS2MJD(MJDTIME* mjd, GPSTIME* gpst);

//MJD转GPS时
void MJD2GPS(GPSTIME* gpst, MJDTIME* mjd);

//通用时转GPS时
void CMT2GPS(GPSTIME* gpst, COMMONTIME* cmt);

#endif

