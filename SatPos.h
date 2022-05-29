/*************************************************************************
名称：卫星位置计算模块头文件
作者 王思翰
学号 2019302141082
修改时间 2021年12月14日
**************************************************************************/
#pragma once
#ifndef SATPOS_H_
#define SATPOS_H_

#include<cmath>
#include<iostream>
#include<fstream>
#include<string.h>
#include"ConstNums.h"
#include"Matrix.h"
#include"Time.h"
#include"Coordinate.h"
#include"Decoder.h"

//计算GPS卫星位置/速度的函数
double GPSPOSVEL(RAWDATA* raw, GPSTIME* gTime);

//计算BDS卫星位置/速度的函数
double BDSPOSVEL(RAWDATA* raw, GPSTIME* gTime);

//计算NEU坐标/高度角/方位角的函数
int calNEU_AE(RAWDATA* raw, int i, int sys);
#endif

