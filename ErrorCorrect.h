/*************************************************************************
名称：误差改正模块头文件
作者：王思翰
学号：2019302141082
修改时间：2021年12月20日
**************************************************************************/
#pragma once
#ifndef _ERRORCORRECT_H_
#define _ERRORCORRECT_H_

#include <math.h>
#include "Coordinate.h"
#include "SatPos.h"
#include "Decoder.h"
#include "ConstNums.h"


//Hopefield模型改正
double Hopfield(RAWDATA* raw, NAVSYS sys, int prn);

//GPS的GF组合观测值
double IF_GPS(double P1, double P2);

//BDS的GF组合观测值
double IF_BDS(double P1, double P2);

//粗差探测
void DetectOutlier(RAWDATA* raw);
#endif



