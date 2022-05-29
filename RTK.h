/*************************************************************************
名称：Matrix定义及计算模块头文件
作者：王思翰
学号：2019302141082
修改时间：2021年11月4日
**************************************************************************/
#pragma once
#ifndef RTK_H_
#define RTK_H_

#include<iostream>
#include<fstream>
#include<string>
#include<iomanip>
#include"ConstNums.h";
#include"Coordinate.h";
#include"Decoder.h";
#include"SatPos.h";
#include<cmath>
#include"Matrix.h"
#include"Time.h"
#include"SPP_SPV.h"
#include"Lambda.h"

using namespace std;

// 时间同步
int TimeMatching(RAWDATA* RawBase, RAWDATA* RawRove, FILE* fpBase, FILE* fpRove);

// 原始数据数组的时间选取
int RawGroupDataSelect(RAWDATA* RawBaseGroup, RAWDATA* RawRoveGroup, RAWDATA* RawBase, RAWDATA* RawRove, GPSTIME* Time, int ValBase, int ValRove);

// 共视卫星选取
void SelectComSats(RAWDATA* rawB, RAWDATA* rawR);

// 参考卫星选取
int SelectRefSats(RAWDATA* rawR, int sys);

// 计算出双差观测方程的L矩阵和B矩阵
void fill_A_L_DOUBLE_DIFF(int prnBG, int prnB, int numGPS, int numBDS, RAWDATA* rawB, RAWDATA* rawR, double* B, double* L, int state);

// 计算出双差观测方程的P矩阵
void fill_COV_DOUBLE_DIFF(int prnBG, int prnB, int numGPS, int numBDS, RAWDATA* rawB, RAWDATA* rawR, double* COV, int state);

// 计算卫星的非差方差分量
double calculatesigma2(double eleangle);

//单历元RTK解算
int SingleEpochRTK(RAWDATA* rawB, RAWDATA* rawR);

// 将结果输出到文件中去
int PrintResulttoFile(string filename, RTK* Rtk);

void PrintHeadRTK(string filename);
#endif
