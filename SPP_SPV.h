/*************************************************************************
名称：SPP和SPV计算模块头文件
作者 王思翰
学号 2019302141082
修改时间 2021年12月22日
**************************************************************************/
#pragma once
#ifndef SPP_SPV_H_
#define SPP_SPV_H_
#include<iostream>
#include<fstream>
#include<cmath>
#include<iomanip>
#include"ConstNums.h";
#include"Coordinate.h";
#include"Decoder.h";
#include"SatPos.h";
#include"Matrix.h";
#include"ErrorCorrect.h";

//枚举型定义SPP解算的状态
//分别为双系统，单GPS和单北斗
enum SPPSTATE
{
	DOUBLESYS, SINGLEGPS, SINGLEBDS
};

//用于计算卫星和测站的之间的距离的函数
double distance_calculate(XYZ xyz1, XYZ xyz2);

//用于SPP解算中B，l矩阵的计算
double fillBl_SPP(RAWDATA* raw, SPPSTATE state, double* B, double* l);

//用于SPP解算中权阵P的计算
double fillP_spp_plain(int size, double* P);
double fillP_spp(int size, double* P, RAWDATA* raw);

//SPP解算
int SPP(RAWDATA* raw);

//用于SPV解算中B，l矩阵的计算
double fillBl_SPV(RAWDATA* raw, double* B, double* l);

//用于SPV解算中权阵P的计算
double fillP_spv_plain(int size, double* P);
double fillP_spv(int size, double* P, RAWDATA* raw);

//SPV解算
int SPV(RAWDATA* raw);
#endif

