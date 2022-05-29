/*************************************************************************
名称：Matrix定义及计算模块头文件
作者：王思翰
学号：2019302141082
修改时间：2021年11月4日
**************************************************************************/
#pragma once
#ifndef MATRIX_H
#define MATRIX_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <Windows.h>
#include <conio.h>
#include <malloc.h>
#include <fstream>

//结构体，用来存放矩阵的行，列以及矩阵内数据
typedef struct matrix
{
	int row;
	int column;
	double* data;
};

//矩阵初始化
void MatrixInit(matrix* newMat, int m, int n, double a[]);

//矩阵复制
void MatrixCopy(matrix* newMat, matrix* oldMat, double b[]);

//矩阵输出
void MatrixPrint(matrix* Mat);
void MatrixPrinttoFile(matrix* Mat, FILE* file);

//求矩阵的行列式
double MatrixDet(matrix* mat);

//矩阵加法
int MatrixAdd(matrix* newMat, matrix* mat1, matrix* mat2, double data[]);

//矩阵减法
int MatrixMinus(matrix* newMat, matrix* mat1, matrix* mat2, double data[]);

//矩阵转置
void MatrixTrans(matrix* newMat, matrix* oldMat, double data[]);

//矩阵数乘
void MatrixTimes(matrix* newMat, matrix* oldMat, double n, double data[]);

//矩阵乘法
int MatrixMulti(matrix* mat1, matrix* mat2, matrix* nerMat, double data[]);

//矩阵求逆
int MatrixInv(matrix* newMat, matrix* oldMat, double data[]);

int MatInv(int n, double a[], double b[]);
void MatrixMultiply(int m1, int n1, int m2, int n2,
	const double M1[], const double M2[], double M3[]);

#endif


