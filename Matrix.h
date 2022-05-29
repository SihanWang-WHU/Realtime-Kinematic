/*************************************************************************
���ƣ�Matrix���弰����ģ��ͷ�ļ�
���ߣ���˼��
ѧ�ţ�2019302141082
�޸�ʱ�䣺2021��11��4��
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

//�ṹ�壬������ž�����У����Լ�����������
typedef struct matrix
{
	int row;
	int column;
	double* data;
};

//�����ʼ��
void MatrixInit(matrix* newMat, int m, int n, double a[]);

//������
void MatrixCopy(matrix* newMat, matrix* oldMat, double b[]);

//�������
void MatrixPrint(matrix* Mat);
void MatrixPrinttoFile(matrix* Mat, FILE* file);

//����������ʽ
double MatrixDet(matrix* mat);

//����ӷ�
int MatrixAdd(matrix* newMat, matrix* mat1, matrix* mat2, double data[]);

//�������
int MatrixMinus(matrix* newMat, matrix* mat1, matrix* mat2, double data[]);

//����ת��
void MatrixTrans(matrix* newMat, matrix* oldMat, double data[]);

//��������
void MatrixTimes(matrix* newMat, matrix* oldMat, double n, double data[]);

//����˷�
int MatrixMulti(matrix* mat1, matrix* mat2, matrix* nerMat, double data[]);

//��������
int MatrixInv(matrix* newMat, matrix* oldMat, double data[]);

int MatInv(int n, double a[], double b[]);
void MatrixMultiply(int m1, int n1, int m2, int n2,
	const double M1[], const double M2[], double M3[]);

#endif


