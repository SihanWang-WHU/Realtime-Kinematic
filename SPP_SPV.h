/*************************************************************************
���ƣ�SPP��SPV����ģ��ͷ�ļ�
���� ��˼��
ѧ�� 2019302141082
�޸�ʱ�� 2021��12��22��
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

//ö���Ͷ���SPP�����״̬
//�ֱ�Ϊ˫ϵͳ����GPS�͵�����
enum SPPSTATE
{
	DOUBLESYS, SINGLEGPS, SINGLEBDS
};

//���ڼ������ǺͲ�վ��֮��ľ���ĺ���
double distance_calculate(XYZ xyz1, XYZ xyz2);

//����SPP������B��l����ļ���
double fillBl_SPP(RAWDATA* raw, SPPSTATE state, double* B, double* l);

//����SPP������Ȩ��P�ļ���
double fillP_spp_plain(int size, double* P);
double fillP_spp(int size, double* P, RAWDATA* raw);

//SPP����
int SPP(RAWDATA* raw);

//����SPV������B��l����ļ���
double fillBl_SPV(RAWDATA* raw, double* B, double* l);

//����SPV������Ȩ��P�ļ���
double fillP_spv_plain(int size, double* P);
double fillP_spv(int size, double* P, RAWDATA* raw);

//SPV����
int SPV(RAWDATA* raw);
#endif

