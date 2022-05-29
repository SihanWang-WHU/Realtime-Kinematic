/*************************************************************************
���ƣ�Matrix���弰����ģ��ͷ�ļ�
���ߣ���˼��
ѧ�ţ�2019302141082
�޸�ʱ�䣺2021��11��4��
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

// ʱ��ͬ��
int TimeMatching(RAWDATA* RawBase, RAWDATA* RawRove, FILE* fpBase, FILE* fpRove);

// ԭʼ���������ʱ��ѡȡ
int RawGroupDataSelect(RAWDATA* RawBaseGroup, RAWDATA* RawRoveGroup, RAWDATA* RawBase, RAWDATA* RawRove, GPSTIME* Time, int ValBase, int ValRove);

// ��������ѡȡ
void SelectComSats(RAWDATA* rawB, RAWDATA* rawR);

// �ο�����ѡȡ
int SelectRefSats(RAWDATA* rawR, int sys);

// �����˫��۲ⷽ�̵�L�����B����
void fill_A_L_DOUBLE_DIFF(int prnBG, int prnB, int numGPS, int numBDS, RAWDATA* rawB, RAWDATA* rawR, double* B, double* L, int state);

// �����˫��۲ⷽ�̵�P����
void fill_COV_DOUBLE_DIFF(int prnBG, int prnB, int numGPS, int numBDS, RAWDATA* rawB, RAWDATA* rawR, double* COV, int state);

// �������ǵķǲ�����
double calculatesigma2(double eleangle);

//����ԪRTK����
int SingleEpochRTK(RAWDATA* rawB, RAWDATA* rawR);

// �����������ļ���ȥ
int PrintResulttoFile(string filename, RTK* Rtk);

void PrintHeadRTK(string filename);
#endif
