/*************************************************************************
���ƣ�������ģ��ͷ�ļ�
���ߣ���˼��
ѧ�ţ�2019302141082
�޸�ʱ�䣺2021��12��20��
**************************************************************************/
#pragma once
#ifndef _ERRORCORRECT_H_
#define _ERRORCORRECT_H_

#include <math.h>
#include "Coordinate.h"
#include "SatPos.h"
#include "Decoder.h"
#include "ConstNums.h"


//Hopefieldģ�͸���
double Hopfield(RAWDATA* raw, NAVSYS sys, int prn);

//GPS��GF��Ϲ۲�ֵ
double IF_GPS(double P1, double P2);

//BDS��GF��Ϲ۲�ֵ
double IF_BDS(double P1, double P2);

//�ֲ�̽��
void DetectOutlier(RAWDATA* raw);
#endif



