/*************************************************************************
���ƣ�����λ�ü���ģ��ͷ�ļ�
���� ��˼��
ѧ�� 2019302141082
�޸�ʱ�� 2021��12��14��
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

//����GPS����λ��/�ٶȵĺ���
double GPSPOSVEL(RAWDATA* raw, GPSTIME* gTime);

//����BDS����λ��/�ٶȵĺ���
double BDSPOSVEL(RAWDATA* raw, GPSTIME* gTime);

//����NEU����/�߶Ƚ�/��λ�ǵĺ���
int calNEU_AE(RAWDATA* raw, int i, int sys);
#endif

