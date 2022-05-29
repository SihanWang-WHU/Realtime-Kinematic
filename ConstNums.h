/*************************************************************************
名称：常用参数模块头文件
作者：王思翰
学号：2019302141082
**************************************************************************/
#pragma once
#ifndef CONSTNUMS_H
#define CONSTNUMS_H

/* RTK 用的一些参数*/
#define MODE           0                  // RTK解算模式，0是网口，1是文件
#define COVL2B         10000.0            // GPS L1，BDS B1I的伪相比
#define COVL2B_L2      10000.00           // GPS L2 的伪相比
#define COVL2B_B1I     90000.00           // BDS B1I的伪相比
#define COVL2B_B3I     250000.00          // BDS B3I的伪相比
#define AMBIGUITY      3                  // 模糊度固定阈值
#define SnrLim         33                 // 卫星的载噪比阈值


/* Math parameters */
#define PI 3.1415926535897932384626433832795
#define C_SPEED 2.99792458e8

/* Physical parameters of the Earth, Sun and Moon  */
#define R_WGS84  6378137.0          /* Radius Earth [m]; WGS-84  */
#define F_WGS84  1.0/298.257223563  /* Flattening; WGS-84   */
#define Omega_WGS 7.2921151467e-5   /*[rad/s], the earth rotation rate */
#define GM_Earth   398600.5e+9     /* [m^3/s^2]; WGS-84 */
#define GM_JGM3   398600.4415e+9     /* [m^3/s^2]; JGM3  */
#define E2_WGS84 0.0066943799901413             


/* Physical parameters of the Earth, Sun and Moon  */
#define R_CGS2K  6378137.0          /* Radius Earth [m]; CGCS2000  */
#define F_CGS2K  1.0/298.257222101  /* Flattening; CGCS2000   */
#define E2_CGS2K 0.0066943800229008 
#define Omega_BDS 7.2921150e-5      /*[rad/s], the earth rotation rate */
#define GM_BDS   398600.4418e+9     /* [m^3/s^2]; CGCS2000  */

/* some constants about GPS satellite signal */
#define  FL1_GPS  1575.42E6             /* L1信号频率 */
#define  FL2_GPS  1227.60E6             /* L2信号频率 */
#define  FG12R    (77/60.0)             /* FG1_Freq/FG2_Freq */
#define  FG12R2   (5929/3600.0)
#define  WL1_GPS  (C_SPEED/FL1_GPS)
#define  WL2_GPS  (C_SPEED/FL2_GPS)
#define  GPS_Miu 3.986005e14   //GM
#define  GPS_OmegaDot 7.2921151467e-5  //地球自转角速度

/* some constants about Compass satellite signal */
#define  FB1_BDS        1561.098E6               /* B1信号的基准频率 */
#define  FB2_BDS        1207.140E6               /* B2信号的基准频率 */
#define  FB3_BDS        1268.520E6               /* B3信号的基准频率 */
#define  BDS_Miu        3.986004418e14  //GM
#define  BDS_OmegaDot   7.2921150e-5  //地球自转角速度
#define  SnrBDS         SnrLim 

#define  FC12R    (FB1_BDS/FB2_BDS)       /* FG1_CPS/FG2_CPS */
#define  FC12R2   (FC12R*FC12R)           /* FG1_CPS^2/FG2_CPS^2 */
#define  FC13R    (FB1_BDS/FB3_BDS)       /* FG1_CPS^2/FG3_CPS^2 */
#define  FC13R2   (FC13R*FC13R)
#define  WB1_BDS  (C_SPEED/FB1_BDS)
#define  WB2_BDS  (C_SPEED/FB2_BDS)
#define  WB3_BDS  (C_SPEED/FB3_BDS)

#define OEM7SYNC1       0xAA    /* oem7/6/4 message start sync code 1 */
#define OEM7SYNC2       0x44    /* oem7/6/4 message start sync code 2 */
#define OEM7SYNC3       0x12    /* oem7/6/4 message start sync code 3 */

#define POLYCRC32 0xEDB88320L   /* crc校验码*/


/* 定义一些Message ID */
#define ID_GPSEPHEM 7           /* GPS星历 */
#define ID_BDSEPHEM 1696        /* BDS星历 */
#define ID_RANGE    43          /* Range数据 */

/* 定义解码的时候一些字符串长度的值 */
#define OEM7HLEN 28             /* oem7 头文件的长度*/
#define MAXBUFLEN 40960         /* 读取文件时的buffer字符串的最长长度 */
#define GPST_BDT  14            /* GPS时与北斗时的差值[s] */
#define MAXCHANNUM 36
#define MAXSATNUM  64
#define MAXGPSPRN  33           /* 最大的GPS卫星的PRN号 */
#define MAXBDSPRN 65            /* 最大的北斗卫星的PRN号 */
#define MAXOBSTYPENUM 9
#define MAXGEOPRN 5             /* 最大的GEO卫星号 */ //不完善，BDS-3的GEO被排除在外了

#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))

#endif



