/*************************************************************************
作者 王思翰
学号 2019302141082
修改时间 2021年11月24日
**************************************************************************/

#include"Decoder.h"

using namespace std;

/*************************************************************************
crc32 检查CRC校验码函数
输入：无符号的字符串buf, 字符串长度len
已测试
**************************************************************************/
unsigned int crc32(const unsigned char* buff, int len)
{
	int i, j;
	unsigned int crc = 0;

	for (i = 0; i < len; i++)
	{
		crc ^= buff[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
			else crc >>= 1;
		}
	}
	return crc;
}

unsigned short U2(unsigned char buf[])
{
	unsigned short r;
	//memcpy函数用于
	//memcpy(目标地址，源地址，字节数);
	memcpy(&r, buf, 2);
	return r;
}

unsigned int U4(unsigned char buf[])
{
	unsigned int r;
	memcpy(&r, buf, 4);
	return r;
}


float F4(unsigned char buf[])
{
	float r;
	memcpy(&r, buf, 4);
	return r;
}

double R8(unsigned char buf[])
{
	double r;
	memcpy(&r, buf, 8);
	return r;
}



/*************************************************************************
DecodeOEM7MessageSock 从网口读取文件的函数
输入：RAWDATA的结构体指针, 这个历元读进来的数据量lenr，上个历元剩余的数据量lenrem，buff
功能：从网口读数据用来实现头文件定位，crc码校验，Message ID 的选择以及进入
**************************************************************************/
int DecodeOEM7MessageSock(unsigned char buff[], int lenr, int& lenrem, RAWDATA* Raw)
{
	int i, msgLen, len = lenr + lenrem;
	unsigned char Buff[10240];
	int MsgID;

	i = 0;
	while (1)
	{
		for (; i < len - 3; i++)
		{
			if (buff[i] == 0xAA && buff[i + 1] == 0x44 && buff[i + 2] == 0x12)
			{
				break;
				//找到跳出 进行后面的读取
			}
		}
		if ((i + 28) < len) //说明这个头文件被这个buff包含
		{
			msgLen = U2(buff + i + 8);
			msgLen += 32;
			// msgLen +28 +4
			// 加上了头文件和CRC校验码
		}
		else
		{
			lenrem = len - i;
			memcpy(buff, buff + i, len - i);
			//把这一包数据中没有算完的部分重新粘到buff前面
			break;
		}
		if ((i + msgLen) <= len)
		{
			memcpy(Buff, buff + i, msgLen);
			//把buff中的数据地址赋值给Buff 用来读取
			i += msgLen;
			// buff的索引往后移msglen位置
			// 让索引跳过这一段消息
		}
		else
		{
			lenrem = len - i;
			memcpy(buff, buff + i, len - i);
			break;
		}

		if (crc32(Buff, msgLen - 4) != U4(Buff + msgLen - 4))
		{
			printf("CRC check failure!\n");
			lenrem = len - i;
			memcpy(buff, buff + i, len - i);
			return 0;
		}
		MsgID = U2(Buff + 4);
		switch (MsgID)
		{
		case 43:
			DecodeObs(Buff, &Raw->obs);
			lenrem = len - i;
			memcpy(buff, buff + i, len - i);
			return 43;
		case 7:
			DecodeGPSEph(Buff, Raw->gpe);
			break;
		case 1696:
			DecodeBDSEph(Buff, Raw->bde);
			break;
		case 42:
			DecodeREFPOS(Buff, &Raw->RefPos);
			break;
		default:
			break;
		}
	}
	//return 0;
}



/*************************************************************************
DecodeGPSEph 读取GPS星历的函数
输入：读取文件的字符串buf, GPS星历结构体GPSEPHEM的结构体指针gpe
功能：读取GPS星历中的数据
已测试
**************************************************************************/
int DecodeGPSEph(unsigned char buf[], GPSEPHEM* gpe)
{
	//p字符串是从去掉了头文件之后的信息开始算的
	unsigned char* p = buf + 28;
	int Prn = U4(p);
	// PRN初步校验
	if (Prn < 0 || Prn>32)
	{
		cout << "GPS卫星不存在Prn号为" << Prn << "的卫星" << endl;
		return -1;
	}
	gpe[Prn - 1].Prn = Prn;
	gpe[Prn - 1].Sys = GPS;
	gpe[Prn - 1].GPS_G.Week = U2(buf + 14);
	gpe[Prn - 1].GPS_G.SecofWeek = 1.0E-3 * U4(buf + 16);
	gpe[Prn - 1].tow = R8(p + 4);
	gpe[Prn - 1].health = U4(p + 12); //卫星的健康状态 //0表示健康，1表示不健康
	gpe[Prn - 1].IODE[0] = U4(p + 16);
	gpe[Prn - 1].IODE[1] = U4(p + 20);
	gpe[Prn - 1].week = U4(p + 24);
	gpe[Prn - 1].zweek = U4(p + 28);
	gpe[Prn - 1].toe = R8(p + 32);
	gpe[Prn - 1].A = R8(p + 40);
	gpe[Prn - 1].DeltaN = R8(p + 48);
	gpe[Prn - 1].M0 = R8(p + 56);
	gpe[Prn - 1].ecc = R8(p + 64);
	gpe[Prn - 1].omega = R8(p + 72);
	gpe[Prn - 1].cuc = R8(p + 80);
	gpe[Prn - 1].cus = R8(p + 88);
	gpe[Prn - 1].crc = R8(p + 96);
	gpe[Prn - 1].crs = R8(p + 104);
	gpe[Prn - 1].cic = R8(p + 112);
	gpe[Prn - 1].cis = R8(p + 120);
	gpe[Prn - 1].I0 = R8(p + 128);
	gpe[Prn - 1].I0Dot = R8(p + 136);
	gpe[Prn - 1].Omega0 = R8(p + 144);
	gpe[Prn - 1].omegaDot = R8(p + 152);
	gpe[Prn - 1].IODC = U4(p + 160);
	gpe[Prn - 1].toc = R8(p + 164);
	gpe[Prn - 1].tgd = R8(p + 172);
	gpe[Prn - 1].af0 = R8(p + 180);
	gpe[Prn - 1].af1 = R8(p + 188);
	gpe[Prn - 1].af2 = R8(p + 196);
	gpe[Prn - 1].N = R8(p + 208);
	gpe[Prn - 1].Valid = true;
	return 0;
}

/*************************************************************************
DecodeBDSEph 读取BDS星历的函数
输入：读取文件的字符串buf, BDS星历结构体BDSEPHEM的结构体指针bpe
功能：读取BDS星历中的数据
已测试
**************************************************************************/
int DecodeBDSEph(unsigned char buf[], BDSEPHEM* bde)
{
	//p字符串是从去掉了头文件之后的信息开始算的
	unsigned char* p = buf + 28;
	int Prn = U4(p);
	// PRN初步校验
	if (Prn <= 0 || Prn >= 63)
	{
		cout << "BDS卫星不存在Prn号为" << Prn << "的卫星" << endl;
		return 0;
	}
	bde[Prn - 1].Sys = BDS;
	bde[Prn - 1].Prn = Prn;
	bde[Prn - 1].week = U4(p + 4);                            //在这里是实际的时间
	bde[Prn - 1].GPSG.Week = U2(buf + 14);
	bde[Prn - 1].GPSG.SecofWeek = 1.0E-3 * U4(buf + 16);
	bde[Prn - 1].health = U4(p + 16);                         //卫星的健康状态
	bde[Prn - 1].tgd1 = R8(p + 20);
	bde[Prn - 1].tgd2 = R8(p + 28);
	bde[Prn - 1].AODC = U4(p + 36);
	bde[Prn - 1].toc = U4(p + 40);
	bde[Prn - 1].a0 = R8(p + 44);
	bde[Prn - 1].a1 = R8(p + 52);
	bde[Prn - 1].a2 = R8(p + 60);
	bde[Prn - 1].AODE = U4(p + 68);
	bde[Prn - 1].toe = U4(p + 72);
	bde[Prn - 1].A = R8(p + 76) * R8(p + 76);
	bde[Prn - 1].ecc = R8(p + 84);
	bde[Prn - 1].DeltaN = R8(p + 100);
	bde[Prn - 1].omega = R8(p + 92);
	bde[Prn - 1].M0 = R8(p + 108);
	bde[Prn - 1].Omega0 = R8(p + 116);
	bde[Prn - 1].omegadot = R8(p + 124);
	bde[Prn - 1].i0 = R8(p + 132);
	bde[Prn - 1].IDOT = R8(p + 140);
	bde[Prn - 1].cuc = R8(p + 148);
	bde[Prn - 1].cus = R8(p + 156);
	bde[Prn - 1].crc = R8(p + 164);
	bde[Prn - 1].crs = R8(p + 172);
	bde[Prn - 1].cic = R8(p + 180);
	bde[Prn - 1].cis = R8(p + 188);
	bde[Prn - 1].Valid = true;
	return 0;
}

int DecodeREFPOS(unsigned char buff[], REFPOS* Psr)
{
	memset(Psr, 0, sizeof(REFPOS));
	unsigned char* p = buff + 28;
	Psr->Blh.lat = R8(p + 8);
	Psr->Blh.lng = R8(p + 16);
	Psr->Blh.h = R8(p + 24);
	Psr->DevB = F4(p + 40);
	Psr->DevL = F4(p + 44);
	Psr->DevH = F4(p + 48);
	Psr->Time.Week = U2(buff + 14);
	Psr->Time.SecofWeek = 1.0E-3 * U4(buff + 16);
	return 1;
}

/*************************************************************************
FindSatObsIndex 寻找单颗卫星观测值的数量的函数
功能：寻找某个卫星在range中的索引的函数
已测试
**************************************************************************/
int FindSatObsIndex(const int Prn, const NAVSYS Sys, EPOCHOBSDATA* data)
{
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if (data->range[i].Prn == 0) return i;
		else if (data->range[i].Prn == Prn && data->range[i].Sys == Sys) return i;
		else continue;
	}
}

/*************************************************************************
DecodeObs 读取观测数据的函数
输入：读取文件的字符串buf, 观测数据EPOCHOBSDATA的结构体指针obs
功能：读取观测值数据
已测试
**************************************************************************/
int DecodeObs(unsigned char buf[], EPOCHOBSDATA* obs)
{
	int n = 0;
	unsigned short PRN;
	int SigType, System, Freq;             /*  信号类型、信号通道、卫星号*/
	unsigned int track;                    /* numbers of seconds of continuous tracking (no cycle slipping) */
	int nobs;                              /* number of the observations */
	NAVSYS sys;                            /* type of navigation system */
	int sat;                               /* 解出来的卫星号，在判断之后赋给PRN*/
	int i, j;
	unsigned char* p = buf + 28;//跳过文件头
	nobs = U4(p);

	// 初始化obs 将obs中当前位置后面的size个字节用0替换并返回obs 
	memset(&obs->gpst, 0, sizeof(GPSTIME));
	memset(&obs->SATNUMS, 0, sizeof(unsigned int));
	memset(obs->range, 0, 36 * sizeof(RANGEDATA));

	//将gps时从头文件中读取出来
	obs->gpst.Week = U2(buf + 14);
	obs->gpst.SecofWeek = 1.0E-3 * U4(buf + 16);

	// 循环得到观测值 每个长44字节 p偏移4位

	for (p += 4, i = 0; i < nobs; i++, p += 44)
	{
		track = U4(p + 40);
		System = (track >> 16) & 0x07;// 与111进行与运算，获取卫星系统
		SigType = (track >> 21) & 0x1F; // 与11111进行与运算，获取信号类型

		if (System == 0)//GPS观测数据
		{
			sys = GPS;
			if (SigType == 0)  Freq = 0;
			else if (SigType == 9) Freq = 1;
			else continue;
		}
		else if (System == 4) //BDS观测数据
		{
			sys = BDS;
			if (SigType == 0 || SigType == 4)  Freq = 0;
			else if (SigType == 2 || SigType == 6)  Freq = 1;
			else continue;
		}
		else continue;

		if (sys == GPS || sys == BDS)
		{
			PRN = U2(p);
		}

		else
		{
			return -1;
		}

		n = FindSatObsIndex(PRN, sys, obs);
		obs->range[n].gpst.Week = U2(buf + 14);
		obs->range[n].gpst.SecofWeek = 1.0E-3 * U4(buf + 16);
		obs->range[n].Prn = PRN;
		obs->range[n].Sys = sys;// 将获取内容存入结构体中

		if (Freq == 0)
		{
			obs->range[n].P1 = R8(p + 4);// 伪距 double
			obs->range[n].L1 = -WL1_GPS * R8(p + 16);// 载波相位（转化成m） double
			obs->range[n].D1 = F4(p + 28);// 多普勒 float
			obs->range[n].P1Noise = F4(p + 12);// 伪距精度 float
			obs->range[n].L1Noise = F4(p + 24);// 载波相位精度 float
			obs->range[n].Snr1 = F4(p + 32);// 载噪比 float
			obs->range[n].flag = true;

		}
		else if (Freq == 1)
		{
			obs->range[n].P2 = R8(p + 4);
			obs->range[n].L2 = -WL2_GPS * R8(p + 16);
			obs->range[n].D2 = F4(p + 28);
			obs->range[n].P2Noise = F4(p + 12);
			obs->range[n].L2Noise = F4(p + 24);
			obs->range[n].Snr2 = F4(p + 32);
			obs->range[n].flag = true;

		}
		else continue;
	}
	for (int i = 0; i < MAXCHANNUM; i++)
	{
		if (obs->range[i].Prn != 0)
			obs->SATNUMS++;
	}
}

/*************************************************************************
NUMofSAT 判断观测值是否读取完整的函数
输入：原始观测数据指针RAWDATA raw
功能：判断是否同时读取了文件中的range数据和GPS星历数据
已测试
**************************************************************************/
bool NUMofSAT(RAWDATA* raw)
{
	int num = 0;
	int num_gps = 0;
	int num_bds = 0;
	for (int i = 0; i < MAXGPSPRN; i++)
	{
		int index = 0;
		index = FindSatObsIndex(i + 1, GPS, &raw->obs);
		if (raw->gpe[i].Valid && raw->obs.range[index].flag)
		{
			num_gps++;
		}
	}

	for (int j = 0; j < MAXBDSPRN; j++)
	{
		int index = 0;
		index = FindSatObsIndex(j + 1, BDS, &raw->obs);
		if (raw->bde[j].Valid && raw->obs.range[index].flag)
		{
			num_bds++;
		}
	}

	num = num_gps + num_bds;

	if (num >= 5)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*************************************************************************
DecodeHOEM7 读取文件头的函数
输入：文件指针file, 原始数据（RAWDATA）结构体指针raw, 原始数据（RAWDATA）结构体对象数组raw_save[5], 判断更新了几次的index
功能：用来实现头文件定位，crc码校验，Message ID 的选择以及进入
已测试
**************************************************************************/
int DecodeHOEM7(FILE* fp, RAWDATA* raw)
{
	//文件头一共有28个字节
	unsigned char buf[MAXBUFLEN];
	// msgType 为接受的文件的类型 0为二进制文件 1为ASCII码文件
	// msgID 为头文件中的Message ID
	// week 为GPS时中GPS的周数
	// len 为文件头中所包含的消息长度信息
	int msgType, msgID, week, len;
	unsigned char* buff = buf;
	while (true)
	{
		// fread返回成功读取的对象个数，不足则退出
		//从给定输入流 stream 读取至多 count 个对象到数组 buffer 中，
		//如同以对每个对象调用 size 次 std::fgetc ，并按顺序存储结果到转译为 unsigned char 数组的 buffer 中的相继位置。
		//流的文件位置指示器前进读取的字符数。
		if (fread(buf + 2, sizeof(unsigned char), 1, fp) < 1)
			//如果这里return-1了就说明文件没有读到
			return	-1;
		if (buf[0] == 0xAA && buf[1] == 0x44 && buf[2] == 0x12)
		{
			break;//寻找到文件头
		}
		else
		{
			//向前进一位
			buf[0] = buf[1];
			buf[1] = buf[2];
		}
	}
	// 再向后读取25字节 读入完整文件头
	if (fread(buf + 3, sizeof(unsigned char), 25, fp) < 25)
		return 0;

	len = U2(buf + 8);// Message Length Ushort

	// 读取整条语句，读不到则返回-1
	if (fread(buf + 28, sizeof(unsigned char), len + 4, fp) < (len + 4))
	{
		cout << "无法读取整条语句" << endl;
		return -1;
	}


	// CRC进行对比校验 int32
	if (crc32(buf, len + 28) != U4(buf + 28 + len))
	{
		return 1;
	}

	// 获取messageID Ushort
	msgID = U2(buf + 4);
	// 获取message Type
	msgType = (U1(buf + 6) >> 4) & 0x3;

	if (msgType != 0) /* message type: 0 for binary file,1 for ascii file*/
	{
		cout << "该文件不是二进制文件" << endl;
		return 0;
	}

	// 根据messageID选择调用不同功能的解码函数
	switch (msgID)
	{
	case 43:// 观测值Range
		DecodeObs(buf, &raw->obs);
		//把观测值此刻的时间赋值给raw中的gpst（便于调用）
		raw->gpst = raw->obs.gpst;
		// 每次在观测值被读取了之后就更新储存的值
		return 43;
		break;
	case 7:// GPS星历
		DecodeGPSEph(buf, raw->gpe);
		return 7;
		break;
	case 1696:// BDS星历
		DecodeBDSEph(buf, raw->bde);
		return 1696;
		break;
	case 42://接收机概略位置
		DecodeREFPOS(buf, &raw->RefPos);
		return 42;
	default:
		return 0;
	}

}