/*************************************************************************
���� ��˼��
ѧ�� 2019302141082
�޸�ʱ�� 2021��11��24��
**************************************************************************/

#include"Decoder.h"

using namespace std;

/*************************************************************************
crc32 ���CRCУ���뺯��
���룺�޷��ŵ��ַ���buf, �ַ�������len
�Ѳ���
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
	//memcpy��������
	//memcpy(Ŀ���ַ��Դ��ַ���ֽ���);
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
DecodeOEM7MessageSock �����ڶ�ȡ�ļ��ĺ���
���룺RAWDATA�Ľṹ��ָ��, �����Ԫ��������������lenr���ϸ���Ԫʣ���������lenrem��buff
���ܣ������ڶ���������ʵ��ͷ�ļ���λ��crc��У�飬Message ID ��ѡ���Լ�����
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
				//�ҵ����� ���к���Ķ�ȡ
			}
		}
		if ((i + 28) < len) //˵�����ͷ�ļ������buff����
		{
			msgLen = U2(buff + i + 8);
			msgLen += 32;
			// msgLen +28 +4
			// ������ͷ�ļ���CRCУ����
		}
		else
		{
			lenrem = len - i;
			memcpy(buff, buff + i, len - i);
			//����һ��������û������Ĳ�������ճ��buffǰ��
			break;
		}
		if ((i + msgLen) <= len)
		{
			memcpy(Buff, buff + i, msgLen);
			//��buff�е����ݵ�ַ��ֵ��Buff ������ȡ
			i += msgLen;
			// buff������������msglenλ��
			// ������������һ����Ϣ
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
DecodeGPSEph ��ȡGPS�����ĺ���
���룺��ȡ�ļ����ַ���buf, GPS�����ṹ��GPSEPHEM�Ľṹ��ָ��gpe
���ܣ���ȡGPS�����е�����
�Ѳ���
**************************************************************************/
int DecodeGPSEph(unsigned char buf[], GPSEPHEM* gpe)
{
	//p�ַ����Ǵ�ȥ����ͷ�ļ�֮�����Ϣ��ʼ���
	unsigned char* p = buf + 28;
	int Prn = U4(p);
	// PRN����У��
	if (Prn < 0 || Prn>32)
	{
		cout << "GPS���ǲ�����Prn��Ϊ" << Prn << "������" << endl;
		return -1;
	}
	gpe[Prn - 1].Prn = Prn;
	gpe[Prn - 1].Sys = GPS;
	gpe[Prn - 1].GPS_G.Week = U2(buf + 14);
	gpe[Prn - 1].GPS_G.SecofWeek = 1.0E-3 * U4(buf + 16);
	gpe[Prn - 1].tow = R8(p + 4);
	gpe[Prn - 1].health = U4(p + 12); //���ǵĽ���״̬ //0��ʾ������1��ʾ������
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
DecodeBDSEph ��ȡBDS�����ĺ���
���룺��ȡ�ļ����ַ���buf, BDS�����ṹ��BDSEPHEM�Ľṹ��ָ��bpe
���ܣ���ȡBDS�����е�����
�Ѳ���
**************************************************************************/
int DecodeBDSEph(unsigned char buf[], BDSEPHEM* bde)
{
	//p�ַ����Ǵ�ȥ����ͷ�ļ�֮�����Ϣ��ʼ���
	unsigned char* p = buf + 28;
	int Prn = U4(p);
	// PRN����У��
	if (Prn <= 0 || Prn >= 63)
	{
		cout << "BDS���ǲ�����Prn��Ϊ" << Prn << "������" << endl;
		return 0;
	}
	bde[Prn - 1].Sys = BDS;
	bde[Prn - 1].Prn = Prn;
	bde[Prn - 1].week = U4(p + 4);                            //��������ʵ�ʵ�ʱ��
	bde[Prn - 1].GPSG.Week = U2(buf + 14);
	bde[Prn - 1].GPSG.SecofWeek = 1.0E-3 * U4(buf + 16);
	bde[Prn - 1].health = U4(p + 16);                         //���ǵĽ���״̬
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
FindSatObsIndex Ѱ�ҵ������ǹ۲�ֵ�������ĺ���
���ܣ�Ѱ��ĳ��������range�е������ĺ���
�Ѳ���
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
DecodeObs ��ȡ�۲����ݵĺ���
���룺��ȡ�ļ����ַ���buf, �۲�����EPOCHOBSDATA�Ľṹ��ָ��obs
���ܣ���ȡ�۲�ֵ����
�Ѳ���
**************************************************************************/
int DecodeObs(unsigned char buf[], EPOCHOBSDATA* obs)
{
	int n = 0;
	unsigned short PRN;
	int SigType, System, Freq;             /*  �ź����͡��ź�ͨ�������Ǻ�*/
	unsigned int track;                    /* numbers of seconds of continuous tracking (no cycle slipping) */
	int nobs;                              /* number of the observations */
	NAVSYS sys;                            /* type of navigation system */
	int sat;                               /* ����������Ǻţ����ж�֮�󸳸�PRN*/
	int i, j;
	unsigned char* p = buf + 28;//�����ļ�ͷ
	nobs = U4(p);

	// ��ʼ��obs ��obs�е�ǰλ�ú����size���ֽ���0�滻������obs 
	memset(&obs->gpst, 0, sizeof(GPSTIME));
	memset(&obs->SATNUMS, 0, sizeof(unsigned int));
	memset(obs->range, 0, 36 * sizeof(RANGEDATA));

	//��gpsʱ��ͷ�ļ��ж�ȡ����
	obs->gpst.Week = U2(buf + 14);
	obs->gpst.SecofWeek = 1.0E-3 * U4(buf + 16);

	// ѭ���õ��۲�ֵ ÿ����44�ֽ� pƫ��4λ

	for (p += 4, i = 0; i < nobs; i++, p += 44)
	{
		track = U4(p + 40);
		System = (track >> 16) & 0x07;// ��111���������㣬��ȡ����ϵͳ
		SigType = (track >> 21) & 0x1F; // ��11111���������㣬��ȡ�ź�����

		if (System == 0)//GPS�۲�����
		{
			sys = GPS;
			if (SigType == 0)  Freq = 0;
			else if (SigType == 9) Freq = 1;
			else continue;
		}
		else if (System == 4) //BDS�۲�����
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
		obs->range[n].Sys = sys;// ����ȡ���ݴ���ṹ����

		if (Freq == 0)
		{
			obs->range[n].P1 = R8(p + 4);// α�� double
			obs->range[n].L1 = -WL1_GPS * R8(p + 16);// �ز���λ��ת����m�� double
			obs->range[n].D1 = F4(p + 28);// ������ float
			obs->range[n].P1Noise = F4(p + 12);// α�ྫ�� float
			obs->range[n].L1Noise = F4(p + 24);// �ز���λ���� float
			obs->range[n].Snr1 = F4(p + 32);// ����� float
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
NUMofSAT �жϹ۲�ֵ�Ƿ��ȡ�����ĺ���
���룺ԭʼ�۲�����ָ��RAWDATA raw
���ܣ��ж��Ƿ�ͬʱ��ȡ���ļ��е�range���ݺ�GPS��������
�Ѳ���
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
DecodeHOEM7 ��ȡ�ļ�ͷ�ĺ���
���룺�ļ�ָ��file, ԭʼ���ݣ�RAWDATA���ṹ��ָ��raw, ԭʼ���ݣ�RAWDATA���ṹ���������raw_save[5], �жϸ����˼��ε�index
���ܣ�����ʵ��ͷ�ļ���λ��crc��У�飬Message ID ��ѡ���Լ�����
�Ѳ���
**************************************************************************/
int DecodeHOEM7(FILE* fp, RAWDATA* raw)
{
	//�ļ�ͷһ����28���ֽ�
	unsigned char buf[MAXBUFLEN];
	// msgType Ϊ���ܵ��ļ������� 0Ϊ�������ļ� 1ΪASCII���ļ�
	// msgID Ϊͷ�ļ��е�Message ID
	// week ΪGPSʱ��GPS������
	// len Ϊ�ļ�ͷ������������Ϣ������Ϣ
	int msgType, msgID, week, len;
	unsigned char* buff = buf;
	while (true)
	{
		// fread���سɹ���ȡ�Ķ���������������˳�
		//�Ӹ��������� stream ��ȡ���� count ���������� buffer �У�
		//��ͬ�Զ�ÿ��������� size �� std::fgetc ������˳��洢�����ת��Ϊ unsigned char ����� buffer �е����λ�á�
		//�����ļ�λ��ָʾ��ǰ����ȡ���ַ�����
		if (fread(buf + 2, sizeof(unsigned char), 1, fp) < 1)
			//�������return-1�˾�˵���ļ�û�ж���
			return	-1;
		if (buf[0] == 0xAA && buf[1] == 0x44 && buf[2] == 0x12)
		{
			break;//Ѱ�ҵ��ļ�ͷ
		}
		else
		{
			//��ǰ��һλ
			buf[0] = buf[1];
			buf[1] = buf[2];
		}
	}
	// ������ȡ25�ֽ� ���������ļ�ͷ
	if (fread(buf + 3, sizeof(unsigned char), 25, fp) < 25)
		return 0;

	len = U2(buf + 8);// Message Length Ushort

	// ��ȡ������䣬�������򷵻�-1
	if (fread(buf + 28, sizeof(unsigned char), len + 4, fp) < (len + 4))
	{
		cout << "�޷���ȡ�������" << endl;
		return -1;
	}


	// CRC���жԱ�У�� int32
	if (crc32(buf, len + 28) != U4(buf + 28 + len))
	{
		return 1;
	}

	// ��ȡmessageID Ushort
	msgID = U2(buf + 4);
	// ��ȡmessage Type
	msgType = (U1(buf + 6) >> 4) & 0x3;

	if (msgType != 0) /* message type: 0 for binary file,1 for ascii file*/
	{
		cout << "���ļ����Ƕ������ļ�" << endl;
		return 0;
	}

	// ����messageIDѡ����ò�ͬ���ܵĽ��뺯��
	switch (msgID)
	{
	case 43:// �۲�ֵRange
		DecodeObs(buf, &raw->obs);
		//�ѹ۲�ֵ�˿̵�ʱ�丳ֵ��raw�е�gpst�����ڵ��ã�
		raw->gpst = raw->obs.gpst;
		// ÿ���ڹ۲�ֵ����ȡ��֮��͸��´����ֵ
		return 43;
		break;
	case 7:// GPS����
		DecodeGPSEph(buf, raw->gpe);
		return 7;
		break;
	case 1696:// BDS����
		DecodeBDSEph(buf, raw->bde);
		return 1696;
		break;
	case 42://���ջ�����λ��
		DecodeREFPOS(buf, &raw->RefPos);
		return 42;
	default:
		return 0;
	}

}