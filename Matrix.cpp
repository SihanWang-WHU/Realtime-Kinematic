#pragma once

#include "Matrix.h"
#define _CRT_SECURE_NO_DEPRECATE


//��������
/*************************************************************************
MatrixInit�����ʼ��
���룺r������c������a����
�����mat����ṹ��
�Ѳ���
**************************************************************************/
void MatrixInit(matrix* newMat, int r, int c, double a[])
{
	newMat->row = r;
	newMat->column = c;//������������������
	newMat->data = a;
}

/*************************************************************************
MatrixInit������
���룺oldMat�����ƾ���bΪ�¾������ݵ�����
�����newMat�¾���ṹ��
�Ѳ���
**************************************************************************/
void MatrixCopy(matrix* newMat, matrix* oldMat, double b[])
{
	int num;
	num = oldMat->row * oldMat->column;
	for (int i = 0; i < num; i++)
	{
		b[i] = oldMat->data[i];
	}
	MatrixInit(newMat, oldMat->row, oldMat->column, b);
}

/*************************************************************************
MatrixPrint�������
���룺mat���������
�����mat����
�Ѳ���
**************************************************************************/
void MatrixPrint(matrix* mat)
{
	int index, i, j;
	for (i = 0; i < mat->row; i++)
	{
		for (j = 0; j < mat->column; j++)
		{
			index = i * mat->column + j;
			printf("%.4f,\t", mat->data[index]);
		}
		printf("\n");
	}
}

void MatrixPrinttoFile(matrix* mat, FILE *fp)
{
	int index, i, j;
	for (i = 0; i < mat->row; i++)
	{
		for (j = 0; j < mat->column; j++)
		{
			index = i * mat->column + j;
			fprintf(fp, "%-12lf\t", mat->data[index]);
		}
		fprintf(fp, "\n");
	}
}

/*************************************************************************
MatrixDet����������ʽ
���룺mat����ľ���
��������������ʽ�ļ���ֵdeterminant
�Ѳ���
**************************************************************************/
double MatrixDet(matrix* mat)
{
	if (mat->column != mat->row || mat->column == 0)
	{
		printf("�����Ƿ���");
		return 1;
	}
	double determinant = 0;

	//���弸������ı����������iΪ����ִ�е��ĵڼ���loop��
	//rΪ������㵽�ڼ��У�cΪ������㵽�ڼ��У�loopΪ�������Ĵ���
	int i, c, r, loop;

	//����ѭ������Ĵ������������Ƕ�ά������ѭ��һ�Σ������ѭ������Ľ�����
	if (mat->column == 2)
	{
		loop = 1;
	}
	else
	{
		loop = mat->column;
	}

	for (i = 0; i < loop; i++)
	{
		double temp = 1;
		for (r = 0, c = i; r < mat->column; r++, c++)
		{
			temp = temp * mat->data[mat->column * r + c];
			determinant += temp;
		}

	}

	for (i = 0; i < loop; i++)
	{
		double temp = 1;
		int flag;
		for (r = 0, c = mat->column - i - 1; r < mat->column; r++, c--)
		{
			if (c >= 0)
			{
				flag = mat->column * r + c;
				temp = temp * mat->data[flag];
				determinant -= temp;
			}
			if (c < 0)
			{
				flag = mat->column * r + c + mat->column;
				temp = temp * mat->data[flag];
				determinant -= temp;
			}

		}

	}
	printf("���������ʽΪ %c\n", determinant);
}

/*************************************************************************
MatrixAdd����ӷ�
���룺mat1,mat2Ϊ��������ӵľ���dataΪ��ž������ݵ�����
�����newMat�¾���ṹ��
�Ѳ���
**************************************************************************/
int MatrixAdd(matrix* newMat, matrix* mat1, matrix* mat2, double data[])
{
	int i;
	//�ж��������������Ƿ���ȫһ��
	if (mat1->row != mat2->row || mat1->column != mat2->column)
	{
		printf("����ӷ�ά������Ӧ��");
		printf("��������˳�\n");
		char redundant = getchar();
		exit(0);
	}

	for (i = 0; i < mat1->column * mat1->row; i++)
		data[i] = mat1->data[i] + mat2->data[i];
	MatrixInit(newMat, mat1->row, mat1->column, data);
	return 0;
}

/*************************************************************************
MatrixMinus�������
���룺mat1,mat2Ϊ����������ľ���dataΪ��ž������ݵ�����
�����newMat�¾���ṹ��
�Ѳ���
**************************************************************************/
int MatrixMinus(matrix* newMat, matrix* mat1, matrix* mat2, double data[])
{
	// mat1 - mat2
	int i;
	//�ж��������������Ƿ���ȫһ��
	if (mat1->row != mat2->row || mat1->column != mat2->column)
	{
		printf("�������ά������Ӧ��");
		printf("��������˳�\n");
		char redundant = getchar();
		exit(0);
	}

	for (i = 0; i < mat1->column * mat1->row; i++)
		data[i] = mat1->data[i] - mat2->data[i];
	MatrixInit(newMat, mat1->row, mat1->column, data);
	return 0;
}

/*************************************************************************
MatrixTrans����ת��
���룺oldMatΪ��ת�þ���dataΪת�ú�������ݵ�����ָ��
�����ת�ý��newMat����ṹ��
�Ѳ���
**************************************************************************/
void MatrixTrans(matrix* newMat, matrix* oldMat, double data[])
{
	int i, j, k;
	k = oldMat->column * oldMat->row;
	for (i = 0; i < oldMat->row; i++)
	{
		for (j = 0; j < oldMat->column; j++)
		{
			//��oldMat��data�ĵ�i��j�У�����ֵi*j-1�����newMat��data�ĵ�j��i��
			data[j * oldMat->row + i] = oldMat->data[i * oldMat->column + j];
		}
		MatrixInit(newMat, oldMat->column, oldMat->row, data);
	}
}

/*************************************************************************
MatrixTimes��������
���룺oldMatΪ���˾���nΪ���������˵�����dataΪ�¾������ݵ�����ָ��
�������˽��newMat����ṹ��
�Ѳ���
**************************************************************************/
void MatrixTimes(matrix* newMat, matrix* oldMat, double n, double data[])
{
	int i;
	for (i = 0; i < oldMat->row * oldMat->column; i++)
	{
		data[i] = n * oldMat->data[i];
	}
	MatrixInit(newMat, oldMat->row, oldMat->column, data);
}

/*************************************************************************
MatrixMulti����˷�
���룺mat1,mat2Ϊ���˾���dataΪ�¾������ݵ�����ָ��
�������˽��newMat����ṹ��
�Ѳ���
**************************************************************************/
int MatrixMulti(matrix* mat1, matrix* mat2, matrix* newMat, double data[])
{
	int i, j, k;
	//�жϵ�һ������������Ƿ���ڵڶ������������
	if (mat1->column != mat2->row)
	{
		printf("����˷�ά������Ӧ��");
		printf("��������˳�\n");
		char redundant = getchar();
		exit(0);
	}

	//���ں���ֱ���Լӣ����������ｫԭʼ�����ʼ��Ϊ0
	for (i = 0; i < mat1->row * mat2->column; i++)
		data[i] = 0;

	for (i = 0; i < mat1->row; i++)
	{
		for (k = 0; k < mat2->column; k++)
		{
			//��mat1��i����mat2��k�����
			for (j = 0; j < mat1->column; j++)
			{
				//ÿ��Ԫ�ض�Ӧ������
				data[i * mat2->column + k] += mat1->data[i * mat1->column + j] * mat2->data[k + j * mat2->column];
			}
		}
	}
	MatrixInit(newMat, mat1->row, mat2->column, data);
	return 0;
}

/****************************************************************************
  MatrixMultiply

  Ŀ�ģ�������� M3 = M1*M2
  ��ţ�01007

  ����:
  m1      M1������
  n1      M1������
  m2      M2������
  n2      M2������
****************************************************************************/
void MatrixMultiply(int m1, int n1, int m2, int n2,
	const double M1[], const double M2[], double M3[])
{
	int i, j, k;
	double Sum;

	if ((n1 != m2) || (m1 <= 0) || (n1 <= 0) || (m2 <= 0) || (n2 <= 0))
	{
		printf("Error dimension in MatrixMultiply!\n");
		return;
	}

	for (i = 0; i < m1; i++)
	{
		for (j = 0; j < n2; j++)
		{
			Sum = 0.0;

			for (k = 0; k < n1; k++)
			{
				Sum = Sum + *(M1 + i * n1 + k) * *(M2 + k * n2 + j);
			}

			*(M3 + i * n2 + j) = Sum;
		}
	}
}

/*************************************************************************
MatrixInv��������
���룺oldMatΪ������ľ���dataΪ�¾������ݵ�����ָ��
�����newMatΪ����֮��ľ���,
�Ѳ���
**************************************************************************/
/****************************************************************************
  MatrixInv

  Ŀ�ģ���������,����ȫѡ��Ԫ��˹-Լ����

  ��ţ�01016

  ����:
  n      M1������������
  a      �������
  b      �������   b=inv(a)
  ����ֵ��1=������0=��������

****************************************************************************/

int MatInv(int n, double a[], double b[])
{
	int i, j, k, l, u, v, is[250] = { 0 }, js[250] = { 0 };   /* matrix dimension <= 250 */
	double d, p;

	if (n <= 0)
	{
		printf("Error dimension in MatrixInv!\n");
		return 0;
	}

	/* ���������ֵ���������b�������b�������棬a���󲻱� */
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			b[i * n + j] = a[i * n + j];
		}
	}

	for (k = 0; k < n; k++)
	{
		d = 0.0;
		for (i = k; i < n; i++)   /* �������½Ƿ�������Ԫ�ص�λ�� */
		{
			for (j = k; j < n; j++)
			{
				l = n * i + j;
				p = fabs(b[l]);
				if (p > d)
				{
					d = p;
					is[k] = i;
					js[k] = j;
				}
			}
		}

		if (d < DBL_EPSILON)   /* ��Ԫ�ؽӽ���0�����󲻿��� */
		{
			//   printf("Divided by 0 in MatrixInv!\n");
			return 0;
		}

		if (is[k] != k)  /* ����Ԫ�����ڵ��������½Ƿ�������н��е��� */
		{
			for (j = 0; j < n; j++)
			{
				u = k * n + j;
				v = is[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		if (js[k] != k)  /* ����Ԫ�����ڵ��������½Ƿ�������н��е��� */
		{
			for (i = 0; i < n; i++)
			{
				u = i * n + k;
				v = i * n + js[k];
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		l = k * n + k;
		b[l] = 1.0 / b[l];  /* �����б任 */
		for (j = 0; j < n; j++)
		{
			if (j != k)
			{
				u = k * n + j;
				b[u] = b[u] * b[l];
			}
		}
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				for (j = 0; j < n; j++)
				{
					if (j != k)
					{
						u = i * n + j;
						b[u] = b[u] - b[i * n + k] * b[k * n + j];
					}
				}
			}
		}
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				u = i * n + k;
				b[u] = -b[u] * b[l];
			}
		}
	}

	for (k = n - 1; k >= 0; k--)  /* ����������е������»ָ� */
	{
		if (js[k] != k)
		{
			for (j = 0; j < n; j++)
			{
				u = k * n + j;
				v = js[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
		if (is[k] != k)
		{
			for (i = 0; i < n; i++)
			{
				u = i * n + k;
				v = is[k] + i * n;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
	}

	return (1);
}

int MatrixInv(matrix* newMat, matrix* oldMat, double data[])
{
	int i, j, k, l, u, v, is[100], js[100], n = oldMat->column;   /* matrix dimension <= 100 */
	double d, p;

	if (oldMat->column != oldMat->row || oldMat->column == 0)
	{
		printf("�����Ƿ���");
		return 1;
	}

	double* a = oldMat->data;
	/* ���������ֵ���������b�������b�������棬a���󲻱� */

	double* b = data;
	for (i = 0; i < n * n; i++)
		b[i] = a[i];

	for (k = 0; k < n; k++)
	{
		d = 0.0;
		for (i = k; i < n; i++)   /* �������½Ƿ�������Ԫ�ص�λ�� */
		{
			for (j = k; j < n; j++)
			{
				l = n * i + j;
				p = fabs(b[l]);
				if (p > d)
				{
					d = p;
					is[k] = i;
					js[k] = j;
				}
			}
		}

		if (d < 1.0E-12)
		{
			printf("�ȿ��޷�����!\n");
			return 1;
		}

		if (is[k] != k)  /* ����Ԫ�����ڵ��������½Ƿ�������н��е��� */
		{
			for (j = 0; j < n; j++)
			{
				u = k * n + j;
				v = is[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		if (js[k] != k)  /* ����Ԫ�����ڵ��������½Ƿ�������н��е��� */
		{
			for (i = 0; i < n; i++)
			{
				u = i * n + k;
				v = i * n + js[k];
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}

		l = k * n + k;
		b[l] = 1.0 / b[l];  /* �����б任 */
		for (j = 0; j < n; j++)
		{
			if (j != k)
			{
				u = k * n + j;
				b[u] = b[u] * b[l];
			}
		}
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				for (j = 0; j < n; j++)
				{
					if (j != k)
					{
						u = i * n + j;
						b[u] = b[u] - b[i * n + k] * b[k * n + j];
					}
				}
			}
		}
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				u = i * n + k;
				b[u] = -b[u] * b[l];
			}
		}
	}

	for (k = n - 1; k >= 0; k--)  /* ����������е������»ָ� */
	{
		if (js[k] != k)
		{
			for (j = 0; j < n; j++)
			{
				u = k * n + j;
				v = js[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
		if (is[k] != k)
		{
			for (i = 0; i < n; i++)
			{
				u = i * n + k;
				v = is[k] + i * n;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		}
	}

	MatrixInit(newMat, n, n, b);
	return 0;
}