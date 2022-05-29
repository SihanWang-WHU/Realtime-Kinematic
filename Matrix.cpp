#pragma once

#include "Matrix.h"
#define _CRT_SECURE_NO_DEPRECATE


//函数定义
/*************************************************************************
MatrixInit矩阵初始化
输入：r行数，c列数，a数据
输出：mat矩阵结构体
已测试
**************************************************************************/
void MatrixInit(matrix* newMat, int r, int c, double a[])
{
	newMat->row = r;
	newMat->column = c;//定义矩阵的行数和列数
	newMat->data = a;
}

/*************************************************************************
MatrixInit矩阵复制
输入：oldMat待复制矩阵，b为新矩阵数据的数组
输出：newMat新矩阵结构体
已测试
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
MatrixPrint矩阵输出
输入：mat待输出矩阵
输出：mat矩阵
已测试
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
MatrixDet求矩阵的行列式
输入：mat待求的矩阵
输出：矩阵的行列式的计算值determinant
已测试
**************************************************************************/
double MatrixDet(matrix* mat)
{
	if (mat->column != mat->row || mat->column == 0)
	{
		printf("矩阵不是方阵");
		return 1;
	}
	double determinant = 0;

	//定义几个计算的变量，这里的i为具体执行到的第几次loop；
	//r为具体计算到第几行，c为具体计算到第几列，loop为矩阵计算的次数
	int i, c, r, loop;

	//定义循环计算的次数，若矩阵是二维矩阵则循环一次，否则就循环矩阵的阶数次
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
	printf("矩阵的行列式为 %c\n", determinant);
}

/*************************************************************************
MatrixAdd矩阵加法
输入：mat1,mat2为两个待相加的矩阵，data为存放矩阵数据的数组
输出：newMat新矩阵结构体
已测试
**************************************************************************/
int MatrixAdd(matrix* newMat, matrix* mat1, matrix* mat2, double data[])
{
	int i;
	//判断两矩阵行列数是否完全一致
	if (mat1->row != mat2->row || mat1->column != mat2->column)
	{
		printf("矩阵加法维数不对应！");
		printf("按任意键退出\n");
		char redundant = getchar();
		exit(0);
	}

	for (i = 0; i < mat1->column * mat1->row; i++)
		data[i] = mat1->data[i] + mat2->data[i];
	MatrixInit(newMat, mat1->row, mat1->column, data);
	return 0;
}

/*************************************************************************
MatrixMinus矩阵减法
输入：mat1,mat2为两个待相减的矩阵，data为存放矩阵数据的数组
输出：newMat新矩阵结构体
已测试
**************************************************************************/
int MatrixMinus(matrix* newMat, matrix* mat1, matrix* mat2, double data[])
{
	// mat1 - mat2
	int i;
	//判断两矩阵行列数是否完全一致
	if (mat1->row != mat2->row || mat1->column != mat2->column)
	{
		printf("矩阵减法维数不对应！");
		printf("按任意键退出\n");
		char redundant = getchar();
		exit(0);
	}

	for (i = 0; i < mat1->column * mat1->row; i++)
		data[i] = mat1->data[i] - mat2->data[i];
	MatrixInit(newMat, mat1->row, mat1->column, data);
	return 0;
}

/*************************************************************************
MatrixTrans矩阵转置
输入：oldMat为待转置矩阵，data为转置后矩阵数据的数组指针
输出：转置结果newMat矩阵结构体
已测试
**************************************************************************/
void MatrixTrans(matrix* newMat, matrix* oldMat, double data[])
{
	int i, j, k;
	k = oldMat->column * oldMat->row;
	for (i = 0; i < oldMat->row; i++)
	{
		for (j = 0; j < oldMat->column; j++)
		{
			//把oldMat的data的第i行j列（索引值i*j-1）编程newMat的data的第j行i列
			data[j * oldMat->row + i] = oldMat->data[i * oldMat->column + j];
		}
		MatrixInit(newMat, oldMat->column, oldMat->row, data);
	}
}

/*************************************************************************
MatrixTimes矩阵数乘
输入：oldMat为待乘矩阵，n为给矩阵数乘的数，data为新矩阵数据的数组指针
输出：相乘结果newMat矩阵结构体
已测试
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
MatrixMulti矩阵乘法
输入：mat1,mat2为待乘矩阵，data为新矩阵数据的数组指针
输出：相乘结果newMat矩阵结构体
已测试
**************************************************************************/
int MatrixMulti(matrix* mat1, matrix* mat2, matrix* newMat, double data[])
{
	int i, j, k;
	//判断第一个矩阵的列数是否等于第二个矩阵的行数
	if (mat1->column != mat2->row)
	{
		printf("矩阵乘法维数不对应！");
		printf("按任意键退出\n");
		char redundant = getchar();
		exit(0);
	}

	//由于后面直接自加，所以在这里将原始数组初始化为0
	for (i = 0; i < mat1->row * mat2->column; i++)
		data[i] = 0;

	for (i = 0; i < mat1->row; i++)
	{
		for (k = 0; k < mat2->column; k++)
		{
			//将mat1的i行于mat2的k列相乘
			for (j = 0; j < mat1->column; j++)
			{
				//每个元素对应相乘相加
				data[i * mat2->column + k] += mat1->data[i * mat1->column + j] * mat2->data[k + j * mat2->column];
			}
		}
	}
	MatrixInit(newMat, mat1->row, mat2->column, data);
	return 0;
}

/****************************************************************************
  MatrixMultiply

  目的：矩阵相乘 M3 = M1*M2
  编号：01007

  参数:
  m1      M1的行数
  n1      M1的列数
  m2      M2的行数
  n2      M2的列数
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
MatrixInv矩阵求逆
输入：oldMat为待求逆的矩阵，data为新矩阵数据的数组指针
输出：newMat为求逆之后的矩阵,
已测试
**************************************************************************/
/****************************************************************************
  MatrixInv

  目的：矩阵求逆,采用全选主元高斯-约当法

  编号：01016

  参数:
  n      M1的行数和列数
  a      输入矩阵
  b      输出矩阵   b=inv(a)
  返回值：1=正常，0=致命错误

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

	/* 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变 */
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
		for (i = k; i < n; i++)   /* 查找右下角方阵中主元素的位置 */
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

		if (d < DBL_EPSILON)   /* 主元素接近于0，矩阵不可逆 */
		{
			//   printf("Divided by 0 in MatrixInv!\n");
			return 0;
		}

		if (is[k] != k)  /* 对主元素所在的行与右下角方阵的首行进行调换 */
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

		if (js[k] != k)  /* 对主元素所在的列与右下角方阵的首列进行调换 */
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
		b[l] = 1.0 / b[l];  /* 初等行变换 */
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

	for (k = n - 1; k >= 0; k--)  /* 将上面的行列调换重新恢复 */
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
		printf("矩阵不是方阵");
		return 1;
	}

	double* a = oldMat->data;
	/* 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变 */

	double* b = data;
	for (i = 0; i < n * n; i++)
		b[i] = a[i];

	for (k = 0; k < n; k++)
	{
		d = 0.0;
		for (i = k; i < n; i++)   /* 查找右下角方阵中主元素的位置 */
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
			printf("秩亏无法求逆!\n");
			return 1;
		}

		if (is[k] != k)  /* 对主元素所在的行与右下角方阵的首行进行调换 */
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

		if (js[k] != k)  /* 对主元素所在的列与右下角方阵的首列进行调换 */
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
		b[l] = 1.0 / b[l];  /* 初等行变换 */
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

	for (k = n - 1; k >= 0; k--)  /* 将上面的行列调换重新恢复 */
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