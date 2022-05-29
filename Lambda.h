#ifndef LAMBDA_H
#define LAMBDA_H

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>


#define MAXCOL      1024
#define LOOPMAX     20000           /* maximum count of search loop */
#define SGN(x)      ((x)<=0.0?-1.0:1.0)
#define ROUND(x)    (floor((x)+0.5))
#define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)

extern double* mat(int n, int m);

int matinv(double* A, int n);

/* lambda/mlambda integer least-square estimation ------------------------------
* integer least-square estimation. reduction is performed by lambda (ref.[1]),
* and search by mlambda (ref.[2]).
* args   : int    n      I  number of float parameters
*          int    m      I  number of fixed solutions , m=2
*          double *a     I  float parameters (n x 1)
*          double *Q     I  covariance matrix of float parameters (n x n)
*          double *F     O  fixed solutions (n x m)
*          double *s     O  sum of squared residuals of fixed solutions (1 x m)
* return : status (0:ok,other:error)
*-----------------------------------------------------------------------------*/
extern int lambda(int n, int m, const double* a, const double* Q, double* F, double* s);


#endif