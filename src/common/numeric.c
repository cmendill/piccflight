#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <netdb.h>
#include <mkl.h>

/******************************************************************************
        NUMERIC_multiply
******************************************************************************/
/*Function: alpha * A * b + beta = result
    alpha  --> scalar multiple
    A      --> vector: size m x n (m columns, n rows) (1-D matrix)
    b      --> vector: size m
    beta   --> scalar offset
    result --> vector: size n
*/
void NUMERIC_multiply(double *A, double *b,double *result,int m, int n) {
  const double alpha = 1.0, beta = 0.0;
  const int incx=1, incy=1;
  cblas_dgemv(CblasColMajor, CblasNoTrans, m, n, alpha, A, m, b, incx, beta, result, incy );
}
