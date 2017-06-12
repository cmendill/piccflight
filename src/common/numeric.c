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
#define MIN(A,B)   ((A) < (B) ? (A) : (B))

/******************************************************************************
        num_dgemv
******************************************************************************/
/*Function: result = A*b
  A      --> double matrix of [m x n] elements stored in a 1d array in column major format
            (i.e. first m elements are the m elements of the first column of the matrix)
  b      --> double vector of [n] elements stored in a 1d array
  result --> double vector of [m] elements stored in a 1d array
  m      --> number of rows of A (also number of elements in result)
  n      --> number of columns of A (also number of elements in b)
*/
void num_dgemv(double *A, double *b, double *result, int m, int n) {
  const double alpha = 1.0, beta = 0.0;
  const int incx=1, incy=1;
  cblas_dgemv(CblasColMajor, CblasNoTrans, m, n, alpha, A, m, b, incx, beta, result, incy );
}

/******************************************************************************
        num_dgesvdi
******************************************************************************/
/*Function: A_inv = svd_inverse(A)
  A      --> double matrix of [m x n] elements stored in a 1d array in column major format
            (i.e. first m elements are the m elements of the first column of the matrix)
  A_inv  --> double matrix of [n x m] elements stored in a 1d array in column major format
            (i.e. first m elements are the m elements of the first column of the matrix)
  m      --> number of rows of A (also number of columns of A_inv)
  n      --> number of columns of A (also number of rows of A_inv)
*/
void num_dgesvdi(double* A, double* A_inv, int m, int n) {
  if (m<=n) return;  // only supports m>n case TODO m<=n case
  int i;
  MKL_INT info;
  double alpha = 1.0, beta = 0.0;
  int idiag[1] = {0};
  int ndiag = 1;
  double* superb;
  if((superb = malloc((MIN(m,n)-1)*sizeof(double))) == NULL)
    perror("malloc()");
  
  double *s, *u, *vt;
  if((s = malloc(n*sizeof(double))) == NULL)
    perror("malloc(s)");
  memset(s, 0, n*sizeof(double));
  
  if((u = malloc(m*n*sizeof(double))) == NULL)
    perror("malloc(u)");
  memset(u, 0, m*n*sizeof(double));

  if((vt = malloc(n*n*sizeof(double))) == NULL)
    perror("malloc(vt)");
  memset(vt, 0, n*n*sizeof(double));

  double *s_inv, *temp;
  if((s_inv = malloc(n*sizeof(double))) == NULL)
    perror("malloc(s_inv)");
  memset(s_inv, 0, n*sizeof(double));

  if((temp = malloc(n*m*sizeof(double))) == NULL)
    perror("malloc(temp)");
  memset(temp, 0, n*m*sizeof(double));

  // singular value decompose
  // A = [u].diagonal_matrix(s).[vt]
  info = LAPACKE_dgesvd(LAPACK_COL_MAJOR, 'A', 'A', m, n, A, m, s, u, m, vt, n, superb);

  // build the s_inv array s_inv = 1/s
  for(i=0; i<n; i++){
    // s_inv[i] = s[i]/(pow(s[i],2) + alpha^2); // tikhonov regularization
    // s_inv[i] = ((1.0/s[i])>cutoff)?0.0:(1.0/s[i]); // cutoff regularization
    s_inv[i] = 1.0/s[i]; // no Regularization
  }
  s_inv[0]=0.0;
  /* transpose u, vt can be transposed at the multiplication cblas_dgemm*/
  mkl_dimatcopy('C', 'T', m, n, alpha, u, m, n);

  /* multiply diagonal_matrix(1/s).[ut] */
  mkl_ddiamm("n", &n, &m, &n, &alpha, "D NC", s_inv, &n, idiag, &ndiag, u, &n, &beta, temp, &n);

  /* multiply [(vt)t].diagonal_matrix(1/s).[ut] */
  cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans, n, m, n, alpha, vt, n, temp, n, beta, A_inv, n);

  free(superb);
  // free(s);  // TODO free without causing segfault
  // free(u);  // TODO free without causing segfault
  // free(vt); // TODO free without causing segfault
  free(s_inv);
  free(temp);

}
