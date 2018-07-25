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
#include "numeric.h"

#define NUMERIC_DEBUG 0
#define MIN(A,B)   ((A) < (B) ? (A) : (B))

/******************************************************************************
        num_dgemv
******************************************************************************/
/*Function: [result(mm)] = [A(mm x nn)]*[b(nn)]
  A      --> double matrix of [mm x nn] elements stored in a 1d array in column major format
             (i.e. first mm elements are the mm elements of the first column of the matrix)
  b      --> double vector of [nn] elements stored in a 1d array
  result --> double vector of [mm] elements stored in a 1d array
  mm     --> number of rows of A (also number of elements of result)
  nn     --> number of columns of A (also number of elements of b)
*/
void num_dgemv(double *A, double *b, double *result, int mm, int nn) {
  const double alpha = 1.0, beta = 0.0;
  const int incx=1, incy=1;
  cblas_dgemv(CblasColMajor, CblasNoTrans, mm, nn, alpha, A, mm, b, incx, beta, result, incy );
}

/******************************************************************************
        num_dgemm
******************************************************************************/
/*Function: [result(mm x kk)] = [A(mm x nn)]*[B(nn x kk)]
  A      --> double matrix of [mm x kk] elements stored in a 1d array in column major format
             (i.e. first mm elements are the mm elements of the first column of the matrix)
  B      --> double matrix of [kk x nn] elements stored in a 1d array in column major format
             (i.e. first kk elements are the kk elements of the first column of the matrix)
  result --> double matrix of [mm x nn] elements stored in a 1d array in column major format
             (i.e. first mm elements are the mm elements of the first column of the matrix)
  mm     --> number of rows of A (also number of rows of result)
  kk     --> number of columns of A (also number of rows of B)
  nn     --> number of columns of B (also number of columns of result)
*/
void num_dgemm(double *A, double *B, double *result, int mm, int kk, int nn) {
  double alpha = 1.0, beta = 0.0;
  cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, mm, nn, kk, alpha, A, mm, B, kk, beta, result, nn);
}


/******************************************************************************
        num_dgesvdi
******************************************************************************/
/*Function: [A_inv(nn x mm)] = svd_inverse([A(mm x nn)])
  A      --> double matrix of [mm x nn] elements stored in a 1d array in column major format
             (i.e. first mm elements are the mm elements of the first column of the matrix)
  A_inv  --> double matrix of [nn x mm] elements stored in a 1d array in column major format
             (i.e. first nn elements are the nn elements of the first column of the matrix)
  mm      --> number of rows of A (also number of columns of A_inv)
  nn      --> number of columns of A (also number of rows of A_inv)
*/
void num_dgesvdi(double* A, double* A_inv, int mm, int nn) {
  if (mm<=nn) return;  // only supports m>n case TODO m<=n case
  int ii;
  MKL_INT info;
  double alpha = 1.0, beta = 0.0;
  int idiag[1] = {0};
  int ndiag = 1;

  double *s, *u, *vt;
  
  if((s = malloc(nn*sizeof(double))) == NULL)
    perror("malloc(s)");
  memset(s, 0, nn*sizeof(double));
  
  if((u = malloc(mm*nn*sizeof(double))) == NULL)
    perror("malloc(u)");
  memset(u, 0, mm*nn*sizeof(double));

  if((vt = malloc(nn*nn*sizeof(double))) == NULL)
    perror("malloc(vt)");
  memset(vt, 0, nn*nn*sizeof(double));

  double *s_inv, *temp;
  if((s_inv = malloc(nn*sizeof(double))) == NULL)
    perror("malloc(s_inv)");
  memset(s_inv, 0, nn*sizeof(double));

  if((temp = malloc(nn*mm*sizeof(double))) == NULL)
    perror("malloc(temp)");
  memset(temp, 0, nn*mm*sizeof(double));

  // for LAPACKE_dgesvd
  // double* superb; 
  // if((superb = malloc((MIN(m,n)-1)*sizeof(double))) == NULL) 
  //   perror("malloc()"); 
  
  // singular value decompose
  // A = [u].diagonal_matrix(s).[vt]
  // info = LAPACKE_dgesvd(LAPACK_COL_MAJOR, 'A', 'A', mm, nn, A, mm, s, u, mm, vt, nn, superb);
  info = LAPACKE_dgesdd(LAPACK_COL_MAJOR, 'S', mm, nn, A, mm, s, u, mm, vt, nn);

  if(NUMERIC_DEBUG) printf("NUM: Singular values : [ ");
  // build the s_inv array s_inv = 1/s
  for(ii=0; ii<nn; ii++){
    // s_inv[ii] = s[ii]/(pow(s[ii],2) + alpha^2); // tikhonov regularization
    // s_inv[ii] = ((1.0/s[ii])>cutoff)?0.0:(1.0/s[ii]); // cutoff regularization
    s_inv[ii] = (s[ii]==0)?0:1.0/s[ii]; // no Regularization
    if(NUMERIC_DEBUG) printf("%.4f, ", s_inv[ii]);
  }
  if(NUMERIC_DEBUG) printf("\b\b ]\n");
  /* transpose u, vt can be transposed at the multiplication cblas_dgemm*/
  mkl_dimatcopy('C', 'T', mm, nn, alpha, u, mm, nn);

  /* multiply diagonal_matrix(1/s).[ut] */
  mkl_ddiamm("n", &nn, &mm, &nn, &alpha, "D NC", s_inv, &nn, idiag, &ndiag, u, &nn, &beta, temp, &nn);

  /* multiply [(vt)t].diagonal_matrix(1/s).[ut] */
  cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans, nn, mm, nn, alpha, vt, nn, temp, nn, beta, A_inv, nn);

  free(s);
  free(u);
  free(vt);
  free(s_inv);
  free(temp);

}
