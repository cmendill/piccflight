#include <string.h> /* string library for memcpy */
#include "numeric.h"

#ifdef mkl
#include "mkl.h"
#endif

#ifdef openblas
#include "cblas.h"
#endif

#define h 16 /* height of the image */
#define w 16 /* width of the image */
#define M h*w /* number of px in the image */
#define N 15 /* number of zernike coefficients*/

#define LDA M
#define INCX 1
#define INCY 1

static int m, n, lda, incx, incy;

static int px;
static double image[M];
static double inv_zernike_matrix[N*M];

void NUMERIC_init(void) {
  m = M, n = N, lda = LDA, incx = INCX, incy = INCY;
}

void NUMERIC_multiply(void* pvAddress, double (*ppCoefficients)[N]) {
  short* psource  = (short*)pvAddress;
  for(px = 0; px < M; px++) {
    image[px] = (double) *(psource++);
  }
  double alpha = 1.0, beta = 0.0;
  cblas_dgemv(CblasColMajor, CblasNoTrans, m, n, alpha, inv_zernike_matrix, lda, image, incx, beta, *ppCoefficients, incy );
}
