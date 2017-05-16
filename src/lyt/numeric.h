#ifndef _NUMERIC
  #define _NUMERIC

  #ifdef mkl
  #include "mkl.h"
  #endif

  #ifdef openblas
  #include "cblas.h"
  #endif

  void NUMERIC_init(void);
  void NUMERIC_multiply(void*, double(*)[15]);

#endif /* _NUMERIC */