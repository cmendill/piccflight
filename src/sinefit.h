typedef struct sinefit_data {
  size_t n;
  double *t;
  double *y;
} sinefit_data_t;

typedef struct sinefit_fit {
  double a;
  double p;
} sinefit_fit_t;

void sinefit(sinefit_data_t *data,sinefit_fit_t *fit );
