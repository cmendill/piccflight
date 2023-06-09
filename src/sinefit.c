#include <stdlib.h>
#include <stdio.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlinear.h>

#include "sinefit.h"

#define DEBUG 0


int sine_f (const gsl_vector * x, void *data, gsl_vector * f){
  size_t n = ((sinefit_data_t *)data)->n;
  double *t = ((sinefit_data_t *)data)->t;
  double *y = ((sinefit_data_t *)data)->y;
  
  double a  = gsl_vector_get (x, 0);
  double p  = gsl_vector_get (x, 1);
  double b  = gsl_vector_get (x, 2);
  
  size_t i;
  
  for (i = 0; i < n; i++){
    /* Model Yi = a * sin(t + p) + b*/
    double Yi = a * sin(t[i] + p) + b;
    gsl_vector_set (f, i, Yi - y[i]);
  }
  
  return GSL_SUCCESS;
}

int sine_df (const gsl_vector * x, void *data, gsl_matrix * J){
  size_t n = ((sinefit_data_t *)data)->n;
  double *t = ((sinefit_data_t *)data)->t;
  
  double a  = gsl_vector_get (x, 0);
  double p  = gsl_vector_get (x, 1);
  
  size_t i;

  for (i = 0; i < n; i++){
    /* J[i][0] = dy/da = sin(t+p) */
    gsl_matrix_set(J, i, 0, sin(t[i]+p));
    /* J[i][1] = dy/dp = a * cos(t[i]+p); */
    gsl_matrix_set (J, i, 1, a * cos(t[i]+p));
    /* J[i][2] = dy/db = 1 */
    gsl_matrix_set (J, i, 2, 1);
  }
  
  return GSL_SUCCESS;
}

void callback(const size_t iter, void *params,const gsl_multifit_nlinear_workspace *w){
  gsl_vector *f = gsl_multifit_nlinear_residual(w);
  gsl_vector *x = gsl_multifit_nlinear_position(w);
  double rcond;
  
  /* compute reciprocal condition number of J(x) */
  gsl_multifit_nlinear_rcond(&rcond, w);

  if(DEBUG){
    fprintf(stderr, "iter %2zu: a = %.4f, p = %.4f, b = %.4f, cond(J) = %8.4f, |f(x)| = %.4f\n",
	    iter,
	    gsl_vector_get(x, 0),
	    gsl_vector_get(x, 1),
	    gsl_vector_get(x, 2),
	    1.0 / rcond,
	    gsl_blas_dnrm2(f));
  }
}

int sinefit(sinefit_data_t *data, sinefit_fit_t *fit, double *guess){
  //guess should be inital values for [amp,phi,offset]
  const gsl_multifit_nlinear_type *T = gsl_multifit_nlinear_trust;
  gsl_multifit_nlinear_workspace *w;
  gsl_multifit_nlinear_fdf fdf;
  gsl_multifit_nlinear_parameters fdf_params = gsl_multifit_nlinear_default_parameters();
  gsl_vector *f;
  gsl_matrix *J;
  gsl_rng * r;
  double chisq, chisq0;
  int status, info;
  size_t i;
  const double xtol = 1e-8;
  const double gtol = 1e-8;
  const double ftol = 0.0;
  const size_t n = data->n; //number of data points
  const size_t p = 3; //number of parameters
  double *weights;

  /* setup */
  gsl_rng_env_setup();
  r = gsl_rng_alloc(gsl_rng_default);

  /* set fitting weights */
  if((weights=(double *)malloc(n*sizeof(double))) == NULL){
    printf("ERROR: sinefit malloc failed\n");
    return 1;
  }
  for(i=0;i<n;i++)
    weights[i]=1;

  /* array & matrix setup */
  gsl_matrix *covar = gsl_matrix_alloc (p, p);
  gsl_vector_view x = gsl_vector_view_array (guess, p);
  gsl_vector_view wts = gsl_vector_view_array(weights, n);

  /* define the function to be minimized */
  fdf.f = sine_f;
  fdf.df = sine_df;   /* set to NULL for finite-difference Jacobian */
  fdf.fvv = NULL;     /* not using geodesic acceleration */
  fdf.n = n;
  fdf.p = p;
  fdf.params = data;
  
  /* allocate workspace with default parameters */
  w = gsl_multifit_nlinear_alloc (T, &fdf_params, n, p);

  /* initialize solver with starting point and weights */
  gsl_multifit_nlinear_winit (&x.vector, &wts.vector, &fdf, w);

  /* compute initial cost function */
  f = gsl_multifit_nlinear_residual(w);
  gsl_blas_ddot(f, f, &chisq0);

  /* solve the system with a maximum of 100 iterations */
  status = gsl_multifit_nlinear_driver(100, xtol, gtol, ftol, callback, NULL, &info, w);

  /* compute covariance of best fit parameters */
  J = gsl_multifit_nlinear_jac(w);
  gsl_multifit_nlinear_covar (J, 0.0, covar);

  /* compute final cost */
  gsl_blas_ddot(f, f, &chisq);

  /* set outputs */
  fit->a = gsl_vector_get(w->x, 0);
  fit->p = gsl_vector_get(w->x, 1);
  fit->b = gsl_vector_get(w->x, 2);
  
  if(DEBUG){
    fprintf(stderr, "summary from method '%s/%s'\n",
	    gsl_multifit_nlinear_name(w),
	    gsl_multifit_nlinear_trs_name(w));
    fprintf(stderr, "number of iterations: %zu\n",
	    gsl_multifit_nlinear_niter(w));
    fprintf(stderr, "function evaluations: %zu\n", fdf.nevalf);
    fprintf(stderr, "Jacobian evaluations: %zu\n", fdf.nevaldf);
    fprintf(stderr, "reason for stopping: %s\n",
	    (info == 1) ? "small step size" : "small gradient");
    fprintf(stderr, "initial |f(x)| = %f\n", sqrt(chisq0));
    fprintf(stderr, "final   |f(x)| = %f\n", sqrt(chisq));
    
    double dof = n - p;
    double c = GSL_MAX_DBL(1, sqrt(chisq / dof));

    fprintf(stderr, "chisq/dof = %g\n", chisq / dof);
    fprintf(stderr, "a = %.5f +/- %.5f\n", gsl_vector_get(w->x, 0), c*sqrt(gsl_matrix_get(covar,0,0)));
    fprintf(stderr, "p = %.5f +/- %.5f\n", gsl_vector_get(w->x, 1), c*sqrt(gsl_matrix_get(covar,1,1)));
    fprintf(stderr, "b = %.5f +/- %.5f\n", gsl_vector_get(w->x, 2), c*sqrt(gsl_matrix_get(covar,2,2)));
    fprintf (stderr, "status = %s\n", gsl_strerror (status));
  }

  gsl_multifit_nlinear_free (w);
  gsl_matrix_free (covar);
  gsl_rng_free (r);
  if(weights) free(weights);

  return status;
}
