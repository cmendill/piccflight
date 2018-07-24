#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#include "dm7820_library.h"
#include "rtdalpao_library.h"
#include "numeric.h"

#define LOWFS_N_ZERNIKE  23
#define ALP_NACT         97
#define MAX_FILENAME     128
#define ZERNIKE2ALP_FILE "data/shk/zern2alp.dat"
#define ALP_FLAT_FILE    "data/alp/alpao_flat_zern.dat";
int cleanup = 0;

/**************************************************************/
/* ALP_ZERN2ALP                                               */
/*  - Convert zernike commands to ALPAO DM commands           */
/**************************************************************/
int alp_zern2alp(double *zernikes,double *actuators){
  FILE *matrix=NULL;
  char matrix_file[MAX_FILENAME];
  static double zern2alp_matrix[LOWFS_N_ZERNIKE*ALP_NACT]={0};
  uint64_t fsize,rsize;
  
  /* Open matrix file */
  //--setup filename
  sprintf(matrix_file,ZERNIKE2ALP_FILE);
  //--open matrix file
  if((matrix = fopen(matrix_file,"r")) == NULL){
    printf("zern2alp file\r");
    perror("fopen");
    return 1;
  }
  
  //--check file size
  fseek(matrix, 0L, SEEK_END);
  fsize = ftell(matrix);
  rewind(matrix);
  rsize = LOWFS_N_ZERNIKE*ALP_NACT*sizeof(double);
  if(fsize != rsize){
    printf("ALP: incorrect zern2alp matrix file size %lu != %lu\n",fsize,rsize);
    return 1;
  }
  
  //--read matrix
  if(fread(zern2alp_matrix,LOWFS_N_ZERNIKE*ALP_NACT*sizeof(double),1,matrix) != 1){
    printf("zern2alp file\r");
    perror("fread");
    return 1;
  }
  //--close file
  fclose(matrix);
  printf("Read: %s\n",matrix_file);
  
  //Do Matrix Multiply
  num_dgemv(zern2alp_matrix,zernikes,actuators, ALP_NACT, LOWFS_N_ZERNIKE);
  
  return 0;
}

//Read flat
int read_flat(char *filename,double *flat){
  FILE *file=NULL;
  
  /* Open file */
  if((file = fopen(filename,"r")) == NULL){
    perror("fopen");
    return 1;
  }
  //--read data
  if(fread(flat,ALP_NACT*sizeof(double),1,file) != 1){
    perror("fread");
    return 1;
  }
  //--close file
  fclose(file);
  printf("Read: %s\n",filename);

  return 0;
}

/* CTRL-C Function */
void ctrlC(int sig)
{
  cleanup=1;
}

int main( int argc, char *argv[] ) {

  DM7820_Board_Descriptor* p_rtd_board;
  DM7820_Error dm7820_status;
  int      dither_enable = 0;
  double   zernike  = 0.01; //microns
  double   data[ALPAO_DEV_N_CHANNEL]={0};
  double   zernike_array[23]={0};
  unsigned long     period   = 100000;
  uint16_t          n_dither = 1;
  double   flat[ALPAO_DEV_N_CHANNEL]={0};
  char     flatfile[128]=ALP_FLAT_FILE;
  int i;
  int opt;


  //Read options
  while ((opt = getopt (argc, argv, "z:dh")) != -1) {
    switch (opt) {
    case 'z': // set zernike value
      zernike = atof(optarg);
      break;
    case 'd': // set number of dither steps
      dither_enable = 1;
      break;
    case 'h': // help
    default:
      printf("Usage:\n");
      printf("\tTo set a Z2 tilt on the ALPAO DM\n");
      printf("Options:\n");
      printf("\t\t-z zernike_coeff_microns\n");
      printf("\t\t-d dither enable\n");
      printf("\t\t-h print this help\n");
      return 0;
      break;
    }
  }

  //Set soft interrupt handler 
  sigset(SIGINT, ctrlC);

  //Read flat file
  read_flat(flatfile,flat);

  //Setup output command
  zernike_array[0]=zernike;
  alp_zern2alp(zernike_array,data);

  //Print tilt command
  for(i=0;i<ALP_NACT;i++)
    printf("ACT %d:  %f\n",i,data[i]);
  
  //Add flat to command
  for(i=0;i<ALP_NACT;i++)
    data[i]+=flat[i];
  
  //Setup dither
  if(dither_enable){
    n_dither = 100;
    period = 1000000.0 * (double)n_dither * ((double)ALPAO_DATA_LENGTH)/((double)RTD_CLK_FREQUENCY);
  }

  printf("Amplitude = %f  Dither = %d  Period = %lu\n",zernike,n_dither,period);

  
  //Open driver
  if((dm7820_status = rtd_open(0, &p_rtd_board)))
    perror("rtd_open");

  //Reset board
  if((dm7820_status = rtd_reset(p_rtd_board)))
    perror("rtd_reset");

  //Clear all
  if((dm7820_status = rtd_clear_all(p_rtd_board)))
    perror("rtd_clear_all");

  //Init ALPAO interface
  if((dm7820_status = rtdalpao_init(p_rtd_board,n_dither)))
    perror("rtdalpao_init");

  //Start timer
  if((dm7820_status = rtdalpao_start_timer(p_rtd_board)))
    perror("rtdalpao_start_timer");

  while(!cleanup){
    //Send frame
    dm7820_status = rtdalpao_send_analog_data(p_rtd_board,data);
    
    //Sleep
    usleep(period);
  }

  /* Sleep before stop timer */
  usleep(period);

  /* Stop timer */
  if((dm7820_status = rtdalpao_stop_timer(p_rtd_board)))
    perror("rtdalpao_stop_timer");

  /* Sleep before cleanup */
  usleep(1500);
  
  //Cleanup ALPAO interface
  if((dm7820_status = rtdalpao_clean(p_rtd_board)))
    perror("rtdalpao_clean");

  //Close driver
  if((dm7820_status = rtd_close(p_rtd_board)))
    perror("rtd_close");

  //Print out
  printf("Done\n");
  return 0;

}
