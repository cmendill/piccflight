// #include <QuickUSB.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <time.h>
#include "acedev5.h"

/* piccflight headers */
#include "../common/controller.h"
#include "../common/common_functions.h"
#include "../common/numeric.h"

/**************************************************************/
/*                      ALP_INIT                             */
/**************************************************************/
void alp_init(alp_t *alp){
  int i;
  for(i=0; i<ALP_NACT; i++)
    alp->act[i] = ALP_BIAS;
}

/**************************************************************/
/*                      ALP_CALIBRATE                         */
/**************************************************************/
int alp_calibrate(int calmode, alp_t *alp, int reset){
  int i,j,index;
  static struct timespec start,this,last,delta;
  static unsigned long int countA=0,countB=0,countF=0;
  static double zernike2alp[LOWFS_N_ZERNIKE*ALP_NACT]={0};
  static double zernike_errors[LOWFS_N_ZERNIKE][ZERNIKE_ERRORS_NUMBER]={{0}};
  const double zernike_timestep = ZERNIKE_ERRORS_PERIOD;
  const int zuse[LOWFS_N_ZERNIKE]={0,0,0,1,1,1, 1,1,1,1,1,1, 0,0,0,0,0,0, 0,0,0,0,0,0};
  static int init=0;
  time_t t;
  FILE *fileptr=NULL;
  char filename[MAX_FILENAME];
  double dt=0,dt0=0,period=0;
  double zernikes[LOWFS_N_ZERNIKE]={0};
  double act[ALP_NACT];

  /* Reset */
  if(reset){
    countA=0;
    countB=0;
    countF=0;
    init=0;
    return calmode;
  }

  /* Initialize */
  if(!init){
    countA=0;
    countB=0;
    countF=0;
    clock_gettime(CLOCK_REALTIME, &start);

    /* Open ZERNIKE2ALP matrix file */
    //--setup filename
    sprintf(filename,ZERNIKE2ALP_FILE);
    //--open file
    if((fileptr = fopen(filename,"r")) == NULL){
      perror("fopen");
      goto endofinit;
    }
    //--check file size
    fseek(fileptr, 0L, SEEK_END);
    if(ftell(fileptr) != sizeof(zernike2alp)){
      printf("SHK: incorrect ALP matrix file size %lu != %lu\n",ftell(fileptr),sizeof(zernike2alp));
      goto endofinit;
    }
    rewind(fileptr);
    //--read matrix
    if(fread(zernike2alp,sizeof(zernike2alp),1,fileptr) != 1){
      perror("fread");
      goto endofinit;
    }

    /* Open zernike errors file */
    //--setup filename
    sprintf(filename,ZERNIKE_ERRORS_FILE);
    //--open file
    if((fileptr = fopen(filename,"r")) == NULL){
      perror("fopen");
      goto endofinit;
    }
    //--check file size
    fseek(fileptr, 0L, SEEK_END);
    if(ftell(fileptr) != sizeof(zernike_errors)){
      printf("SHK: incorrect zernike_errors file size %lu != %lu\n",ftell(fileptr),sizeof(zernike_errors));
      goto endofinit;
    }
    rewind(fileptr);
    //--read matrix
    if(fread(zernike_errors,sizeof(zernike_errors),1,fileptr) != 1){
      perror("fread");
      goto endofinit;
    }

  endofinit:
    //--close file
    if(fileptr != NULL) fclose(fileptr);
    init=1;
  }

  /* Calculate times */
  clock_gettime(CLOCK_REALTIME, &this);
  if(timespec_subtract(&delta,&this,&start))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);

  /* CALMODE 1: Scan through acuators poking one at a time */
  if(calmode == 1){
    //Set all ALP actuators to bias
    for(i=0;i<ALP_NACT;i++)
      alp->act[i]=ALP_BIAS;
    //poke one actuator
    alp->act[(countA/ALP_NCALIM) % ALP_NACT] = ALP_BIAS + ALP_POKE;
    // usleep(500000);
    countA++;
    return calmode;
  }

  /* CALMODE 2: Scan through acuators poking one at a time.
   *            Set flat inbetween each poke                */

  if(calmode == 2){
    if(countA >= 0 && countA < (2*ALP_NACT*ALP_NCALIM)){
      //set all ALP actuators to bias
      for(i=0;i<ALP_NACT;i++)
	alp->act[i]=ALP_BIAS;

      //poke one actuator
      if((countA/ALP_NCALIM) % 2 == 1){
	alp->act[(countB/ALP_NCALIM) % ALP_NACT] = ALP_BIAS + ALP_POKE;
	countB++;
      }
      countA++;
    }
    else{
      //Turn off calibration
      printf("ALP: Stopping ALP calmode 2\n");
      calmode = 0;
      init = 0;
    }
    return calmode;

  }

  /* CALMODE 3: Set Zernike on ALP */
  if(calmode == 3){
    double zerns[24] = {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      zernikes[i] = zerns[i];
    }
    num_dgemv(zernike2alp, zernikes, act, ALP_NACT, LOWFS_N_ZERNIKE);
    for(i=0;i<ALP_NACT;i++){
  	   alp->act[i] = (double)0.5*act[i];
    }
    // countA++;
    return calmode;
  }

  /* CALMODE 4: Flight Simulator */
  if(calmode == 4){
    if(countF == 0)
      dt0 = dt;
    if(countF == 1)
      period = dt-dt0;
    //Set index
    index = (int)((dt-dt0)/zernike_timestep);
    //Get zernikes for this timestep

    for(i=0;i<LOWFS_N_ZERNIKE;i++){

      if(zuse[i]){
	       zernikes[i] = zernike_errors[i][index % ZERNIKE_ERRORS_NUMBER];
      }
    }
    //Convert to ALP DAC codes
    num_dgemv(zernike2alp, zernikes, act, ALP_NACT, LOWFS_N_ZERNIKE);
    //Add offsets to ALP position
    for(i=0;i<ALP_NACT;i++)
      alp->act[i] += (double)act[i]*1.0;
    //Check if we are done
    if((index + (int)(1.1*period/zernike_timestep)) > ZERNIKE_ERRORS_NUMBER){
      //Turn off calibration
      printf("ALP: Stopping ALP calmode 4\n");
      calmode = 0;
      init = 0;
      return calmode;
    }
    countF++;
  }



    //Return calmode
  return calmode;
}



/**************************************************************/
/*                      ALP_CHECK                             */
/**************************************************************/
void alp_check(alp_t *alp){
  int i;
  for(i=0;i<ALP_NACT;i++){
    alp->act[i] = alp->act[i] > ALP_DMAX ? ALP_DMAX : alp->act[i];
    alp->act[i] = alp->act[i] < ALP_DMIN ? ALP_DMIN : alp->act[i];
  }
}

/**************************************************************/
/*                      ALP_OPEN                              */
/**************************************************************/
const int* alp_open(int* dmId){
  int             nbAct = 0, act, idx, i;
  acecsCOMPL_STAT ret;
  const int       nDm = 1;
  // int             dmIds[1];
  double          data[ALP_NACT];
  int             i_tmp[ALP_NACT];
  char            serial[128] = "BAX197";

  ret = acedev5Init(1, dmId, serial);
  // printf("ALP: finished acedev5 init\n");

  if ( ret != acecsSUCCESS ){
      // return -1;
      printf("ALP: init failed.\n");
  }

  /* Get the number of actuators */
  ret = acedev5GetNbActuator( nDm, dmId, i_tmp );
  for(idx = 0; idx < nDm; idx++){
    nbAct += i_tmp[idx];
  }

  /* Check errors */
  if ( ret != acecsSUCCESS ){
      // return -1;
      printf("ALP: Errors.\n");
  }

  printf("ALP: Total number of actuators: %i\n", nbAct );

  /* Initialize data */
  // data = (double*)calloc(nbAct, sizeof(double));
  for ( idx = 0 ; idx < nbAct ; idx++ ){
      data[idx] = 0;
  }
  // acecsErrDisplay();

  return dmId;
}

/**************************************************************/
/*                      ALP_WRITE                              */
/**************************************************************/
int alp_write(const int* alp_dev, alp_t* alp){
  int ret;
  if(*alp_dev >= 0){
    #if ALP_ENABLE
        //Check ALP
        alp_check(alp);
    #endif
    }
    // int act = 1;
    // double* cmd = alp->act;
    // printf("Sending %f to act %i\n", *cmd, act);
    ret = acedev5Send(1, alp_dev, alp->act);
    if ( ret != acecsSUCCESS ){
        return -1;

    }
  return ret;
}

/**************************************************************/
/*                      ALP_CLOSEDEV                          */
/**************************************************************/
int alp_closeDev(const int* alp_dev){

  acedev5Release(1, alp_dev);
  printf("ALP: Device closed.\r\n");
  return 0;
}


/**************************************************************/
/*                      ALP_ZERO                              */
/**************************************************************/
int alp_zero(const int* alp_dev){
  double data[ALP_NACT];
  int ret, i;
  for(i=0; i<ALP_NACT; i++){
    data[i] = 0;
  }
  ret = acedev5Send(1, alp_dev, data);
  if(ret != acecsSUCCESS){
    printf("ALP: alp_write failed!\n");
    return 1;
  }
  return 0;
}
