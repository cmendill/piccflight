#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <time.h>
#include <acedev5.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "numeric.h"
#include "alp_functions.h"

/**************************************************************/
/* ALP_INIT                                                   */
/*  - Initialize ALPAO command structure                      */
/**************************************************************/
void alp_init(alp_t *alp){
  //Set everything to zero
  memset(alp,0,sizeof(alp_t));
}

/**************************************************************/
/* ALP_INIT_CALMODE                                           */
/*  - Initialize ALP calmode structure                        */
/**************************************************************/
void alp_init_calmode(int calmode, calmode_t *alp){
  //ALP_CALMODE_NONE
  if(calmode == ALP_CALMODE_NONE){
    sprintf(alp->name,"ALP_CALMODE_NONE");
    sprintf(alp->cmd,"none");
  }
  //ALP_CALMODE_POKE
  if(calmode == ALP_CALMODE_POKE){
    sprintf(alp->name,"ALP_CALMODE_POKE");
    sprintf(alp->cmd,"poke");
  }
  //ALP_CALMODE_ZPOKE
  if(calmode == ALP_CALMODE_ZPOKE){
    sprintf(alp->name,"ALP_CALMODE_ZPOKE");
    sprintf(alp->cmd,"zpoke");
  }
  //ALP_CALMODE_FLIGHT
  if(calmode == ALP_CALMODE_FLIGHT){
    sprintf(alp->name,"ALP_CALMODE_FLIGHT");
    sprintf(alp->cmd,"flight");
  }

}

/**************************************************************/
/* ALP_ZERN2ALP                                               */
/*  - Convert zernike commands to ALPAO DM commands           */
/**************************************************************/
int alp_zern2alp(alp_t *alp){
  FILE *matrix=NULL;
  char matrix_file[MAX_FILENAME];
  static int init=0;
  static double zern2alp_matrix[LOWFS_N_ZERNIKE*ALP_NACT]={0};
  uint64 fsize,rsize;
  int c,i;
  
  if(!init){
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
    
    //--set init flag
    init=1;
  }

  //Do Matrix Multiply
  num_dgemv(zern2alp_matrix,alp->zernike_cmd,alp->act_cmd, ALP_NACT, LOWFS_N_ZERNIKE);
  
  return 0;
}

/**************************************************************/
/* ALP_CALIBRATE                                              */
/* - Run calibration routines for ALPAO DM                    */
/**************************************************************/
int alp_calibrate(int calmode, alp_t *alp, int reset){
  int i,j,index;
  static struct timespec start,this,last,delta;
  static unsigned long int countA=0,countB=0;
  static double zernike2alp[LOWFS_N_ZERNIKE*ALP_NACT]={0};
  static double zernike_errors[LOWFS_N_ZERNIKE][ZERNIKE_ERRORS_NUMBER]={{0}};
  const double zernike_timestep = ZERNIKE_ERRORS_PERIOD;
  const int flight_zuse[LOWFS_N_ZERNIKE]={0,0,0,1,1,1, 1,1,1,1,1,1, 0,0,0,0,0,0, 0,0,0,0,0,0};
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
    init=0;
    return calmode;
  }

  /* Initialize */
  if(!init){
    countA=0;
    countB=0;
    clock_gettime(CLOCK_REALTIME, &start);

    /* Open ZERNIKE2ALP matrix file */
    //--setup filename
    sprintf(filename,ZERNIKE2ALP_FILE);
    //--open file
    if((fileptr = fopen(filename,"r")) == NULL){
      printf("Zernike2alp file\r\n");
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
      printf("Zernike Errors file\r\n");
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

  /* ALP_CALMODE_NONE: Do nothing. Just reset counters.            */
  if(calmode==ALP_CALMODE_NONE){
    countA=0;
    countB=0;
    return calmode;
  }

  /* ALP_CALMODE_POKE: Scan through acuators poking one at a time. */
  /*                   Set flat in between each poke.              */
  if(calmode == ALP_CALMODE_POKE){
    if(countA >= 0 && countA < (2*ALP_NACT*ALP_NCALIM)){
      //set all ALP actuators to bias
      for(i=0;i<ALP_NACT;i++)
	alp->act_cmd[i]=ALP_BIAS;

      //poke one actuator
      if((countA/ALP_NCALIM) % 2 == 1){
	alp->act_cmd[(countB/ALP_NCALIM) % ALP_NACT] = ALP_BIAS + ALP_POKE;
	countB++;
      }
      countA++;
    }else{
      //Turn off calibration
      printf("ALP: Stopping ALP calmode %d\n",ALP_CALMODE_POKE);
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }
    return calmode;

  }

  /* ALP_CALMODE_ZPOKE: Poke Zernikes one at a time    */
  /*                    Set flat in between each poke. */
  if(calmode == ALP_CALMODE_ZPOKE){
    if(countA >= 0 && countA < (2*LOWFS_N_ZERNIKE*ALP_NCALIM)){
      //set all Zernikes to zero
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
        alp->zernike_cmd[i] = 0.0;
      
      //set all ALP actuators to bias
      for(i=0;i<ALP_NACT;i++)
	alp->act_cmd[i]=ALP_BIAS;
      
      //poke one zernike
      if((countA/ALP_NCALIM) % 2 == 1){
	alp->zernike_cmd[(countB/ALP_NCALIM) % LOWFS_N_ZERNIKE] = 0.1;
	num_dgemv(zernike2alp, alp->zernike_cmd, act, ALP_NACT, LOWFS_N_ZERNIKE);
        for(i=0; i<ALP_NACT; i++)
          alp->act_cmd[i] += act[i];
       	countB++;
      }
      countA++;
    }else{
      printf("ALP: Stopping calmode %d\n",ALP_CALMODE_ZPOKE);
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }
    return calmode;
  }

  /* ALP_CALMODE_FLIGHT: Flight Simulator */
  if(calmode == ALP_CALMODE_FLIGHT){
    if(countA == 0)
      dt0 = dt;
    if(countA == 1)
      period = dt-dt0;
    //Set index
    index = (int)((dt-dt0)/zernike_timestep);
    if(index < ZERNIKE_ERRORS_NUMBER){
      //Get zernikes for this timestep
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(flight_zuse[i])
	  zernikes[i] = zernike_errors[i][index % ZERNIKE_ERRORS_NUMBER];
      //Convert to ALP DAC codes
      num_dgemv(zernike2alp, zernikes, act, ALP_NACT, LOWFS_N_ZERNIKE);
      //Add offsets to ALP position
      for(i=0;i<ALP_NACT;i++)
	alp->act_cmd[i] += act[i];
      countA++;
    }else{
      //Turn off calibration
      printf("ALP: Stopping ALP calmode %d\n",ALP_CALMODE_FLIGHT);
      calmode = ALP_CALMODE_NONE;
      init = 0;
      return calmode;
    }
  }
  
  //Return calmode
  return calmode;
}


/**************************************************************/
/* ALP_CHECK                                                  */
/* - Check and fix ALP command values                         */
/**************************************************************/
void alp_check(alp_t *alp){
  int i;
  double max_diff = 0.1;
  for(i=1;i<ALP_NACT;i++){
    if((alp->act_cmd[i]-alp->act_cmd[i-1] > max_diff) || (alp->act_cmd[i]-alp->act_cmd[i-1] < max_diff)){
      alp->act_cmd[i] -= (alp->act_cmd[i]-alp->act_cmd[i-1]-max_diff);
    }
    for(i=0;i<ALP_NACT;i++){
      alp->act_cmd[i] = alp->act_cmd[i] < ALP_DMIN ? ALP_DMIN : alp->act_cmd[i];
      alp->act_cmd[i] = alp->act_cmd[i] > ALP_DMAX ? ALP_DMAX : alp->act_cmd[i];
    }
  }
}


/**************************************************************/
/* ALP_OPEN                                                   */
/* - Open ALPAO device                                        */
/**************************************************************/
int alp_open(char *name){
  int devId;
  int ret;
  //Open device
  ret = acedev5Init(1, &devId, name);
  if ( ret != acecsSUCCESS ){
    return -1;
    printf("ALP: init failed.\n");
  }
  return devId;
}

/**************************************************************/
/* ALP_WRITE                                                  */
/* - Write commands to ALPAO DM                               */
/**************************************************************/
int alp_write(int devId, alp_t* alp){
  int ret;
  if(devId >= 0){
    //Check ALP
    alp_check(alp);
    //Send command
    ret = acedev5Send(1, &devId, alp->act_cmd);
    if( ret != acecsSUCCESS )
      return 1;
    else
      return 0;
  }
  return 1;
}

/**************************************************************/
/* ALP_CLOSE                                                  */
/* - Close ALPAO device                                       */
/**************************************************************/
int alp_close(int devId){
  acedev5Release(1, &devId);
  return 0;
}


/**************************************************************/
/* ALP_ZERO                                                   */
/* - Send all zeros to ALPAO DM                               */
/**************************************************************/
int alp_zero(int devId){
  double data[ALP_NACT]={0};
  int ret;
  ret = acedev5Send(1, &devId, data);
  if(ret != acecsSUCCESS){
    printf("ALP: alp_zero failed!\n");
    return 1;
  }
  return 0;
}
