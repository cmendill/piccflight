#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <time.h>
#include <libgen.h>
#include <sys/stat.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "tgt_functions.h"

/**************************************************************/
/* TGT_INIT_CALMODE                                           */
/*  - Initialize TGT calmode structure                        */
/**************************************************************/
void tgt_init_calmode(int calmode, calmode_t *tgt){
  int i;
  const double shk_zpoke[LOWFS_N_ZERNIKE]={0.2,0.2,0.05,0.05,0.05,0.05,0.05,0.03,0.03,0.03,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02};
  const double lyt_zpoke[LOWFS_N_ZERNIKE]={0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005};
  const double sci_zpoke[LOWFS_N_ZERNIKE]={0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1};
  const double shk_zramp[LOWFS_N_ZERNIKE]={4.0,4.0,1.25,1.5,1.5,0.5,0.5,1.0,1.0,0.3,0.3,0.3,0.7,0.7,0.2,0.2,0.2,0.2,0.6,0.6,0.2,0.2,0.2};

  //DEFAULTS
  tgt->shk_ncalim = TGT_SHK_NCALIM;
  tgt->lyt_ncalim = TGT_LYT_NCALIM;
  tgt->sci_ncalim = TGT_SCI_NCALIM;
  tgt->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    tgt->shk_zpoke[i]  = TGT_SHK_ZPOKE;
    tgt->lyt_zpoke[i]  = TGT_LYT_ZPOKE;
    tgt->sci_zpoke[i]  = TGT_SCI_ZPOKE;
  }
  
  //TGT_CALMODE_NONE
  if(calmode == TGT_CALMODE_NONE){
    sprintf(tgt->name,"TGT_CALMODE_NONE");
    sprintf(tgt->cmd,"none");
  }
  //TGT_CALMODE_ZPOKE
  if(calmode == TGT_CALMODE_ZPOKE){
    sprintf(tgt->name,"TGT_CALMODE_ZPOKE");
    sprintf(tgt->cmd,"zpoke");
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      tgt->shk_zpoke[i]  = shk_zpoke[i];
      tgt->lyt_zpoke[i]  = lyt_zpoke[i];
    }
  }
  //TGT_CALMODE_ZRAMP
  if(calmode == TGT_CALMODE_ZRAMP){
    sprintf(tgt->name,"TGT_CALMODE_ZRAMP");
    sprintf(tgt->cmd,"zramp");
    tgt->shk_ncalim = 100;
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      tgt->shk_zpoke[i]  = shk_zramp[i];
      tgt->lyt_zpoke[i]  = 0.01*shk_zramp[i];
    }
  }
  //TGT_CALMODE_ZRAND
  if(calmode == TGT_CALMODE_ZRAND){
    sprintf(tgt->name,"TGT_CALMODE_ZRAND");
    sprintf(tgt->cmd,"zrand");
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      tgt->shk_zpoke[i]  = shk_zpoke[i];
      tgt->lyt_zpoke[i]  = lyt_zpoke[i];
    }
  }
}

/**************************************************************/
/* TGT_CALIBRATE                                              */
/* - Run calibration routines for TGT                         */
/**************************************************************/
int tgt_calibrate(sm_t *sm_p, int calmode, double *zernikes, uint32_t *step, int procid, int reset){
  int i,j,z;
  time_t trand;
  static int init=0;
  double zpoke[LOWFS_N_ZERNIKE]={0};
  static calmode_t tgtcalmodes[TGT_NCALMODES];
  static double zrand[LOWFS_N_ZERNIKE]={0};
  struct timespec this,delta;
  int ncalim=0;
  
  /* Initialize */
  if(!init || reset){
    //Reset counters
    memset((void *)sm_p->tgtcal.countA,0,sizeof(sm_p->tgtcal.countA));
    memset((void *)sm_p->tgtcal.countB,0,sizeof(sm_p->tgtcal.countB));
    //Init TGT calmodes
    for(i=0;i<TGT_NCALMODES;i++)
      tgt_init_calmode(i,&tgtcalmodes[i]);
    for(i=0;i<LOWFS_N_ZERNIKE;i++) zrand[i] = (2*(rand() / (double) RAND_MAX) - 1);
    init=1;
    //Return if reset
    if(reset == FUNCTION_RESET_RETURN) return calmode;
  }

  /* Set calibration parameters */
  if(procid == SHKID){
    memcpy(zpoke,tgtcalmodes[calmode].shk_zpoke,sizeof(zpoke));
    ncalim = tgtcalmodes[calmode].shk_ncalim;
  }
  if(procid == LYTID){
    memcpy(zpoke,tgtcalmodes[calmode].lyt_zpoke,sizeof(zpoke));
    ncalim = tgtcalmodes[calmode].lyt_ncalim;
  }
  if(procid == SCIID){
    memcpy(zpoke,tgtcalmodes[calmode].sci_zpoke,sizeof(zpoke));
    ncalim = tgtcalmodes[calmode].sci_ncalim;
  }

  /* Get time */
  clock_gettime(CLOCK_REALTIME, &this);

  /* Init times and TGT commands */
  if((calmode != TGT_CALMODE_NONE) && (sm_p->tgtcal.countA[calmode] == 0)){
    //Save start time
    memcpy((struct timespec *)&sm_p->tgtcal.start[calmode],&this,sizeof(struct timespec));
    //Save tgt starting position
    memcpy((double *)sm_p->tgtcal.tgt_start[calmode].zcmd,zernikes,sizeof(sm_p->tgtcal.tgt_start[calmode].zcmd));
  }
  
  
  /* TGT_CALMODE_NONE: Do nothing. Just reset counters.            */
  if(calmode==TGT_CALMODE_NONE){
    //Reset counters
    memset((void *)sm_p->tgtcal.countA,0,sizeof(sm_p->tgtcal.countA));
    memset((void *)sm_p->tgtcal.countB,0,sizeof(sm_p->tgtcal.countB));

    //Return calmode
    return calmode;
  }

  /* TGT_CALMODE_ZPOKE: Poke Zernikes one at a time                    */
  /*                    Set to starting position in between each poke. */
  if(calmode == TGT_CALMODE_ZPOKE){
    //Check counters
    if(sm_p->tgtcal.countA[calmode] >= 0 && sm_p->tgtcal.countA[calmode] < (2*LOWFS_N_ZERNIKE*ncalim)){
      //Zet all Zernikes to starting position
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	zernikes[i] = sm_p->tgtcal.tgt_start[calmode].zcmd[i];

      //Poke one zernike by adding it on top of the flat
      if((sm_p->tgtcal.countA[calmode]/ncalim) % 2 == 1){
	z = (sm_p->tgtcal.countB[calmode]/ncalim) % LOWFS_N_ZERNIKE;
	zernikes[z] += zpoke[z];
	sm_p->tgtcal.countB[calmode]++;
      }
    }
    else{
      //Calibration done
      //Set tgt back to starting position
      memcpy(zernikes,(double *)sm_p->tgtcal.tgt_start[calmode].zcmd,sizeof(sm_p->tgtcal.tgt_start[calmode].zcmd));
      //Turn off calibration
      printf("TGT: Stopping calmode TGT_CALMODE_ZPOKE\n");
      init = 0;
      calmode = TGT_CALMODE_NONE;
    }
    
    //Set step counter
    *step = (sm_p->tgtcal.countA[calmode]/ncalim);

    //Increment counter
    sm_p->tgtcal.countA[calmode]++;
    
    return calmode;
  }

  /* TGT_CALMODE_ZRAND: Poke all Zernikes to a random value */
  if(calmode == TGT_CALMODE_ZRAND){
    //Check counters
    if(sm_p->tgtcal.countA[calmode] >= 0 && sm_p->tgtcal.countA[calmode] < (2*ncalim)){
      //Zet all Zernikes to starting position
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	zernikes[i] = sm_p->tgtcal.tgt_start[calmode].zcmd[i];
      
      //Poke all zernikes a random amount
      if((sm_p->tgtcal.countA[calmode]/ncalim) % 2 == 1){
	for(i=0;i<LOWFS_N_ZERNIKE;i++)
	  zernikes[i] += zpoke[i] * zrand[i];
      }
    }
    else{
      //Calibration done
      //Set tgt back to starting position
      memcpy(zernikes,(double *)sm_p->tgtcal.tgt_start[calmode].zcmd,sizeof(sm_p->tgtcal.tgt_start[calmode].zcmd));
      //Turn off calibration
      printf("TGT: Stopping calmode TGT_CALMODE_ZRAND\n");
      init = 0;
      calmode = TGT_CALMODE_NONE;
    }
    
    //Set step counter
    *step = (sm_p->tgtcal.countA[calmode]/ncalim);

    //Increment counter
    sm_p->tgtcal.countA[calmode]++;
    
    return calmode;
  }

  /* TGT_CALMODE_ZRAMP: Ramp Zernikes one at a time                    */
  /*                    Set to starting position in between each ramp. */
  if(calmode == TGT_CALMODE_ZRAMP){
    //Check counters
    if(sm_p->tgtcal.countA[calmode] >= 0 && sm_p->tgtcal.countA[calmode] < (2*LOWFS_N_ZERNIKE*ncalim)){
      //Zet all Zernikes to starting position
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	zernikes[i] = sm_p->tgtcal.tgt_start[calmode].zcmd[i];

      //Poke one zernike by adding it on top of the flat
      if((sm_p->tgtcal.countA[calmode]/ncalim) % 2 == 1){
	z = (sm_p->tgtcal.countB[calmode]/ncalim) % LOWFS_N_ZERNIKE;
	zernikes[z] = (sm_p->tgtcal.countB[calmode] % ncalim) * (zpoke[z]/ncalim);
	sm_p->tgtcal.countB[calmode]++;
      }
    }
    else{
      //Calibration done
      //Set tgt back to starting position
      memcpy(zernikes,(double *)sm_p->tgtcal.tgt_start[calmode].zcmd,sizeof(sm_p->tgtcal.tgt_start[calmode].zcmd));
      //Turn off calibration
      printf("TGT: Stopping calmode TGT_CALMODE_ZRAMP\n");
      init = 0;
      calmode = TGT_CALMODE_NONE;
    }
    
    //Set step counter
    *step = (sm_p->tgtcal.countA[calmode]/ncalim);

    //Increment counter
    sm_p->tgtcal.countA[calmode]++;
    
    return calmode;
  }

  //Return calmode
  return calmode;
}
