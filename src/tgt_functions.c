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
  const double shk_zramp[LOWFS_N_ZERNIKE]={4.0,4.0,1.25,1.5,1.5,0.5,0.5,1.0,1.0,0.3,0.3,0.3,0.7,0.7,0.2,0.2,0.2,0.2,0.6,0.6,0.2,0.2,0.2};

  //DEFAULTS
  tgt->shk_ncalim = TGT_SHK_NCALIM;
  tgt->lyt_ncalim = TGT_LYT_NCALIM;
  tgt->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    tgt->shk_zpoke[i]  = TGT_SHK_ZPOKE;
    tgt->lyt_zpoke[i]  = TGT_LYT_ZPOKE;
  }
  
  //TGT_CALMODE_NONE
  if(calmode == TGT_CALMODE_NONE){
    sprintf(tgt->name,"TGT_CALMODE_NONE");
    sprintf(tgt->cmd,"none");
  }
  //TGT_CALMODE_ZERO
  if(calmode == TGT_CALMODE_ZERO){
    sprintf(tgt->name,"TGT_CALMODE_ZERO");
    sprintf(tgt->cmd,"zero");
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
    tgt->shk_ncalim = 40;
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
int tgt_calibrate(int calmode, double *zernikes, uint32_t *step, int procid, int reset){
  int i,j,z;
  time_t trand;
  static int init=0;
  static long countA=0,countB=0,ncalim=0;
  double zpoke[LOWFS_N_ZERNIKE]={0};
  static calmode_t tgtcalmodes[TGT_NCALMODES];
  static double zrand[LOWFS_N_ZERNIKE]={0};
  
  /* Initialize */
  if(!init || reset){
    countA = 0;
    countB = 0;
    //Init TGT calmodes
    for(i=0;i<TGT_NCALMODES;i++)
      tgt_init_calmode(i,&tgtcalmodes[i]);
    for(i=0;i<LOWFS_N_ZERNIKE;i++) zrand[i] = (2*(rand() / (double) RAND_MAX) - 1);
    init=1;
    //Return if reset
    if(reset) return calmode;
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

  
  /* TGT_CALMODE_NONE: Do nothing. Just reset counters.            */
  if(calmode==TGT_CALMODE_NONE){
    //Reset counters
    countA = 0;
    countB = 0;
    return calmode;
  }

  /* TGT_CALMODE_ZERO: Set all zernikes to 0 */
  if(calmode==TGT_CALMODE_ZERO){
    //Reset counters
    countA = 0;
    countB = 0;
    //Set all zernikes to 0
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      zernikes[i]=0;
    return calmode;
  }

  /* TGT_CALMODE_ZPOKE: Poke Zernikes one at a time                    */
  /*                    Set to zero in between each poke.              */
  if(calmode == TGT_CALMODE_ZPOKE){
    //Zet all Zernikes to zero
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      zernikes[i] = 0;

    //Check counters
    if(countA >= 0 && countA < (2*LOWFS_N_ZERNIKE*ncalim)){
      //Set step counter
      *step = (countA/ncalim);
      //Poke one zernike by adding it on top of the flat
      if((countA/ncalim) % 2 == 1){
	z = (countB/ncalim) % LOWFS_N_ZERNIKE;
	zernikes[z] = zpoke[z];
	countB++;
      }
      countA++;
    }
    else{
      //Calibration done
      //Turn off calibration
      printf("TGT: Stopping calmode TGT_CALMODE_ZPOKE\n");
      init = 0;
      calmode = TGT_CALMODE_NONE;
    }
    
    return calmode;
  }

  /* TGT_CALMODE_ZRAND: Poke all Zernikes to a random value */
  if(calmode == TGT_CALMODE_ZRAND){
    //Zet all Zernikes to zero
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      zernikes[i] = 0;
    
    //Check counters
    if(countA >= 0 && countA < (2*ncalim)){
      //Set step counter
      *step = (countA/ncalim);
      //Poke all zernikes by random amount
      if((countA/ncalim) % 2 == 1){
	for(i=0;i<LOWFS_N_ZERNIKE;i++)
	  zernikes[i] = zpoke[i] * zrand[i];
      }
      countA++;
    }
    else{
      //Calibration done
      //Turn off calibration
      printf("TGT: Stopping calmode TGT_CALMODE_ZRAND\n");
      init = 0;
      calmode = TGT_CALMODE_NONE;
    }
    return calmode;
  }

  /* TGT_CALMODE_ZRAMP: Ramp Zernikes one at a time                    */
  /*                    Set to starting position in between each ramp. */
  if(calmode == TGT_CALMODE_ZRAMP){
    //Set all Zernikes to zero
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      zernikes[i] = 0.0;
    //Check counters
    if(countA >= 0 && countA < (2*LOWFS_N_ZERNIKE*ncalim)){
      
      //set step counter
      *step = (countA/ncalim);
      
      //ramp one zernike at a time
      if((countA/ncalim) % 2 == 1){
	z = (countB/ncalim) % LOWFS_N_ZERNIKE;
	zernikes[z] = (countB % ncalim) * (zpoke[z]/ncalim);
	countB++;
      }
      countA++;
    }else{
      //Turn off calibration
      printf("TGT: Stopping calmode TGT_CALMODE_ZRAMP\n");
      calmode = TGT_CALMODE_NONE;
      init = 0;
    }
    return calmode;
  }
  
  //Return calmode
  return calmode;
}
