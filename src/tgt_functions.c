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

//Defaults
#define TGT_SHK_NCALIM        100   //shk number of calibration images per tgt step
#define TGT_LYT_NCALIM        200   //lyt number of calibration images per tgt step
#define TGT_LYT_ZPOKE         0.005 //lyt tgt zpoke [microns rms]
#define TGT_SHK_ZPOKE         0.02  //shk tgt zpoke [microns rms]

/**************************************************************/
/* TGT_INIT_CALMODE                                           */
/*  - Initialize TGT calmode structure                        */
/**************************************************************/
void tgt_init_calmode(int calmode, calmode_t *tgt){
  //DEFAULTS
  tgt->shk_ncalim = TGT_SHK_NCALIM;
  tgt->shk_zpoke  = TGT_SHK_ZPOKE;
  tgt->lyt_ncalim = TGT_LYT_NCALIM;
  tgt->lyt_zpoke  = TGT_LYT_ZPOKE;

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
  }
}

/**************************************************************/
/* TGT_CALIBRATE                                              */
/* - Run calibration routines for TGT                         */
/**************************************************************/
int tgt_calibrate(int calmode, double *zernikes, uint32_t *step, int procid, int reset){
  int i,j,z;
  time_t t;
  static int init=0;
  static long countA=0,countB=0,ncalim=0;
  const double shk_zpoke[LOWFS_N_ZERNIKE]={0.2,0.2,0.05,0.05,0.05,0.05,0.05,0.03,0.03,0.03,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02};
  const double lyt_zpoke[LOWFS_N_ZERNIKE]={0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005,0.005};
  double zpoke[LOWFS_N_ZERNIKE]={0};
  static calmode_t tgtcalmodes[TGT_NCALMODES];

  /* Initialize */
  if(!init || reset){
    countA = 0;
    countB = 0;
    //Init TGT calmodes
    for(i=0;i<TGT_NCALMODES;i++)
      tgt_init_calmode(i,&tgtcalmodes[i]);
    init=1;
    //Return if reset
    if(reset) return calmode;
  }

  /* Set calibration parameters */
  if(procid == SHKID){
    memcpy(zpoke,shk_zpoke,sizeof(zpoke));
    ncalim = tgtcalmodes[calmode].shk_ncalim;
  }
  if(procid == LYTID){
    memcpy(zpoke,lyt_zpoke,sizeof(zpoke));
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
  
  //Return calmode
  return calmode;
}
