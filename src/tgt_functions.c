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

/**************************************************************/
/* TGT_INIT_CALMODE                                           */
/*  - Initialize TGT calmode structure                        */
/**************************************************************/
void tgt_init_calmode(int calmode, calmode_t *tgt){
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
int tgt_calibrate(int calmode, double *zernikes, int procid, int reset){
  int i,j;
  time_t t;
  static int init=0;
  static long countA=0,countB=0,ncalim=0;
  static double zpoke=0;
  
  
  /* Initialize */
  if(!init || reset){
    countA = 0;
    countB = 0
    init=1;
    //Return if reset
    if(reset) return calmode;
  }

  /* Set calibration parameters */
  if(procid == SHKID){
    zpoke  = ALP_SHK_ZPOKE;
    ncalim = ALP_SHK_NCALIM;
  }
  if(procid == LYTID){
    zpoke  = ALP_LYT_ZPOKE;
    ncalim = ALP_LYT_NCALIM;
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
      //Poke one zernike by adding it on top of the flat
      if((countA[calmode]/ncalim) % 2 == 1){
	zernikes[(countB[calmode]/ncalim) % LOWFS_N_ZERNIKE] = zpoke;
	countB++;
      }
      countA++;
    }
    else{
      //Calibration done
      countA = 0;
      countB = 0;
      return TGT_CALMODE_NONE;
    }
    
    return calmode;
  }
  
  //Return calmode
  return calmode;
}
