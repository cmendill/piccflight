#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <time.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "numeric.h"
#include "alp_functions.h"
#include "alpao_map.h"
#include "rtd_functions.h"

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
  //ALP_CALMODE_FLAT
  if(calmode == ALP_CALMODE_FLAT){
    sprintf(alp->name,"ALP_CALMODE_FLAT");
    sprintf(alp->cmd,"flat");
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
int alp_zern2alp(double *zernikes,double *actuators){
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
    printf("ALP: Read: %s\n",matrix_file);
    
    //--set init flag
    init=1;
  }

  //Do Matrix Multiply
  num_dgemv(zern2alp_matrix,zernikes,actuators, ALP_NACT, LOWFS_N_ZERNIKE);
  
  return 0;
}

/**************************************************************/
/* ALP_GET_COMMAND                                            */
/* - Function to get the last command sent to the ALPAO DM    */
/* - Use atomic operations to prevent two processes from      */
/*   accessing the commands at the same time                  */
/**************************************************************/
void alp_get_command(sm_t *sm_p, alp_t *cmd){
  //Atomically test and set ALP command lock using GCC built-in function
  while(__sync_lock_test_and_set(&sm_p->alp_command_lock,1));
  //Copy command
  memcpy(cmd,(alp_t *)&sm_p->alp_command,sizeof(alp_t));
  //Release lock
  __sync_lock_release(&sm_p->alp_command_lock);
}

/**************************************************************/
/* ALP_SEND_COMMAND                                           */
/* - Function to command the ALPAO DM                         */
/* - Use atomic operations to prevent two processes from      */
/*   sending commands at the same time                        */
/* - Return 1 if the command was sent and 0 if it wasn't      */
/**************************************************************/
int alp_send_command(sm_t *sm_p, alp_t *cmd, int proc_id, uint32_t n_dither){
  int retval=0;
  static int last_n_dither=0;
  
  //Atomically test and set ALP command lock using GCC built-in function
  if(__sync_lock_test_and_set(&sm_p->alp_command_lock,1)==0){

    //Check if the commanding process is the ALP commander
    if(proc_id == sm_p->state_array[sm_p->state].alp_commander){
      
      //Check if we need to re-initalize the RTD board
      if(n_dither != last_n_dither){
	//Init ALPAO RTD interface
	if(rtd_init_alp(sm_p->p_rtd_board,n_dither))
	  perror("rtd_init_alp");
	else
	  last_n_dither = n_dither;
      }
      
      //Send the command
      if(rtd_send_alp(sm_p->p_rtd_board,cmd->act_cmd) == 0){
	//Copy command to current position
	memcpy((alp_t *)&sm_p->alp_command,cmd,sizeof(alp_t));
 	//Set return value
	retval=1;
      }

    }
    //Release lock
    __sync_lock_release(&sm_p->alp_command_lock);
  }
  
  //Return
  return retval;
}


/**************************************************************/
/* ALP_CALIBRATE                                              */
/* - Run calibration routines for ALPAO DM                    */
/**************************************************************/
int alp_calibrate(int calmode, alp_t *alp, uint32_t *step, int reset){
  int i,j,index;
  static struct timespec start,this,last,delta;
  static uint64 countA=0,countB=0;
  static double zernike_errors[LOWFS_N_ZERNIKE][ZERNIKE_ERRORS_NUMBER]={{0}};
  const double zernike_timestep = ZERNIKE_ERRORS_PERIOD;
  const int flight_zuse[LOWFS_N_ZERNIKE]={0,0,1,1,1, 1,1,1,1,1,1, 0,0,0,0,0,0, 0,0,0,0,0,0};
  static int init=0;
  time_t t;
  FILE *fileptr=NULL;
  char filename[MAX_FILENAME];
  double dt=0,dt0=0,period=0;
  double zernikes[LOWFS_N_ZERNIKE]={0};
  double act[ALP_NACT];
  const double flat[ALP_NACT] = ALP_OFFSET;
  
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
      printf("ALP: incorrect zernike_errors file size %lu != %lu\n",ftell(fileptr),sizeof(zernike_errors));
      goto endofinit;
    }
    rewind(fileptr);
    //--read data
    if(fread(zernike_errors,sizeof(zernike_errors),1,fileptr) != 1){
      perror("fread");
      goto endofinit;
    }
  endofinit:
    //--close file
    if(fileptr != NULL){
      printf("ALP: Read: %s\n",filename);
      fclose(fileptr);
    }
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

  /* ALP_CALMODE_FLAT: Set all actuators to flat map */
  if(calmode==ALP_CALMODE_FLAT){
    //Reset counters
    countA=0;
    countB=0;
    //Set all ALP actuators to flat
    for(i=0;i<ALP_NACT;i++)
      alp->act_cmd[i]=flat[i];
    return calmode;
  }
  
  /* ALP_CALMODE_POKE: Scan through acuators poking one at a time. */
  /*                   Set flat in between each poke.              */
  if(calmode == ALP_CALMODE_POKE){
    //Check counters
    if(countA >= 0 && countA < (2*ALP_NACT*ALP_NCALIM)){
      //set all ALP actuators to flat
      for(i=0;i<ALP_NACT;i++)
	alp->act_cmd[i]=flat[i];

      //set step counter
      *step = (countA/ALP_NCALIM);

      //poke one actuator
      if((countA/ALP_NCALIM) % 2 == 1){
	alp->act_cmd[(countB/ALP_NCALIM) % ALP_NACT] += ALP_POKE;
	countB++;
      }
      countA++;
    }else{
      //Turn off calibration
      printf("ALP: Stopping ALP calmode ALP_CALMODE_POKE\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }
    return calmode;

  }

  /* ALP_CALMODE_ZPOKE: Poke Zernikes one at a time    */
  /*                    Set flat in between each poke. */
  if(calmode == ALP_CALMODE_ZPOKE){
    //Check counters
    if(countA >= 0 && countA < (2*LOWFS_N_ZERNIKE*ALP_NCALIM)){
      //set all Zernikes to zero
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	alp->zernike_cmd[i] = 0.0;
      
      //set all ALP actuators to flat
      for(i=0;i<ALP_NACT;i++)
	alp->act_cmd[i]=flat[i];

      //set step counter
      *step = (countA/ALP_NCALIM);

      //poke one zernike by adding it on top of the flat
      if((countA/ALP_NCALIM) % 2 == 1){
	alp->zernike_cmd[(countB/ALP_NCALIM) % LOWFS_N_ZERNIKE] = ALP_ZPOKE;
	alp_zern2alp(alp->zernike_cmd,act);
	for(i=0; i<ALP_NACT; i++)
	  alp->act_cmd[i] += act[i];
	countB++;
      }
      countA++;
    }else{
      printf("ALP: Stopping calmode ALP_CALMODE_ZPOKE\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }
    return calmode;
  }

  /* ALP_CALMODE_FLIGHT: Flight Simulator */
  if(calmode == ALP_CALMODE_FLIGHT){
    //Setup counters
    if(countA == 0)
      dt0 = dt;
    if(countA == 1)
      period = dt-dt0;
    //Set step counter
    *step = countA;
    //Set index
    index = (int)((dt-dt0)/zernike_timestep);
    if(index < ZERNIKE_ERRORS_NUMBER){
      //Get zernikes for this timestep
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(flight_zuse[i])
	  zernikes[i] = zernike_errors[i][index % ZERNIKE_ERRORS_NUMBER];
      //Convert zernikes to actuators
      alp_zern2alp(zernikes,act);
      //Add offsets to ALP position
      for(i=0;i<ALP_NACT;i++)
	alp->act_cmd[i] += act[i];
      countA++;
    }else{
      //Turn off calibration
      printf("ALP: Stopping ALP calmode ALP_CALMODE_FLIGHT\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
      return calmode;
    }
  }
  //Return calmode
  return calmode;
}





