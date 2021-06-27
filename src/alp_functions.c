#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <time.h>
#include <math.h>
#include <libgen.h>
#include <sys/stat.h>
#include <sys/io.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "numeric.h"
#include "alp_functions.h"
#include "alpao_map.h"
#include "rtd_functions.h"
#include "../drivers/phxdrv/picc_dio.h"

/**************************************************************/
/* ALP_FUNCTION_RESET                                         */
/*  - Reset all ALP functions                                */
/**************************************************************/
void alp_function_reset(sm_t *sm_p){
  alp_zern2alp(NULL,NULL,FUNCTION_RESET);
  alp_alp2zern(NULL,NULL,FUNCTION_RESET);
  alp_calibrate(sm_p, 0, NULL, NULL, NULL, 0, FUNCTION_RESET);
}

  
  /**************************************************************/
/* ALP_INIT_CALMODE                                           */
/*  - Initialize ALP calmode structure                        */
/**************************************************************/
void alp_init_calmode(int calmode, calmode_t *alp){
  int i;
  const double shk_zramp[LOWFS_N_ZERNIKE]={4.0,4.0,1.25,1.5,1.5,0.5,0.5,1.0,1.0,0.3,0.3,0.3,0.7,0.7,0.2,0.2,0.2,0.2,0.6,0.6,0.2,0.2,0.2};
  const double lyt_zramp[LOWFS_N_ZERNIKE]={0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.05};
  
  //DEFAULTS
  alp->shk_ncalim = ALP_SHK_NCALIM;
  alp->shk_poke   = ALP_SHK_POKE;
  alp->lyt_ncalim = ALP_LYT_NCALIM;
  alp->lyt_poke   = ALP_LYT_POKE;
  alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    alp->shk_zpoke[i]  = ALP_SHK_ZPOKE;
    alp->lyt_zpoke[i]  = ALP_LYT_ZPOKE;
  }
  
  //ALP_CALMODE_NONE
  if(calmode == ALP_CALMODE_NONE){
    sprintf(alp->name,"ALP_CALMODE_NONE");
    sprintf(alp->cmd,"none");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  }
  //ALP_CALMODE_TIMER
  if(calmode == ALP_CALMODE_TIMER){
    sprintf(alp->name,"ALP_CALMODE_TIMER");
    sprintf(alp->cmd,"timer");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  }
  //ALP_CALMODE_POKE
  if(calmode == ALP_CALMODE_POKE){
    sprintf(alp->name,"ALP_CALMODE_POKE");
    sprintf(alp->cmd,"poke");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  }
  //ALP_CALMODE_ZPOKE
  if(calmode == ALP_CALMODE_ZPOKE){
    sprintf(alp->name,"ALP_CALMODE_ZPOKE");
    sprintf(alp->cmd,"zpoke");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  }
  //ALP_CALMODE_FLIGHT
  if(calmode == ALP_CALMODE_FLIGHT){
    sprintf(alp->name,"ALP_CALMODE_FLIGHT");
    sprintf(alp->cmd,"flight");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  }
  //ALP_CALMODE_RAMP
  if(calmode == ALP_CALMODE_RAMP){
    sprintf(alp->name,"ALP_CALMODE_RAMP");
    sprintf(alp->cmd,"ramp");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_MAX;
  }
  //ALP_CALMODE_ZRAMP
  if(calmode == ALP_CALMODE_ZRAMP){
    sprintf(alp->name,"ALP_CALMODE_ZRAMP");
    sprintf(alp->cmd,"zramp");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_MAX;
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      alp->shk_zpoke[i]  = shk_zramp[i];
      alp->lyt_zpoke[i]  = lyt_zramp[i];
    }
    alp->shk_ncalim = 100;
  }
  //ALP_CALMODE_RAND
  if(calmode == ALP_CALMODE_RAND){
    sprintf(alp->name,"ALP_CALMODE_RAND");
    sprintf(alp->cmd,"rand");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  }
  //ALP_CALMODE_ZRAND
  if(calmode == ALP_CALMODE_ZRAND){
    sprintf(alp->name,"ALP_CALMODE_ZRAND");
    sprintf(alp->cmd,"zrand");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  }
  //ALP_CALMODE_PMZPOKE
  if(calmode == ALP_CALMODE_PMZPOKE){
    sprintf(alp->name,"ALP_CALMODE_PMZPOKE");
    sprintf(alp->cmd,"pmzpoke");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  }

}

/**************************************************************/
/* ALP_ZERN2ALP                                               */
/*  - Convert zernike commands to ALP actuator commands       */
/**************************************************************/
int alp_zern2alp(double *zernikes,double *actuators,int reset){
  static int init=0;
  static double zern2alp_matrix[LOWFS_N_ZERNIKE*ALP_NACT]={0};

  if(!init || reset){
    //Read matrix file
    if(read_file(SHKZER2ALPACT_FILE,zern2alp_matrix,sizeof(zern2alp_matrix))){
      memset(zern2alp_matrix,0,sizeof(zern2alp_matrix));
      return 1;
    }
    printf("ALP: Read zern2alp file\n");
    
    //Set init flag
    init=1;
    
    //Return if reset
    if(reset) return 0;
  }

  //Do Matrix Multiply
  num_dgemv(zern2alp_matrix,zernikes,actuators, ALP_NACT, LOWFS_N_ZERNIKE);

  return 0;
}

/**************************************************************/
/* ALP_ALP2ZERN                                               */
/*  - Convert ALP actuator commands to zernike commands       */
/**************************************************************/
int alp_alp2zern(double *actuators, double *zernikes,int reset){
  static int init=0;
  static double alp2zern_matrix[LOWFS_N_ZERNIKE*ALP_NACT]={0};

  if(!init || reset){
    //Read matrix file
    if(read_file(SHKZER2ALPACT_FILE,alp2zern_matrix,sizeof(alp2zern_matrix))){
      memset(alp2zern_matrix,0,sizeof(alp2zern_matrix));
      return 1;
    }
    printf("ALP: Read alp2zern file\n");
    
    //Set init flag
    init=1;
    
    //Return if reset
    if(reset) return 0;
  }

  //Do Matrix Multiply
  num_dgemv(alp2zern_matrix,actuators,zernikes, LOWFS_N_ZERNIKE, ALP_NACT);

  return 0;
}

/**************************************************************/
/* ALP_GET_COMMAND                                            */
/* - Function to get the last command sent to the ALPAO DM    */
/* - Use atomic operations to prevent two processes from      */
/*   accessing the commands at the same time                  */
/**************************************************************/
int alp_get_command(sm_t *sm_p, alp_t *cmd){
  int retval = 1;
  
  //Atomically test and set ALP command lock using GCC built-in function
  if(__sync_lock_test_and_set(&sm_p->alp_command_lock,1)==0){
    //Copy command
    memcpy(cmd,(alp_t *)&sm_p->alp_command,sizeof(alp_t));
    //Release lock
    __sync_lock_release(&sm_p->alp_command_lock);
    //Return 0 on success
    retval = 0;
  }
  
  //Return
  return retval;
}

/**************************************************************/
/* ALP_SET_SHK2LYT                                            */
/* - Function to send Zernike commands from SHK to LYT        */
/* - Use atomic operations to prevent two processes from      */
/*   accessing the commands at the same time                  */
/**************************************************************/
int alp_set_shk2lyt(sm_t *sm_p, alp_t *cmd){
  int retval = 1;
  
  //Atomically test and set ALP command lock using GCC built-in function
  if(__sync_lock_test_and_set(&sm_p->alp_shk2lyt_lock,1)==0){
    //Check if command is empty
    if(!sm_p->alp_shk2lyt_set){
      //Copy command
      memcpy((alp_t *)&sm_p->alp_shk2lyt,cmd,sizeof(alp_t));
      //Set command
      sm_p->alp_shk2lyt_set = 1;
      //Return 0 on success
      retval = 0;
    }
    //Release lock
    __sync_lock_release(&sm_p->alp_shk2lyt_lock);
  }
  
  //Return
  return retval;
}

/**************************************************************/
/* ALP_GET_SHK2LYT                                            */
/* - Function to receive Zernike commands from SHK to LYT     */
/* - Use atomic operations to prevent two processes from      */
/*   accessing the commands at the same time                  */
/**************************************************************/
int alp_get_shk2lyt(sm_t *sm_p, alp_t *cmd){
  int retval = 1;
  
  //Atomically test and set ALP command lock using GCC built-in function
  if(__sync_lock_test_and_set(&sm_p->alp_shk2lyt_lock,1)==0){
    //Check if command is set
    if(sm_p->alp_shk2lyt_set){
      //Copy command
      memcpy(cmd,(alp_t *)&sm_p->alp_shk2lyt,sizeof(alp_t));
      //Unset command
      sm_p->alp_shk2lyt_set = 0;
      //Return 0 on success
      retval = 0;
    }
    //Release lock
    __sync_lock_release(&sm_p->alp_shk2lyt_lock);
  }
  
  //Return
  return retval;
}

/**************************************************************/
/* ALP_SEND_COMMAND                                           */
/* - Function to command the ALPAO DM                         */
/* - Use atomic operations to prevent two processes from      */
/*   sending commands at the same time                        */
/* - Return 0 if the command was sent and 1 if it wasn't      */
/**************************************************************/
int alp_send_command(sm_t *sm_p, alp_t *cmd, int proc_id, int n_dither){
  int retval = 1;
  
  //Atomically test and set ALP command lock using GCC built-in function
  if(__sync_lock_test_and_set(&sm_p->alp_command_lock,1)==0){
    
    //Check if the commanding process is the ALP commander
    if(proc_id == sm_p->state_array[sm_p->state].alp_commander){
      
      //Set DIO bit A0
      #if PICC_DIO_ENABLE
      outb(0x01,PICC_DIO_BASE+PICC_DIO_PORTA);
      #endif

      //Check if we need to re-initalize the RTD board
      if((proc_id != sm_p->alp_proc_id) || (n_dither != sm_p->alp_n_dither)){
	//Init ALPAO RTD interface
	printf("ALP: Initializing RTD board for %s with %d dither steps\n",sm_p->w[proc_id].name,n_dither);
	if(rtd_init_alp(sm_p->p_rtd_alp_board,n_dither)){
	  perror("ALP: rtd_init_alp");
	}
	else{
	  sm_p->alp_proc_id = proc_id;
	  sm_p->alp_n_dither = n_dither;
	}
      }
      
      //Send the command
      if(!rtd_send_alp(sm_p->p_rtd_alp_board,cmd->acmd)){
	//Copy command to current position
	memcpy((alp_t *)&sm_p->alp_command,cmd,sizeof(alp_t));
	//Set retval for good command
	retval = 0;
      }
      
      //Unset DIO bit A0
      #if PICC_DIO_ENABLE
      outb(0x00,PICC_DIO_BASE+PICC_DIO_PORTA);
      #endif
      
    }
    
    //Release lock
    __sync_lock_release(&sm_p->alp_command_lock);
  }

  //Return
  return retval;
}


/**************************************************************/
/* ALP_SET_BIAS                                               */
/* - Set all actuators to the same value                      */
/**************************************************************/
int alp_set_bias(sm_t *sm_p, double bias, int proc_id){
  alp_t alp;
  int i;
  
  //Set bias
  for(i=0;i<ALP_NACT;i++)
    alp.acmd[i] = bias;

  //Clear zernike commands
  memset(alp.zcmd,0,sizeof(alp.zcmd));
  
  //Send command
  return(alp_send_command(sm_p,&alp,proc_id,1));
}

/**************************************************************/
/* ALP_SET_RANDOM                                             */
/* - Add a random perturbation to the ALP                     */
/**************************************************************/
int alp_set_random(sm_t *sm_p, int proc_id){
  alp_t alp;
  time_t t;
  int i;
  
  //Init random numbers
  srand((unsigned) time(&t));
  
  //Get current command
  if(alp_get_command(sm_p,&alp))
    return 1;

  //Add perturbation
  for(i=0;i<ALP_NACT;i++)
    alp.acmd[i] += (2*(rand() / (double) RAND_MAX) - 1) * ALP_SHK_POKE;

  //Send command
  return(alp_send_command(sm_p,&alp,proc_id,1));
}

/**************************************************************/
/* ALP_SET_ZRANDOM                                            */
/* - Add a random perturbation to the ALP zernikes            */
/**************************************************************/
int alp_set_zrandom(sm_t *sm_p, int proc_id){
  alp_t alp;
  time_t t;
  int i;
  double dz[LOWFS_N_ZERNIKE] = {0};
  double da[ALP_NACT] = {0};
  
  //Init random numbers
  srand((unsigned) time(&t));
  
  //Get current command
  if(alp_get_command(sm_p,&alp))
    return 1;

  //Calculate zernike perturbation 
  for(i=0;i<LOWFS_N_ZERNIKE;i++)
    dz[i] = (2*(rand() / (double) RAND_MAX) - 1) * ALP_SHK_ZPOKE;

  //Convert to actuators deltas
  alp_zern2alp(dz,da,FUNCTION_NO_RESET);

  //Add to current command
  for(i=0;i<LOWFS_N_ZERNIKE;i++)
    alp.zcmd[i] += dz[i]; 
  for(i=0;i<ALP_NACT;i++)
    alp.acmd[i] += da[i]; 

  //Send command
  return(alp_send_command(sm_p,&alp,proc_id,1));
}

/**************************************************************/
/* ALP_ZERO_FLAT                                              */
/* - Set ALP to all zeros                                     */
/**************************************************************/
int alp_zero_flat(sm_t *sm_p, int proc_id){
  alp_t alp;
  memset(&alp,0,sizeof(alp_t));
  return(alp_send_command(sm_p,&alp,proc_id,1));
}

/**************************************************************/
/* ALP_REVERT_FLAT                                            */
/* - Set ALP to #defined flat map                             */
/**************************************************************/
int alp_revert_flat(sm_t *sm_p, int proc_id){
  alp_t alp;
  const double flat[ALP_NACT] = ALP_OFFSET;
  memset(&alp,0,sizeof(alp_t));
  memcpy(alp.acmd,flat,sizeof(alp.acmd));
  return(alp_send_command(sm_p,&alp,proc_id,1));
}

/**************************************************************/
/* ALP_SAVE_FLAT                                              */
/* - Save current ALP position to file                        */
/**************************************************************/
int alp_save_flat(sm_t *sm_p){
  alp_t alp;  

  //Get current command
  if(alp_get_command(sm_p,&alp))
    return 1;

  //Clear zernike commands
  memset(alp.zcmd,0,sizeof(alp.zcmd));

  //Write output file
  check_and_mkdir(ALP_FLAT_FILE);
  write_file(ALP_FLAT_FILE,&alp,sizeof(alp));
  printf("ALP: Wrote: %s\n",ALP_FLAT_FILE);

  return 0;
}

/***************************************************************/
/* ALP_LOAD_FLAT                                               */
/*  - Loads ALP flat from file                                 */
/***************************************************************/
int alp_load_flat(sm_t *sm_p,int proc_id){
  alp_t alp;
    
  //Read file
  if(read_file(ALP_FLAT_FILE,&alp,sizeof(alp)))
    return 1;
  
  //Clear zernike commands
  memset(alp.zcmd,0,sizeof(alp.zcmd));
  
  //Send flat to ALP
  return(alp_send_command(sm_p,&alp,proc_id,1));
}


/**************************************************************/
/* ALP_INIT_CALIBRATION                                       */
/* - Initialize calibration structure                         */
/**************************************************************/
void alp_init_calibration(sm_t *sm_p){
  int i;

  //Zero out calibration struct
  memset((void *)&sm_p->alpcal,0,sizeof(alpcal_t));

  //Read Zernike errors file
  if(read_file(ZERNIKE_ERRORS_FILE,sm_p->alpcal.zernike_errors,sizeof(sm_p->alpcal.zernike_errors)))
    memset(sm_p->alpcal.zernike_errors,0,sizeof(sm_p->alpcal.zernike_errors));
  
  /* Initialize other elements */
  sm_p->alpcal.timer_length = CALMODE_TIMER_SEC;
  sm_p->alpcal.command_scale = 1;
  
  return; 
}

/**************************************************************/
/* ALP_CALIBRATE                                              */
/* - Run calibration routines for ALPAO DM                    */
/**************************************************************/
int alp_calibrate(sm_t *sm_p, int calmode, alp_t *alp, uint32_t *step, double *zoutput, int procid, int reset){
  const double zernike_timestep = ZERNIKE_ERRORS_PERIOD;
  static int init=0;
  static double zrand[LOWFS_N_ZERNIKE]={0};
  static double arand[ALP_NACT]={0};
  static calmode_t alpcalmodes[ALP_NCALMODES];
  uint64_t i,j,z,index;
  struct timespec this,delta;
  time_t trand;
  double dt,step_fraction;
  double dz[LOWFS_N_ZERNIKE]={0};
  double this_zernike[LOWFS_N_ZERNIKE]={0};
  double act[ALP_NACT];
  double poke=0,zpoke[LOWFS_N_ZERNIKE]={0};
  int    ncalim=0;

  /* Reset & Quick Init*/
  if(reset || !init){
    //Reset counters
    memset((void *)sm_p->alpcal.countA,0,sizeof(sm_p->alpcal.countA));
    memset((void *)sm_p->alpcal.countB,0,sizeof(sm_p->alpcal.countB));
    //Reset random numbers
    srand((unsigned) time(&trand));
    for(i=0;i<LOWFS_N_ZERNIKE;i++) zrand[i] = (2*(rand() / (double) RAND_MAX) - 1);
    for(i=0;i<ALP_NACT;i++) arand[i] = (2*(rand() / (double) RAND_MAX) - 1);
    //Init ALP calmodes
    for(i=0;i<ALP_NCALMODES;i++)
      alp_init_calmode(i,&alpcalmodes[i]);
    init = 1;
    printf("ALP: Calibration initialized\n");
    if(reset) return calmode;
  }
  
  /* Set calibration parameters */
  if(procid == SHKID){
    poke   = alpcalmodes[calmode].shk_poke;
    memcpy(zpoke,alpcalmodes[calmode].shk_zpoke,sizeof(zpoke));
    ncalim = alpcalmodes[calmode].shk_ncalim;
  }
  if(procid == LYTID){
    poke   = alpcalmodes[calmode].lyt_poke;
    memcpy(zpoke,alpcalmodes[calmode].lyt_zpoke,sizeof(zpoke));
    ncalim = alpcalmodes[calmode].lyt_ncalim;
  }
  
  /* Get time */
  clock_gettime(CLOCK_REALTIME, &this);

  /* Init times and ALP commands */
  if((calmode != ALP_CALMODE_NONE) && (sm_p->alpcal.countA[calmode] == 0)){
    //Save start time
    memcpy((struct timespec *)&sm_p->alpcal.start[calmode],&this,sizeof(struct timespec));
    //Save alp starting position
    memcpy((alp_t *)&sm_p->alpcal.alp_start[calmode],alp,sizeof(alp_t));
  }
  
  /* ALP_CALMODE_NONE: Do nothing. Just reset counters.*/
  if(calmode==ALP_CALMODE_NONE){
    //Reset counters
    memset((void *)sm_p->alpcal.countA,0,sizeof(sm_p->alpcal.countA));
    memset((void *)sm_p->alpcal.countB,0,sizeof(sm_p->alpcal.countB));

    //Return calmode
    return calmode;
  }
  
  /* ALP_CALMODE_TIMER: Do nothing. End after a defined amount of time     */
  if(calmode==ALP_CALMODE_TIMER){
    //Get time delta
    if(timespec_subtract(&delta,&this,(struct timespec *)&sm_p->alpcal.start[calmode]))
      printf("ALP: alp_calibrate --> timespec_subtract error!\n");
    ts2double(&delta,&dt);
    
    if(dt > sm_p->alpcal.timer_length){
      //Turn off calibration
      printf("ALP: Stopping ALP calmode ALP_CALMODE_TIMER\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }
    
    //Set step counter
    *step = sm_p->alpcal.countA[calmode];
    
    //Increment counter
    sm_p->alpcal.countA[calmode]++;
    
    //Return calmode
    return calmode;
  }
  
  /* ALP_CALMODE_POKE: Scan through acuators poking one at a time.    */
  /*                   Set to starting position in between each poke. */
  if(calmode==ALP_CALMODE_POKE){
    //Check counters
    if(sm_p->alpcal.countA[calmode] >= 0 && sm_p->alpcal.countA[calmode] < (2*ALP_NACT*ncalim)){
      //Set all ALP actuators to starting position
      for(i=0;i<ALP_NACT;i++)
	alp->acmd[i]=sm_p->alpcal.alp_start[calmode].acmd[i];
      
      //Poke one actuator
      if((sm_p->alpcal.countA[calmode]/ncalim) % 2 == 1){
	alp->acmd[(sm_p->alpcal.countB[calmode]/ncalim) % ALP_NACT] += poke * sm_p->alpcal.command_scale;
	sm_p->alpcal.countB[calmode]++;
      }
    }else{
      //Set alp back to starting position
      memcpy(alp,(alp_t *)&sm_p->alpcal.alp_start[calmode],sizeof(alp_t));
      //Turn off calibration
      printf("ALP: Stopping ALP calmode ALP_CALMODE_POKE\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }
    
    //Set step counter
    *step = (sm_p->alpcal.countA[calmode]/ncalim);
    
    //Increment counter
    sm_p->alpcal.countA[calmode]++;
    
    //Return calmode
    return calmode;
    
  }
  
  /* ALP_CALMODE_ZPOKE: Poke Zernikes one at a time                    */
  /*                    Set to starting position in between each poke. */
  if(calmode == ALP_CALMODE_ZPOKE){
    //Check counters
    if(sm_p->alpcal.countA[calmode] >= 0 && sm_p->alpcal.countA[calmode] < (2*LOWFS_N_ZERNIKE*ncalim)){
      //Set all Zernikes to zero
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	alp->zcmd[i] = 0.0;

      //Set all ALP actuators to starting position
      for(i=0;i<ALP_NACT;i++)
	alp->acmd[i]=sm_p->alpcal.alp_start[calmode].acmd[i];
      
      //Poke one zernike by adding it on top of the starting position
      if((sm_p->alpcal.countA[calmode]/ncalim) % 2 == 1){
	z = (sm_p->alpcal.countB[calmode]/ncalim) % LOWFS_N_ZERNIKE;
	alp->zcmd[z] = zpoke[z] * sm_p->alpcal.command_scale;
	alp_zern2alp(alp->zcmd,act,FUNCTION_NO_RESET);
	for(i=0; i<ALP_NACT; i++)
	  alp->acmd[i] += act[i];
	sm_p->alpcal.countB[calmode]++;
      }
    }else{
      //Set alp back to starting position
      memcpy(alp,(alp_t *)&sm_p->alpcal.alp_start[calmode],sizeof(alp_t));
      //Turn off calibration
      printf("ALP: Stopping calmode ALP_CALMODE_ZPOKE\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }

    //Set step counter
    *step = (sm_p->alpcal.countA[calmode]/ncalim);

    //Increment counter
    sm_p->alpcal.countA[calmode]++;
    
    //Return calmode
    return calmode;
  }

  /* ALP_CALMODE_PMZPOKE: Poke Zernikes one at a time @ +/- poke amplitude. */
  /*                      Set to starting position in between each poke.    */
  if(calmode == ALP_CALMODE_PMZPOKE){
    //Check counters
    if(sm_p->alpcal.countA[calmode] >= 0 && sm_p->alpcal.countA[calmode] < (3*LOWFS_N_ZERNIKE*ncalim)){
      //Set all Zernikes to zero
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	alp->zcmd[i] = 0.0;

      //Set all ALP actuators to starting position
      for(i=0;i<ALP_NACT;i++)
	alp->acmd[i]=sm_p->alpcal.alp_start[calmode].acmd[i];
      
      //Poke one zernike UP
      if((sm_p->alpcal.countA[calmode]/ncalim) % 3 == 1){
	z = (sm_p->alpcal.countB[calmode]/ncalim) % LOWFS_N_ZERNIKE;
	alp->zcmd[z] = zpoke[z] * sm_p->alpcal.command_scale;
	alp_zern2alp(alp->zcmd,act,FUNCTION_NO_RESET);
	for(i=0; i<ALP_NACT; i++)
	  alp->acmd[i] += act[i];
      }
      //Poke one zernike DOWN
      if((sm_p->alpcal.countA[calmode]/ncalim) % 3 == 2){
	z = (sm_p->alpcal.countB[calmode]/ncalim) % LOWFS_N_ZERNIKE;
	alp->zcmd[z] = -1 * zpoke[z] * sm_p->alpcal.command_scale;
	alp_zern2alp(alp->zcmd,act,FUNCTION_NO_RESET);
	for(i=0; i<ALP_NACT; i++)
	  alp->acmd[i] += act[i];
	//Increment Zernike counter
	sm_p->alpcal.countB[calmode]++;
      }
    }else{
      //Set alp back to starting position
      memcpy(alp,(alp_t *)&sm_p->alpcal.alp_start[calmode],sizeof(alp_t));
      //Turn off calibration
      printf("ALP: Stopping calmode ALP_CALMODE_PMZPOKE\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }

    //Set step counter
    *step = (sm_p->alpcal.countA[calmode]/ncalim);

    //Increment counter
    sm_p->alpcal.countA[calmode]++;
    
    //Return calmode
    return calmode;
  }

  /* ALP_CALMODE_RAMP: Scan through acuators ramping one at a time.    */
  /*                   Set to starting position in between each ramp. */
  if(calmode == ALP_CALMODE_RAMP){
    //Check counters
    if(sm_p->alpcal.countA[calmode] >= 0 && sm_p->alpcal.countA[calmode] < (2*ALP_NACT*ncalim)){
      //Set all ALP actuators to starting position -- only the first interation
      if((sm_p->alpcal.countB[calmode] % ncalim) == 0)
	for(i=0;i<ALP_NACT;i++)
	  alp->acmd[i]=sm_p->alpcal.alp_start[calmode].acmd[i];

      //Ramp one actuator
      if((sm_p->alpcal.countA[calmode]/ncalim) % 2 == 1){
	alp->acmd[(sm_p->alpcal.countB[calmode]/ncalim) % ALP_NACT] += 5*poke*sm_p->alpcal.command_scale/ncalim;
	sm_p->alpcal.countB[calmode]++;
      }
    }else{
      //Set alp back to starting position
      memcpy(alp,(alp_t *)&sm_p->alpcal.alp_start[calmode],sizeof(alp_t));
      //Turn off calibration
      printf("ALP: Stopping ALP calmode ALP_CALMODE_RAMP\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }

    //Set step counter
    *step = (sm_p->alpcal.countA[calmode]/ncalim);
    
    //Increment counter
    sm_p->alpcal.countA[calmode]++;
    
    //Return calmode
    return calmode;
  }

  /* ALP_CALMODE_ZRAMP: Ramp Zernikes one at a time                    */
  /*                    Set to starting position in between each ramp. */
  if(calmode == ALP_CALMODE_ZRAMP){
    //Check counters
    if(sm_p->alpcal.countA[calmode] >= 0 && sm_p->alpcal.countA[calmode] < (2*LOWFS_N_ZERNIKE*ncalim)){
      //Set all Zernikes to zero -- only the first interation
      if((sm_p->alpcal.countB[calmode] % ncalim) == 0)
	for(i=0;i<LOWFS_N_ZERNIKE;i++)
	  alp->zcmd[i] = 0.0;

      //Set all ALP actuators to starting position
      for(i=0;i<ALP_NACT;i++)
	alp->acmd[i]=sm_p->alpcal.alp_start[calmode].acmd[i];

      //Ramp one zernike by adding it on top of the starting position
      if((sm_p->alpcal.countA[calmode]/ncalim) % 2 == 1){
	z = (sm_p->alpcal.countB[calmode]/ncalim) % LOWFS_N_ZERNIKE;
	//Set starting position as negative full ramp amplitude
	if(sm_p->alpcal.countB[calmode] % ncalim == 0) alp->zcmd[z] = -zpoke[z] * sm_p->alpcal.command_scale;
	alp->zcmd[z] += 2*zpoke[z]*sm_p->alpcal.command_scale/ncalim;
	alp_zern2alp(alp->zcmd,act,FUNCTION_NO_RESET);
	for(i=0; i<ALP_NACT; i++)
	  alp->acmd[i] += act[i];
	sm_p->alpcal.countB[calmode]++;
      }
    }else{
      //Set alp back to starting position
      memcpy(alp,(alp_t *)&sm_p->alpcal.alp_start[calmode],sizeof(alp_t));
      //Turn off calibration
      printf("ALP: Stopping calmode ALP_CALMODE_ZRAMP\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }
 
    //Set step counter
    *step = (sm_p->alpcal.countA[calmode]/ncalim);

    //Increment counter
    sm_p->alpcal.countA[calmode]++;

    //Return calmode
    return calmode;
  }
  
  /* ALP_CALMODE_RAND: Poke all actuators by a random amount           */
  if(calmode == ALP_CALMODE_RAND){
    //Check counters
    if(sm_p->alpcal.countA[calmode] >= 0 && sm_p->alpcal.countA[calmode] < (2*ncalim)){
      //Poke all actuators by random amount (just once)
      if(sm_p->alpcal.countA[calmode] == ncalim){
	for(i=0; i<ALP_NACT; i++)
	  alp->acmd[i] = sm_p->alpcal.alp_start[calmode].acmd[i] + poke * arand[i] * sm_p->alpcal.command_scale;
      }
    }else{
      //Set alp back to starting position
      memcpy(alp,(alp_t *)&sm_p->alpcal.alp_start[calmode],sizeof(alp_t));
      //Turn off calibration
      printf("ALP: Stopping calmode ALP_CALMODE_RAND\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }

    //Set step counter
    *step = (sm_p->alpcal.countA[calmode]/ncalim);
    
    //Increment counter
    sm_p->alpcal.countA[calmode]++;

    //Return calmode
    return calmode;
  }

  /* ALP_CALMODE_ZRAND: Poke all zernikes by a random amount           */
  if(calmode == ALP_CALMODE_ZRAND){
    //Check counters
    if(sm_p->alpcal.countA[calmode] >= 0 && sm_p->alpcal.countA[calmode] < (2*ncalim)){
      //Set all Zernikes to zero
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	alp->zcmd[i] = 0.0;
      //Poke all zernikes by random amount (just once)
      if(sm_p->alpcal.countA[calmode] >= ncalim){
	for(i=0; i<LOWFS_N_ZERNIKE; i++)
	  alp->zcmd[i] = zpoke[i] * zrand[i] * sm_p->alpcal.command_scale;
	alp_zern2alp(alp->zcmd,act,FUNCTION_NO_RESET);
	for(i=0; i<ALP_NACT; i++)
	  alp->acmd[i] = sm_p->alpcal.alp_start[calmode].acmd[i]+act[i];
      }
    }else{
      //Set alp back to starting position
      memcpy(alp,(alp_t *)&sm_p->alpcal.alp_start[calmode],sizeof(alp_t));
      //Turn off calibration
      printf("ALP: Stopping calmode ALP_CALMODE_ZRAND\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }

    //Set step counter
    *step = (sm_p->alpcal.countA[calmode]/ncalim);

    //Increment counter
    sm_p->alpcal.countA[calmode]++;

    //Return calmode
    return calmode;
  }
  
  /* ALP_CALMODE_FLIGHT: Flight Simulator */
  if(calmode == ALP_CALMODE_FLIGHT){
    //Get time delta
    if(timespec_subtract(&delta,&this,(struct timespec *)&sm_p->alpcal.start[calmode]))
      printf("ALP: alp_calibrate --> timespec_subtract error!\n");
    ts2double(&delta,&dt);
    //Set data index
    index = (uint64_t)(dt/zernike_timestep);
    if((index < ZERNIKE_ERRORS_NUMBER-1) && (dt <= sm_p->alpcal.timer_length)){
      //Interpolate between steps, convert from [Microns RMS Wavefront] to [Microns RMS Surface], calc zernike deltas
      step_fraction = fmod(dt,zernike_timestep)/zernike_timestep;
      for(i=0;i<LOWFS_N_ZERNIKE;i++){
	this_zernike[i] = 0.5*((1-step_fraction)*sm_p->alpcal.zernike_errors[i][index] + step_fraction*sm_p->alpcal.zernike_errors[i][index+1]);
	this_zernike[i] *= sm_p->alpcal.command_scale;
	if(sm_p->alp_zernike_control[i])
	  dz[i] = this_zernike[i] - sm_p->alpcal.last_zernike[i];
      }
      
      //Wait for the 2nd iteration to move the mirror to prevent large deltas
      if(sm_p->alpcal.countA[calmode] > 0){
	//Add zernike deltas to ALP command
	for(i=0;i<LOWFS_N_ZERNIKE;i++)
	  alp->zcmd[i] += dz[i];
	//Convert zernike deltas to actuator deltas
	alp_zern2alp(dz,act,FUNCTION_NO_RESET);
	//Add offsets to ALP position
	for(i=0;i<ALP_NACT;i++)
	  alp->acmd[i] += act[i];
      }
      //Save zernikes
      memcpy((double *)sm_p->alpcal.last_zernike,this_zernike,sizeof(sm_p->alpcal.last_zernike));
      //Return current zernikes to user
      memcpy(zoutput, this_zernike, sizeof(this_zernike));
    }else{
      //Don't set alp back to starting position since we are probably in closed loop
      //Turn off calibration
      printf("ALP: Stopping ALP calmode ALP_CALMODE_FLIGHT\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }
  }

  //Set step counter
  *step = sm_p->alpcal.countA[calmode];

  //Increment counter
  sm_p->alpcal.countA[calmode]++;

  //Return calmode
  return calmode;
}
