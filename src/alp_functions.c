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
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  }
  //ALP_CALMODE_ZERO
  if(calmode == ALP_CALMODE_ZERO){
    sprintf(alp->name,"ALP_CALMODE_ZERO");
    sprintf(alp->cmd,"zero");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  }
  //ALP_CALMODE_FLAT
  if(calmode == ALP_CALMODE_FLAT){
    sprintf(alp->name,"ALP_CALMODE_FLAT");
    sprintf(alp->cmd,"flat");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;
  }
  //ALP_CALMODE_POKE
  if(calmode == ALP_CALMODE_POKE){
    sprintf(alp->name,"ALP_CALMODE_POKE");
    sprintf(alp->cmd,"poke");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_MAX;
  }
  //ALP_CALMODE_ZPOKE
  if(calmode == ALP_CALMODE_ZPOKE){
    sprintf(alp->name,"ALP_CALMODE_ZPOKE");
    sprintf(alp->cmd,"zpoke");
    alp->shk_boxsize_cmd = SHK_BOXSIZE_CMD_MAX;
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
  }

}

/**************************************************************/
/* ALP_ZERN2ALP                                               */
/*  - Convert zernike commands to ALPAO DM commands           */
/**************************************************************/
int alp_zern2alp(double *zernikes,double *actuators,int reset){
  FILE *matrix=NULL;
  char matrix_file[MAX_FILENAME];
  static int init=0;
  static double zern2alp_matrix[LOWFS_N_ZERNIKE*ALP_NACT]={0};
  uint64 fsize,rsize;
  int c,i;

  if(!init || reset){
    /* Open matrix file */
    //--setup filename
    sprintf(matrix_file,SHKZER2ALPACT_FILE);
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

    //--return if reset
    if(reset) return 0;
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
int alp_send_command(sm_t *sm_p, alp_t *cmd, int proc_id, int n_dither){
  int retval=0;

  //Atomically test and set ALP command lock using GCC built-in function
  if(__sync_lock_test_and_set(&sm_p->alp_command_lock,1)==0){

    //Check if the commanding process is the ALP commander
    if(proc_id == sm_p->state_array[sm_p->state].alp_commander){

      //Check if we need to re-initalize the RTD board
      if((proc_id != sm_p->alp_proc_id) || (n_dither != sm_p->alp_n_dither)){
	//Init ALPAO RTD interface
	printf("ALP: Initializing RTD board for %s with %d dither steps\n",sm_p->w[proc_id].name,n_dither);
	if(rtd_init_alp(sm_p->p_rtd_board,n_dither))
	  perror("rtd_init_alp");
	else{
	  sm_p->alp_proc_id = proc_id;
	  sm_p->alp_n_dither = n_dither;
	}
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
  alp_get_command(sm_p,&alp);

  //Add perturbation
  for(i=0;i<ALP_NACT;i++)
    alp.act_cmd[i] += (2*(rand() / (double) RAND_MAX) - 1) * ALP_SHK_POKE;

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
  alp_get_command(sm_p,&alp);

  //Calculate zernike perturbation 
  for(i=0;i<LOWFS_N_ZERNIKE;i++)
    dz[i] += (2*(rand() / (double) RAND_MAX) - 1) * ALP_SHK_ZPOKE;

  //Convert to actuators deltas
  alp_zern2alp(dz,da,FUNCTION_NO_RESET);

  //Add to current command
  for(i=0;i<LOWFS_N_ZERNIKE;i++)
    alp.zernike_cmd[i] += dz[i]; 
  for(i=0;i<ALP_NACT;i++)
    alp.act_cmd[i] += da[i]; 

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
  memcpy(alp.act_cmd,flat,sizeof(flat));
  return(alp_send_command(sm_p,&alp,proc_id,1));
}

/**************************************************************/
/* ALP_SAVE_FLAT                                              */
/* - Save current ALP position to file                        */
/**************************************************************/
int alp_save_flat(sm_t *sm_p){
  alp_t alp;
  struct stat st = {0};
  FILE *fd=NULL;
  static char outfile[MAX_FILENAME];
  char temp[MAX_FILENAME];
  char path[MAX_FILENAME];

  //Get current command
  alp_get_command(sm_p,&alp);

  //Open output file
  //--setup filename
  sprintf(outfile,"%s",ALP_FLAT_FILE);
  //--create output folder if it does not exist
  strcpy(temp,outfile);
  strcpy(path,dirname(temp));
  if (stat(path, &st) == -1){
    printf("ALP: creating folder %s\n",path);
    recursive_mkdir(path, 0777);
  }
  //--open file
  if((fd = fopen(outfile, "w")) == NULL){
    perror("ALP: alp_save_flat fopen()\n");
    return 1;
  }
  
  //Save flat
  if(fwrite(&alp,sizeof(alp_t),1,fd) != 1){
    printf("ALP: alp_save_flat fwrite error!\n");
    fclose(fd);
    return 1;
  }
  printf("ALP: Wrote: %s\n",outfile);

  //Close file
  fclose(fd);
  return 0;
}

/***************************************************************/
/* ALP_LOAD_FLAT                                               */
/*  - Loads ALP flat from file                                 */
/***************************************************************/
int alp_load_flat(sm_t *sm_p,int proc_id){
  FILE *fd=NULL;
  char filename[MAX_FILENAME];
  uint64 fsize,rsize;
  alp_t alp;
    
  //Open file
  //--setup filename
  sprintf(filename,ALP_FLAT_FILE);
  //--open file
  if((fd = fopen(filename,"r")) == NULL){
    perror("ALP: alp_load_flat fopen");
    return 0;
  }
  //--check file size
  fseek(fd, 0L, SEEK_END);
  fsize = ftell(fd);
  rewind(fd);
  rsize = sizeof(alp_t);
  if(fsize != rsize){
    printf("ALP: incorrect ALP_FLAT_FILE size %lu != %lu\n",fsize,rsize);
    fclose(fd);
    return 0;
  }
  
  //Read file
  if(fread(&alp,rsize,1,fd) != 1){
    perror("ALP: alp_load_flat fread");
    fclose(fd);
    return 0;
  }
  //Close file
  fclose(fd);
  printf("ALP: Read: %s\n",filename);

  //Send flat to ALP
  return(alp_send_command(sm_p,&alp,proc_id,1));
}

/**************************************************************/
/* ALP_CALIBRATE                                              */
/* - Run calibration routines for ALPAO DM                    */
/**************************************************************/
int alp_calibrate(int calmode, alp_t *alp, uint32_t *step, int procid, int reset){
  int i,j,index;
  static struct timespec start,this,last,delta;
  static double zernike_errors[LOWFS_N_ZERNIKE][ZERNIKE_ERRORS_NUMBER]={{0}};
  const double zernike_timestep = ZERNIKE_ERRORS_PERIOD;
  static int init=0;
  time_t t;
  FILE *fileptr=NULL;
  char filename[MAX_FILENAME];
  double dt=0,dt0=0,period=0;
  double zernikes[LOWFS_N_ZERNIKE]={0};
  double act[ALP_NACT];
  double poke=0,zpoke=0;
  int    ncalim=0;
  const double flat[ALP_NACT] = ALP_OFFSET;
  static int mode_init[ALP_NCALMODES] = {0};
  static alp_t alp_start[ALP_NCALMODES];
  static uint64 countA[ALP_NCALMODES] = {0};
  static uint64 countB[ALP_NCALMODES] = {0};

  /* Set calibration parameters */
  if(procid == SHKID){
    poke   = ALP_SHK_POKE;
    zpoke  = ALP_SHK_ZPOKE;
    ncalim = ALP_SHK_NCALIM;
  }
  if(procid == LYTID){
    poke   = ALP_LYT_POKE;
    zpoke  = ALP_LYT_ZPOKE;
    ncalim = ALP_LYT_NCALIM;
  }
  
  /* Reset */
  if(reset){
    memset(countA,0,sizeof(countA));
    memset(countB,0,sizeof(countB));
    memset(mode_init,0,sizeof(mode_init));
    memset(alp_start,0,sizeof(alp_start));
    init=0;
    return calmode;
  }

  /* Initialize */
  if(!init){
    memset(countA,0,sizeof(countA));
    memset(countB,0,sizeof(countB));
    memset(mode_init,0,sizeof(mode_init));
    memset(alp_start,0,sizeof(alp_start));
    clock_gettime(CLOCK_REALTIME, &start);

    /* Open zernike errors file */
    //--setup filename
    sprintf(filename,ZERNIKE_ERRORS_FILE);
    //--open file
    if((fileptr = fopen(filename,"r")) == NULL){
      perror("ALP: Zernike errors file: fopen");
      goto endofinit;
    }
    //--check file size
    fseek(fileptr, 0L, SEEK_END);
    if(ftell(fileptr) != sizeof(zernike_errors)){
      printf("ALP: incorrect zernike_errors file size (%lu) != expected (%lu)\n",ftell(fileptr),sizeof(zernike_errors));
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
    memset(countA,0,sizeof(countA));
    memset(countB,0,sizeof(countB));
    return calmode;
  }

  /* ALP_CALMODE_ZERO: Set all actuators to 0 */
  if(calmode==ALP_CALMODE_ZERO){
    //Reset counters
    memset(countA,0,sizeof(countA));
    memset(countB,0,sizeof(countB));
    //Set all ALP actuators to 0
    for(i=0;i<ALP_NACT;i++)
      alp->act_cmd[i]=0;
    return calmode;
  }

  /* ALP_CALMODE_FLAT: Set all actuators to flat map */
  if(calmode==ALP_CALMODE_FLAT){
    //Reset counters
    memset(countA,0,sizeof(countA));
    memset(countB,0,sizeof(countB));
    //Set all ALP actuators to flat
    for(i=0;i<ALP_NACT;i++)
      alp->act_cmd[i]=flat[i];
    return calmode;
  }

  /* ALP_CALMODE_POKE: Scan through acuators poking one at a time.    */
  /*                   Set to starting position in between each poke. */
  if(calmode == ALP_CALMODE_POKE){
    //Save alp starting position
    if(!mode_init[calmode]){
      memcpy(&alp_start[calmode],alp,sizeof(alp_t));
      mode_init[calmode]=1;
    }
    //Check counters
    if(countA[calmode] >= 0 && countA[calmode] < (2*ALP_NACT*ncalim)){
      //set all ALP actuators to starting position
      for(i=0;i<ALP_NACT;i++)
	alp->act_cmd[i]=alp_start[calmode].act_cmd[i];

      //set step counter
      *step = (countA[calmode]/ncalim);

      //poke one actuator
      if((countA[calmode]/ncalim) % 2 == 1){
	alp->act_cmd[(countB[calmode]/ncalim) % ALP_NACT] += poke;
	countB[calmode]++;
      }
      countA[calmode]++;
    }else{
      //Set alp back to starting position
      memcpy(alp,&alp_start[calmode],sizeof(alp_t));
      mode_init[calmode]=0;
      //Turn off calibration
      printf("ALP: Stopping ALP calmode ALP_CALMODE_POKE\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }
    return calmode;

  }

  /* ALP_CALMODE_ZPOKE: Poke Zernikes one at a time                    */
  /*                    Set to starting position in between each poke. */
  if(calmode == ALP_CALMODE_ZPOKE){
    //Save alp starting position
    if(!mode_init[calmode]){
      memcpy(&alp_start[calmode],alp,sizeof(alp_t));
      mode_init[calmode]=1;
    }
    //Check counters
    if(countA[calmode] >= 0 && countA[calmode] < (2*LOWFS_N_ZERNIKE*ncalim)){
      //set all Zernikes to zero
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	alp->zernike_cmd[i] = 0.0;

      //set all ALP actuators to starting position
      for(i=0;i<ALP_NACT;i++)
	alp->act_cmd[i]=alp_start[calmode].act_cmd[i];

      //set step counter
      *step = (countA[calmode]/ncalim);

      //poke one zernike by adding it on top of the flat
      if((countA[calmode]/ncalim) % 2 == 1){
	alp->zernike_cmd[(countB[calmode]/ncalim) % LOWFS_N_ZERNIKE] = zpoke;
	alp_zern2alp(alp->zernike_cmd,act,FUNCTION_NO_RESET);
	for(i=0; i<ALP_NACT; i++)
	  alp->act_cmd[i] += act[i];
	countB[calmode]++;
      }
      countA[calmode]++;
    }else{
      //Set alp back to starting position
      memcpy(alp,&alp_start[calmode],sizeof(alp_t));
      mode_init[calmode]=0;
      //Turn off calibration
      printf("ALP: Stopping calmode ALP_CALMODE_ZPOKE\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }
    return calmode;
  }

  /* ALP_CALMODE_RAMP: Scan through acuators ramping one at a time.    */
  /*                   Set to starting position in between each ramp. */
  if(calmode == ALP_CALMODE_RAMP){
    //Save alp starting position
    if(!mode_init[calmode]){
      memcpy(&alp_start[calmode],alp,sizeof(alp_t));
      mode_init[calmode]=1;
    }
    //Check counters
    if(countA[calmode] >= 0 && countA[calmode] < (2*ALP_NACT*ncalim)){
      //set all ALP actuators to starting position -- only the first interation
      if((countB[calmode] % ncalim) == 0)
	for(i=0;i<ALP_NACT;i++)
	  alp->act_cmd[i]=alp_start[calmode].act_cmd[i];

      //set step counter
      *step = (countA[calmode]/ncalim);

      //ramp one actuator
      if((countA[calmode]/ncalim) % 2 == 1){
	alp->act_cmd[(countB[calmode]/ncalim) % ALP_NACT] += 5*poke/ncalim;
	countB[calmode]++;
      }
      countA[calmode]++;
    }else{
      //Set alp back to starting position
      memcpy(alp,&alp_start[calmode],sizeof(alp_t));
      mode_init[calmode]=0;
      //Turn off calibration
      printf("ALP: Stopping ALP calmode ALP_CALMODE_RAMP\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }
    return calmode;

  }

  /* ALP_CALMODE_ZRAMP: Ramp Zernikes one at a time                    */
  /*                    Set to starting position in between each ramp. */
  if(calmode == ALP_CALMODE_ZRAMP){
    //Save alp starting position
    if(!mode_init[calmode]){
      memcpy(&alp_start[calmode],alp,sizeof(alp_t));
      mode_init[calmode]=1;
    }
    //Check counters
    if(countA[calmode] >= 0 && countA[calmode] < (2*LOWFS_N_ZERNIKE*ncalim)){
      //set all Zernikes to zero -- only the first interation
      if((countB[calmode] % ncalim) == 0)
	for(i=0;i<LOWFS_N_ZERNIKE;i++)
	  alp->zernike_cmd[i] = 0.0;

      //set all ALP actuators to starting position
      for(i=0;i<ALP_NACT;i++)
	alp->act_cmd[i]=alp_start[calmode].act_cmd[i];

      //set step counter
      *step = (countA[calmode]/ncalim);

      //ramp one zernike by adding it on top of the flat
      if((countA[calmode]/ncalim) % 2 == 1){
	alp->zernike_cmd[(countB[calmode]/ncalim) % LOWFS_N_ZERNIKE] += 5*zpoke/ncalim;
	alp_zern2alp(alp->zernike_cmd,act,FUNCTION_NO_RESET);
	for(i=0; i<ALP_NACT; i++)
	  alp->act_cmd[i] += act[i];
	countB[calmode]++;
      }
      countA[calmode]++;
    }else{
      //Set alp back to starting position
      memcpy(alp,&alp_start[calmode],sizeof(alp_t));
      mode_init[calmode]=0;
      //Turn off calibration
      printf("ALP: Stopping calmode ALP_CALMODE_ZRAMP\n");
      calmode = ALP_CALMODE_NONE;
      init = 0;
    }
    return calmode;
  }

  /* ALP_CALMODE_FLIGHT: Flight Simulator */
  if(calmode == ALP_CALMODE_FLIGHT){
    //Save alp starting position
    if(!mode_init[calmode]){
      memcpy(&alp_start[calmode],alp,sizeof(alp_t));
      mode_init[calmode]=1;
    }
    //Setup counters
    if(countA[calmode] == 0)
      dt0 = dt;
    if(countA[calmode] == 1)
      period = dt-dt0;
    //Set step counter
    *step = countA[calmode];
    //Set index
    index = (int)((dt-dt0)/zernike_timestep);
    if(index < ZERNIKE_ERRORS_NUMBER){
      //Get zernikes for this timestep
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	zernikes[i] = zernike_errors[i][index % ZERNIKE_ERRORS_NUMBER];
      //Convert zernikes to actuators
      alp_zern2alp(zernikes,act,FUNCTION_NO_RESET);
      //Add offsets to ALP position
      for(i=0;i<ALP_NACT;i++)
	alp->act_cmd[i] += act[i];
      countA[calmode]++;
    }else{
      //Set alp back to starting position
      memcpy(alp,&alp_start[calmode],sizeof(alp_t));
      mode_init[calmode]=0;
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
