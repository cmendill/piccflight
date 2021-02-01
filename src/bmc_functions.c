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
#include "bmc_functions.h"

/**************************************************************/
/* BMC_INIT_CALMODE                                           */
/*  - Initialize BMC calmode structure                        */
/**************************************************************/
void bmc_init_calmode(int calmode, calmode_t *bmc){
  int i;
  
  //DEFAULTS
  bmc->sci_ncalim = BMC_SCI_NCALIM;
  bmc->sci_poke   = BMC_SCI_POKE;
   
  //BMC_CALMODE_NONE
  if(calmode == BMC_CALMODE_NONE){
    sprintf(bmc->name,"BMC_CALMODE_NONE");
    sprintf(bmc->cmd,"none");
  }
  //BMC_CALMODE_TIMER
  if(calmode == BMC_CALMODE_TIMER){
    sprintf(bmc->name,"BMC_CALMODE_TIMER");
    sprintf(bmc->cmd,"timer");
  }
  //BMC_CALMODE_POKE
  if(calmode == BMC_CALMODE_POKE){
    sprintf(bmc->name,"BMC_CALMODE_POKE");
    sprintf(bmc->cmd,"poke");
  }
  //BMC_CALMODE_RAND
  if(calmode == BMC_CALMODE_RAND){
    sprintf(bmc->name,"BMC_CALMODE_RAND");
    sprintf(bmc->cmd,"rand");
  }
  //BMC_CALMODE_PROBE
  if(calmode == BMC_CALMODE_PROBE){
    sprintf(bmc->name,"BMC_CALMODE_PROBE");
    sprintf(bmc->cmd,"probe");
  }
}

/**************************************************************/
/* BMC_GET_COMMAND                                            */
/* - Function to get the last command sent to the BMC DM      */
/* - Use atomic operations to prevent two processes from      */
/*   accessing the commands at the same time                  */
/**************************************************************/
int bmc_get_command(sm_t *sm_p, bmc_t *cmd){
  int retval = 1;
  
  //Atomically test and set BMC command lock using GCC built-in function
  if(__sync_lock_test_and_set(&sm_p->bmc_command_lock,1)==0){
    //Copy command
    memcpy(cmd,(bmc_t *)&sm_p->bmc_command,sizeof(bmc_t));
    //Release lock
    __sync_lock_release(&sm_p->bmc_command_lock);
    //Return 0 on success
    retval = 0;
  }
  
  //Return
  return retval;
}

/**************************************************************/
/* BMC_SEND_COMMAND                                           */
/* - Function to command the BMC DM                           */
/* - Use atomic operations to prevent two processes from      */
/*   sending commands at the same time                        */
/* - Return 0 if the command was sent and 1 if it wasn't      */
/**************************************************************/
int bmc_send_command(sm_t *sm_p, bmc_t *cmd, int proc_id){
  int retval = 1;
  
  //Atomically test and set BMC command lock using GCC built-in function
  if(__sync_lock_test_and_set(&sm_p->bmc_command_lock,1)==0){
    
    //Check if the commanding process is the BMC commander
    if(proc_id == sm_p->state_array[sm_p->state].bmc_commander){
      
      //Send the command
      if(!libbmc_set_acts_tstpnts(sm_p->libbmc_device, cmd->acmd, cmd->tcmd)){
	//Copy command to current position
	memcpy((bmc_t *)&sm_p->bmc_command,cmd,sizeof(bmc_t));
	//Set retval for good command
	retval = 0;
      }
    }
    
    //Release lock
    __sync_lock_release(&sm_p->bmc_command_lock);
  }

  //Return
  return retval;
}

/**************************************************************/
/* BMC_SET_BIAS                                               */
/* - Set all actuators to the same value                      */
/**************************************************************/
int bmc_set_bias(sm_t *sm_p, float bias, int proc_id){
  bmc_t bmc={};
  int i;
  
  //Set bias
  for(i=0;i<BMC_NACT;i++)
    bmc.acmd[i] = bias;
  
  //Send command
  return(bmc_send_command(sm_p,&bmc,proc_id));
}

/**************************************************************/
/* BMC_SET_RANDOM                                             */
/* - Add a random perturbation to the BMC                     */
/**************************************************************/
int bmc_set_random(sm_t *sm_p, int proc_id){
  bmc_t bmc;
  time_t t;
  int i;
  
  //Init random numbers
  srand((unsigned) time(&t));
  
  //Get current command
  if(bmc_get_command(sm_p,&bmc))
    return 1;

  //Add perturbation
  for(i=0;i<BMC_NACT;i++)
    bmc.acmd[i] += (2*(rand() / (double) RAND_MAX) - 1) * BMC_SCI_POKE;

  //Send command
  return(bmc_send_command(sm_p,&bmc,proc_id));
}

/**************************************************************/
/* BMC_ZERO_FLAT                                              */
/* - Set BMC to all zeros                                     */
/**************************************************************/
int bmc_zero_flat(sm_t *sm_p, int proc_id){
  bmc_t bmc;
  memset(&bmc,0,sizeof(bmc_t));
  return(bmc_send_command(sm_p,&bmc,proc_id));
}

/**************************************************************/
/* BMC_REVERT_FLAT                                            */
/* - Set BMC to #defined flat map                             */
/**************************************************************/
int bmc_revert_flat(sm_t *sm_p, int proc_id){
  bmc_t bmc;
  const float flat[BMC_NACT] = BMC_OFFSET;
  memset(&bmc,0,sizeof(bmc_t));
  memcpy(bmc.acmd,flat,sizeof(bmc.acmd));
  return(bmc_send_command(sm_p,&bmc,proc_id,1));
}

/**************************************************************/
/* BMC_SAVE_FLAT                                              */
/* - Save current BMC position to file                        */
/**************************************************************/
int bmc_save_flat(sm_t *sm_p){
  bmc_t bmc;
  struct stat st = {0};
  FILE *fd=NULL;
  static char outfile[MAX_FILENAME];
  char temp[MAX_FILENAME];
  char path[MAX_FILENAME];

  //Get current command
  if(bmc_get_command(sm_p,&bmc))
    return 1;

  //Clear zernike commands
  memset(bmc.zcmd,0,sizeof(bmc.zcmd));

  //Open output file
  //--setup filename
  sprintf(outfile,"%s",BMC_FLAT_FILE);
  //--create output folder if it does not exist
  strcpy(temp,outfile);
  strcpy(path,dirname(temp));
  if (stat(path, &st) == -1){
    printf("BMC: creating folder %s\n",path);
    recursive_mkdir(path, 0777);
  }
  //--open file
  if((fd = fopen(outfile, "w")) == NULL){
    perror("BMC: bmc_save_flat fopen()\n");
    return 1;
  }
  
  //Save flat
  if(fwrite(&bmc,sizeof(bmc_t),1,fd) != 1){
    printf("BMC: bmc_save_flat fwrite error!\n");
    fclose(fd);
    return 1;
  }
  printf("BMC: Wrote: %s\n",outfile);

  //Close file
  fclose(fd);
  return 0;
}

/***************************************************************/
/* BMC_LOAD_FLAT                                               */
/*  - Loads BMC flat from file                                 */
/***************************************************************/
int bmc_load_flat(sm_t *sm_p,int proc_id){
  FILE *fd=NULL;
  char filename[MAX_FILENAME];
  uint64 fsize,rsize;
  bmc_t bmc;
    
  //Open file
  //--setup filename
  sprintf(filename,BMC_FLAT_FILE);
  //--open file
  if((fd = fopen(filename,"r")) == NULL){
    perror("BMC: bmc_load_flat fopen");
    return 0;
  }
  //--check file size
  fseek(fd, 0L, SEEK_END);
  fsize = ftell(fd);
  rewind(fd);
  rsize = sizeof(bmc_t);
  if(fsize != rsize){
    printf("BMC: incorrect BMC_FLAT_FILE size %lu != %lu\n",fsize,rsize);
    fclose(fd);
    return 0;
  }
  
  //Read file
  if(fread(&bmc,rsize,1,fd) != 1){
    perror("BMC: bmc_load_flat fread");
    fclose(fd);
    return 0;
  }

  //Close file
  fclose(fd);
  printf("BMC: Read: %s\n",filename);

  //Clear zernike commands
  memset(bmc.zcmd,0,sizeof(bmc.zcmd));

  //Send flat to BMC
  return(bmc_send_command(sm_p,&bmc,proc_id,1));
}


/**************************************************************/
/* BMC_INIT_CALIBRATION                                       */
/* - Initialize calibration structure                         */
/**************************************************************/
void bmc_init_calibration(sm_t *sm_p){
  int i;
  FILE *fd=NULL;
  char filename[MAX_FILENAME];

  //Zero out calibration struct
  memset((void *)&sm_p->bmccal,0,sizeof(bmccal_t));

  /* Initialize other elements */
  sm_p->bmccal.timer_length = CALMODE_TIMER_SEC;
  sm_p->bmccal.command_scale = 1;
  
  
}

/**************************************************************/
/* BMC_ADD_NM                                                 */
/* - Add NM command to current voltage command                */
/**************************************************************/
void bmc_add_nm(float *v, double *dn){
  int i;
  double a[BMC_NACT]={0};
  double b[BMC_NACT]={0};
  double n;
  static int init=0;
  //n = a*v^2 + b*v
  //0 = a*v^2 + b*v -n
  //v = (sqrt(b^2 + 4*a*n) - b) / 2*a
  if(!init){
    if(read_file(BMC_CAL_A_FILE,a,sizeof(a)))
      memset(a,0,sizeof(a));
    if(read_file(BMC_CAL_B_FILE,b,sizeof(b)))
      memset(b,0,sizeof(b));
    init=1;
  }
  for(i=0;i<BMC_NACT;i++){
    n    = a[i]*v[i]^2 + b[i]*v[i];
    n   += dn[i];
    v[i] = (sqrt(b[i]^2 + 4*a[i]*n) - b[i]) / (2*a[i]);
  }
  return;
}



/**************************************************************/
/* BMC_CALIBRATE                                              */
/* - Run calibration routines for BMC DM                      */
/**************************************************************/
int bmc_calibrate(sm_t *sm_p, int calmode, bmc_t *bmc, uint32_t *step, int procid, int reset){
  static int init=0;
  static double arand[BMC_NACT]={0};
  static calmode_t bmccalmodes[BMC_NCALMODES];
  uint64_t i,j,z,index;
  struct timespec this,delta;
  time_t trand;
  double dt,step_fraction;
  double act[BMC_NACT];
  double poke=0;
  int    ncalim=0;
  double probe[BMC_NACT][HOWFS_NPROBE];
  char   filename[MAX_FILENAME];
  
  /* Reset & Quick Init*/
  if(reset || !init){
    //Reset counters
    memset((void *)sm_p->bmccal.countA,0,sizeof(sm_p->bmccal.countA));
    memset((void *)sm_p->bmccal.countB,0,sizeof(sm_p->bmccal.countB));
    //Reset random numbers
    srand((unsigned) time(&trand));
    for(i=0;i<BMC_NACT;i++) arand[i] = (2*(rand() / (double) RAND_MAX) - 1);
    //Init BMC calmodes
    for(i=0;i<BMC_NCALMODES;i++)
      bmc_init_calmode(i,&bmccalmodes[i]);
    //Read probes
    for(i=0;i<SCI_HOWFS_NPROBE;i++){
      sprintf(filename,HOWFS_PROBE_FILE,i);
      if(read_file(filename,&probe[0][i])){
	memset(&probe[0][0],0,sizeof(probe));
	break;
      }
    }
    init = 1;
    if(reset) return calmode;
  }
  
  /* Set calibration parameters */
  if(procid == SCIID){
    poke   = bmccalmodes[calmode].sci_poke;
    ncalim = bmccalmodes[calmode].sci_ncalim;
  }
    
  /* Get time */
  clock_gettime(CLOCK_REALTIME, &this);

  /* Init times and BMC commands */
  if((calmode != BMC_CALMODE_NONE) && (sm_p->bmccal.countA[calmode] == 0)){
    //Save start time
    memcpy((struct timespec *)&sm_p->bmccal.start[calmode],&this,sizeof(struct timespec));
    //Save bmc starting position
    memcpy((bmc_t *)&sm_p->bmccal.bmc_start[calmode],bmc,sizeof(bmc_t));
  }
  
  /* BMC_CALMODE_NONE: Do nothing. Just reset counters.*/
  if(calmode==BMC_CALMODE_NONE){
    //Reset counters
    memset((void *)sm_p->bmccal.countA,0,sizeof(sm_p->bmccal.countA));
    memset((void *)sm_p->bmccal.countB,0,sizeof(sm_p->bmccal.countB));

    //Return calmode
    return calmode;
  }
  
  /* BMC_CALMODE_TIMER: Do nothing. End after a defined amount of time     */
  if(calmode==BMC_CALMODE_TIMER){
    //Get time delta
    if(timespec_subtract(&delta,&this,(struct timespec *)&sm_p->bmccal.start[calmode]))
      printf("BMC: bmc_calibrate --> timespec_subtract error!\n");
    ts2double(&delta,&dt);
    
    if(dt > sm_p->bmccal.timer_length){
      //Turn off calibration
      printf("BMC: Stopping BMC calmode BMC_CALMODE_TIMER\n");
      calmode = BMC_CALMODE_NONE;
      init = 0;
    }
    
    //Set step counter
    *step = sm_p->bmccal.countA[calmode];
    
    //Increment counter
    sm_p->bmccal.countA[calmode]++;
    
    //Return calmode
    return calmode;
  }
  
  /* BMC_CALMODE_POKE: Scan through acuators poking one at a time.    */
  /*                   Set to starting position in between each poke. */
  if(calmode==BMC_CALMODE_POKE){
    //Check counters
    if(sm_p->bmccal.countA[calmode] >= 0 && sm_p->bmccal.countA[calmode] < (2*BMC_NACT*ncalim)){
      //Set all BMC actuators to starting position
      for(i=0;i<BMC_NACT;i++)
	bmc->acmd[i]=sm_p->bmccal.bmc_start[calmode].acmd[i];
      
      //Poke one actuator
      if((sm_p->bmccal.countA[calmode]/ncalim) % 2 == 1){
	bmc->acmd[(sm_p->bmccal.countB[calmode]/ncalim) % BMC_NACT] += poke * sm_p->bmccal.command_scale;
	sm_p->bmccal.countB[calmode]++;
      }
    }else{
      //Set bmc back to starting position
      memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      //Turn off calibration
      printf("BMC: Stopping BMC calmode BMC_CALMODE_POKE\n");
      calmode = BMC_CALMODE_NONE;
      init = 0;
    }
    
    //Set step counter
    *step = (sm_p->bmccal.countA[calmode]/ncalim);
    
    //Increment counter
    sm_p->bmccal.countA[calmode]++;
    
    //Return calmode
    return calmode;
    
  }

  /* BMC_CALMODE_RAND: Poke all actuators by a random amount           */
  if(calmode == BMC_CALMODE_RAND){
    //Check counters
    if(sm_p->bmccal.countA[calmode] >= 0 && sm_p->bmccal.countA[calmode] < (2*ncalim)){
      //Poke all actuators by random amount (just once)
      if(sm_p->bmccal.countA[calmode] == ncalim){
	for(i=0; i<BMC_NACT; i++)
	  bmc->acmd[i] = sm_p->bmccal.bmc_start[calmode].acmd[i] + poke * arand[i] * sm_p->bmccal.command_scale;
      }
    }else{
      //Set bmc back to starting position
      memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      //Turn off calibration
      printf("BMC: Stopping calmode BMC_CALMODE_RAND\n");
      calmode = BMC_CALMODE_NONE;
      init = 0;
    }

    //Set step counter
    *step = (sm_p->bmccal.countA[calmode]/ncalim);
    
    //Increment counter
    sm_p->bmccal.countA[calmode]++;

    //Return calmode
    return calmode;
  }

  /* BMC_CALMODE_PROBE: Step through the HOWFS probe patterns         */
  if(calmode==BMC_CALMODE_PROBE){
    //Set step counter
    *step = (sm_p->bmccal.countA[calmode]/ncalim);

    //Check counters
    if(sm_p->bmccal.countA[calmode] >= 0 && sm_p->bmccal.countA[calmode] < ncalim*SCI_HOWFS_NPROBE){
      //Set all BMC actuators to starting position
      memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      //Add probe pattern
      bmc_add_nm(&bmc->acmd,probe[0][*step]);
    }else{
      //Set bmc back to starting position
      memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      //Turn off calibration
      printf("BMC: Stopping BMC calmode BMC_CALMODE_PROBE\n");
      calmode = BMC_CALMODE_NONE;
      init = 0;
    }
    
    //Increment counter
    sm_p->bmccal.countA[calmode]++;
    
    //Return calmode
    return calmode;
    
  }

  /* Unknown calmode (impossible) */
  printf("BMC: Unknown calmode\n");
  calmode = BMC_CALMODE_NONE;
  init = 0;
  
  //Return calmode
  return calmode;
}
