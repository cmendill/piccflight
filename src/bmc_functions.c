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
/* BMC_FUNCTION_RESET                                         */
/* - Reset all BMC functions                                  */
/**************************************************************/
void bmc_function_reset(sm_t *sm_p){
  bmc_rotate_command(NULL, FUNCTION_RESET_RETURN);
  bmc_add_length(NULL, NULL, NULL, FUNCTION_RESET_RETURN);
  bmc_calibrate(sm_p, 0, NULL, NULL, 0, 0, 0, FUNCTION_RESET_RETURN);
}

/**************************************************************/
/* BMC_INIT_CALMODE                                           */
/*  - Initialize BMC calmode structure                        */
/**************************************************************/
void bmc_init_calmode(int calmode, calmode_t *bmc){
  int i;
  
  //DEFAULTS
  bmc->sci_ncalim = BMC_SCI_NCALIM;
  bmc->sci_poke   = BMC_SCI_POKE;
  bmc->sci_vpoke  = BMC_SCI_VPOKE;
   
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
  //BMC_CALMODE_VPOKE
  if(calmode == BMC_CALMODE_VPOKE){
    sprintf(bmc->name,"BMC_CALMODE_VPOKE");
    sprintf(bmc->cmd,"vpoke");
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
  //BMC_CALMODE_SINE
  if(calmode == BMC_CALMODE_SINE){
    sprintf(bmc->name,"BMC_CALMODE_SINE");
    sprintf(bmc->cmd,"sine");
  }
  //BMC_CALMODE_SINE_DIFF
  if(calmode == BMC_CALMODE_SINE_DIFF){
    sprintf(bmc->name,"BMC_CALMODE_SINE_DIFF");
    sprintf(bmc->cmd,"sdiff");
  }
}

/**************************************************************/
/* BMC_LIMIT_COMMAND                                          */
/* - Limit BMC command to voltage limits                      */
/**************************************************************/
void bmc_limit_command(bmc_t *cmd){
  int i;

  for(i=0;i<BMC_NACT;i++){
    cmd->acmd[i] = cmd->acmd[i] < BMC_VMIN ? BMC_VMIN : cmd->acmd[i];
    cmd->acmd[i] = cmd->acmd[i] > BMC_VMAX ? BMC_VMAX : cmd->acmd[i];
  }
  for(i=0;i<BMC_NTEST;i++){
    cmd->tcmd[i] = cmd->tcmd[i] < BMC_VMIN ? BMC_VMIN : cmd->tcmd[i];
    cmd->tcmd[i] = cmd->tcmd[i] > BMC_VMAX ? BMC_VMAX : cmd->tcmd[i];
  }
}

/**************************************************************/
/* BMC_ROTATE_COMMAND                                         */
/* - Apply BMC command rotation                               */
/**************************************************************/
void bmc_rotate_command(bmc_t *cmd, int reset){
  static int16_t bmcrot[BMC_NACT]={0};
  static int init=0;
  bmc_t rot;
  int i;
  if(!init || reset){
    if(read_file(BMC_ROTATE_FILE,bmcrot,sizeof(bmcrot)))
      memset(bmcrot,0,sizeof(bmcrot));
    init=1;
    printf("BMC: Loaded rotation file\n");
    if(reset == FUNCTION_RESET_RETURN) return;
  }
  
  for(i=0;i<BMC_NACT;i++)
    rot.acmd[bmcrot[i]] = cmd->acmd[i];
  for(i=0;i<BMC_NTEST;i++)
    rot.tcmd[i] = cmd->tcmd[i];

  memcpy(cmd,&rot,sizeof(bmc_t));
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
/* BMC_GET_FLAT                                               */
/* - Function to get the last flat sent to the BMC DM         */
/* - Use atomic operations to prevent two processes from      */
/*   accessing the commands at the same time                  */
/* - iflat==0 --> the last flat written                       */
/* - iflat==1 --> the second to last flat written             */
/**************************************************************/
int bmc_get_flat(sm_t *sm_p, bmc_t *cmd, int iflat){
  int retval = 1;
  int bmc_iflat = (sm_p->bmc_iflat - iflat) % BMC_NFLAT;
  if((bmc_iflat >= BMC_NFLAT) || (bmc_iflat < 0) || (((int64_t)sm_p->bmc_iflat - iflat) < 0)){
    printf("BMC: Bad iflat argument %d\n",iflat);
    return 1;
  }
  //printf("BMC: iflat = %d  |  index = %d  | counter = %d\n",iflat,bmc_iflat,sm_p->bmc_iflat);
  
  
  //Atomically test and set BMC command lock using GCC built-in function
  if(__sync_lock_test_and_set(&sm_p->bmc_command_lock,1)==0){
    //Copy flat
    memcpy(cmd,(bmc_t *)&sm_p->bmc_flat[bmc_iflat],sizeof(bmc_t));
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
int bmc_send_command(sm_t *sm_p, bmc_t *cmd, int proc_id, int set_flat){
  int retval = 1;
  int i;
  bmc_t bmc_rotate;

  //Set test points -- preserved in command
  for(i=0;i<BMC_NTEST;i++)
    cmd->tcmd[i] = 100 + i;
  
  //Apply command limits -- preserved in command
  bmc_limit_command(cmd);
  
  //Apply rotation -- not preserved in command
  memcpy(&bmc_rotate,cmd,sizeof(bmc_t));
  bmc_rotate_command(&bmc_rotate,FUNCTION_NO_RESET);
  
  //Check if controller is ready
  if(sm_p->bmc_ready && sm_p->bmc_hv_on){

    //Atomically test and set BMC command lock using GCC built-in function
    if(__sync_lock_test_and_set(&sm_p->bmc_command_lock,1)==0){
    
      //Check if the commanding process is the BMC commander
      if(proc_id == sm_p->state_array[sm_p->state].bmc_commander){
      
	//Send the command
	if(!libbmc_set_acts_tstpnts((libbmc_device_t *)&sm_p->libbmc_device, bmc_rotate.acmd, bmc_rotate.tcmd)){
	  //Copy command to current position
	  memcpy((bmc_t *)&sm_p->bmc_command,cmd,sizeof(bmc_t));
	  //Set flat
	  if(set_flat == BMC_SET_FLAT) memcpy((bmc_t *)&sm_p->bmc_flat[++sm_p->bmc_iflat % BMC_NFLAT],cmd,sizeof(bmc_t));
	  if(set_flat == BMC_OW_FLAT)  memcpy((bmc_t *)&sm_p->bmc_flat[sm_p->bmc_iflat % BMC_NFLAT],cmd,sizeof(bmc_t));
	  //Set retval for good command
	  retval = 0;
	}
      }
    
      //Release lock
      __sync_lock_release(&sm_p->bmc_command_lock);
    }
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
  return(bmc_send_command(sm_p,&bmc,proc_id,BMC_NOSET_FLAT));
}

/**************************************************************/
/* BMC_SET_RANDOM                                             */
/* - Add a random perturbation to the BMC                     */
/**************************************************************/
int bmc_set_random(sm_t *sm_p, int proc_id){
  bmc_t bmc;
  double dl[BMC_NACT];
  time_t t;
  int i;
  
  //Init random numbers
  srand((unsigned) time(&t));
  
  //Get current command (volts)
  if(bmc_get_command(sm_p,&bmc))
    return 1;

  //Get random perturbations (length)
  for(i=0;i<BMC_NACT;i++)
    dl[i] = (2*(rand() / (double) RAND_MAX) - 1) * BMC_SCI_POKE;

  //Add perturbations to command
  bmc_add_length(bmc.acmd,bmc.acmd,dl,FUNCTION_NO_RESET);
  
  //Send command
  return(bmc_send_command(sm_p,&bmc,proc_id,BMC_NOSET_FLAT));
}

/**************************************************************/
/* BMC_ZERO_FLAT                                              */
/* - Set BMC to all zeros                                     */
/**************************************************************/
int bmc_zero_flat(sm_t *sm_p, int proc_id){
  bmc_t bmc;
  memset(&bmc,0,sizeof(bmc_t));
  return(bmc_send_command(sm_p,&bmc,proc_id,BMC_SET_FLAT));
}

/***************************************************************/
/* BMC_REVERT_FLAT                                             */
/*  - Loads BMC flat from default file                         */
/***************************************************************/
int bmc_revert_flat(sm_t *sm_p,int proc_id){
  bmc_t bmc;

  //Read flat file
  read_file(BMC_DEFAULT_FILE,bmc.acmd,sizeof(bmc.acmd));
  
  //Send flat to BMC
  return(bmc_send_command(sm_p,&bmc,proc_id,BMC_SET_FLAT));
}


/**************************************************************/
/* BMC_SAVE_FLAT                                              */
/* - Save current BMC position to file                        */
/**************************************************************/
int bmc_save_flat(sm_t *sm_p){
  bmc_t bmc;

  //Get current flat
  if(bmc_get_flat(sm_p,&bmc,sm_p->bmc_iflat))
    return 1;

  //Save command to file
  check_and_mkdir(BMC_FLAT_FILE);
  write_file(BMC_FLAT_FILE,&bmc,sizeof(bmc_t));
    printf("BMC: Wrote: %s\n",BMC_FLAT_FILE);

  return 0;
}

/***************************************************************/
/* BMC_LOAD_FLAT                                               */
/*  - Loads BMC flat from file                                 */
/***************************************************************/
int bmc_load_flat(sm_t *sm_p,int proc_id){
  bmc_t bmc;

  //Read flat file
  read_file(BMC_FLAT_FILE,&bmc,sizeof(bmc));
  
  //Send flat to BMC
  return(bmc_send_command(sm_p,&bmc,proc_id,BMC_SET_FLAT));
}

/**************************************************************/
/* BMC_RECALL_FLAT                                            */
/* - Recall previous flat                                     */
/**************************************************************/
int bmc_recall_flat(sm_t *sm_p,int proc_id, int iflat){
  bmc_t bmc;
  
  //Get current flat
  if(bmc_get_flat(sm_p,&bmc,iflat))
    return 1;
  
  //Send flat to BMC
  return(bmc_send_command(sm_p,&bmc,proc_id,BMC_OW_FLAT));

  return 0;
}

/**************************************************************/
/* BMC_INIT_CALIBRATION                                       */
/* - Initialize calibration structure                         */
/**************************************************************/
void bmc_init_calibration(sm_t *sm_p){
  
  //Zero out calibration struct
  memset((void *)&sm_p->bmccal,0,sizeof(bmccal_t));
  
}

/**************************************************************/
/* BMC_ADD_LENGTH                                             */
/* - Add length command to current voltage command            */
/* - Output can be same pointer as input                      */
/**************************************************************/
void bmc_add_length(float *input, float *output, double *dl, int reset){
  int i;
  static double a[BMC_NACT]={0};
  static double b[BMC_NACT]={0};
  static double polarity=0;
  double l;
  static int init=0;
  //l = a*v^2 + b*v
  //0 = a*v^2 + b*v - l
  //v = (sqrt(b^2 + 4*a*l) - b) / 2*a
  if(!init || reset){
    if(read_file(BMC_CAL_A_FILE,a,sizeof(a)))
      memset(a,0,sizeof(a));
    if(read_file(BMC_CAL_B_FILE,b,sizeof(b)))
      memset(b,0,sizeof(b));
    if(read_file(BMC_POLARITY_FILE,&polarity,sizeof(polarity)))
      polarity = 0;
    init=1;
    printf("BMC: Loaded calibration file\n");
    printf("BMC: Polarity is %f\n",polarity);
    if(reset == FUNCTION_RESET_RETURN) return;
  }
  for(i=0;i<BMC_NACT;i++){
    l    = a[i]*(double)input[i]*(double)input[i] + b[i]*(double)input[i];
    l   += polarity * dl[i];
    output[i] = (float)((sqrt(b[i]*b[i] + 4*a[i]*l) - b[i]) / (2*a[i]));
  }
  return;
}

/**************************************************************/
/* BMC_ADD_PROBE                                              */
/* - Add PROBE pattern to BMC command                         */
/* - Output can be same pointer as input                      */
/**************************************************************/
void bmc_add_probe(float *input, float *output, int dhrot, int istep, int amp){
  double dl[BMC_NACT]={0};
  char filename[MAX_FILENAME];
  int i;

  //Read command file
  sprintf(filename,BMC_PROBE_FILE,istep,dhrot,amp);
  if(read_file(filename,dl,sizeof(dl)))
    memset(dl,0,sizeof(dl));

  //Add to flat
  bmc_add_length(input,output,dl,FUNCTION_NO_RESET);
  return;
}

/**************************************************************/
/* BMC_ADD_TEST                                               */
/* - Add TEST pattern to BMC command                          */
/* - Output can be same pointer as input                      */
/**************************************************************/
void bmc_add_test(float *input, float *output,int istep, double scale){
  double dl[BMC_NACT]={0};
  char filename[MAX_FILENAME];
  int i;

  //Read command file
  sprintf(filename,BMC_TEST_FILE,istep);
  if(read_file(filename,dl,sizeof(dl)))
    memset(dl,0,sizeof(dl));

  //Apply scale factor
  if(scale != 1)
    for(i=0;i<BMC_NACT;i++)
      dl[i] *= scale;
  
  //Add to flat
  bmc_add_length(input,output,dl,FUNCTION_NO_RESET);
  return;
}

/**************************************************************/
/* BMC_ADD_SINE                                               */
/* - Add SINE pattern to BMC command                          */
/* - Output can be same pointer as input                      */
/**************************************************************/
void bmc_add_sine(float *input, float *output,int istep, double scale){
  double dl[BMC_NACT]={0};
  char filename[MAX_FILENAME];
  int i;

  //Read command file
  sprintf(filename,BMC_SINE_FILE,istep);
  if(read_file(filename,dl,sizeof(dl)))
    memset(dl,0,sizeof(dl));

  //Apply scale factor
  if(scale != 1)
    for(i=0;i<BMC_NACT;i++)
      dl[i] *= scale;
  
  //Add to flat
  bmc_add_length(input,output,dl,FUNCTION_NO_RESET);
  return;
}

/**************************************************************/
/* BMC_ADD_SPECKLE                                            */
/* - Add SINE pattern to BMC command for speckle nulling     */
/* - Output can be same pointer as input                      */
/**************************************************************/
void bmc_add_speckle(float *input, float *output, double amp, double phi, double ang, double freq){
  static int init=0;
  static double bmcx[BMC_NACT];
  static double bmcy[BMC_NACT];
  char filename[MAX_FILENAME];
  double dl[BMC_NACT]={0};
  double xrot;
  int i;
  
  //Initialize
  if(!init){
    sprintf(filename,BMC_X_FILE);
    if(read_file(filename,bmcx,sizeof(bmcx)))
      memset(bmcx,0,sizeof(bmcx));
    sprintf(filename,BMC_Y_FILE);
    if(read_file(filename,bmcy,sizeof(bmcy)))
      memset(bmcy,0,sizeof(bmcy));
    init=1;
  }

  //Calculate sine wave for input speckle
  for(i=0;i<BMC_NACT;i++){
    xrot  = bmcx[i]*cos(ang) + bmcy[i]*sin(ang); //rotation matrix (coordinate transform)
    dl[i] = amp * sin(2 * M_PI * freq * xrot + phi);
  }
  
  //Add to flat
  bmc_add_length(input,output,dl,FUNCTION_NO_RESET);
  return;
}

/**************************************************************/
/* BMC_CALIBRATE                                              */
/* - Run calibration routines for BMC DM                      */
/**************************************************************/
int bmc_calibrate(sm_t *sm_p, int calmode, bmc_t *bmc, uint32_t *step, int advance, int delta_only, int procid, int reset){
  static int init=0;
  static double arand[BMC_NACT]={0};
  static calmode_t bmccalmodes[BMC_NCALMODES];
  uint64_t i,j,z,index;
  struct timespec this,delta;
  time_t trand;
  double dt,step_fraction,multiplier;
  double act[BMC_NACT];
  double poke=0,vpoke=0;
  int    ncalim=0;
  
  /* Reset & Quick Init*/
  if(reset || !init){
    //Reset counters
    memset((void *)sm_p->bmccal.countA,0,sizeof(sm_p->bmccal.countA));
    memset((void *)sm_p->bmccal.countB,0,sizeof(sm_p->bmccal.countB));
    //Reset random numbers
    srand((unsigned) time(&trand));
    for(i=0;i<BMC_NACT;i++) arand[i] = (2*(rand() / (double) RAND_MAX) - 1);
    //Init calibration
    bmc_init_calibration(sm_p);
    //Init BMC calmodes
    for(i=0;i<BMC_NCALMODES;i++)
      bmc_init_calmode(i,&bmccalmodes[i]);
    init = 1;
    printf("BMC: Calibration initialized\n");
    if(reset == FUNCTION_RESET_RETURN) return calmode;
  }
  
  /* Set calibration parameters */
  if(procid == SCIID){
    poke   = bmccalmodes[calmode].sci_poke;
    vpoke  = bmccalmodes[calmode].sci_vpoke;
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
    //Print status
    printf("BMC: %f/%f seconds\n",dt,sm_p->bmc_cal_timer_length);
    if(dt > sm_p->bmc_cal_timer_length){
      //Turn off calibration
      printf("BMC: Stopping BMC calmode BMC_CALMODE_TIMER\n");
      calmode = BMC_CALMODE_NONE;
      init = 0;
    }
    
    //Set step counter
    *step = sm_p->bmccal.countA[calmode];
    
    //Increment counter
    if(advance) sm_p->bmccal.countA[calmode]++;
    
    //Return calmode
    return calmode;
  }
  
  /* BMC_CALMODE_POKE: Scan through acuators poking one at a time.    */
  /*                   Set to starting position in between each poke. */
  if(calmode==BMC_CALMODE_POKE){
    //Check counters
    if(sm_p->bmccal.countA[calmode] >= 0 && sm_p->bmccal.countA[calmode] < (2*BMC_NACT*ncalim)){
      //Set all BMC actuators to starting position
      if(!delta_only) memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      
      //Zero command delta
      memset(act,0,sizeof(act));
      
      //Poke one actuator
      if((sm_p->bmccal.countA[calmode]/ncalim) % 2 == 1){
	i = (sm_p->bmccal.countB[calmode]/ncalim) % BMC_NACT;
	printf("BMC: %lu of %d\n",i,BMC_NACT);
	act[i] = poke * sm_p->bmc_cal_scale;
	bmc_add_length(bmc->acmd,bmc->acmd,act,FUNCTION_NO_RESET);
	if(advance) sm_p->bmccal.countB[calmode]++;
      }
      else{
	printf("BMC: flat\n");

      }
    }else{
      //Set bmc back to starting position
      if(!delta_only) memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      //Turn off calibration
      printf("BMC: Stopping BMC calmode BMC_CALMODE_POKE\n");
      calmode = BMC_CALMODE_NONE;
      init = 0;
    }
    
    //Set step counter
    *step = (sm_p->bmccal.countA[calmode]/ncalim);
    
    //Increment counter
    if(advance) sm_p->bmccal.countA[calmode]++;
    
    //Return calmode
    return calmode;
    
  }

  /* BMC_CALMODE_VPOKE: Scan through acuators poking one at a time.    */
  /*                   Set to starting position in between each vpoke. */
  if(calmode==BMC_CALMODE_VPOKE){
    //Check counters
    if(sm_p->bmccal.countA[calmode] >= 0 && sm_p->bmccal.countA[calmode] < (2*BMC_NACT*ncalim)){
      //Set all BMC actuators to starting position
      if(!delta_only)
	for(i=0;i<BMC_NACT;i++)
	  bmc->acmd[i]=sm_p->bmccal.bmc_start[calmode].acmd[i];
      
      //Poke one actuator
      if((sm_p->bmccal.countA[calmode]/ncalim) % 2 == 1){
	i = (sm_p->bmccal.countB[calmode]/ncalim) % BMC_NACT;
	printf("BMC: %lu of %d\n",i,BMC_NACT);
	bmc->acmd[i] += vpoke * sm_p->bmc_cal_scale;
	if(advance) sm_p->bmccal.countB[calmode]++;
      }
      else{
	printf("BMC: flat\n");
      }
    }else{
      //Set bmc back to starting position
      if(!delta_only) memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      //Turn off calibration
      printf("BMC: Stopping BMC calmode BMC_CALMODE_VPOKE\n");
      calmode = BMC_CALMODE_NONE;
      init = 0;
    }
    
    //Set step counter
    *step = (sm_p->bmccal.countA[calmode]/ncalim);
    
    //Increment counter
    if(advance) sm_p->bmccal.countA[calmode]++;
    
    //Return calmode
    return calmode;
    
  }

  /* BMC_CALMODE_RAND: Poke all actuators by a random amount           */
  if(calmode == BMC_CALMODE_RAND){
    //Check counters
    if(sm_p->bmccal.countA[calmode] >= 0 && sm_p->bmccal.countA[calmode] < (2*ncalim)){

      //Set all BMC actuators to starting position
      if(!delta_only) memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));

      //Poke all actuators by random amount (just once)
      if(sm_p->bmccal.countA[calmode] == ncalim){
	for(i=0; i<BMC_NACT; i++)
	  bmc->acmd[i] += poke * arand[i] * sm_p->bmc_cal_scale;
      }
    }else{
      //Set bmc back to starting position
      if(!delta_only) memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      //Turn off calibration
      printf("BMC: Stopping calmode BMC_CALMODE_RAND\n");
      calmode = BMC_CALMODE_NONE;
      init = 0;
    }

    //Set step counter
    *step = (sm_p->bmccal.countA[calmode]/ncalim);
    
    //Increment counter
    if(advance) sm_p->bmccal.countA[calmode]++;

    //Return calmode
    return calmode;
  }

  /* BMC_CALMODE_PROBE: Step through the HOWFS probe patterns         */
  if(calmode==BMC_CALMODE_PROBE){
    //Check counters
    if(sm_p->bmccal.countA[calmode] >= 0 && sm_p->bmccal.countA[calmode] < ncalim*SCI_HOWFS_NPROBE){
      //Set all BMC actuators to starting position
      if(!delta_only) memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      //Add probe pattern
      bmc_add_probe(bmc->acmd,bmc->acmd,sm_p->efc_dhrot,sm_p->bmccal.countA[calmode]/ncalim,sm_p->efc_probe_amp);
      //Print status
      printf("BMC: %lu/%d steps\n",sm_p->bmccal.countA[calmode]/ncalim + 1,SCI_HOWFS_NPROBE);
     }else{
      //Set bmc back to starting position
      if(!delta_only) memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      //Turn off calibration
      printf("BMC: Stopping BMC calmode BMC_CALMODE_PROBE\n");
      calmode = BMC_CALMODE_NONE;
      init = 0;
    }
    
    //Set step counter
    *step = (sm_p->bmccal.countA[calmode]/ncalim);

    //Increment counter
    if(advance) sm_p->bmccal.countA[calmode]++;
    
    //Return calmode
    return calmode;
    
  }

  /* BMC_CALMODE_SINE: Step through sine patterns */
  if(calmode==BMC_CALMODE_SINE){
    //Check counters
    if(sm_p->bmccal.countA[calmode] >= 0 && sm_p->bmccal.countA[calmode] < ncalim*BMC_NSINE){
      //Set all BMC actuators to starting position
      if(!delta_only) memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      //Add sine pattern
      bmc_add_sine(bmc->acmd,bmc->acmd,sm_p->bmccal.countA[calmode]/ncalim,sm_p->bmc_cal_scale);
      //Print status
      printf("BMC: %lu/%d steps\n",sm_p->bmccal.countA[calmode]/ncalim + 1,BMC_NSINE);
    }else{
      //Set bmc back to starting position
      if(!delta_only) memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      //Turn off calibration
      printf("BMC: Stopping BMC calmode BMC_CALMODE_SINE\n");
      calmode = BMC_CALMODE_NONE;
      init = 0;
    }
    
    //Set step counter
    *step = (sm_p->bmccal.countA[calmode]/ncalim);

    //Increment counter
    if(advance) sm_p->bmccal.countA[calmode]++;
    
    //Return calmode
    return calmode;
    
  }
  
  /* BMC_CALMODE_SINE_DIFF: Step through sine patterns, use differential +/- patterns */
  if(calmode==BMC_CALMODE_SINE_DIFF){
    //Check counters
    if(sm_p->bmccal.countA[calmode] >= 0 && sm_p->bmccal.countA[calmode] < ncalim*BMC_NSINE*3){
      //Set all BMC actuators to starting position
      if(!delta_only) memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      
      //Get sub-step
      i = (sm_p->bmccal.countA[calmode]/ncalim) % 3;
      if(i==0) multiplier = 0;
      if(i==1) multiplier = 1;
      if(i==2) multiplier = -1;
      
      //Add sine pattern
      bmc_add_sine(bmc->acmd,bmc->acmd,sm_p->bmccal.countA[calmode]/ncalim/3,sm_p->bmc_cal_scale*multiplier);
      //Print status
      printf("BMC: %lu/%d [%lu/3] steps\n",sm_p->bmccal.countA[calmode]/ncalim/3 + 1,BMC_NSINE,i+1);
    }else{
      //Set bmc back to starting position
      if(!delta_only) memcpy(bmc,(bmc_t *)&sm_p->bmccal.bmc_start[calmode],sizeof(bmc_t));
      //Turn off calibration
      printf("BMC: Stopping BMC calmode BMC_CALMODE_SINE\n");
      calmode = BMC_CALMODE_NONE;
      init = 0;
    }
    
    //Set step counter
    *step = (sm_p->bmccal.countA[calmode]/ncalim);

    //Increment counter
    if(advance) sm_p->bmccal.countA[calmode]++;
    
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
