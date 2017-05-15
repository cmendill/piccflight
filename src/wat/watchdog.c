#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/io.h>

#include "../common/controller.h"
#include "../common/semun.h"
#include "../common/common_functions.h"
#include "../fsm/fsm_functions.h"
#include "watchdog.h"
#include "../hed/cameracmd.h"
#include "../hed/dmmap.h"
#include "../hed/dmmap2.h"
#define STDIN 0  // file descriptor for standard input

/* Processes to Watch */
extern void mpu_proc(void); //ctu commands & housekeeping
extern void ctu_proc(void); //receive data
extern void img_proc(void); //new way to receive data
extern void tlm_proc(void); //telemetry out
extern void bin_proc(void); //receive packets from bu
extern void fsm_proc(void); //state machine & unwrapper
extern void hsk_proc(void); //housekeeping loop

/* Report Fake Modes */
void report_fake_modes(void){
  printf("************************************************\n");
  printf("*                   FAKE MODES                 *\n");
  printf("************************************************\n");
  printf("0: Disabled\n");
  printf("1: TM test pattern\n");
  printf("2: Generate images w/ CTU sync\n");
  printf("3: Read images w/ CTU sync\n");
  printf("4: Generate images w/ timer sync\n");
  printf("5: Read images w/ timer sync\n");
  printf("6: ABCD stepping w/ timer sync\n");
  printf("\n");
}

/* Kill Process */
void kill_proc(sm_t *sm_p,int id){ 
  char rmmod[100] = "modprobe -r ";
  int i;
  int keep_going=1;

  if(WAT_DEBUG) printf("WAT: killing: %s\n",sm_p->w[id].name);
  if(sm_p->w[id].rtl){
    // Real-Time Process
    strncat(rmmod,(char *)sm_p->w[id].mod,6);
    //ask process to die
    sm_p->w[id].die = 1;
    for(i=0;i<RTL_UNLOAD_RETRY;i++){
      if(sm_p->w[id].done)
	break;
      else
	sleep(RTL_UNLOAD_SLEEP);
      if(WAT_DEBUG) printf("WAT: retry KILL: %s\n",sm_p->w[id].name);
    }
    
    //remove the module
    system(rmmod);
    if(WAT_DEBUG) printf("WAT: %s unloaded\n",sm_p->w[id].name);
    sm_p->w[id].pid = -1;
      }
  else{
    // User-Space Process
    keep_going=1;
    if(sm_p->w[id].ask){
      //wait for process to kill itself
      sm_p->w[id].die=1;
      if(!procwait(sm_p->w[id].pid)){
	if(WAT_DEBUG) printf("WAT: unloaded: %s\n",sm_p->w[id].name);
	keep_going=0;
      }
    }
    if(keep_going){
      //ask process to die
      kill(sm_p->w[id].pid,SIGINT);
      //wait for process to die
      if(procwait(sm_p->w[id].pid)){
	//kill process
	kill(sm_p->w[id].pid,SIGKILL);
	//wait for process to die
	if(procwait(sm_p->w[id].pid)){
	  if(WAT_DEBUG) printf(WARNING);
	  if(WAT_DEBUG) printf("WAT: could not kill %s!\n",sm_p->w[id].name);
	}
	else{
	  if(WAT_DEBUG) printf("WAT: unloaded with SIGKILL: %s\n",sm_p->w[id].name);
	}
      }
      else{
	if(WAT_DEBUG) printf("WAT: unloaded with SIGINT: %s\n",sm_p->w[id].name);
      }  
    }
  }
  //setup for next time
  sm_p->w[id].pid = -1;
}


/* Launch Process */
void launch_proc(sm_t *sm_p,int id){
  int pid=-1;
  void (*launch)(void);
  char insmod[100] = "insmod ";
  launch = sm_p->w[id].launch;
  //reset values
  sm_p->w[id].die  =  0;
  sm_p->w[id].done =  0;
  sm_p->w[id].res  =  0;
  sm_p->w[id].chk  =  0;
  sm_p->w[id].rec  =  0;
  sm_p->w[id].cnt  =  0;
  

  if(sm_p->w[id].rtl){
    // Real-Time Process
    strcat(insmod,(char *)sm_p->w[id].mod);
    system(insmod);
    sm_p->w[id].pid = 1;
    if(WAT_DEBUG) printf("WAT: started: %s:%s\n",sm_p->w[id].name,sm_p->w[id].mod);
  }
  else {
    // User-Space Process
    // --fork
    pid = fork();
    if(pid != 0){
      //we are the parent
      sm_p->w[id].pid = pid;
      if(sm_p->w[id].pid < 0){
	if(WAT_DEBUG) printf("WAT: %s fork failed!\n",sm_p->w[id].name);
      }
      else{
	if(WAT_DEBUG) printf("WAT: started: %s:%d\n",sm_p->w[id].name,sm_p->w[id].pid);
      }    
    }
    else{
      //we are the child
      launch();
      exit(0);
    }
  }
  
}


void wat_proc(void){
  int i;
  volatile uint32 chk;
  sm_t *sm_p;
  int shmfd;
  
  /**********************************
   *     Open shared memory
   *********************************/
  if((sm_p = openshm(&shmfd)) == NULL){
    printf("openshm fail: wat_proc\n");
    printf(WARNING);
    printf(REBOOT);
    printf("openshm fail: wat_proc\n");
    close(shmfd);
    exit(1);
  }
  

  /* Start Watchdog */
  while(1){
    /*(SECTION 0): If process has died, reset its pid*/
    for(i=0;i<NCLIENTS;i++)
      if(!sm_p->w[i].rtl)
	if(i != WATID)
	  if(sm_p->w[i].pid != -1)
	    if(waitpid(sm_p->w[i].pid,NULL,WNOHANG) == sm_p->w[i].pid){
	      sm_p->w[i].pid = -1; 
	      if(WAT_DEBUG) printf("WAT: %s crashed!\n",sm_p->w[i].name);
	    }    
    
    /*(SECTION 1): Kill everything if we've been turned off*/
    if(sm_p->die){
      for(i=0;i<NCLIENTS;i++)
	if(i != WATID)
	  if(sm_p->w[i].pid != -1)
	    kill_proc(sm_p,i);
      break;
    }
    
    /*(SECTION 2): If loop has died or been dead for last LOOP_TIMEOUT checks: KILL */
    for(i=0;i<NCLIENTS;i++)
      if(i != WATID)
	if(sm_p->w[i].run){
	  if((((sm_p->w[i].cnt > sm_p->w[i].tmo) && (sm_p->w[i].tmo > 0)) || sm_p->w[i].die) && sm_p->w[i].pid != -1){
	    printf("WAT: %s timeout: %d > %d\n",sm_p->w[i].name,sm_p->w[i].cnt,sm_p->w[i].tmo);
	    kill_proc(sm_p,i);
	  }
	}

    /*(SECTION 3): If loop is not running and should be: LAUNCH */
    for(i=0;i<NCLIENTS;i++)
      if(i != WATID)
	if(sm_p->w[i].run)
	  if(sm_p->w[i].pid == -1)
	    launch_proc(sm_p,i);
    
    /*(SECTION 4): If loop is running and shouldn't be: KILL */
    for(i=0;i<NCLIENTS;i++)
      if(i != WATID)
	if(!sm_p->w[i].run)
	  if(sm_p->w[i].pid != -1)
	    kill_proc(sm_p,i);
 
    /*(SECTION 5): If loop returned and needs to be restarted */
    for(i=0;i<NCLIENTS;i++)
      if(i != WATID)
	if(sm_p->w[i].run)
	  if(sm_p->w[i].pid != -1)
	    if((sm_p->w[i].done == 1 && sm_p->w[i].die == 0) || sm_p->w[i].res == 1){
	      kill_proc(sm_p,i);
	      launch_proc(sm_p,i);
	    }

    /*(SECTION 6): If loop is dead-> Increment COUNT. Else-> save checkin count and zero cnt */
    /* Loops will increment chk every time thru*/
    for(i=0;i<NCLIENTS;i++){
      if(i != WATID){
	if(sm_p->w[i].run){
	  chk = sm_p->w[i].chk;
	  if((chk == sm_p->w[i].rec) && (sm_p->w[i].pid != -1)){
	    sm_p->w[i].cnt++;
	  }
	  else{
	    sm_p->w[i].rec = chk;
	    sm_p->w[i].cnt = 0;
	  }
	}
      }
    }
        
    /*(SECTION 7): Print checkin counts*/
    if(CHK_DEBUG){
      printf("\n");
      for(i=0;i<NCLIENTS;i++){
	if(i != WATID){
	  printf("WAT: %s chk:rec:cnt:tmo = %lu:%lu:%d:%d\n",sm_p->w[i].name,sm_p->w[i].chk,sm_p->w[i].rec,sm_p->w[i].cnt,sm_p->w[i].tmo);
	}
      }
      printf("\n");
    }
    /*(SECTION 8): Sleep*/
    sleep(sm_p->w[WATID].per);
  }
  close(shmfd);
  return;
}


//function to load dm maps from file
void load_dmfiles(sm_t *sm_p){
  char dmpokefile[MAX_FILENAME];
  char dmflatfile[MAX_FILENAME];
  char dmtestfile[MAX_FILENAME];
  int fd;
  int i,j;
  int16 dmread[DMXS][DMYS];
  /* DM Images */
  sprintf(dmpokefile,"data/dmpoke.dat");
  sprintf(dmflatfile,"data/dmflat.dat");
  sprintf(dmtestfile,"data/dmtest.dat");
  
  /* Get flat image */
  fd = open(dmflatfile,O_RDONLY);
  if(fd < 0){
    close(fd);
    printf("WAT: Error opening %s.\n",dmflatfile);
  }
  else{
    printf("WAT: Reading %s\n",dmflatfile);
    read(fd,(void *)&dmread[0][0],sizeof dmread);
    close(fd);
    //convert to phase
    for(i=0;i<DMXS;i++)
      for(j=0;j<DMYS;j++)
	sm_p->dmflat[i][j]=dm_dac2phase(dmread[i][j]);
  }

  /* Get poke image */
  fd = open(dmpokefile,O_RDONLY);
  if(fd < 0){
    close(fd);
    printf("WAT: Error opening %s.\n",dmpokefile);
  }
  else{
    printf("WAT: Reading %s\n",dmpokefile);
    read(fd,(void *)&dmread[0][0],sizeof dmread);
    close(fd);
    //convert to phase
    for(i=0;i<DMXS;i++)
      for(j=0;j<DMYS;j++)
	sm_p->dmpoke[i][j]=dm_dac2phase(dmread[i][j]);
  }

  /* Get test image */
  fd = open(dmtestfile,O_RDONLY);
  if(fd < 0){
    close(fd);
    printf("WAT: Error opening %s.\n",dmtestfile);
  }
  else{
    printf("WAT: Reading %s\n",dmtestfile);
    read(fd,(void *)&dmread[0][0],sizeof dmread);
    close(fd);
    //convert to phase
    for(i=0;i<DMXS;i++)
      for(j=0;j<DMYS;j++)
	sm_p->dmtest[i][j]=dm_dac2phase(dmread[i][j]);
  }
}



int main(int argc,char **argv){
  #include "../fsm/flight_states.h"
  
  char line[JPLCMD_MAX_LENGTH];
  char cmd[JPLCMD_MAX_LENGTH];
  char smode[2];
  int  imode;
  int i,j,n;
  float ftemp;
  int   itemp;
  char  stemp[JPLCMD_MAX_LENGTH];
  char *delim  = " ";
  char *result = NULL;
  int  count=0;
  int shutdown=0;

 /**********************************
   *     Open shared memory
   *********************************/
  sm_t *sm_p;
  int shmfd;
  if((sm_p = openshm(&shmfd)) == NULL){
    printf(WARNING);
    printf("openshm fail: main\n");
    close(shmfd);
    exit(0);
  } 

  /**********************************
   *     Erase Shared Memory
   *********************************/
  memset((char *)sm_p,0,sizeof(sm_t));

  /**********************************
   *     Init Semephore
   *********************************/
  key_t key;
  int semid;
  union semun arg;
  
  
  /* Semaphore key */
  if((key = ftok(SEMA_FILE, SEMA_ID)) == -1) {
    perror("ftok");
    exit(1);
  }
  
  /* init the semaphore set*/
  if((semid = initsem(key, 1)) == -1) {
    perror("initsem");
    exit(1);
  }
  
  /* set the semaphores to 0 */
  arg.val = 0;
  if(semctl(semid, FRAME_SEM, SETVAL, arg)==-1){
    perror("semctl");
    exit(1);
  }
  
  /* Initialize Watchdog Structure*/ 
  uint8 proctmo[NCLIENTS]  = PROCTMO;
  uint8 procask[NCLIENTS]  = PROCASK;
  uint8 procrun[NCLIENTS]  = PROCRUN;
  char *procnam[NCLIENTS]  = PROCNAM;
  char *procmod[NCLIENTS]  = PROCMOD;
  uint8 procrtl[NCLIENTS]  = PROCRTL;
  uint8 procper[NCLIENTS]  = PROCPER;
  uint8 procpri[NCLIENTS]  = PROCPRI;
  for(i=0;i<NCLIENTS;i++){
    sm_p->w[i].pid  = -1;
    sm_p->w[i].run  =  procrun[i];
    sm_p->w[i].die  =  0;
    sm_p->w[i].done =  0;
    sm_p->w[i].chk  =  0;
    sm_p->w[i].rec  =  0;
    sm_p->w[i].cnt  =  0;
    sm_p->w[i].tmo  =  proctmo[i];
    sm_p->w[i].ask  =  procask[i];
    sm_p->w[i].rtl  =  procrtl[i];
    sm_p->w[i].per  =  procper[i];
    sm_p->w[i].pri  =  procpri[i];
    sm_p->w[i].name =  procnam[i];
    sm_p->w[i].mod  =  procmod[i];


    /*Set the user space processes */
    switch(i){
    case WATID:sm_p->w[i].launch = wat_proc; break;
    case CTUID:sm_p->w[i].launch = ctu_proc; break;
    case IMGID:sm_p->w[i].launch = img_proc; break;
    case MPUID:sm_p->w[i].launch = mpu_proc; break;
    case TLMID:sm_p->w[i].launch = tlm_proc; break;
    case BINID:sm_p->w[i].launch = bin_proc; break;
    case FSMID:sm_p->w[i].launch = fsm_proc; break;
    case HSKID:sm_p->w[i].launch = hsk_proc; break;
    }
  }
  
  /* Set Runtime Defaults */
  /* All shmem numbers are ZERO unless defined here */
  sm_p->memlock    = 0;  
  sm_p->die        = 0;
  sm_p->mpu_ready  = 0;
  sm_p->fake_mode  = FAKE_MODE_DEFAULT;  // Fake data mode
  sm_p->state      = STATE_DEFAULT;      // startup state
  sm_p->state_auto = N_STATES;
  sm_p->state_user = N_STATES;
  sm_p->state_loop_count = 0;
  sm_p->bksub      = BKSUB_DEFAULT;
  sm_p->dm_default = DM_DEFAULT;
  sm_p->fine_poke  = FINE_POKE_DEFAULT;
  sm_p->dither     = DITHER_DEFAULT;
  sm_p->mask_reset = MASK_RESET_DEFAULT;
  sm_p->unwrapper  = UNWRAPPER_DEFAULT;

  //dm gain
  sm_p->DMkP       = DMkP_DEFAULT;
  sm_p->DMkI       = DMkI_DEFAULT;
  sm_p->DMkD       = DMkD_DEFAULT;
  
  //dm auto piston 
  sm_p->dm_piston  = DM_PISTON_DEFAULT;

  //npzt gain
  sm_p->NkP        = NkP_HIGH_DEFAULT;
  sm_p->NkI        = NkI_HIGH_DEFAULT;
  sm_p->NkD        = NkD_HIGH_DEFAULT;
  
  //image2dm
  sm_p->dm_affine.M[0][0] = DM_AFFINE_M00_DEFAULT;
  sm_p->dm_affine.M[0][1] = DM_AFFINE_M01_DEFAULT;
  sm_p->dm_affine.M[1][0] = DM_AFFINE_M10_DEFAULT;
  sm_p->dm_affine.M[1][1] = DM_AFFINE_M11_DEFAULT;
  sm_p->dm_affine.T[0]    = DM_AFFINE_T0_DEFAULT;
  sm_p->dm_affine.T[1]    = DM_AFFINE_T1_DEFAULT;

  //image cropping
  sm_p->wfs_xoff    = WFSXOFF_DEFAULT;
  sm_p->wfs_yoff    = WFSYOFF_DEFAULT;

  //PZT DEFAULTS
  sm_p->Ithresh      = ITHRESH_DEFAULT;
  sm_p->Vthresh      = VTHRESH_DEFAULT;
  sm_p->pztscan_max  = PZTSCAN_MAX_DEFAULT;
  sm_p->pztscan_min  = PZTSCAN_MIN_DEFAULT;
  sm_p->pztscan_step = PZTSCAN_STEP_DEFAULT;
  sm_p->Ntip         = NTIP_DEFAULT;
  sm_p->Ntilt        = NTILT_DEFAULT;
  sm_p->Npiston      = NPISTON_DEFAULT;
  sm_p->npzt_null    = NPZT_NULL_DEFAULT;

  //pzts
  sm_p->NpztA0 = NPZTA0_DEFAULT;
  sm_p->NpztB0 = NPZTB0_DEFAULT;
  sm_p->NpztC0 = NPZTC0_DEFAULT;
  sm_p->PpztA0 = PPZTA0_DEFAULT;
  sm_p->PpztB0 = PPZTB0_DEFAULT;
  sm_p->PpztC0 = PPZTC0_DEFAULT;
  sm_p->CpztA0 = CPZTA0_DEFAULT;
  sm_p->CpztB0 = CPZTB0_DEFAULT;
  sm_p->CpztC0 = CPZTC0_DEFAULT;

  //fake WFS data intensity
  sm_p->fake_intensity  = FAKE_INTENSITY_DEFAULT;

  //WFS image rotation
  sm_p->wfs_affine_enable = WFS_AFFINE_ENABLE_DEFAULT;
  sm_p->wfs_affine_angle  = WFS_AFFINE_ANGLE_DEFAULT;
  sm_p->wfs_affine_xoff   = WFS_AFFINE_XOFF_DEFAULT;
  sm_p->wfs_affine_yoff   = WFS_AFFINE_YOFF_DEFAULT;
  sm_p->wfs_affine_xscale = WFS_AFFINE_XSCALE_DEFAULT;
  sm_p->wfs_affine_yscale = WFS_AFFINE_YSCALE_DEFAULT;

  //Dither amplitude
  sm_p->dither_amp        = DITHER_AMP_DEFAULT;

  /* Exposure Times */
  sm_p->sci_mode = SCI_MODE_DEFAULT;
  sm_p->wfs_mode = WFS_MODE_DEFAULT;
  if(change_sci_mode(sm_p,0))
    printf("WAT: Error setting default SCI exposure time\n");
  if(change_wfs_mode(sm_p,0))
    printf("WAT: Error setting default WFS exposure time\n");

  /* DM Mapping */
  memcpy((void *)&sm_p->dmmap[0][0],(void *)&dmmap[0][0],sizeof(dmmap));
  memcpy((void *)&sm_p->dmmap2[0][0],(void *)&dmmap2[0][0],sizeof(dmmap2));
 
  /* DM Files */
  load_dmfiles(sm_p);

  /* Insert RTLinux Modules */
  system("modprobe rtl");
  system("modprobe rtl_time");
  system("modprobe rtl_posixio");
  system("modprobe rtl_fifo");
  system("modprobe rtl_sched");

  /* Launch Watchdog */
  if(sm_p->w[WATID].run){
    if(sm_p->w[WATID].pid == -1){
      //launch process
      launch_proc(sm_p,WATID);
    }
  }

  /* Below this line, if something bad happens */
  /* we need to kill the watchdog              */
  
  /*-------------------------------------------*/

 
  /* Enter foreground loop and wait for kill signal */
  while(1){
    /* The foreground will now wait for an input from the console */
    fgets(line,JPLCMD_MAX_LENGTH,stdin);
    
    /****************************************
     * SYSTEM COMMANDS
     ***************************************/
    //quit: stop cameras, reset assert on, kill watchdog, shutdown BU, shutdown JPL
    if(!strncasecmp(line,"quit",4)){
      //Reset AE
      printf("WAT: Asserting HW reset\n");
      if(mpucmd("mpuf com/reset_assert.com",sm_p))
	printf("mpucmd failed - reset_assert\n");
      
      //Shutdown BU CPU
      printf("WAT: Shutting down BU CPU NOW!\n");
      sprintf(cmd,"bucpu shutdown");
      if(send2bu(cmd,strlen(cmd)*sizeof(char)))
	printf("WAT: send2bu failed!\n");
      else
	printf("WAT: sent bucmd: %s\n",cmd);
      
      //Setup for shutdown
      shutdown=1;
      
      break;
    }
    
    //preflight exit: kill watchdog, shutdown BU, shutdown JPL
    if(!strncasecmp(line,"preflight exit",14)){
      //Shutdown BU CPU
      printf("WAT: Shutting down BU CPU NOW!\n");
      sprintf(cmd,"bucpu shutdown");
      if(send2bu(cmd,strlen(cmd)*sizeof(char)))
	printf("WAT: send2bu failed!\n");
      else
	printf("WAT: sent bucmd: %s\n",cmd);
      
      //Setup for shutdown
      shutdown=1;

      break;
    }

    //just exit
    if(!strncasecmp(line,"exit",4)){
      printf("shutting down watchdog\n");
      break;
    }
    
    //erase flight data
    if(!strncasecmp(line,"erase flight data",17)){
      printf("Erasing flight data...\n");
      printf(" -- Stopping Telemetry\n");
      sm_p->w[TLMID].run = 0;    
      for(i=0;i<ERASE_TIMEOUT;i++){
	if((sm_p->w[TLMID].pid == -1)){
	  printf(" -- Erasing data\n");
	  system("rm -r data/flight_data/*");
	  printf(" -- Starting Telemetry\n");
	  sm_p->w[TLMID].run = 1;
	  break;
	}
	sleep(1);
      }
      if(i >= ERASE_TIMEOUT){
	printf(" -- failed!  quitting...\n");
	break;
      }
      printf("Flight data erased.\n");
    }
    
    //resend mpuinit.com
    if(!strncasecmp(line,"resetmpu",8)){
      printf("Initializing MPU\n");
      if(mpucmd("mpuf com/mpuinit.com",sm_p))
	printf("resetmpu failed!\n");
    }
    
    /****************************************
     * FLIGHT COMMANDS
     ***************************************/
    //*****JPL Commands******//
    
    //change state to aligned
    if(!strncasecmp(line,"null",4)){
      //only allow if we are in STATE_FINE_MODE
      if(sm_p->state == STATE_FINE_MODE){
	sm_p->aligned = 1;
	printf("Got command NULL\n");
      }
      else
	printf("Must be in STATE_FINE_MODE to NULL\n");
    }
    //change state to not aligned
    if(!strncasecmp(line,"abcd",4)){
      sm_p->aligned = 0;
      printf("Got command ABCD\n");
    }
    //change state to lost
    if(!strncasecmp(line,"lost",4)){
      sm_p->state_user = STATE_LOST;
      printf("Switching to state: STATE_LOST\n");
    }
    //reset dm
    if(!strncasecmp(line,"reset dm",8)){
      sm_p->resetdm = 1;
      load_dmfiles(sm_p);
      printf("Resetting DM\n");
    }
    //reset mask
    if(!strncasecmp(line,"reset mask",10)){
      sm_p->mask_reset = 1;
      printf("Resetting mask\n");
    }

    //target left
    if(!strncasecmp(line,"target left",11)){
      if(send2bu(line,(strlen(line))*sizeof(char)))
	printf("WAT: send2bu failed!\n");
      else{
	printf("WAT: sent bucmd: %s\n",line);
	if(flight_states[sm_p->state]){
	  //change to state lost
	  sm_p->state_user = STATE_LOST;
	  printf("Switching to state: STATE_LOST\n");
	}
      }
    }

    //target right
    if(!strncasecmp(line,"target right",12)){
      if(send2bu(line,(strlen(line))*sizeof(char)))
	printf("WAT: send2bu failed!\n");
      else{
	printf("WAT: sent bucmd: %s\n",line);
	if(flight_states[sm_p->state]){
	  //change to state lost
	  sm_p->state_user = STATE_LOST;
	  printf("Switching to state: STATE_LOST\n");
	}
      }
    }
    
    //turn dmmap_default
    if(!strncasecmp(line,"dmmap 0",7)){
      sm_p->dmmap_default = 1;
      printf("Using Default DM2IMAGE Map\n");
    }
    //turn dmmap_default
    if(!strncasecmp(line,"dmmap 1",7)){
      sm_p->dmmap_default = 0;
      printf("Using Poked DM2IMAGE Map\n");
    }
    //npzt high gain
    if(!strncasecmp(line,"pzth",4)){
      sm_p->NkP        = NkP_HIGH_DEFAULT;
      sm_p->NkI        = NkI_HIGH_DEFAULT;
      sm_p->NkD        = NkD_HIGH_DEFAULT;
      printf("NPZT --> High Gain\n");
    }
    //npzt low gain
    if(!strncasecmp(line,"pztl",4)){
      sm_p->NkP        = NkP_LOW_DEFAULT;
      sm_p->NkI        = NkI_LOW_DEFAULT;
      sm_p->NkD        = NkD_LOW_DEFAULT;
      printf("NPZT --> LOW Gain\n");
    }
    //turn angle tracker override on
    if(!strncasecmp(line,"atover",6)){
      sm_p->at_override = 1;
      printf("AT Override ON\n");
    }
    //turn angle tracker override off
    if(!strncasecmp(line,"atwait",6)){
      sm_p->at_override = 0;
      printf("AT Override OFF\n");
    }
    //move wfs offset right
    if(!strncasecmp(line,"rr",2)){
      itemp = sm_p->wfs_xoff - 1;
      if(itemp < WFSOFF_MIN)
	itemp=WFSOFF_MIN;
      if(itemp > WFSOFF_MAX)
	itemp=WFSOFF_MAX;
      sm_p->wfs_xoff = itemp;
      printf("WFS XOFF = %d\n",sm_p->wfs_xoff);
      printf("Resetting mask\n");
      sm_p->mask_reset = 1;
    }
    //move wfs offset left
    if(!strncasecmp(line,"ll",2)){
      itemp = sm_p->wfs_xoff + 1;
      if(itemp < WFSOFF_MIN)
	itemp=WFSOFF_MIN;
      if(itemp > WFSOFF_MAX)
	itemp=WFSOFF_MAX;
      sm_p->wfs_xoff = itemp;
      printf("WFS XOFF = %d\n",sm_p->wfs_xoff);
      printf("Resetting mask\n");
      sm_p->mask_reset = 1;
    }
    //move wfs offset up
    if(!strncasecmp(line,"uu",2)){
      itemp = sm_p->wfs_yoff - 1;
      if(itemp < WFSOFF_MIN)
	itemp=WFSOFF_MIN;
      if(itemp > WFSOFF_MAX)
	itemp=WFSOFF_MAX;
      sm_p->wfs_yoff = itemp;
      printf("WFS YOFF = %d\n",sm_p->wfs_yoff);
      printf("Resetting mask\n");
      sm_p->mask_reset = 1;
    }
    //move wfs offset down
    if(!strncasecmp(line,"dd",2)){
      itemp = sm_p->wfs_yoff + 1;
      if(itemp < WFSOFF_MIN)
	itemp=WFSOFF_MIN;
      if(itemp > WFSOFF_MAX)
	itemp=WFSOFF_MAX;
      sm_p->wfs_yoff = itemp;
      printf("WFS YOFF = %d\n",sm_p->wfs_yoff);
      printf("Resetting mask\n");
      sm_p->mask_reset = 1;
    }
    

    //turn TM on
    if(!strncasecmp(line,"tlm on",6)){
      sm_p->w[TLMID].run = 1;    
      printf("Telemetry ON\n");
    }
    //turn TM off
    if(!strncasecmp(line,"tlm off",7)){
      sm_p->w[TLMID].run = 0;    
      printf("Telemetry OFF\n");
    }

    //turn Npzt dither on
    if(!strncasecmp(line,"dither on",9)){
      sm_p->dither = 1;
      printf("NPZT Dither ON\n");
    }
    //turn Npzt dither off
    if(!strncasecmp(line,"dither off",10)){
      sm_p->dither = 0;
      printf("NPZT Dither OFF\n");
    }
    //turn dm auto-piston on
    if(!strncasecmp(line,"dmpiston on",11)){
      sm_p->dm_piston = 1;
      printf("DM Auto-Piston ON\n");
    }
    //turn dm auto-piston off
    if(!strncasecmp(line,"dmpiston off",12)){
      sm_p->dm_piston = 0;
      printf("DM Auto-Piston OFF\n");
    }
    
    /****************************************
     * AE BOX COMMANDS
     ***************************************/
    //start camera
    if(!strncasecmp(line,"onestart",8)){
      printf("starting camera\n");
      if(mpucmd("mpuf com/restartpix.com",sm_p))
	printf("mpucmd failed - restartpix\n");
      if(mpucmd("mpuf com/onestart.com",sm_p))
	printf("mpucmd failed - onestart\n");
    }

    //stop camera
    if(!strncasecmp(line,"onestop",7)){
      printf("stopping camera\n");
      if(mpucmd("mpuf com/onestop.com",sm_p))
	printf("mpucmd failed - onestop\n");
    }

    //start cameras
    if(!strncasecmp(line,"start",5)){
      printf("starting both cameras\n");
      if(mpucmd("mpuf com/restartpix.com",sm_p))
	printf("mpucmd failed - restartpix\n");
      if(mpucmd("mpuf com/start.com",sm_p))
	printf("mpucmd failed - start\n");
    }

    //stop cameras
    if(!strncasecmp(line,"stop",4)){
      printf("stopping both cameras\n");
      if(mpucmd("mpuf com/stop.com",sm_p))
	printf("mpucmd failed - stop\n");
    }

    //command science camera
    if(!strncasecmp(line,"camera sci",10)){
      printf("Selecting SCI\n");
      if(mpucmd("mpu ctucmd camera 0",sm_p))
	printf("mpucmd failed - camera sci\n");
    }

    //command wavefront sensor camera
    if(!strncasecmp(line,"camera wfs",10)){
      printf("Selecting WFS\n");
      if(mpucmd("mpu ctucmd camera 1",sm_p))
	printf("mpucmd failed - camera wfs\n");
    }

    //command both cameras
    if(!strncasecmp(line,"camera both",11)){
      printf("Selecting SCI & WFS\n");
      if(mpucmd("mpu ctucmd camera 01",sm_p))
	printf("mpucmd failed - camera both\n");
    }
    //load science pattern
    if(!strncasecmp(line,"load sci",8)){
      if(change_sci_mode(sm_p,1))
	printf("change_sci_mode failed!\n");
    }
    //load wfs pattern
    if(!strncasecmp(line,"load wfs",8)){
      if(change_wfs_mode(sm_p,1))
	printf("change_wfs_mode failed!\n");
    }


    //sci mode
    if(!strncasecmp(line,"scimode",7)){
      strncpy(smode,line+8,4);
      imode = atoi(smode);
      printf("setting sci_mode = %2.2d\n",imode);
      sm_p->sci_mode = imode;
      if(change_sci_mode(sm_p,1))
       printf("change_sci_mode failed!\n");
    }
    
    //wfs mode
    if(!strncasecmp(line,"wfsmode",7)){
      strncpy(smode,line+8,4);
      imode = atoi(smode);
      printf("setting wfs_mode = %2.2d\n",imode);
      sm_p->wfs_mode = imode;
      if(change_wfs_mode(sm_p,1))
	printf("change_wfs_mode failed!\n");
      
    }

    //reset AE
    if(!strncmp(line,"resetAE",7)){
      printf("resetting AE\n");
      if(reset_AE(sm_p))
	printf("reset_AE failed! - resetAE\n");
    }
    
    //hk on
    if(!strncasecmp(line,"hk on",5)){
      printf("Starting HK Loop\n");
      sm_p->w[HSKID].run = 1;      
    }

    //hk off
    if(!strncasecmp(line,"hk off",6)){
      printf("Stopping HK Loop\n");
      sm_p->w[HSKID].run = 0;      
    }
    //dump housekeeping
    if(!strncasecmp(line,"hkdump",6)){
      if(mpucmd("mpuf com/hk/hkdump.com",sm_p))
	printf("mpucmd failed - hkdump\n");
    }
    //issue command
    if(!strncasecmp(line,"mpu",3)){
      if(mpucmd(line,sm_p))
	printf("mpucmd failed - issue command\n");
    }
    
    //command tec
    if(!strncasecmp(line,"settec",6)){
      if(sscanf(line,"%s %d",stemp,&itemp)==2){
	//check tec current limit percentage
	if(itemp >=0 && itemp <= 100){
	  sprintf(cmd,"mpu T_WRITE_CR to T_CR_CHAN5 with %d",itemp);
	  printf("Setting TEC current limit: %d%% \n",itemp);
	  if(mpucmd(cmd,sm_p))
	    printf("mpucmd failed - hkdump\n");
	  
	}
	else{
	  printf("Invalid TEC limit: %d. Must be 0-100\n",itemp);
	}
      }
    }
    

    /****************************************
     * SHARED MEMORY COMMANDS
     ***************************************/
    //set unwrapper
    if(!strncasecmp(line,"unwrap picard",13)){
      sm_p->unwrapper = PICARD_UNWRAP;
      printf("Using Picard unwrapper\n");
    }
    if(!strncasecmp(line,"unwrap herraez",14)){
      sm_p->unwrapper = HERRAEZ_UNWRAP;
      printf("Using Herraez unwrapper\n");
    }

    //set fringehop
    if(!strncasecmp(line,"hop",3)){
      itemp = atoi(line+4);
      sm_p->pzthop = itemp;
      printf("Setting fringehop: %d\n",sm_p->pzthop);
    }
    
    //set WFS X offset
    if(!strncasecmp(line,"wfsxoff",7)){
      itemp = atoi(line+8);
      if(itemp >= 0 && itemp <= (WFSXS - MAPXS*MAPBIN)){
	sm_p->wfs_xoff = itemp;
	printf("Setting WFS X Offset: %d\n",sm_p->wfs_xoff);
      }
      else
	printf("WFS X Offset out of range\n");
    }
    
    //set WFS Y offset
    if(!strncasecmp(line,"wfsyoff",7)){
      itemp = atoi(line+8);
      if(itemp >= 0 && itemp <= (WFSYS - MAPYS*MAPBIN)){
	sm_p->wfs_yoff = itemp;
	printf("Setting WFS Y Offset: %d\n",sm_p->wfs_yoff);
      }
      else
	printf("WFS Y Offset out of range\n");
    }
    
    //set fake intensity
    if(!strncasecmp(line,"fake intensity",14)){
      ftemp = atof(line+15);
      if(ftemp >= FAKE_WFS_MIN && ftemp <= FAKE_WFS_MAX){
	printf("Setting fake WFS intensity: %8.1f\n",ftemp);
	sm_p->fake_intensity = ftemp;
	printf("Resetting mask\n");
	sm_p->mask_reset = 1;
      }
      else
	printf("Fake intensity out of range: [%d-%d]\n",FAKE_WFS_MIN,FAKE_WFS_MAX);
      
    }

    //set Ithresh
    if(!strncasecmp(line,"ithresh",7)){
      ftemp = atof(line+8);
      if(ftemp >= ITHRESH_MIN && ftemp <= ITHRESH_MAX){
	printf("Setting mask intensity threshold: %8.3f\n",ftemp);
	sm_p->Ithresh = ftemp;
	printf("Resetting mask\n");
	sm_p->mask_reset = 1;
      }
      else
	printf("Ithresh out of range: [%d-%d]\n",ITHRESH_MIN,ITHRESH_MAX);
      
    }
    
    //set Vthresh
    if(!strncasecmp(line,"vthresh",7)){
      ftemp = atof(line+8);
      if(ftemp >= VTHRESH_MIN && ftemp <= VTHRESH_MAX){
	printf("Setting mask visibility threshold: %8.3f\n",ftemp);
	sm_p->Vthresh = ftemp;
	printf("Resetting mask\n");
	sm_p->mask_reset = 1;
      }
      else
	printf("Vthresh out of range: [%d-%d]\n",VTHRESH_MIN,VTHRESH_MAX);
      
    }
    
    //set sleep time
    if(!strncasecmp(line,"sleep time",10)){
      ftemp = atof(line+11);
      printf("Setting sleep time: %8.3f\n",ftemp);
      sm_p->sleep_time = ftemp;
    }

    //set dither amplitude
    if(!strncasecmp(line,"dither amp",10)){
      ftemp = atof(line+11);
      if(ftemp >= DITHER_AMP_MIN && ftemp <= DITHER_AMP_MAX){
	printf("Setting dither amplitude: %8.3f\n",ftemp);
	sm_p->dither_amp = ftemp;
      }
      else
	printf("Dither amplitude %f out of range [%f, %f]\n",ftemp,DITHER_AMP_MIN,DITHER_AMP_MAX);
    }

    //set Ntip
    if(!strncasecmp(line,"ntip",4)){
      ftemp = atof(line+5);
      printf("Setting Ntip: %8.3f\n",ftemp);
      sm_p->Ntip = ftemp;
    }
    //set Ntilt
    if(!strncasecmp(line,"ntilt",5)){
      ftemp = atof(line+6);
      printf("Setting Ntilt: %8.3f\n",ftemp);
      sm_p->Ntilt = ftemp;
    }
    //set Npiston
    if(!strncasecmp(line,"npiston",7)){
      ftemp = atof(line+8);
      printf("Setting Npiston: %8.3f\n",ftemp);
      sm_p->Npiston = ftemp;
    }
    //set NPZT null position
    if(!strncasecmp(line,"set npzt null",13)){
      if(isspace(*(line+13))){
	itemp = atoi(line+14);
	if(itemp >= 0 && itemp <=3){
	  printf("Setting NPZT null position to: %d\n",itemp);
	  sm_p->npzt_null = itemp;
	}else{
	  printf("Invalid NPZT null position: %d, Use: [0,1,2,3]\n",itemp);
	}
      }
    }
    //set nP gain
    if(!strncasecmp(line,"npgain",6)){
      ftemp = atof(line+7);
      if(ftemp <= 0){
	printf("Setting NkP: %8.3f\n",ftemp);
	sm_p->NkP = ftemp;
      }
      else
	printf("PZT Gains must be less than zero\n");
    }
    //set nI gain
    if(!strncasecmp(line,"nigain",6)){
      ftemp = atof(line+7);
      if(ftemp <= 0){
	printf("Setting NkI: %8.3f\n",ftemp);
	sm_p->NkI = ftemp;
      }
      else
	printf("PZT Gains must be less than zero\n");
    }
    //set nD gain
    if(!strncasecmp(line,"ndgain",6)){
      ftemp = atof(line+7);
      if(ftemp <= 0){
	printf("Setting NkD: %8.3f\n",ftemp);
	sm_p->NkD = ftemp;
      }
      else
	printf("PZT Gains must be less than zero\n");
    }
    //set dmP gain
    if(!strncasecmp(line,"dmpgain",7)){
      ftemp = atof(line+7);
      if(ftemp >= 0){
	printf("Setting DMkP: %8.3f\n",ftemp);
	sm_p->DMkP = ftemp;
      }
      else
	printf("DM Gains must be greater than zero\n");
    }
    //set dmI gain
    if(!strncasecmp(line,"dmigain",7)){
      ftemp = atof(line+7);
      if(ftemp >= 0){
	printf("Setting DMkI: %8.3f\n",ftemp);
	sm_p->DMkI = ftemp;
      }
      else
	printf("DM Gains must be greater than zero\n");
    }
    //set dmD gain
    if(!strncasecmp(line,"dmdgain",7)){
      ftemp = atof(line+7);
      if(ftemp >= 0){
	printf("Setting DMkD: %8.3f\n",ftemp);
	sm_p->DMkD = ftemp;
      }
      else
	printf("DM Gains must be greater than zero\n");
    }
    //set dm_default
    if(!strncasecmp(line,"dmdefault",9)){
      itemp = atoi(line+10);
      sm_p->dm_default = itemp;
      printf("Setting dm_default: %d\n",sm_p->dm_default);
    }
 
    

    //set pztscan params
    if(!strncasecmp(line,"pztscan",7)){
      count = 0;
      delim = " ";
      char args[10][20];
      uint32 min,max,step;
      result = strtok(line,delim);
      while(result != NULL){
	if(!isspace(*result)){
	  strcpy(args[count],result);
	  count++;
	}
	result = strtok(NULL,delim);
      }
      if(count == 4){
	min  = (uint32)(atof(args[1])*(PZT_DMAX/10.));
	max  = (uint32)(atof(args[2])*(PZT_DMAX/10.));
	step = (uint32)(NPZT_DWAVE/atof(args[3]));

	//check for range
	if(min  >= PZT_DMIN && 
	   min  <= PZT_DMAX &&
	   max  >= PZT_DMIN &&
	   max  <= PZT_DMAX &&
	   min  <  max      &&
	   step >= PZT_DMIN &&
	   step <= PZT_DMAX){
	  
	  sm_p->pztscan_min  = min;
	  sm_p->pztscan_max  = max;
	  sm_p->pztscan_step = step;
	  sm_p->state_reset = 1;
	}
	else
	  printf("USAGE: pztscan min[microns] max[microns] step[1/X WAVE]\n");
	   
	  
      }
      
      printf("Setting pztscan: min, max, step\n");
    }
    
    
    //set Npzt default position
    if(!strncasecmp(line,"Npzt",4) || !strncasecmp(line,"Ppzt",4) || !strncasecmp(line,"Cpzt",4)){
      count = 0;
      delim = " ";
      int set = 0;
      char args[10][20];
      uint32 pztA0,pztB0,pztC0;
      double  alpha,beta,zeta;
      double npzt_m2d = PZT_DMAX/NPZT_STROKE;
      double ppzt_m2d = PZT_DMAX/PPZT_STROKE;
      double cpzt_m2d = PZT_DMAX/CPZT_STROKE;

      result = strtok(line,delim);
      while(result != NULL){
	if(!isspace(*result)){
	  strcpy(args[count],result);
	  count++;
	}
	result = strtok(NULL,delim);
      }
      if(count == 4){
	if(!strncasecmp(line+4,"M",1)){
	  if(!strncasecmp(line,"N",1)){
	    pztA0  = (uint32)(atof(args[1])*npzt_m2d);
	    pztB0  = (uint32)(atof(args[2])*npzt_m2d);
	    pztC0  = (uint32)(atof(args[3])*npzt_m2d);
	    set    = 1;
	  }
	  if(!strncasecmp(line,"P",1)){
	    pztA0  = (uint32)(atof(args[1])*ppzt_m2d);
	    pztB0  = (uint32)(atof(args[2])*ppzt_m2d);
	    pztC0  = (uint32)(atof(args[3])*ppzt_m2d);
	    set    = 1;
	  }
	  if(!strncasecmp(line,"C",1)){
	    pztA0  = (uint32)(atof(args[1])*cpzt_m2d);
	    pztB0  = (uint32)(atof(args[2])*cpzt_m2d);
	    pztC0  = (uint32)(atof(args[3])*cpzt_m2d);
	    set    = 1;
	  }
	}
	
	if(!strncasecmp(line+4,"A",1)){
	  alpha = atof(args[1]);
	  beta  = atof(args[2]);
	  zeta  = atof(args[3]);
	  if(!strncasecmp(line,"N",1)){
	    abz2abc(alpha,beta,zeta,npzt_m2d,&pztA0,&pztB0,&pztC0);
	    set    = 1;
	  }
	  if(!strncasecmp(line,"P",1)){
	    abz2abc(alpha,beta,zeta,ppzt_m2d,&pztA0,&pztB0,&pztC0);
	    set    = 1;
	  }
	  if(!strncasecmp(line,"C",1)){
	    abz2abc(alpha,beta,zeta,cpzt_m2d,&pztA0,&pztB0,&pztC0);
	    set    = 1;
	  }
	  
	}
	
	//check for range
	if(set){
	  if(pztA0  >= PZT_DMIN && 
	     pztA0  <= PZT_DMAX &&
	     pztB0  >= PZT_DMIN &&
	     pztB0  <= PZT_DMAX &&
	     pztC0  >= PZT_DMIN &&
	     pztC0  <= PZT_DMAX){
	    if(!strncasecmp(line,"N",1)){
	      sm_p->NpztA0 = pztA0;
	      sm_p->NpztB0 = pztB0;
	      sm_p->NpztC0 = pztC0;
	      printf("Setting Npzt: %ld, %ld, %ld\n",sm_p->NpztA0,sm_p->NpztB0,sm_p->NpztC0);
	    }
	    if(!strncasecmp(line,"P",1)){
	      sm_p->PpztA0 = pztA0;
	      sm_p->PpztB0 = pztB0;
	      sm_p->PpztC0 = pztC0;
	      printf("Setting Ppzt: %ld, %ld, %ld\n",sm_p->PpztA0,sm_p->PpztB0,sm_p->PpztC0);
	    }
	    if(!strncasecmp(line,"C",1)){
	      sm_p->CpztA0 = pztA0;
	      sm_p->CpztB0 = pztB0;
	      sm_p->CpztC0 = pztC0;
	      printf("Setting Cpzt: %ld, %ld, %ld\n",sm_p->CpztA0,sm_p->CpztB0,sm_p->CpztC0);
	    }
	  }
	}
      }
      else{
	printf("USAGE: [N,P,C]pztM A0[microns] B0[microns] C0[microns]\n");
	printf("USAGE: [N,P,C]pztV Alpha[as] Beta[as] Z[microns]\n");
      }
    }
    
    //set impoke params
    if(!strncasecmp(line,"impoke",6)){
      count = 0;
      delim = " ";
      char args[10][20];
      result = strtok(line,delim);
      while(result != NULL){
	if(!isspace(*result)){
	  strcpy(args[count],result);
	  count++;
	}
	result = strtok(NULL,delim);
      }
      if(count == 4){
	sm_p->imxpoke  = atoi(args[1]);
	sm_p->imypoke  = atoi(args[2]);
	sm_p->imzpoke  = atof(args[3]);
      }
      printf("Setting impoke: %d, %d, %f\n",sm_p->imxpoke,sm_p->imypoke,sm_p->imzpoke);
    }
    
      
    //set dmpoke params
    if(!strncasecmp(line,"dmpoke",6)){
      count = 0;
      delim = " ";
      char args[10][20];
      result = strtok(line,delim);
      while(result != NULL){
	if(!isspace(*result)){
	  strcpy(args[count],result);
	  count++;
	}
	result = strtok(NULL,delim);
      }
      if(count == 4){
	sm_p->dmxpoke  = atoi(args[1]);
	sm_p->dmypoke  = atoi(args[2]);
	sm_p->dmzpoke  = atof(args[3]);
      }
      printf("Setting dmpoke: %d, %d, %f\n",sm_p->dmxpoke,sm_p->dmypoke,sm_p->dmzpoke);
    }
    
    //load dark,flat,read_flat,bkpix
    if(!strncasecmp(line,"loadcal",7)){
      printf("Loading calibration images\n");
      sm_p->gotcal=0;
    }
    
    //turn fine_poke on
    if(!strncasecmp(line,"fine_poke on",12)){
      sm_p->fine_poke = 1;
      printf("FINE_MODE DM Poke ON\n");
    }
    //turn fine_poke off
    if(!strncasecmp(line,"fine_poke off",13)){
      sm_p->fine_poke = 0;
      printf("FINE_MODE DM Poke OFF\n");
    }
    //turn background subtraction on
    if(!strncasecmp(line,"bksub on",8)){
      sm_p->bksub = 1;
      printf("Background Subtration ON\n");
    }
    //turn background subtraction off
    if(!strncasecmp(line,"bksub off",9)){
      sm_p->bksub = 0;
      printf("Background Subtration OFF\n");
    }
    
    /****************************************
     * CHANGE STATE
     ***************************************/
    //goto state X
    if(!strncasecmp(line,"state",5)){
      
      if(!strncasecmp(line+6,"lost",4)){
	sm_p->state_user = STATE_LOST;
	printf("Switching to state: STATE_LOST\n");
      }
      if(!strncasecmp(line+6,"fine_mode",9)){
	sm_p->state_user = STATE_FINE_MODE;
	printf("Switching to state: STATE_FINE_MODE\n");
      }
      if(!strncasecmp(line+6,"zero",4)){
	sm_p->state_user = STATE_ZERO;
	printf("Switching to state: STATE_ZERO\n");
      }
      if(!strncasecmp(line+6,"max",3)){
	sm_p->state_user = STATE_MAX;
	printf("Switching to state: STATE_MAX\n");
      }
      if(!strncasecmp(line+6,"nstep",5)){
	sm_p->state_user = STATE_NSTEP;
	printf("Switching to state: STATE_NSTEP\n");
      }
      if(!strncasecmp(line+6,"nscan",5)){
	sm_p->state_user = STATE_NSCAN;
	printf("Switching to state: STATE_NSCAN\n");
      }
      if(!strncasecmp(line+6,"npiston",7)){
	sm_p->state_user = STATE_NPISTON;
	printf("Switching to state: STATE_NPISTON\n");
      }
      if(!strncasecmp(line+6,"nullscan",8)){
	sm_p->state_user = STATE_NULLSCAN;
	printf("Switching to state: STATE_NULLSCAN\n");
      }
      if(!strncasecmp(line+6,"cscan",5)){
	sm_p->state_user = STATE_CSCAN;
	printf("Switching to state: STATE_CSCAN\n");
      }
      if(!strncasecmp(line+6,"pztbest",7)){
	sm_p->state_user = STATE_PZTBEST;
	printf("Switching to state: STATE_PZTBEST\n");
      }
      if(!strncasecmp(line+6,"psearch",7)){
	sm_p->state_user = STATE_PSEARCH;
	printf("Switching to state: STATE_PSEARCH\n");
      }
      if(!strncasecmp(line+6,"pspiral",7)){
	sm_p->state_user = STATE_PSPIRAL;
	printf("Switching to state: STATE_PSPIRAL\n");
      }
      if(!strncasecmp(line+6,"npid",4)){
	sm_p->state_user = STATE_NPID;
	printf("Switching to state: STATE_NPID\n");
      }
      if(!strncasecmp(line+6,"setttp",6)){
	sm_p->state_user = STATE_SETTTP;
	printf("Switching to state: STATE_SETTTP\n");
      }
      if(!strncasecmp(line+6,"setn",4)){
	sm_p->state_user = STATE_SETN;
	printf("Switching to state: STATE_SETN\n");
      }
      if(!strncasecmp(line+6,"nacal",5)){
	sm_p->state_user = STATE_NACAL;
	printf("Switching to state: NACAL\n");
      }
      if(!strncasecmp(line+6,"nbcal",5)){
	sm_p->state_user = STATE_NBCAL;
	printf("Switching to state: NBCAL\n");
      }
      if(!strncasecmp(line+6,"nccal",5)){
	sm_p->state_user = STATE_NCCAL;
	printf("Switching to state: NCCAL\n");
      }
      if(!strncasecmp(line+6,"dmpoke",6)){
	sm_p->state_user = STATE_DMPOKE;
	printf("Switching to state: STATE_DMPOKE\n");
      }
      if(!strncasecmp(line+6,"dmxypoke",8)){
	sm_p->state_user = STATE_DMXYPOKE;
	printf("Switching to state: STATE_DMXYPOKE\n");
      }
     if(!strncasecmp(line+6,"dmscan",6)){
	sm_p->state_user = STATE_DMSCAN;
	printf("Switching to state: STATE_DMSCAN\n");
      }
      if(!strncasecmp(line+6,"dmraster",8)){
	sm_p->state_user = STATE_DMRASTER;
	printf("Switching to state: STATE_DMRASTER\n");
      }
      if(!strncasecmp(line+6,"testdmmap",9)){
	sm_p->state_user = STATE_TESTDMMAP;
	printf("Switching to state: STATE_TESTDMMAP\n");
      }
      if(!strncasecmp(line+6,"imraster",8)){
	sm_p->state_user = STATE_IMRASTER;
	printf("Switching to state: STATE_IMRASTER\n");
      }
      if(!strncasecmp(line+6,"imxypoke",8)){
	sm_p->state_user = STATE_IMXYPOKE;
	printf("Switching to state: STATE_IMXYPOKE\n");
      }
      if(!strncasecmp(line+6,"trackat",7)){
	sm_p->state_user = STATE_TRACKAT;
	printf("Switching to state: STATE_TRACKAT\n");
      }
      if(!strncasecmp(line+6,"sleep",5)){
	sm_p->state_user = STATE_SLEEP;
	printf("Switching to state: STATE_SLEEP\n");
      }
       if(!strncasecmp(line+6,"nothing",7)){
	sm_p->state_user = STATE_NOTHING;
	printf("Switching to state: STATE_NOTHING\n");
      }
      if(!strncasecmp(line+6,"reset",5)){
	sm_p->state_reset=1;
	printf("Resetting State\n");
      }
      if(!DM_ENABLE)
	printf("DM DISABLED\n");
      if(!PZT_ENABLE)
	printf("PZT DISABLED\n");
    }
    
    /***************************************/
    /* FAKE DATA COMMANDS                  */
    /***************************************/
    //change fake mode
    if(!strncasecmp(line,"fake mode",9)){
      if(sscanf(line,"%s %s %d\n",stemp,stemp,&itemp)==3){
	switch(itemp){
	case 0: //Fake data disabled
	  sm_p->fake_mode = 0; 
	  printf("Changing to fake mode %d: Disabled\n",itemp);
	  sm_p->w[CTUID].res=1;
	  printf("Resetting mask\n");
	  sm_p->mask_reset = 1;
	  break;
	case 1: //Send TM test pattern
	  sm_p->fake_mode = FAKE_TM_TEST_PATTERN; 
	  printf("Changing to fake mode %d: TM test pattern\n",itemp);
	  sm_p->w[CTUID].res=1;
	  printf("Resetting mask\n");
	  sm_p->mask_reset = 1;
	  break;
	case 2: //Generate fake images and process (ctu sync)
	  sm_p->fake_mode = FAKE_IMAGES | FAKE_GEN; 
	  printf("Changing to fake mode %d: Generate images w/ CTU sync\n",itemp);
	  sm_p->w[CTUID].res=1;
	  printf("Resetting mask\n");
	  sm_p->mask_reset = 1;
	  break;
	case 3: //Read fake images and process (ctu sync)
	  sm_p->fake_mode = FAKE_IMAGES; 
	  printf("Changing to fake mode %d: Read images w/ CTU sync\n",itemp);
	  sm_p->w[CTUID].res=1;
	  printf("Resetting mask\n");
	  sm_p->mask_reset = 1;
	  break;
	case 4: //Generate fake images and process (timer sync)
	  sm_p->fake_mode = FAKE_IMAGES | FAKE_GEN | FAKE_TIMER; 
	  printf("Changing to fake mode %d: Generate images w/ timer sync\n",itemp);
	  sm_p->w[CTUID].res=1;
	  printf("Resetting mask\n");
	  sm_p->mask_reset = 1;
	  break;
	case 5: //Read fake images and process (timer sync)
	  sm_p->fake_mode = FAKE_IMAGES | FAKE_TIMER; 
	  printf("Changing to fake mode %d: Read images w/ timer sync\n",itemp);
	  sm_p->w[CTUID].res=1;
	  printf("Resetting mask\n");
	  sm_p->mask_reset = 1;
	  break;
	case 6: //Run PZTs & DM ABCD stepping on timer
	  sm_p->fake_mode = FAKE_ABCD; 
	  printf("Changing to fake mode %d: ABCD stepping w/ timer sync\n",itemp);
	  sm_p->w[CTUID].res=1;
	  printf("Resetting mask\n");
	  sm_p->mask_reset = 1;
	  break;
	default:
	  printf("Invalid fake mode: %d\n",itemp);
	  report_fake_modes();
	}
      }
      else report_fake_modes();
    }
    
    //WFS affine on
    if(!strncasecmp(line,"wfs affine on",13)){
      printf("Enabling WFS affine transformations\n");
      sm_p->wfs_affine_enable=1;      
    }

    //WFS affine off
    if(!strncasecmp(line,"wfs affine off",14)){
      printf("Disabling WFS affine transformations\n");
      sm_p->wfs_affine_enable=0;      
    }

    //WFS affine reset
    if(!strncasecmp(line,"wfs affine reset",16)){
      printf("Resetting WFS affine transform\n");
      sm_p->wfs_affine_angle  = WFS_AFFINE_ANGLE_DEFAULT;
      sm_p->wfs_affine_xoff   = WFS_AFFINE_XOFF_DEFAULT;
      sm_p->wfs_affine_yoff   = WFS_AFFINE_YOFF_DEFAULT;
      sm_p->wfs_affine_xscale = WFS_AFFINE_XSCALE_DEFAULT;
      sm_p->wfs_affine_yscale = WFS_AFFINE_YSCALE_DEFAULT;
    }

    //WFS angle
    if(!strncasecmp(line,"wfs affine angle",16)){
      ftemp = atof(line+17);
      sm_p->wfs_affine_angle=ftemp;
      printf("Setting WFS angle: %-.1f degrees\n",sm_p->wfs_affine_angle);
    }

    //WFS xoff
    if(!strncasecmp(line,"wfs affine xoff",15)){
      ftemp = atof(line+16);
      sm_p->wfs_affine_xoff=ftemp;
      printf("Setting WFS X-offset: %-.1f\n",sm_p->wfs_affine_xoff);
    }
  
    //WFS yoff
    if(!strncasecmp(line,"wfs affine yoff",15)){
      ftemp = atof(line+16);
      sm_p->wfs_affine_yoff=ftemp;
      printf("Setting WFS Y-offset: %-.1f\n",sm_p->wfs_affine_yoff);
    }

    //WFS xscale
    if(!strncasecmp(line,"wfs affine xscale",17)){
      ftemp = atof(line+18);
      sm_p->wfs_affine_xscale=ftemp;
      printf("Setting WFS X-scale: %-.1f\n",sm_p->wfs_affine_xscale);
    }
  
    //WFS yscale
    if(!strncasecmp(line,"wfs affine yscale",17)){
      ftemp = atof(line+18);
      sm_p->wfs_affine_yscale=ftemp;
      printf("Setting WFS Y-scale: %-.1f\n",sm_p->wfs_affine_yscale);
    }
    
    /***************************************/
    /* Send Commands to BU Computer        */
    /***************************************/
    if(!strncasecmp(line,"bucmd",5)){
      if(send2bu(line+6,(strlen(line)-6)*sizeof(char)))
	printf("WAT: send2bu failed!\n");
      else
	printf("WAT: sent bucmd: %s\n",line+6);
    }

}
  
 cleanup:

  /* Clean Up */
  printf("WAT: cleaning up...\n");
  //tell all subthreads to die
  sm_p->die = 1;

  //Wait for watchdog to exit
  if(procwait_to(sm_p->w[WATID].pid,EXIT_WAITTIME))
    printf("WAT: cleanup timeout!\n");
  else
    printf("WAT: cleanup done.\n");
  
  close(shmfd);


  //Shutdown JPL CPU
  if(shutdown){
    printf("WAT: Shutting down JPL CPU NOW!\n");
    fflush(stdout); 
    system("shutdown -h now");
    
    //Go to sleep
    sleep(60);
  }
  return 0;
}
	
