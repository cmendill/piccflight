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
#include <sys/prctl.h>
#include <dm7820_library.h>

/* piccflight headers */
#include "watchdog.h"
#include "handle_command.h"
#include "controller.h"
#include "common_functions.h"
#include "alp_functions.h"
#include "rtd_functions.h"
#include "hex_functions.h"
#include "fakemodes.h"

/* Constants */
#define STDIN 0  // file descriptor for standard input

/* Prototypes */
int handle_command(char *line, sm_t *sm_p);
void init_state(int state_number, state_t *state);

//Flight Processes
extern void sci_proc(void); //science camera
extern void shk_proc(void); //shack-hartmann camera
extern void lyt_proc(void); //lyot lowfs camera
extern void tlm_proc(void); //telemetry
extern void acq_proc(void); //acquisition camera
extern void mtr_proc(void); //motor controller
extern void thm_proc(void); //thermal controller
extern void srv_proc(void); //data server
extern void dia_proc(void); //diagnostic program

/* Kill Process */
void kill_proc(sm_t *sm_p,int id){
  char rmmod[100] = "modprobe -r ";
  int i;
  int keep_going=1;

  if(WAT_DEBUG) printf("WAT: killing: %s\n",sm_p->w[id].name);
  keep_going=1;
  if(sm_p->w[id].ask){
    //wait for process to kill itself
    sm_p->w[id].die=1;
    if(!procwait(sm_p->w[id].pid,PROC_TIMEOUT)){
      if(WAT_DEBUG) printf("WAT: unloaded: %s\n",sm_p->w[id].name);
      keep_going=0;
    }
  }
  if(keep_going){
    //ask process to die
    kill(sm_p->w[id].pid,SIGINT);
    //wait for process to die
    if(procwait(sm_p->w[id].pid,PROC_TIMEOUT)){
      //kill process
      kill(sm_p->w[id].pid,SIGKILL);
      //wait for process to die
      if(procwait(sm_p->w[id].pid,PROC_TIMEOUT)){
	printf(WARNING);
	printf("WAT: could not kill %s!\n",sm_p->w[id].name);
      }
      else{
	printf("WAT: unloaded with SIGKILL: %s\n",sm_p->w[id].name);
      }
    }
    else{
      if(WAT_DEBUG) printf("WAT: unloaded with SIGINT: %s\n",sm_p->w[id].name);
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
  char procname[100];

  launch = sm_p->w[id].launch;
  //reset values
  sm_p->w[id].die  =  0;
  sm_p->w[id].done =  0;
  sm_p->w[id].res  =  0;
  sm_p->w[id].chk  =  0;
  sm_p->w[id].rec  =  0;
  sm_p->w[id].cnt  =  0;

  //set procname
  sprintf(procname,"picc_%s",sm_p->w[id].name);

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
    //--set the process name (as shown in top)
    prctl(PR_SET_NAME, (unsigned long)procname, 0, 0, 0);
    //--launch the process routine
    launch();
    //--kill the process when the routine returns
    exit(0);
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
      if(i != WATID)
	if(sm_p->w[i].pid != -1)
	  if(waitpid(sm_p->w[i].pid,NULL,WNOHANG) == sm_p->w[i].pid){
	    sm_p->w[i].pid = -1;
	    printf("WAT: %s crashed!\n",sm_p->w[i].name);
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
    if(WAT_DEBUG){
      printf("\n");
      for(i=0;i<NCLIENTS;i++){
	if(i != WATID){
	  printf("WAT: %s chk:rec:cnt:tmo = %d:%d:%d:%d\n",sm_p->w[i].name,sm_p->w[i].chk,sm_p->w[i].rec,sm_p->w[i].cnt,sm_p->w[i].tmo);
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

/**********************************
 *     MAIN WATCHDOG PROGRAM
 *********************************/
int main(int argc,char **argv){
  char line[CMD_MAX_LENGTH];
  int i;
  int retval;
  int shutdown=0;
  DM7820_Error dm7820_status;
  DM7820_Board_Descriptor* p_rtd_board;
  int hexfd;

  /* Open Shared Memory */
  sm_t *sm_p;
  int shmfd;
  if((sm_p = openshm(&shmfd)) == NULL){
    printf(WARNING);
    printf("openshm fail: main\n");
    close(shmfd);
    exit(0);
  }

  /* Erase Shared Memory */
  memset((char *)sm_p,0,sizeof(sm_t));


  /* Initialize Watchdog Structure */
  uint8 proctmo[NCLIENTS]  = PROCTMO;
  uint8 procask[NCLIENTS]  = PROCASK;
  uint8 procrun[NCLIENTS]  = PROCRUN;
  char *procnam[NCLIENTS]  = PROCNAM;
  char *procmod[NCLIENTS]  = PROCMOD;
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
    sm_p->w[i].per  =  procper[i];
    sm_p->w[i].pri  =  procpri[i];
    sm_p->w[i].name =  procnam[i];
    sm_p->w[i].mod  =  procmod[i];
    sm_p->w[i].reset    = 0;
    sm_p->w[i].fakemode = FAKEMODE_NONE;


    /* Assign Sub-Processes */
    switch(i){
    case WATID:sm_p->w[i].launch = wat_proc; break;
    case SCIID:sm_p->w[i].launch = sci_proc; break;
    case SHKID:sm_p->w[i].launch = shk_proc; break;
    case LYTID:sm_p->w[i].launch = lyt_proc; break;
    case TLMID:sm_p->w[i].launch = tlm_proc; break;
    case ACQID:sm_p->w[i].launch = acq_proc; break;
    case MTRID:sm_p->w[i].launch = mtr_proc; break;
    case THMID:sm_p->w[i].launch = thm_proc; break;
    case SRVID:sm_p->w[i].launch = srv_proc; break;
    case DIAID:sm_p->w[i].launch = dia_proc; break;
    }
  }

  /* Set Runtime Defaults */
  /* All shmem numbers are ZERO unless defined here */
  sm_p->memlock            = 0;
  sm_p->die                = 0;
  sm_p->state              = STATE_STANDBY;
  sm_p->sci_exptime        = SCI_EXPTIME_DEFAULT;
  sm_p->shk_exptime        = SHK_EXPTIME_DEFAULT;
  sm_p->lyt_exptime        = LYT_EXPTIME_DEFAULT;
  sm_p->acq_exptime        = ACQ_EXPTIME_DEFAULT;
  sm_p->shk_boxsize        = SHK_BOXSIZE_DEFAULT;
  sm_p->hex_tilt_correct   = HEX_TILT_CORRECT_DEFAULT;
  sm_p->alp_n_dither       = -1;
  sm_p->alp_proc_id        = -1;
  
  //Enable control of all zernikes by default
  for(i=0;i<LOWFS_N_ZERNIKE;i++) sm_p->zernike_control[i] = 1;

  //SHK PID Gains
  float shk_gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID] = SHK_GAIN_ALP_ZERN_DEFAULT;
  memcpy((float *)&sm_p->shk_gain_alp_zern[0][0],&shk_gain_alp_zern[0][0],sizeof(shk_gain_alp_zern));
  float shk_gain_alp_cell[LOWFS_N_PID] = SHK_GAIN_ALP_CELL_DEFAULT;
  memcpy((float *)sm_p->shk_gain_alp_cell,shk_gain_alp_cell,sizeof(shk_gain_alp_cell));
  float shk_gain_hex_zern[LOWFS_N_PID] = SHK_GAIN_HEX_ZERN_DEFAULT;
  memcpy((float *)sm_p->shk_gain_hex_zern,shk_gain_hex_zern,sizeof(shk_gain_hex_zern));

  //LYT PID Gains
  float lyt_gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID] = LYT_GAIN_ALP_ZERN_DEFAULT;
  memcpy((float *)&sm_p->lyt_gain_alp_zern[0][0],&lyt_gain_alp_zern[0][0],sizeof(lyt_gain_alp_zern));
  float lyt_gain_alp_act[LOWFS_N_PID] = LYT_GAIN_ALP_ACT_DEFAULT;
  memcpy((float *)&sm_p->lyt_gain_alp_act,&lyt_gain_alp_act,sizeof(lyt_gain_alp_act));
  
  /* Initialize States */
  for(i=0;i<NSTATES;i++)
    init_state(i,(state_t *)&sm_p->state_array[i]);

  /* Configure Circular Buffers */
  //-- Event buffers
  sm_p->circbuf[BUFFER_SCIEVENT].buffer  = (void *)sm_p->scievent;
  sm_p->circbuf[BUFFER_SCIEVENT].nbytes  = sizeof(scievent_t);
  sm_p->circbuf[BUFFER_SCIEVENT].bufsize = SCIEVENTSIZE;
  sprintf((char *)sm_p->circbuf[BUFFER_SCIEVENT].name,"SCIEVENT");
  sm_p->circbuf[BUFFER_SHKEVENT].buffer  = (void *)sm_p->shkevent;
  sm_p->circbuf[BUFFER_SHKEVENT].nbytes  = sizeof(shkevent_t);
  sm_p->circbuf[BUFFER_SHKEVENT].bufsize = SHKEVENTSIZE;
  sprintf((char *)sm_p->circbuf[BUFFER_SHKEVENT].name,"SHKEVENT");
  sm_p->circbuf[BUFFER_LYTEVENT].buffer  = (void *)sm_p->lytevent;
  sm_p->circbuf[BUFFER_LYTEVENT].nbytes  = sizeof(lytevent_t);
  sm_p->circbuf[BUFFER_LYTEVENT].bufsize = LYTEVENTSIZE;
  sprintf((char *)sm_p->circbuf[BUFFER_LYTEVENT].name,"LYTEVENT");
  sm_p->circbuf[BUFFER_ACQEVENT].buffer  = (void *)sm_p->acqevent;
  sm_p->circbuf[BUFFER_ACQEVENT].nbytes  = sizeof(acqevent_t);
  sm_p->circbuf[BUFFER_ACQEVENT].bufsize = ACQEVENTSIZE;
  sprintf((char *)sm_p->circbuf[BUFFER_ACQEVENT].name,"ACQEVENT");
  sm_p->circbuf[BUFFER_THMEVENT].buffer  = (void *)sm_p->thmevent;
  sm_p->circbuf[BUFFER_THMEVENT].nbytes  = sizeof(thmevent_t);
  sm_p->circbuf[BUFFER_THMEVENT].bufsize = THMEVENTSIZE;
  sprintf((char *)sm_p->circbuf[BUFFER_MTREVENT].name,"THMEVENT");
  sm_p->circbuf[BUFFER_MTREVENT].buffer  = (void *)sm_p->mtrevent;
  sm_p->circbuf[BUFFER_MTREVENT].nbytes  = sizeof(mtrevent_t);
  sm_p->circbuf[BUFFER_MTREVENT].bufsize = MTREVENTSIZE;
  sprintf((char *)sm_p->circbuf[BUFFER_MTREVENT].name,"MTREVENT");

  //-- Full frame buffers
  sm_p->circbuf[BUFFER_SCIFULL].buffer  = (void *)sm_p->scifull;
  sm_p->circbuf[BUFFER_SCIFULL].nbytes  = sizeof(scifull_t);
  sm_p->circbuf[BUFFER_SCIFULL].bufsize = SCIFULLSIZE;
  sprintf((char *)sm_p->circbuf[BUFFER_SCIFULL].name,"SCIFULL");
  sm_p->circbuf[BUFFER_SHKFULL].buffer  = (void *)sm_p->shkfull;
  sm_p->circbuf[BUFFER_SHKFULL].nbytes  = sizeof(shkfull_t);
  sm_p->circbuf[BUFFER_SHKFULL].bufsize = SHKFULLSIZE;
  sprintf((char *)sm_p->circbuf[BUFFER_SHKFULL].name,"SHKFULL");
  sm_p->circbuf[BUFFER_LYTFULL].buffer  = (void *)sm_p->lytfull;
  sm_p->circbuf[BUFFER_LYTFULL].nbytes  = sizeof(lytfull_t);
  sm_p->circbuf[BUFFER_LYTFULL].bufsize = LYTFULLSIZE;
  sprintf((char *)sm_p->circbuf[BUFFER_LYTFULL].name,"LYTFULL");
  sm_p->circbuf[BUFFER_ACQFULL].buffer  = (void *)sm_p->acqfull;
  sm_p->circbuf[BUFFER_ACQFULL].nbytes  = sizeof(acqfull_t);
  sm_p->circbuf[BUFFER_ACQFULL].bufsize = ACQFULLSIZE;
  sprintf((char *)sm_p->circbuf[BUFFER_ACQFULL].name,"ACQFULL");

  /* Setup IO Permissions for all ISA boards */
  if(ioperm(REL_BASE,REL_IOPORT_LENGTH,1)){
    perror("WAT: REL ioperm()");
  }
  if(ioperm(SSR_BASE,SSR_IOPORT_LENGTH,1)){
    perror("WAT: SSR ioperm()");
  }
  if(ioperm(ADC1_BASE,ADC_IOPORT_LENGTH,1)){
    perror("WAT: ADC1 ioperm()");
  }
  if(ioperm(ADC2_BASE,ADC_IOPORT_LENGTH,1)){
    perror("WAT: ADC2 ioperm()");
  }
  if(ioperm(ADC3_BASE,ADC_IOPORT_LENGTH,1)){
    perror("WAT: ADC3 ioperm()");
  }

  
  /* Setup Debugging DIO Pulses */
  if(DIO_ENABLE){
    //configure ADC DIO output
    outb(ADC_PORTA_PAGE,ADC2_BASE+ADC_PAGE_OFFSET);
    outb(ADC_PORTA_CONFIG,ADC2_BASE+ADC_PORTA_CONFIG_OFFSET);
    //set DIO ready
    sm_p->dio_ready=1;
  }

  /* Init RTD Driver */
  if(ALP_ENABLE || TLM_ENABLE){
    printf("WAT: Opening RTD driver\n");
    //Open driver
    if((dm7820_status = rtd_open(RTD_BOARD_MINOR, &p_rtd_board))){
      perror("WAT: rtd_open");
      printf("WAT: ERROR: RTD init failed!\n");
    }
    else{
      //Reset board
      if((dm7820_status = rtd_reset(p_rtd_board))){
	perror("WAT: rtd_reset");
	printf("WAT: ERROR: RTD init failed!\n");
      }
      else{
	//Set device handle
	sm_p->p_rtd_board = p_rtd_board;
	if(ALP_ENABLE) sm_p->alp_ready = 1;
	if(TLM_ENABLE) sm_p->tlm_ready = 1;
	printf("WAT: RTD ready\n");
      }
    }
  }

  /* Init HEX Driver */
  if(HEX_ENABLE){
    printf("WAT: Opening HEX driver\n");
    if(hex_init(&hexfd)){
      perror("hex_init");
      printf("WAT: ERROR: HEX init failed!\n");
    }
    else{
      sm_p->hexfd = hexfd;
      sm_p->hex_ready = 1;
      printf("WAT: HEX ready\n");
    }
  }
  
  /* Set initial ALP position */
  if(sm_p->alp_ready){
    if(alp_load_flat(sm_p,WATID)==0){
      if(alp_revert_flat(sm_p,WATID)==0){
	printf("WAT: ERROR: alp_load_flat & alp_revert_flat failed!\n");
      }
      else{
	printf("WAT: Set ALP to default flat\n");
      }
    }
    else{
      printf("WAT: Loaded ALP flat from file\n");
    }
  }

  /* Launch Watchdog */
  if(sm_p->w[WATID].run){
    if(sm_p->w[WATID].pid == -1){
      //launch process
      launch_proc(sm_p,WATID);
    }
  }

  /* Enter foreground loop and wait for kill signal */
  while(1){
    /* The foreground will now wait for an input from the console */
    retval=CMD_NORMAL;
    if(fgets(line,CMD_MAX_LENGTH,stdin) != NULL)
      retval = handle_command(line,sm_p);

    /* Check return value */
    if(retval == CMD_NORMAL){
      //Normal command -- do nothing
    }
    if(retval == CMD_NOT_FOUND){
      //Bad command
      printf("Command not found: %s\n",line);
    }
    if(retval == CMD_EXIT_WATCHDOG){
      //Exit watchdog, no shutdown
      shutdown=0;
      break;
    }
    if(retval == CMD_SHUTDOWN){
      //Exit watchdog, shutdown cpu
      shutdown=1;
      break;
    }


}

 cleanup:

  /* Clean Up */
  printf("WAT: cleaning up...\n");
  //tell all subthreads to die
  sm_p->die = 1;

  //Wait for watchdog to exit
  if(procwait(sm_p->w[WATID].pid,EXIT_TIMEOUT))
    printf("WAT: cleanup timeout!\n");
  else
    printf("WAT: cleanup done.\n");

  //Clean up IOPERM
  if(ioperm(REL_BASE,REL_IOPORT_LENGTH,0)){
    perror("WAT: REL ioperm()");
  }
  if(ioperm(SSR_BASE,SSR_IOPORT_LENGTH,0)){
    perror("WAT: SSR ioperm()");
  }
  if(ioperm(ADC1_BASE,ADC_IOPORT_LENGTH,0)){
    perror("WAT: ADC1 ioperm()");
  }
  if(ioperm(ADC2_BASE,ADC_IOPORT_LENGTH,0)){
    perror("WAT: ADC2 ioperm()");
  }
  if(ioperm(ADC3_BASE,ADC_IOPORT_LENGTH,0)){
    perror("WAT: ADC3 ioperm()");
  }

  //Cleanup RTD
  if(ALP_ENABLE || TLM_ENABLE){
    if((dm7820_status = rtd_alp_cleanup(p_rtd_board)))
      perror("rtd_alp_cleanup");
    if((dm7820_status = rtd_tlm_cleanup(p_rtd_board)))
      perror("rtd_tlm_cleanup");

    //Close RTD driver
    if((dm7820_status = rtd_close(p_rtd_board)))
      perror("rtd_close");
    else
      printf("WAT: RTD closed\n");

  }

  //Cleanup HEX
  if(HEX_ENABLE){
    hex_disconnect(hexfd);
    printf("WAT: HEX closed\n");
  }

  //Close shared memory
  close(shmfd);


  //Shutdown CPU
  if(shutdown){
    printf("WAT: Shutting down CPU NOW!\n");
    fflush(stdout);
    if(system("shutdown -h now"))
      printf("WAT: shutdown command failed! (%d)\n",errno);

    //Go to sleep
    sleep(60);
  }
  return 0;
}
