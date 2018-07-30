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
#include <dm7820_library.h>

/* piccflight headers */
#include "watchdog.h"
#include "handle_command.h"
#include "controller.h"
#include "common_functions.h"
#include "alp_functions.h"
#include "rtd_functions.h"

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
extern void mot_proc(void); //motor controller
extern void thm_proc(void); //thermal controller
extern void srv_proc(void); //data server
extern void hex_proc(void); //hexapod controller
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
  launch = sm_p->w[id].launch;
  //reset values
  sm_p->w[id].die  =  0;
  sm_p->w[id].done =  0;
  sm_p->w[id].res  =  0;
  sm_p->w[id].chk  =  0;
  sm_p->w[id].rec  =  0;
  sm_p->w[id].cnt  =  0;


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


  /* Initialize Watchdog Structure*/
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


    /*Assign Sub-Processes */
    switch(i){
    case WATID:sm_p->w[i].launch = wat_proc; break;
    case SCIID:sm_p->w[i].launch = sci_proc; break;
    case SHKID:sm_p->w[i].launch = shk_proc; break;
    case LYTID:sm_p->w[i].launch = lyt_proc; break;
    case TLMID:sm_p->w[i].launch = tlm_proc; break;
    case ACQID:sm_p->w[i].launch = acq_proc; break;
    case MOTID:sm_p->w[i].launch = mot_proc; break;
    case THMID:sm_p->w[i].launch = thm_proc; break;
    case SRVID:sm_p->w[i].launch = srv_proc; break;
    case HEXID:sm_p->w[i].launch = hex_proc; break;
    case DIAID:sm_p->w[i].launch = dia_proc; break;
    }
  }

  /* Set Runtime Defaults */
  /* All shmem numbers are ZERO unless defined here */
  sm_p->memlock          = 0;
  sm_p->die              = 0;
  sm_p->state            = STATE_STANDBY;
  sm_p->sci_mode         = SCI_MODE_DEFAULT;
  sm_p->shk_mode         = SHK_MODE_DEFAULT;
  sm_p->lyt_mode         = LYT_MODE_DEFAULT;
  sm_p->acq_mode         = ACQ_MODE_DEFAULT;
  sm_p->shk_boxsize      = SHK_BOXSIZE_DEFAULT;
  sm_p->shk_kP_alp_cell  = SHK_KP_ALP_CELL_DEFAULT;
  sm_p->shk_kI_alp_cell  = SHK_KI_ALP_CELL_DEFAULT;
  sm_p->shk_kD_alp_cell  = SHK_KD_ALP_CELL_DEFAULT;
  sm_p->shk_kP_alp_zern  = SHK_KP_ALP_ZERN_DEFAULT;
  sm_p->shk_kI_alp_zern  = SHK_KI_ALP_ZERN_DEFAULT;
  sm_p->shk_kD_alp_zern  = SHK_KD_ALP_ZERN_DEFAULT;
  sm_p->shk_kP_hex_zern  = SHK_KP_HEX_ZERN_DEFAULT;
  sm_p->shk_kI_hex_zern  = SHK_KI_HEX_ZERN_DEFAULT;
  sm_p->shk_kD_hex_zern  = SHK_KD_HEX_ZERN_DEFAULT;
  sm_p->hex_tilt_correct = HEX_TILT_CORRECT_DEFAULT;
  /* Initialize States */
  for(i=0;i<NSTATES;i++)
    init_state(i,(state_t *)&sm_p->state_array[i]);
  
  /* Configure Circular Buffers */
  //-- Event buffers
  sm_p->circbuf[SCIEVENT].buffer  = (void *)sm_p->scievent;
  sm_p->circbuf[SCIEVENT].nbytes  = sizeof(scievent_t);
  sm_p->circbuf[SCIEVENT].bufsize = SCIEVENTSIZE;
  sprintf((char *)sm_p->circbuf[SCIEVENT].name,"SCIEVENT");
  sm_p->circbuf[SHKEVENT].buffer  = (void *)sm_p->shkevent;
  sm_p->circbuf[SHKEVENT].nbytes  = sizeof(shkevent_t);
  sm_p->circbuf[SHKEVENT].bufsize = SHKEVENTSIZE;
  sprintf((char *)sm_p->circbuf[SHKEVENT].name,"SHKEVENT");
  sm_p->circbuf[LYTEVENT].buffer  = (void *)sm_p->lytevent;
  sm_p->circbuf[LYTEVENT].nbytes  = sizeof(lytevent_t);
  sm_p->circbuf[LYTEVENT].bufsize = LYTEVENTSIZE;
  sprintf((char *)sm_p->circbuf[LYTEVENT].name,"LYTEVENT");
  sm_p->circbuf[ACQEVENT].buffer  = (void *)sm_p->acqevent;
  sm_p->circbuf[ACQEVENT].nbytes  = sizeof(acqevent_t);
  sm_p->circbuf[ACQEVENT].bufsize = ACQEVENTSIZE;
  sprintf((char *)sm_p->circbuf[ACQEVENT].name,"ACQEVENT");
  sm_p->circbuf[HEXRECV].buffer  = (void *)sm_p->hexrecv;
  sm_p->circbuf[HEXRECV].nbytes  = sizeof(hexevent_t);
  sm_p->circbuf[HEXRECV].bufsize = HEXRECVSIZE;
  sprintf((char *)sm_p->circbuf[HEXRECV].name,"HEXRECV");
  sm_p->circbuf[SHK_HEXSEND].buffer  = (void *)sm_p->shk_hexsend;
  sm_p->circbuf[SHK_HEXSEND].nbytes  = sizeof(hexevent_t);
  sm_p->circbuf[SHK_HEXSEND].bufsize = HEXSENDSIZE;
  sprintf((char *)sm_p->circbuf[SHK_HEXSEND].name,"SHK_HEXSEND");
  sm_p->circbuf[LYT_HEXSEND].buffer  = (void *)sm_p->lyt_hexsend;
  sm_p->circbuf[LYT_HEXSEND].nbytes  = sizeof(hexevent_t);
  sm_p->circbuf[LYT_HEXSEND].bufsize = HEXSENDSIZE;
  sprintf((char *)sm_p->circbuf[LYT_HEXSEND].name,"LYT_HEXSEND");
  sm_p->circbuf[ACQ_HEXSEND].buffer  = (void *)sm_p->acq_hexsend;
  sm_p->circbuf[ACQ_HEXSEND].nbytes  = sizeof(hexevent_t);
  sm_p->circbuf[ACQ_HEXSEND].bufsize = HEXSENDSIZE;
  sprintf((char *)sm_p->circbuf[ACQ_HEXSEND].name,"ACQ_HEXSEND");
  sm_p->circbuf[WAT_HEXSEND].buffer  = (void *)sm_p->wat_hexsend;
  sm_p->circbuf[WAT_HEXSEND].nbytes  = sizeof(hexevent_t);
  sm_p->circbuf[WAT_HEXSEND].bufsize = HEXSENDSIZE;
  sprintf((char *)sm_p->circbuf[WAT_HEXSEND].name,"WAT_HEXSEND");
  sm_p->circbuf[SHK_HEXRECV].buffer  = (void *)sm_p->shk_hexrecv;
  sm_p->circbuf[SHK_HEXRECV].nbytes  = sizeof(hexevent_t);
  sm_p->circbuf[SHK_HEXRECV].bufsize = HEXRECVSIZE;
  sprintf((char *)sm_p->circbuf[SHK_HEXRECV].name,"SHK_HEXRECV");
  sm_p->circbuf[LYT_HEXRECV].buffer  = (void *)sm_p->lyt_hexrecv;
  sm_p->circbuf[LYT_HEXRECV].nbytes  = sizeof(hexevent_t);
  sm_p->circbuf[LYT_HEXRECV].bufsize = HEXRECVSIZE;
  sprintf((char *)sm_p->circbuf[LYT_HEXRECV].name,"LYT_HEXRECV");
  sm_p->circbuf[ACQ_HEXRECV].buffer  = (void *)sm_p->acq_hexrecv;
  sm_p->circbuf[ACQ_HEXRECV].nbytes  = sizeof(hexevent_t);
  sm_p->circbuf[ACQ_HEXRECV].bufsize = HEXRECVSIZE;
  sprintf((char *)sm_p->circbuf[ACQ_HEXRECV].name,"ACQ_HEXRECV");
  sm_p->circbuf[WAT_HEXRECV].buffer  = (void *)sm_p->wat_hexrecv;
  sm_p->circbuf[WAT_HEXRECV].nbytes  = sizeof(hexevent_t);
  sm_p->circbuf[WAT_HEXRECV].bufsize = HEXRECVSIZE;
  sprintf((char *)sm_p->circbuf[WAT_HEXRECV].name,"WAT_HEXRECV");
  
  //-- Full frame buffers
  sm_p->circbuf[SCIFULL].buffer  = (void *)sm_p->scifull;
  sm_p->circbuf[SCIFULL].nbytes  = sizeof(scifull_t);
  sm_p->circbuf[SCIFULL].bufsize = SCIFULLSIZE;
  sprintf((char *)sm_p->circbuf[SCIFULL].name,"SCIFULL");
  sm_p->circbuf[SHKFULL].buffer  = (void *)sm_p->shkfull;
  sm_p->circbuf[SHKFULL].nbytes  = sizeof(shkfull_t);
  sm_p->circbuf[SHKFULL].bufsize = SHKFULLSIZE;
  sprintf((char *)sm_p->circbuf[SHKFULL].name,"SHKFULL");
  sm_p->circbuf[LYTFULL].buffer  = (void *)sm_p->lytfull;
  sm_p->circbuf[LYTFULL].nbytes  = sizeof(lytfull_t);
  sm_p->circbuf[LYTFULL].bufsize = LYTFULLSIZE;
  sprintf((char *)sm_p->circbuf[LYTFULL].name,"LYTFULL");
  sm_p->circbuf[ACQFULL].buffer  = (void *)sm_p->acqfull;
  sm_p->circbuf[ACQFULL].nbytes  = sizeof(acqfull_t);
  sm_p->circbuf[ACQFULL].bufsize = ACQFULLSIZE;
  sprintf((char *)sm_p->circbuf[ACQFULL].name,"ACQFULL");
  
  /* Init RTD Driver */
  //Open driver
  if((dm7820_status = rtd_open(0, &sm_p->p_rtd_board)))
    perror("rtd_open");
  
  //Reset board
  if((dm7820_status = rtd_reset(sm_p->p_rtd_board)))
    perror("rtd_reset");
  
  //Install interrupt handler -- pass pointer to shmem
  if((dm7820_status = rtd_install_isr(sm_p->p_rtd_board,sm_p)))
    perror("rtd_install_isr");
    
  
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


  //Close RTD driver
  if((dm7820_status = rtd_close(sm_p->p_rtd_board)))
    perror("rtd_close");

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
