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
#include <libbmc.h>

/* piccflight headers */
#include "watchdog.h"
#include "handle_command.h"
#include "controller.h"
#include "common_functions.h"
#include "alp_functions.h"
#include "rtd_functions.h"
#include "hex_functions.h"
#include "thm_functions.h"
#include "fakemodes.h"

/* STDIN file descriptor */
#define STDIN 0

/* Prototypes */
int handle_command(char *line, sm_t *sm_p);
void init_state(int state_number, state_t *state);
void change_state(sm_t *sm_p, int state);
int getirq(char *driver);
int setirq_affinity(int irq, int proc);

//Flight Processes
extern void sci_proc(void); //science camera
extern void shk_proc(void); //shack-hartmann camera
extern void lyt_proc(void); //lyot lowfs camera
extern void tlm_proc(void); //telemetry
extern void acq_proc(void); //acquisition camera
extern void mtr_proc(void); //motor controller
extern void thm_proc(void); //thermal controller
extern void msg_proc(void); //message capture
extern void dia_proc(void); //diagnostic program

/* Kill Process */
void kill_proc(sm_t *sm_p,int id){
  char rmmod[100] = "modprobe -r ";
  int i;
  int keep_going=1;

  if(WAT_DEBUG) printf("WAT: killing: %s\n",sm_p->w[id].name);
  keep_going=1;
  if(sm_p->w[id].ask){
    //ask process to die
    sm_p->w[id].die=1;
    //wait for process to exit
    if(!procwait(sm_p->w[id].pid,PROC_TIMEOUT)){
      if(WAT_DEBUG) printf("WAT: unloaded: %s\n",sm_p->w[id].name);
      keep_going=0;
    }
  }
  if(keep_going){
    //interrupt process with SIGINT
    kill(sm_p->w[id].pid,SIGINT);
    //wait for process to exit
    if(procwait(sm_p->w[id].pid,PROC_TIMEOUT)){
      //kill process with SIGKILL
      kill(sm_p->w[id].pid,SIGKILL);
      //wait for process to exit
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
  int state;

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
    
    //Check process enable flags
    for(i=0;i<NCLIENTS;i++)
      if(!sm_p->w[i].ena)
	sm_p->w[i].run=0;

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
	    if(sm_p->w[i].res == 1){
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
  char *pch;
  int i;
  int retval;
  int shutdown=0;
  DM7820_Board_Descriptor* p_rtd_alp_board;
  DM7820_Board_Descriptor* p_rtd_tlm_board;
  int irq;
  fd_set readset;
  int fdcmd;
  struct termios t;
  
  
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

  /* Set stdout to line buffering */
  setvbuf(stdout,NULL,_IOLBF,0);

  /* Initialize Watchdog Structure */
  uint8 proctmo[NCLIENTS]  = PROCTMO;
  uint8 procask[NCLIENTS]  = PROCASK;
  uint8 procrun[NCLIENTS]  = PROCRUN;
  char *procnam[NCLIENTS]  = PROCNAM;
  uint8 procper[NCLIENTS]  = PROCPER;
  for(i=0;i<NCLIENTS;i++){
    sm_p->w[i].pid  = -1;
    sm_p->w[i].run  =  procrun[i];
    sm_p->w[i].ena  =  1;
    sm_p->w[i].die  =  0;
    sm_p->w[i].chk  =  0;
    sm_p->w[i].rec  =  0;
    sm_p->w[i].cnt  =  0;
    sm_p->w[i].tmo  =  proctmo[i];
    sm_p->w[i].ask  =  procask[i];
    sm_p->w[i].per  =  procper[i];
    sm_p->w[i].name =  procnam[i];
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
    case MSGID:sm_p->w[i].launch = msg_proc; break;
    case DIAID:sm_p->w[i].launch = dia_proc; break;
    }
  }

  /* Set Runtime Defaults */
  /* All shmem numbers are ZERO unless defined here */
  sm_p->die                  = 0;
  sm_p->sci_exptime          = SCI_EXPTIME_DEFAULT;
  sm_p->sci_frmtime          = SCI_FRMTIME_DEFAULT;
  sm_p->shk_exptime          = SHK_EXPTIME_DEFAULT;
  sm_p->shk_frmtime          = SHK_FRMTIME_DEFAULT;
  sm_p->lyt_exptime          = LYT_EXPTIME_DEFAULT;
  sm_p->lyt_frmtime          = LYT_FRMTIME_DEFAULT;
  sm_p->acq_exptime          = ACQ_EXPTIME_DEFAULT;
  sm_p->acq_frmtime          = ACQ_FRMTIME_DEFAULT;
  sm_p->shk_boxsize          = SHK_BOXSIZE_DEFAULT;
  sm_p->alp_n_dither         = -1;
  sm_p->alp_proc_id          = -1;
  sm_p->lyt_xorigin          = LYT_XORIGIN_DEFAULT;
  sm_p->lyt_yorigin          = LYT_YORIGIN_DEFAULT;
  sm_p->sci_tec_enable       = SCI_TEC_ENABLE_DEFAULT;
  sm_p->sci_tec_setpoint     = SCI_TEC_SETPOINT_DEFAULT;
  sm_p->sci_phase_n_zernike  = SCI_PHASE_N_ZERNIKE_DEFAULT;
  sm_p->sci_phase_expscale   = SCI_PHASE_EXPSCALE_DEFAULT;
  sm_p->sci_optmode          = SCI_OPTMODE_NMSIMPLEX2;
  sm_p->acq_thresh           = ACQ_THRESH_DEFAULT;
  sm_p->thm_enable_vref      = THM_ENABLE_VREF_DEFAULT;
  sm_p->hex_spiral_autostop  = HEX_SPIRAL_AUTOSTOP_DEFAULT;
  sm_p->lyt_mag_enable       = 0;
  sm_p->lyt_mag              = 1;
  sm_p->efc_bmc_max          = EFC_BMC_MAX_DEFAULT;
  sm_p->efc_sci_thresh       = EFC_SCI_THRESH_DEFAULT;
  sm_p->efc_gain             = EFC_GAIN_DEFAULT;
  sm_p->efc_probe_amp        = EFC_PROBE_AMP_DEFAULT;
  sm_p->speckle_scale        = SPECKLE_SCALE_DEFAULT;
  sm_p->alp_cal_scale        = ALP_CAL_SCALE_DEFAULT;
  sm_p->bmc_cal_scale        = BMC_CAL_SCALE_DEFAULT;
  sm_p->alp_cal_timer_length = ALP_CAL_TIMER_LENGTH_DEFAULT;
  sm_p->bmc_cal_timer_length = BMC_CAL_TIMER_LENGTH_DEFAULT;

  //Zernike control defaults
  //--SHK
  for(i=0;i<LOWFS_N_ZERNIKE;i++)
    sm_p->shk_zernike_control[i] = 1;
  //--LYT
  sm_p->lyt_zernike_control[0] = 1;
  sm_p->lyt_zernike_control[1] = 1;
  //--ALP
  for(i=0;i<LOWFS_N_ZERNIKE;i++)
    sm_p->alp_zernike_control[i] = 1;

  //SHK PID Gains
  double shk_gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID] = SHK_GAIN_ALP_ZERN_DEFAULT;
  memcpy((double *)&sm_p->shk_gain_alp_zern[0][0],&shk_gain_alp_zern[0][0],sizeof(shk_gain_alp_zern));
  double shk_gain_alp_cell[LOWFS_N_PID] = SHK_GAIN_ALP_CELL_DEFAULT;
  memcpy((double *)sm_p->shk_gain_alp_cell,shk_gain_alp_cell,sizeof(shk_gain_alp_cell));
  double shk_gain_hex_zern[LOWFS_N_PID] = SHK_GAIN_HEX_ZERN_DEFAULT;
  memcpy((double *)sm_p->shk_gain_hex_zern,shk_gain_hex_zern,sizeof(shk_gain_hex_zern));
  
  //LYT PID Gains
  double lyt_gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID] = LYT_GAIN_ALP_ZERN_DEFAULT;
  memcpy((double *)&sm_p->lyt_gain_alp_zern[0][0],&lyt_gain_alp_zern[0][0],sizeof(lyt_gain_alp_zern));
  
  //LYT ROI
  sm_p->lyt_roi[0] = LYT_ROI_X_DEFAULT;
  sm_p->lyt_roi[1] = LYT_ROI_Y_DEFAULT;
  sm_p->lyt_roi[2] = LYTREADXS;
  sm_p->lyt_roi[3] = LYTREADYS;

  //LYT Centroid Control
  sm_p->lyt_cen_enable = LYT_CEN_ENABLE_DEFAULT;

  //SCI ROI Origin
  uint32 sci_xorigin[SCI_NBANDS] = SCI_XORIGIN_DEFAULT;
  uint32 sci_yorigin[SCI_NBANDS] = SCI_YORIGIN_DEFAULT;
  memcpy((uint32 *)sm_p->sci_xorigin,sci_xorigin,sizeof(sci_xorigin));
  memcpy((uint32 *)sm_p->sci_yorigin,sci_yorigin,sizeof(sci_yorigin));

  /* Initialize States */
  for(i=0;i<NSTATES;i++)
    init_state(i,(state_t *)&sm_p->state_array[i]);

  /* Set startup state */
  change_state(sm_p,STATE_LOW_POWER);
  
  /* Configure Circular Buffers */
  //-- Event buffers
  sm_p->circbuf[BUFFER_SCIEVENT].buffer  = (void *)sm_p->scievent;
  sm_p->circbuf[BUFFER_SCIEVENT].nbytes  = sizeof(scievent_t);
  sm_p->circbuf[BUFFER_SCIEVENT].bufsize = SCIEVENTSIZE;
  sm_p->circbuf[BUFFER_SCIEVENT].write   = WRITE_SCIEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SCIEVENT].read    = READ_SCIEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SCIEVENT].send    = SEND_SCIEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SCIEVENT].save    = SAVE_SCIEVENT_DEFAULT;
  sprintf((char *)sm_p->circbuf[BUFFER_SCIEVENT].name,"scievent");
  sm_p->circbuf[BUFFER_WFSEVENT].buffer  = (void *)sm_p->wfsevent;
  sm_p->circbuf[BUFFER_WFSEVENT].nbytes  = sizeof(wfsevent_t);
  sm_p->circbuf[BUFFER_WFSEVENT].bufsize = WFSEVENTSIZE;
  sm_p->circbuf[BUFFER_WFSEVENT].write   = WRITE_WFSEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_WFSEVENT].read    = READ_WFSEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_WFSEVENT].send    = SEND_WFSEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_WFSEVENT].save    = SAVE_WFSEVENT_DEFAULT;
  sprintf((char *)sm_p->circbuf[BUFFER_WFSEVENT].name,"wfsevent");
  sm_p->circbuf[BUFFER_SHKEVENT].buffer  = (void *)sm_p->shkevent;
  sm_p->circbuf[BUFFER_SHKEVENT].nbytes  = sizeof(shkevent_t);
  sm_p->circbuf[BUFFER_SHKEVENT].bufsize = SHKEVENTSIZE;
  sm_p->circbuf[BUFFER_SHKEVENT].write   = WRITE_SHKEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKEVENT].read    = READ_SHKEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKEVENT].send    = SEND_SHKEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKEVENT].save    = SAVE_SHKEVENT_DEFAULT;
  sprintf((char *)sm_p->circbuf[BUFFER_SHKEVENT].name,"shkevent");
  sm_p->circbuf[BUFFER_LYTEVENT].buffer  = (void *)sm_p->lytevent;
  sm_p->circbuf[BUFFER_LYTEVENT].nbytes  = sizeof(lytevent_t);
  sm_p->circbuf[BUFFER_LYTEVENT].bufsize = LYTEVENTSIZE;
  sm_p->circbuf[BUFFER_LYTEVENT].write   = WRITE_LYTEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTEVENT].read    = READ_LYTEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTEVENT].send    = SEND_LYTEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTEVENT].save    = SAVE_LYTEVENT_DEFAULT;
  sprintf((char *)sm_p->circbuf[BUFFER_LYTEVENT].name,"lytevent");
  sm_p->circbuf[BUFFER_ACQEVENT].buffer  = (void *)sm_p->acqevent;
  sm_p->circbuf[BUFFER_ACQEVENT].nbytes  = sizeof(acqevent_t);
  sm_p->circbuf[BUFFER_ACQEVENT].bufsize = ACQEVENTSIZE;
  sm_p->circbuf[BUFFER_ACQEVENT].write   = WRITE_ACQEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_ACQEVENT].read    = READ_ACQEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_ACQEVENT].send    = SEND_ACQEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_ACQEVENT].save    = SAVE_ACQEVENT_DEFAULT;
  sprintf((char *)sm_p->circbuf[BUFFER_ACQEVENT].name,"acqevent");
  sm_p->circbuf[BUFFER_THMEVENT].buffer  = (void *)sm_p->thmevent;
  sm_p->circbuf[BUFFER_THMEVENT].nbytes  = sizeof(thmevent_t);
  sm_p->circbuf[BUFFER_THMEVENT].bufsize = THMEVENTSIZE;
  sm_p->circbuf[BUFFER_THMEVENT].write   = WRITE_THMEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_THMEVENT].read    = READ_THMEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_THMEVENT].send    = SEND_THMEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_THMEVENT].save    = SAVE_THMEVENT_DEFAULT;
  sprintf((char *)sm_p->circbuf[BUFFER_THMEVENT].name,"thmevent");
  sm_p->circbuf[BUFFER_MTREVENT].buffer  = (void *)sm_p->mtrevent;
  sm_p->circbuf[BUFFER_MTREVENT].nbytes  = sizeof(mtrevent_t);
  sm_p->circbuf[BUFFER_MTREVENT].bufsize = MTREVENTSIZE;
  sm_p->circbuf[BUFFER_MTREVENT].write   = WRITE_MTREVENT_DEFAULT;
  sm_p->circbuf[BUFFER_MTREVENT].read    = READ_MTREVENT_DEFAULT;
  sm_p->circbuf[BUFFER_MTREVENT].send    = SEND_MTREVENT_DEFAULT;
  sm_p->circbuf[BUFFER_MTREVENT].save    = SAVE_MTREVENT_DEFAULT;
  sprintf((char *)sm_p->circbuf[BUFFER_MTREVENT].name,"mtrevent");
  sm_p->circbuf[BUFFER_MSGEVENT].buffer  = (void *)sm_p->msgevent;
  sm_p->circbuf[BUFFER_MSGEVENT].nbytes  = sizeof(msgevent_t);
  sm_p->circbuf[BUFFER_MSGEVENT].bufsize = MSGEVENTSIZE;
  sm_p->circbuf[BUFFER_MSGEVENT].write   = WRITE_MSGEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_MSGEVENT].read    = READ_MSGEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_MSGEVENT].send    = SEND_MSGEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_MSGEVENT].save    = SAVE_MSGEVENT_DEFAULT;
  sprintf((char *)sm_p->circbuf[BUFFER_MSGEVENT].name,"msgevent");

  //-- Packet buffers
  sm_p->circbuf[BUFFER_SHKPKT].buffer  = (void *)sm_p->shkpkt;
  sm_p->circbuf[BUFFER_SHKPKT].nbytes  = sizeof(shkpkt_t);
  sm_p->circbuf[BUFFER_SHKPKT].bufsize = SHKPKTSIZE;
  sm_p->circbuf[BUFFER_SHKPKT].write   = WRITE_SHKPKT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKPKT].read    = READ_SHKPKT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKPKT].send    = SEND_SHKPKT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKPKT].save    = SAVE_SHKPKT_DEFAULT;
  sprintf((char *)sm_p->circbuf[BUFFER_SHKPKT].name,"shkpkt");
  sm_p->circbuf[BUFFER_LYTPKT].buffer  = (void *)sm_p->lytpkt;
  sm_p->circbuf[BUFFER_LYTPKT].nbytes  = sizeof(lytpkt_t);
  sm_p->circbuf[BUFFER_LYTPKT].bufsize = LYTPKTSIZE;
  sm_p->circbuf[BUFFER_LYTPKT].write   = WRITE_LYTPKT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTPKT].read    = READ_LYTPKT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTPKT].send    = SEND_LYTPKT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTPKT].save    = SAVE_LYTPKT_DEFAULT;
  sprintf((char *)sm_p->circbuf[BUFFER_LYTPKT].name,"lytpkt");

  //-- Full frame buffers
  sm_p->circbuf[BUFFER_SHKFULL].buffer  = (void *)sm_p->shkfull;
  sm_p->circbuf[BUFFER_SHKFULL].nbytes  = sizeof(shkfull_t);
  sm_p->circbuf[BUFFER_SHKFULL].bufsize = SHKFULLSIZE;
  sm_p->circbuf[BUFFER_SHKFULL].write   = WRITE_SHKFULL_DEFAULT;
  sm_p->circbuf[BUFFER_SHKFULL].read    = READ_SHKFULL_DEFAULT;
  sm_p->circbuf[BUFFER_SHKFULL].send    = SEND_SHKFULL_DEFAULT;
  sm_p->circbuf[BUFFER_SHKFULL].save    = SAVE_SHKFULL_DEFAULT;
  sprintf((char *)sm_p->circbuf[BUFFER_SHKFULL].name,"shkfull");
  sm_p->circbuf[BUFFER_ACQFULL].buffer  = (void *)sm_p->acqfull;
  sm_p->circbuf[BUFFER_ACQFULL].nbytes  = sizeof(acqfull_t);
  sm_p->circbuf[BUFFER_ACQFULL].bufsize = ACQFULLSIZE;
  sm_p->circbuf[BUFFER_ACQFULL].write   = WRITE_ACQFULL_DEFAULT;
  sm_p->circbuf[BUFFER_ACQFULL].read    = READ_ACQFULL_DEFAULT;
  sm_p->circbuf[BUFFER_ACQFULL].send    = SEND_ACQFULL_DEFAULT;
  sm_p->circbuf[BUFFER_ACQFULL].save    = SAVE_ACQFULL_DEFAULT;
  sprintf((char *)sm_p->circbuf[BUFFER_ACQFULL].name,"acqfull");

  /* Initialize Heater Settings */
  for(i=0;i<SSR_NCHAN;i++)
    thm_init_heater(i,(htr_t *)&sm_p->htr[i]);
   
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

  /* Set IRQ affinity for drivers */
  if((irq = getirq("phx0")) >= 0){
    if(setirq_affinity(irq,CPU_AFFINITY_PHX0))
      printf("WAT: setirq_affinity(%d,%d) failed\n",irq,CPU_AFFINITY_PHX0);
    else
      printf("WAT: phx0 on IRQ: %d CPU: %d\n",irq,CPU_AFFINITY_PHX0);
  }
  if((irq = getirq("phx1")) >= 0){
    if(setirq_affinity(irq,CPU_AFFINITY_PHX1))
      printf("WAT: setirq_affinity(%d,%d) failed\n",irq,CPU_AFFINITY_PHX1);
    else
      printf("WAT: phx1 on IRQ: %d CPU: %d\n",irq,CPU_AFFINITY_PHX1);
  }
  if((irq = getirq("xhci_hcd")) >= 0){
    if(setirq_affinity(irq,CPU_AFFINITY_XHCI_HCD))
      printf("WAT: setirq_affinity(%d,%d) failed\n",irq,CPU_AFFINITY_XHCI_HCD);
    else
      printf("WAT: xhci_hcd on IRQ: %d CPU: %d\n",irq,CPU_AFFINITY_XHCI_HCD);
  }
  
  /* Init RTD Driver for ALP */
  if(ALP_ENABLE){
    printf("WAT: Opening RTD ALP driver\n");
    //Open driver
    if(rtd_open(RTD_ALP_BOARD_MINOR, &p_rtd_alp_board)){
      rtd_close(p_rtd_alp_board);
      perror("WAT: rtd_open (ALP)");
      printf("WAT: ERROR: RTD ALP init failed!\n");
    }
    else{
      //Reset board
      if(rtd_reset(p_rtd_alp_board)){
	rtd_close(p_rtd_alp_board);
	perror("WAT: rtd_reset (ALP)");
	printf("WAT: ERROR: RTD ALP init failed!\n");
      }
      else{
	//Set device handle
	sm_p->p_rtd_alp_board = p_rtd_alp_board;
	sm_p->alp_ready = 1;
	printf("WAT: RTD ALP ready\n");
      }
    }
  }
  
  /* Init RTD Driver for TLM */
  if(TLM_ENABLE){
    printf("WAT: Opening RTD TLM driver\n");
    if(RTD_TLM_BOARD_MINOR == RTD_ALP_BOARD_MINOR && sm_p->alp_ready){
      //Using a single RTD board and it was already opened by ALP
      p_rtd_tlm_board = p_rtd_alp_board;
      sm_p->p_rtd_tlm_board = p_rtd_tlm_board;
      sm_p->tlm_ready = 1;
      printf("WAT: RTD TLM ready\n");
    }
    else{
      //Open driver
      if(rtd_open(RTD_TLM_BOARD_MINOR, &p_rtd_tlm_board)){
	rtd_close(p_rtd_tlm_board);
	perror("WAT: rtd_open (TLM)");
	printf("WAT: ERROR: RTD TLM init failed!\n");
      }
      else{
	//Reset board
	if(rtd_reset(p_rtd_tlm_board)){
	  rtd_close(p_rtd_tlm_board);
	  perror("WAT: rtd_reset (TLM)");
	  printf("WAT: ERROR: RTD TLM init failed!\n");
	}
	else{
	  //Set device handle
	  sm_p->p_rtd_tlm_board = p_rtd_tlm_board;
	  sm_p->tlm_ready = 1;
	  printf("WAT: RTD TLM ready\n");
	}
      }
    }
  }

  /* Init HEX Driver */
  if(HEX_ENABLE){
    printf("WAT: Opening HEX driver\n");
    if(hex_init((int *)&sm_p->hexfd)){
      sm_p->hex_ready = 0;
      printf("WAT: ERROR: HEX init failed!\n");
    }
    else{
      sm_p->hex_ready = 1;
      printf("WAT: HEX ready\n");
    }
  }

  /* Initialize BMC DM */
  if(BMC_ENABLE){
    /* Open Device */
    if((retval = libbmc_open_device((libbmc_device_t *)&sm_p->libbmc_device)) < 0){
      printf("WAT: Failed to find the bmc device: %s - %s \n", libbmc_error_name(retval), libbmc_strerror(retval));
      sm_p->bmc_ready = 0;
    }else{
      printf("WAT: BMC device found and opened\n");
      sm_p->bmc_ready = 1;
      printf("WAT: BMC ready\n");
    }
    /* Turn off LEDs */
    if(libbmc_toggle_leds_off((libbmc_device_t *)&sm_p->libbmc_device))
      printf("WAT: ERROR (libbmc_toggle_leds_off)\n");
    usleep(LIBBMC_LONG_USLEEP);
    /* Disable HV by default*/
    sm_p->bmc_hv_enable = 0;
  }

  
  /* Set initial ALP position */
  if(sm_p->alp_ready){
    if(alp_load_flat(sm_p,WATID)){
      if(alp_revert_flat(sm_p,WATID)){
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
  
  
  /* Open command uplink */
  if ((fdcmd = open(UPLINK_DEVICE,O_RDONLY))<0) {
    printf("WAT: Cannot open %s\n",UPLINK_DEVICE);
  }

  /* Configure command uplink interface */
  tcgetattr(fdcmd,&t);
  t.c_iflag = 0;
  t.c_oflag = 0;
  t.c_cflag = CS8 | CREAD | CLOCAL  ;
  t.c_lflag = 0;
  t.c_line = 0;
  cfsetospeed(&t,B1200);
  cfsetispeed(&t,B1200);
  tcsetattr(fdcmd,TCSANOW,&t);
  
  
  /* Launch Watchdog */
  if(sm_p->w[WATID].run){
    if(sm_p->w[WATID].pid == -1){
      //launch process
      launch_proc(sm_p,WATID);
    }
  }
  
  /* Enter foreground loop and wait for commands */
  while(1){
    //Init retval
    retval = CMD_NORMAL;
    
    /* Configure readset */
    FD_ZERO(&readset);
    FD_SET(STDIN,&readset); //console input
    FD_SET(fdcmd,&readset); //command uplink

    //Clear line
    memset(line,0,sizeof(line));
    
    //Select on readset with no timeout (blocking)
    if(select(FD_SETSIZE,&readset,NULL,NULL,NULL) < 0){
      perror("select");
    }
    for(i=0;i<FD_SETSIZE;i++){
      if(FD_ISSET(i,&readset)){
	if(i == STDIN){
	  //Process standard command
	  if(fgets(line,CMD_MAX_LENGTH,stdin) != NULL)
	    retval = handle_command(line,sm_p);
	  break;
	}
	if(i == fdcmd){
	  //Process uplink command
	  if(read_uplink(line,CMD_MAX_LENGTH,fdcmd) > 0){
	    //Loop over embedded carriage returns
	    pch = strtok(line,"\n");
	    while(pch != NULL){
	      retval = handle_command(pch,sm_p);
	      pch = strtok(NULL,"\n");
	    }
	  }
	  break;
	}
      }
    }
    
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
    if(retval == CMD_REBOOT){
      //Exit watchdog, reboot cpu
      shutdown=2;
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
    printf("WAT: process cleanup timeout!\n");
  
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

  //Cleanup RTD ALP
  if(ALP_ENABLE){
    if(sm_p->alp_ready){
      if(rtd_alp_cleanup(p_rtd_alp_board))
	perror("rtd_alp_cleanup");
    }
  }
  
  //Cleanup RTD TLM
  if(TLM_ENABLE){
    if(sm_p->tlm_ready){
      if(rtd_tlm_cleanup(p_rtd_tlm_board))
	perror("rtd_tlm_cleanup");
    }
  }
  
  //Close RTD ALP driver
  if(ALP_ENABLE){
    if(sm_p->alp_ready){
      if(rtd_close(p_rtd_alp_board))
	perror("rtd_close");
      else
	printf("WAT: RTD ALP closed\n");
    }
  }
  
  //Close RTD TLM driver
  if(TLM_ENABLE){
    if(sm_p->tlm_ready){
      if(RTD_TLM_BOARD_MINOR == RTD_ALP_BOARD_MINOR && sm_p->alp_ready){
	//Using a single RTD board and it was already closed by ALP
	printf("WAT: RTD TLM closed\n");
      }else{
	if(rtd_close(p_rtd_tlm_board))
	  perror("rtd_close");
	else
	  printf("WAT: RTD TLM closed\n");
      }
    }
  }
  
  //Cleanup HEX
  if(HEX_ENABLE){
    if(sm_p->hex_ready){
      hex_disconnect(sm_p->hexfd);
      printf("WAT: HEX closed\n");
    }
  }

  //Cleanup BMC
  if(BMC_ENABLE){
    if(sm_p->bmc_ready){
      /* Turn BMC HV off */
      libbmc_hv_off((libbmc_device_t *)&sm_p->libbmc_device);
      /* Turn ON LEDs */
      if(libbmc_toggle_leds_on((libbmc_device_t *)&sm_p->libbmc_device))
	printf("WAT: ERROR (libbmc_toggle_leds_on)\n");
      usleep(LIBBMC_LONG_USLEEP);
      /* Close BMC device */
      libbmc_close_device((libbmc_device_t *)&sm_p->libbmc_device);
      printf("WAT: BMC closed\n");
    }
  }

  //Close shared memory
  close(shmfd);

  //Print
  printf("WAT: cleanup done\n");
  
  //Shutdown CPU
  if(shutdown==1){
    printf("WAT: Shutting down CPU NOW!\n");
    fflush(stdout);
    if(system("shutdown -h now"))
      printf("WAT: shutdown command failed! (%d)\n",errno);

    //Go to sleep
    sleep(60);
  }
  if(shutdown==2){
    printf("WAT: Rebooting CPU NOW!\n");
    fflush(stdout);
    if(system("reboot"))
      printf("WAT: reboot command failed! (%d)\n",errno);

    //Go to sleep
    sleep(60);
  }
  return 0;
}
