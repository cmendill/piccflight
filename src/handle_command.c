#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/io.h>
#include <libbmc.h>

/* piccflight headers */
#include "watchdog.h"
#include "handle_command.h"
#include "controller.h"
#include "hex_functions.h"
#include "alp_functions.h"
#include "bmc_functions.h"
#include "tgt_functions.h"
#include "common_functions.h"
#include "fakemodes.h"

/* Prototypes */
void getshk_proc(void); //get shkevents
void getlyt_proc(void); //get lytevents
void getsci_proc(void); //get scievents
void init_fakemode(int fakemode, calmode_t *fake);
void change_state(sm_t *sm_p, int state);
void sci_init_phasemode(int phasemode, phasemode_t *sci);
void sci_init_optmode(int optmode, optmode_t *sci);

/**************************************************************/
/* PRINT_PROC_STATUS                                          */
/*  - Print out current process status                        */
/**************************************************************/
void print_proc_status(sm_t *sm_p){
  int i;
  char run[10];
  char ena[10];
  printf("************* Process Status *************\n");
  printf("%-10s %-10s %-10s\n","Process","Enabled","Running");
  for(i=0;i<NCLIENTS;i++){
    if(sm_p->w[i].run) sprintf(run,"YES"); else sprintf(run,"NO");
    if(sm_p->w[i].ena) sprintf(ena,"YES"); else sprintf(ena,"NO");
    printf("%-10s %-10s %-10s\n",sm_p->w[i].name,ena,run);
  }
  printf("******************************************\n");
}

/**************************************************************/
/* PRINT_CIRCBUF_STATUS                                       */
/*  - Print out current circular buffer status                */
/**************************************************************/
void print_circbuf_status(sm_t *sm_p){
  int i;
  char write[10];
  char read[10];
  char save[10];
  char send[10];
  printf("******************* Buffer Status *******************\n");
  printf("%-10s %-10s %-10s %-10s %-10s\n","Buffer","Write","Read","Save","Send");
  for(i=0;i<NCIRCBUF;i++){
    if(sm_p->circbuf[i].write) sprintf(write,"YES"); else sprintf(write,"NO");
    if(sm_p->circbuf[i].save) sprintf(save,"YES"); else sprintf(save,"NO");
    if(sm_p->circbuf[i].send) sprintf(send,"YES"); else sprintf(send,"NO");
    if(sm_p->circbuf[i].read==0) sprintf(read,"NO");
    if(sm_p->circbuf[i].read==1) sprintf(read,"YES");
    if(sm_p->circbuf[i].read==2) sprintf(read,"NEW");
    
    printf("%-10s %-10s %-10s %-10s %-10s\n",sm_p->circbuf[i].name,write,read,save,send);
  }
  printf("*****************************************************\n");
}

/**************************************************************/
/* RESET_CIRCBUF                                              */
/*  - Resets circular buffer settings                         */
/**************************************************************/
void reset_circbuf(sm_t *sm_p){
  sm_p->circbuf[BUFFER_SCIEVENT].write   = WRITE_SCIEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SCIEVENT].read    = READ_SCIEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SCIEVENT].send    = SEND_SCIEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SCIEVENT].save    = SAVE_SCIEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_WFSEVENT].write   = WRITE_WFSEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_WFSEVENT].read    = READ_WFSEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_WFSEVENT].send    = SEND_WFSEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_WFSEVENT].save    = SAVE_WFSEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKEVENT].write   = WRITE_SHKEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKEVENT].read    = READ_SHKEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKEVENT].send    = SEND_SHKEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKEVENT].save    = SAVE_SHKEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTEVENT].write   = WRITE_LYTEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTEVENT].read    = READ_LYTEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTEVENT].send    = SEND_LYTEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTEVENT].save    = SAVE_LYTEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_ACQEVENT].write   = WRITE_ACQEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_ACQEVENT].read    = READ_ACQEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_ACQEVENT].send    = SEND_ACQEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_ACQEVENT].save    = SAVE_ACQEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_THMEVENT].write   = WRITE_THMEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_THMEVENT].read    = READ_THMEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_THMEVENT].send    = SEND_THMEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_THMEVENT].save    = SAVE_THMEVENT_DEFAULT;
  sm_p->circbuf[BUFFER_MTREVENT].write   = WRITE_MTREVENT_DEFAULT;
  sm_p->circbuf[BUFFER_MTREVENT].read    = READ_MTREVENT_DEFAULT;
  sm_p->circbuf[BUFFER_MTREVENT].send    = SEND_MTREVENT_DEFAULT;
  sm_p->circbuf[BUFFER_MTREVENT].save    = SAVE_MTREVENT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKPKT].write     = WRITE_SHKPKT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKPKT].read      = READ_SHKPKT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKPKT].send      = SEND_SHKPKT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKPKT].save      = SAVE_SHKPKT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTPKT].write     = WRITE_LYTPKT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTPKT].read      = READ_LYTPKT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTPKT].send      = SEND_LYTPKT_DEFAULT;
  sm_p->circbuf[BUFFER_LYTPKT].save      = SAVE_LYTPKT_DEFAULT;
  sm_p->circbuf[BUFFER_SHKFULL].write    = WRITE_SHKFULL_DEFAULT;
  sm_p->circbuf[BUFFER_SHKFULL].read     = READ_SHKFULL_DEFAULT;
  sm_p->circbuf[BUFFER_SHKFULL].send     = SEND_SHKFULL_DEFAULT;
  sm_p->circbuf[BUFFER_SHKFULL].save     = SAVE_SHKFULL_DEFAULT;
  sm_p->circbuf[BUFFER_ACQFULL].write    = WRITE_ACQFULL_DEFAULT;
  sm_p->circbuf[BUFFER_ACQFULL].read     = READ_ACQFULL_DEFAULT;
  sm_p->circbuf[BUFFER_ACQFULL].send     = SEND_ACQFULL_DEFAULT;
  sm_p->circbuf[BUFFER_ACQFULL].save     = SAVE_ACQFULL_DEFAULT;
}

/**************************************************************/
/* PRINT_STATES                                               */
/*  - Print out available states                              */
/**************************************************************/
void print_states(state_t *state, int current){
  int i;
  printf("************ Available States ************\n");
  printf("#    Command    Name\n");
  for(i=0;i<NSTATES;i++){
    if(current == i)
      printf("%02d   %-7s   *%s\n",i,state[i].cmd,state[i].name);
    else
      printf("%02d   %-7s    %s\n",i,state[i].cmd,state[i].name);
  }
  printf("******************************************\n");

}

/**************************************************************/
/* PRINT_SCI_PHASEMODES                                         */
/*  - Print available SCI phasemodes                          */
/**************************************************************/
void print_sci_phasemodes(phasemode_t *sci, int current){
  int i;
  printf("************ Available SCI Phasemodes  ************\n");
  printf("#    Command    Name\n");
  for(i=0;i<SCI_NPHASEMODES;i++){
    if(current == i)
      printf("%02d   %-7s   *%s\n",i,sci[i].cmd,sci[i].name);
    else
      printf("%02d   %-7s    %s\n",i,sci[i].cmd,sci[i].name);
  }
  printf("*************************************************\n");
}

/**************************************************************/
/* PRINT_SCI_OPTMODES                                         */
/*  - Print available SCI optmodes                          */
/**************************************************************/
void print_sci_optmodes(optmode_t *sci, int current){
  int i;
  printf("************ Available SCI Optmodes  ************\n");
  printf("#    Command    Name\n");
  for(i=0;i<SCI_NOPTMODES;i++){
    if(current == i)
      printf("%02d   %-7s   *%s\n",i,sci[i].cmd,sci[i].name);
    else
      printf("%02d   %-7s    %s\n",i,sci[i].cmd,sci[i].name);
  }
  printf("*************************************************\n");
}

/**************************************************************/
/* PRINT_ALP_CALMODES                                         */
/*  - Print available ALP calmodes                          */
/**************************************************************/
void print_alp_calmodes(calmode_t *alp, int current){
  int i;
  printf("************ Available ALP Calmodes  ************\n");
  printf("#    Command    Name\n");
  for(i=0;i<ALP_NCALMODES;i++){
    if(current == i)
      printf("%02d   %-7s   *%s\n",i,alp[i].cmd,alp[i].name);
    else
      printf("%02d   %-7s    %s\n",i,alp[i].cmd,alp[i].name);
  }
  printf("*************************************************\n");
}

/**************************************************************/
/* PRINT_HEX_CALMODES                                         */
/*  - Print available HEX calmodes                            */
/**************************************************************/
void print_hex_calmodes(calmode_t *hex, int current){
  int i;
  printf("************ Available HEX Calmodes  ************\n");
  printf("#    Command    Name\n");
  for(i=0;i<HEX_NCALMODES;i++){
    if(current == i)
      printf("%02d   %-7s   *%s\n",i,hex[i].cmd,hex[i].name);
    else
      printf("%02d   %-7s    %s\n",i,hex[i].cmd,hex[i].name);
  }
  printf("*************************************************\n");
}

/**************************************************************/
/* PRINT_BMC_CALMODES                                         */
/*  - Print available BMC calmodes                            */
/**************************************************************/
void print_bmc_calmodes(calmode_t *bmc, int current){
  int i;
  printf("************ Available BMC Calmodes  ************\n");
  printf("#    Command    Name\n");
  for(i=0;i<BMC_NCALMODES;i++){
    if(current == i)
      printf("%02d   %-7s   *%s\n",i,bmc[i].cmd,bmc[i].name);
    else
      printf("%02d   %-7s    %s\n",i,bmc[i].cmd,bmc[i].name);
  }
  printf("*************************************************\n");
}

/**************************************************************/
/* PRINT_TGT_CALMODES                                         */
/*  - Print available TGT calmodes                          */
/**************************************************************/
void print_tgt_calmodes(calmode_t *tgt, int current){
  int i;
  printf("************ Available TGT Calmodes  ************\n");
  printf("#    Command    Name\n");
  for(i=0;i<TGT_NCALMODES;i++){
    if(current == i)
      printf("%02d   %-7s   *%s\n",i,tgt[i].cmd,tgt[i].name);
    else
      printf("%02d   %-7s    %s\n",i,tgt[i].cmd,tgt[i].name);
  }
  printf("*************************************************\n");
}

/**************************************************************/
/* PRINT_FAKEMODES                                            */
/*  - Print available fake data modes                         */
/**************************************************************/
void print_fakemodes(calmode_t *fake, int current){
  int i;
  printf("******************* Available Fake Modes  *******************\n");
  printf("#    Command                Name\n");
  for(i=0;i<NFAKEMODES;i++){
    if(current == i)
      printf("%02d   %-18s    *%s\n",i,fake[i].cmd,fake[i].name);
    else
      printf("%02d   %-18s     %s\n",i,fake[i].cmd,fake[i].name);
  }
  printf("*************************************************************\n");
}

/**************************************************************/
/* PRINT_COMMANDERS                                           */
/*  - Print actuator commanders                               */
/**************************************************************/
void print_commanders(sm_t *sm_p){
  char bmc[10];
  char alp[10];
  char hex[10];
  char tgt[10];;
  int i;
  for(i=0;i<NCLIENTS;i++){
    if(sm_p->state_array[sm_p->state].bmc_commander == i) strcpy(bmc,sm_p->w[i].name);
    if(sm_p->state_array[sm_p->state].alp_commander == i) strcpy(alp,sm_p->w[i].name);
    if(sm_p->state_array[sm_p->state].hex_commander == i) strcpy(hex,sm_p->w[i].name);
    if(sm_p->state_array[sm_p->state].tgt_commander == i) strcpy(tgt,sm_p->w[i].name);
  }

  printf("******************* Actuator Commanders  *******************\n");
  printf("BMC: %s\n",bmc);
  printf("ALP: %s\n",alp);
  printf("HEX: %s\n",hex);
  printf("TGT: %s\n",tgt);
  printf("*************************************************************\n");
}

/**************************************************************/
/* PRINT_SHK_ZERNIKES                                             */
/*  - Print out SHK zernike control status                        */
/**************************************************************/
void print_shk_zernikes(sm_t *sm_p){
  int i;
  printf("************ SHK Zernike Control ************\n");
  printf("Zernike   Controlled\n");
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    if(sm_p->shk_zernike_control[i] == 1) printf("%02d        YES\n",i);        
     else
   if(sm_p->shk_zernike_control[i] == 0) printf("%02d        NO\n",i);
  }
  printf("******************************************\n");

}

/**************************************************************/
/* PRINT_LYT_ZERNIKES                                             */
/*  - Print out LYT zernike control status                        */
/**************************************************************/
void print_lyt_zernikes(sm_t *sm_p){
  int i;
  printf("************ LYT Zernike Control ************\n");
  printf("Zernike   Controlled\n");
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    if(sm_p->lyt_zernike_control[i] == 1) printf("%02d        YES\n",i);        
     else
   if(sm_p->lyt_zernike_control[i] == 0) printf("%02d        NO\n",i);
  }
  printf("******************************************\n");

}

/**************************************************************/
/* PRINT_ALP_ZERNIKES                                         */
/*  - Print out ALP zernike control status                    */
/**************************************************************/
void print_alp_zernikes(sm_t *sm_p){
  int i;
  printf("************ ALP Zernike Control ************\n");
  printf("Zernike   Controlled\n");
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    if(sm_p->alp_zernike_control[i] == 1) printf("%02d        YES\n",i);        
     else
   if(sm_p->alp_zernike_control[i] == 0) printf("%02d        NO\n",i);
  }
  printf("******************************************\n");

}

/**************************************************************/
/* PRINT_LYT_ZLIMIT                                           */
/*  - Print out LYT zernike command limits                    */
/**************************************************************/
void print_lyt_zlimit(sm_t *sm_p){
  int i;
  printf("************ LYT Zernike Command Limits ************\n");
  printf("%10s %10s\n","Zernike","Limit");
  for(i=0;i<LOWFS_N_ZERNIKE;i++)
    printf("%10d %10.3f\n",i,sm_p->lyt_zcmd_limit[i]);
  printf("****************************************************\n");

}

/**************************************************************/
/* PRINT_HTR_STATUS                                           */
/*  - Print out current heater status                         */
/**************************************************************/
void print_htr_status(sm_t *sm_p){
  int i;

  printf("******************************** Heater Status *********************************\n");
  printf("%2s%5s%5s%5s%5s%5s%5s%5s%6s%6s%6s%6s%6s%6s\n","#","Name","Ena","Over","Pow","Max","ADC","CH","Temp","SetP","kP","kI","kD","Imax");
  for(i=0;i<SSR_NCHAN;i++)
    printf("%2d%5s%5d%5d%5d%5d%5d%5d%6.1f%6.1f%6.1f%6.1f%6.1f%6ld\n",i,sm_p->htr[i].name,sm_p->htr[i].enable,sm_p->htr[i].override,sm_p->htr[i].power,sm_p->htr[i].maxpower,sm_p->htr[i].adc,sm_p->htr[i].ch,sm_p->htr[i].temp,sm_p->htr[i].setpoint,sm_p->htr[i].kP,sm_p->htr[i].kI,sm_p->htr[i].kD,(long)sm_p->htr[i].intmax);
  printf("********************************************************************************\n");

}

/**************************************************************/
/* HANDLE_COMMAND                                             */
/*  - Handle user commands                                    */
/**************************************************************/
int handle_command(char *line, sm_t *sm_p){
  double ftemp,pgain,igain,dgain;
  int    itemp,ich,iadc,iband,npix;
  char   stemp[CMD_MAX_LENGTH];
  char   cmd[CMD_MAX_LENGTH];
  int    cmdfound=0;
  int    i=0,j=0,hex_axis=0;
  double hex_poke=0;
  int    calmode=0,phasemode=0,optmode=0;
  uint16_t led;
  static double trl_poke = HEX_TRL_POKE;
  static double rot_poke = HEX_ROT_POKE;
  static calmode_t alpcalmodes[ALP_NCALMODES];
  static calmode_t hexcalmodes[HEX_NCALMODES];
  static calmode_t bmccalmodes[BMC_NCALMODES];
  static calmode_t tgtcalmodes[TGT_NCALMODES];
  static calmode_t fakemodes[NFAKEMODES];
  static phasemode_t sciphasemodes[SCI_NPHASEMODES];
  static optmode_t scioptmodes[SCI_NOPTMODES];
  static int init=0;
  const char hex_str_axes[HEX_NAXES][5] = {"X","Y","Z","U","V","W"};
  const char hex_str_unit[HEX_NAXES][5] = {"mm","mm","mm","deg","deg","deg"};
  const double hexhome[HEX_NAXES] = HEX_POS_HOME;
  const double hexdef[HEX_NAXES]  = HEX_POS_DEFAULT;
  const double shk_gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID] = SHK_GAIN_ALP_ZERN_DEFAULT;
  const double shk_gain_alp_cell[LOWFS_N_PID]                  = SHK_GAIN_ALP_CELL_DEFAULT;
  const double lyt_gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID] = LYT_GAIN_ALP_ZERN_DEFAULT;
  hex_t hexcmd;
  memset(&hexcmd,0,sizeof(hex_t));
  char *pch;
  char  dim[1];
  alp_t alp;
  bmc_t bmc;
  double dz[LOWFS_N_ZERNIKE] = {0};
  double da[ALP_NACT] = {0};
  int lyt_roi[2] = {0};
  int ret;
  
  //Intitalize
  if(!init){
    //Init ALP calmodes
    for(i=0;i<ALP_NCALMODES;i++)
      alp_init_calmode(i,&alpcalmodes[i]);
    //Init HEX calmodes
    for(i=0;i<HEX_NCALMODES;i++)
      hex_init_calmode(i,&hexcalmodes[i]);
    //Init BMC calmodes
    for(i=0;i<BMC_NCALMODES;i++)
      bmc_init_calmode(i,&bmccalmodes[i]);
    //Init TGT calmodes
    for(i=0;i<TGT_NCALMODES;i++)
      tgt_init_calmode(i,&tgtcalmodes[i]);
    //Init fake modes
    for(i=0;i<NFAKEMODES;i++)
      init_fakemode(i,&fakemodes[i]);
    //Init SCI phasemodes
    for(i=0;i<SCI_NPHASEMODES;i++)
      sci_init_phasemode(i,&sciphasemodes[i]);
    //Init SCI optmodes
    for(i=0;i<SCI_NOPTMODES;i++)
      sci_init_optmode(i,&scioptmodes[i]);
    //Set init flag
    init=1;
  }

  /****************************************
   * SYSTEM COMMANDS
   ***************************************/
  //exit: exit watchdog
  sprintf(cmd,"exit");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    return(CMD_EXIT_WATCHDOG);
  }
  
  //shutdown: exit watchdog and shutdown computer
  sprintf(cmd,"shutdown");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    return(CMD_SHUTDOWN);
  }

  //reboot: exit watchdog and reboot computer
  sprintf(cmd,"reboot");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    return(CMD_REBOOT);
  }

  //sysinfo: print CPU Memory and Disk usage
  sprintf(cmd,"sysinfo");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    //print top 5 process lines from top
    if(system("top -bn1 | head -n 12 | tail -n 6") == -1)
      printf("CMD: system command error\n");
    return(CMD_NORMAL);
  }
  
  //print packet info
  sprintf(cmd,"packet info");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: pktheadr = %lu bytes\n",sizeof(pkthed_t));
    printf("CMD: scievent = %lu bytes\n",sizeof(scievent_t));
    printf("CMD: shkevent = %lu bytes\n",sizeof(shkevent_t));
    printf("CMD: lytevent = %lu bytes\n",sizeof(lytevent_t));
    printf("CMD: acqevent = %lu bytes\n",sizeof(acqevent_t));
    printf("CMD: thmevent = %lu bytes\n",sizeof(thmevent_t));
    printf("CMD: mtrevent = %lu bytes\n",sizeof(mtrevent_t));
    printf("CMD: shkpkt   = %lu bytes\n",sizeof(shkpkt_t));
    printf("CMD: lytpkt   = %lu bytes\n",sizeof(lytpkt_t));
    printf("CMD: shkfull  = %lu bytes\n",sizeof(shkfull_t));
    printf("CMD: acqfull  = %lu bytes\n",sizeof(acqfull_t));
    return(CMD_NORMAL);
  }
  
  //erase flight data
  sprintf(cmd,"erase flight data");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Erasing flight data...\n");
    printf("CMD: -- Stopping Telemetry\n");
    sm_p->w[TLMID].run = 0;    
    for(i=0;i<PROC_TIMEOUT;i++){
      if((sm_p->w[TLMID].pid == -1)){
	printf("CMD: -- Erasing data\n");
	if(system(CMD_ERASE_FLIGHT_DATA)==-1){
	  printf("CMD: -- Erase command failed\n");
	  return(CMD_NORMAL);
	}
	printf("CMD: -- Starting Telemetry\n");
	sm_p->w[TLMID].run = 1;
	break;
      }
      sleep(1);
    }
    if(i >= PROC_TIMEOUT){
      printf("CMD: -- timeout!\n");
    }
    printf("CMD: Flight data erased\n");
    return(CMD_NORMAL);
  }

  //erase calibration data
  sprintf(cmd,"erase cal data");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Erasing calibration data...\n");
    printf("CMD: -- Stopping Telemetry\n");
    sm_p->w[TLMID].run = 0;    
    for(i=0;i<PROC_TIMEOUT;i++){
      if((sm_p->w[TLMID].pid == -1)){
	printf("CMD: -- Erasing data\n");
	if(system(CMD_ERASE_CAL_DATA)==-1){
	  printf("CMD: -- Erase command failed\n");
	  return(CMD_NORMAL);
	}
	printf("CMD: -- Starting Telemetry\n");
	sm_p->w[TLMID].run = 1;
	break;
      }
      sleep(1);
    }
    if(i >= PROC_TIMEOUT){
      printf("CMD: -- timeout!\n");
    }
    printf("CMD: Calibration data erased\n");
    return(CMD_NORMAL);
  }

  //Sleep
  sprintf(cmd,"sleep");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= 0 && ftemp <= 5){
      printf("CMD: Input console sleeping for %f seconds\n",ftemp);
      usleep(ftemp*ONE_MILLION);
    }
    else
      printf("CMD: sleep range = [0,5] seconds\n");
    return(CMD_NORMAL);
  }
  

  /****************************************
   * PROCESS CONTROL
   ***************************************/

  //Process control commands
  for(i=0;i<NCLIENTS;i++){
    if(i != WATID){
      //ON command
      sprintf(cmd,"%s_proc on",sm_p->w[i].name);
      for(j=0;j<strlen(cmd);j++) cmd[j]=tolower(cmd[j]);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	if(sm_p->w[i].ena){
	  printf("CMD: Turning %s ON\n",sm_p->w[i].name);
	  sm_p->w[i].run = 1;
	}else{
	  printf("CMD: Process %s DISABLED\n",sm_p->w[i].name);
	}
	return(CMD_NORMAL);
      }
      //OFF command
      sprintf(cmd,"%s_proc off",sm_p->w[i].name);
      for(j=0;j<strlen(cmd);j++) cmd[j]=tolower(cmd[j]);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	printf("CMD: Turning %s OFF\n",sm_p->w[i].name);
	sm_p->w[i].run = 0;
	return(CMD_NORMAL);
      }
      //ENABLE command
      sprintf(cmd,"%s_proc enable",sm_p->w[i].name);
      for(j=0;j<strlen(cmd);j++) cmd[j]=tolower(cmd[j]);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	printf("CMD: Enabling %s\n",sm_p->w[i].name);
	sm_p->w[i].ena = 1;
	return(CMD_NORMAL);
      }
      //DISABLE command
      sprintf(cmd,"%s_proc disable",sm_p->w[i].name);
      for(j=0;j<strlen(cmd);j++) cmd[j]=tolower(cmd[j]);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	printf("CMD: Disabling %s\n",sm_p->w[i].name);
	sm_p->w[i].ena = 0;
	return(CMD_NORMAL);
      }
      //RESTART command
      sprintf(cmd,"%s_proc restart",sm_p->w[i].name);
      for(j=0;j<strlen(cmd);j++) cmd[j]=tolower(cmd[j]);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	printf("CMD: Restarting %s\n",sm_p->w[i].name);
	sm_p->w[i].res = 1;
	return(CMD_NORMAL);
      }
    }
  }

  //Get process status
  sprintf(cmd,"proc status");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    print_proc_status(sm_p);
    return(CMD_NORMAL);
  }

  /****************************************
   * CIRCULAR BUFFER SETTINGS
   ***************************************/
  //Buffer control commands
  for(i=0;i<NCIRCBUF;i++){
    //Write ON
    sprintf(cmd,"circbuf %s write on",sm_p->circbuf[i].name);
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Turning circbuf %s write ON\n",sm_p->circbuf[i].name);
      sm_p->circbuf[i].write = 1;
      return(CMD_NORMAL);
    }
    //Write OFF
    sprintf(cmd,"circbuf %s write off",sm_p->circbuf[i].name);
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Turning circbuf %s write OFF\n",sm_p->circbuf[i].name);
      sm_p->circbuf[i].write = 0;
      return(CMD_NORMAL);
    }
    //Read ON
    sprintf(cmd,"circbuf %s read on",sm_p->circbuf[i].name);
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Turning circbuf %s read ON\n",sm_p->circbuf[i].name);
      sm_p->circbuf[i].read = 1;
      return(CMD_NORMAL);
    }
    //Read NEW
    sprintf(cmd,"circbuf %s read new",sm_p->circbuf[i].name);
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Turning circbuf %s read NEWEST\n",sm_p->circbuf[i].name);
      sm_p->circbuf[i].read = 2;
      return(CMD_NORMAL);
    }
     //Read OFF
    sprintf(cmd,"circbuf %s read off",sm_p->circbuf[i].name);
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Turning circbuf %s read OFF\n",sm_p->circbuf[i].name);
      sm_p->circbuf[i].read = 0;
      return(CMD_NORMAL);
    }
    //Save ON
    sprintf(cmd,"circbuf %s save on",sm_p->circbuf[i].name);
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Turning circbuf %s save ON\n",sm_p->circbuf[i].name);
      sm_p->circbuf[i].save = 1;
      return(CMD_NORMAL);
    }
    //Save OFF
    sprintf(cmd,"circbuf %s save off",sm_p->circbuf[i].name);
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Turning circbuf %s save OFF\n",sm_p->circbuf[i].name);
      sm_p->circbuf[i].save = 0;
      return(CMD_NORMAL);
    }
    //Send ON
    sprintf(cmd,"circbuf %s send on",sm_p->circbuf[i].name);
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Turning circbuf %s send ON\n",sm_p->circbuf[i].name);
      sm_p->circbuf[i].send = 1;
      return(CMD_NORMAL);
    }
    //Send OFF
    sprintf(cmd,"circbuf %s send off",sm_p->circbuf[i].name);
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Turning circbuf %s send OFF\n",sm_p->circbuf[i].name);
      sm_p->circbuf[i].send = 0;
      return(CMD_NORMAL);
    }
  }
  
  //Get circbuf status
  sprintf(cmd,"circbuf status");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    print_circbuf_status(sm_p);
    return(CMD_NORMAL);
  }

  //Reset circbuf to defaults
  sprintf(cmd,"circbuf reset");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Resetting circular buffer defaults\n");
    reset_circbuf(sm_p);
    print_circbuf_status(sm_p);
    return(CMD_NORMAL);
  }
  

  /****************************************
   * FAKE DATA MODES
   ***************************************/

  //Fake Data Modes
  for(i=0;i<NCLIENTS;i++){
    if(i != WATID){
      sprintf(cmd,"%s fakemode",sm_p->w[i].name);
      for(j=0;j<strlen(cmd);j++) cmd[j]=tolower(cmd[j]);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	cmdfound = 0;
	for(j=0;j<NFAKEMODES;j++){
	  if(!strncasecmp(line+strlen(cmd)+1,fakemodes[j].cmd,strlen(fakemodes[j].cmd))){
	    sm_p->w[i].fakemode = j;
	    printf("CMD: Changed %s fake mode to %s\n",sm_p->w[i].name,fakemodes[j].name);
	    cmdfound = 1;
	    break;
	  }
	}
	if(!cmdfound)
	  print_fakemodes(fakemodes,sm_p->w[i].fakemode);
	return(CMD_NORMAL);
      }
    }
  }
  
  /****************************************
   * CALIBRATION MODES
   ***************************************/
  
  //ALP Calmodes
  sprintf(cmd,"alp calmode");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    cmdfound = 0;
    for(i=0;i<ALP_NCALMODES;i++){
      if(!strncasecmp(line+strlen(cmd)+1,alpcalmodes[i].cmd,strlen(alpcalmodes[i].cmd))){
	sm_p->alp_calmode=i;
	printf("CMD: Changed ALP calibration mode to %s\n",alpcalmodes[i].name);
	cmdfound = 1;
      }
    }
    if(!cmdfound)
      print_alp_calmodes(alpcalmodes,sm_p->alp_calmode);
    
    return(CMD_NORMAL);
  }
    
  //HEX Calmodes
  sprintf(cmd,"hex calmode");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    cmdfound = 0;
    for(i=0;i<HEX_NCALMODES;i++){
      if(!strncasecmp(line+strlen(cmd)+1,hexcalmodes[i].cmd,strlen(hexcalmodes[i].cmd))){
	sm_p->hex_calmode=i;
	printf("CMD: Changed HEX calibration mode to %s\n",hexcalmodes[i].name);
	cmdfound = 1;
      }
    }
    if(!cmdfound)
      print_hex_calmodes(hexcalmodes,sm_p->hex_calmode);

    return(CMD_NORMAL);
  }

  //BMC Calmodes
  sprintf(cmd,"bmc calmode");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    cmdfound = 0;
    for(i=0;i<BMC_NCALMODES;i++){
      if(!strncasecmp(line+strlen(cmd)+1,bmccalmodes[i].cmd,strlen(bmccalmodes[i].cmd))){
	sm_p->bmc_calmode=i;
	printf("CMD: Changed BMC calibration mode to %s\n",bmccalmodes[i].name);
	cmdfound = 1;
      }
    }
    if(!cmdfound)
      print_bmc_calmodes(bmccalmodes,sm_p->bmc_calmode);

    return(CMD_NORMAL);
  }

  //TGT Calmodes
  sprintf(cmd,"tgt calmode");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    cmdfound = 0;
    for(i=0;i<TGT_NCALMODES;i++){
      if(!strncasecmp(line+strlen(cmd)+1,tgtcalmodes[i].cmd,strlen(tgtcalmodes[i].cmd))){
	sm_p->tgt_calmode=i;
	printf("CMD: Changed TGT calibration mode to %s\n",tgtcalmodes[i].name);
	cmdfound = 1;
      }
    }
    if(!cmdfound)
      print_tgt_calmodes(tgtcalmodes,sm_p->tgt_calmode);

    return(CMD_NORMAL);
  }

  /****************************************
   * CALIBRATION SETTINGS
   ***************************************/

  //ALP timer length
  sprintf(cmd,"alp timer length");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp > 0 && ftemp <= ALP_CAL_TIMER_MAX){
      sm_p->alp_cal_timer_length = ftemp;
      printf("CMD: Changed ALP calibration timer length to %.1f seconds\n",sm_p->alp_cal_timer_length);
    }else{
      printf("CMD: ALP calibration timer out of bounds [%d,%f]\n",0,(double)ALP_CAL_TIMER_MAX);
    }
    return(CMD_NORMAL);
  }
  
  //ALP calibration scale
  sprintf(cmd,"alp cal scale");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= 0 && ftemp <= ALP_CAL_SCALE_MAX){
      sm_p->alp_cal_scale = ftemp;
      printf("CMD: Changed ALP calibration scale to %.3f\n",sm_p->alp_cal_scale);
    }else{
      printf("CMD: ALP calibration scale out of bounds [%d,%d]\n",0,ALP_CAL_SCALE_MAX);
    }
    return(CMD_NORMAL);
  }
  
  //BMC timer length
  sprintf(cmd,"bmc timer length");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp > 0 && ftemp <= BMC_CAL_TIMER_MAX){
      sm_p->bmc_cal_timer_length = ftemp;
      printf("CMD: Changed BMC calibration timer length to %.1f seconds\n",sm_p->bmc_cal_timer_length);
    }else{
      printf("CMD: BMC calibration timer out of bounds [%d,%f]\n",0,(double)BMC_CAL_TIMER_MAX);
    }
    return(CMD_NORMAL);
  }
  
  //BMC calibration scale
  sprintf(cmd,"bmc cal scale");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= 0 && ftemp <= BMC_CAL_SCALE_MAX){
      sm_p->bmc_cal_scale = ftemp;
      printf("CMD: Changed BMC calibration scale to %.3f\n",sm_p->bmc_cal_scale);
    }else{
      printf("CMD: BMC calibration scale out of bounds [%d,%d]\n",0,BMC_CAL_SCALE_MAX);
    }
    return(CMD_NORMAL);
  }


  /****************************************
   * STATES
   ***************************************/
  
  //Change state
  sprintf(cmd,"state");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    cmdfound=0;
    for(i=0;i<NSTATES;i++){
      if(!strncasecmp(line+strlen(cmd)+1,(char *)sm_p->state_array[i].cmd,strlen((char *)sm_p->state_array[i].cmd))){
	change_state(sm_p,i);
	printf("CMD: Changing state to %s\n",sm_p->state_array[i].name);
	cmdfound = 1;
      }
    }
    if(!cmdfound) print_states((state_t *)sm_p->state_array,sm_p->state);
    return(CMD_NORMAL);
  }

  //Print actuator commanders
  sprintf(cmd,"commander");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    print_commanders(sm_p);
    return(CMD_NORMAL);
  }
  
  //Change actuator commanders
  sprintf(cmd,"bmc commander sci");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->state_array[sm_p->state].bmc_commander = SCIID;
    printf("CMD: State %s BMC commander set to SCI\n",sm_p->state_array[sm_p->state].name);
    return(CMD_NORMAL);
  }
  sprintf(cmd,"bmc commander wat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->state_array[sm_p->state].bmc_commander = WATID;
    printf("CMD: State %s BMC commander set to WAT\n",sm_p->state_array[sm_p->state].name);
    return(CMD_NORMAL);
  }
  sprintf(cmd,"alp commander shk");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->state_array[sm_p->state].alp_commander = SHKID;
    printf("CMD: State %s ALP commander set to SHK\n",sm_p->state_array[sm_p->state].name);
    return(CMD_NORMAL);
  }
  sprintf(cmd,"alp commander lyt");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->state_array[sm_p->state].alp_commander = LYTID;
    printf("CMD: State %s ALP commander set to LYT\n",sm_p->state_array[sm_p->state].name);
    return(CMD_NORMAL);
  }
  sprintf(cmd,"alp commander sci");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->state_array[sm_p->state].alp_commander = SCIID;
    printf("CMD: State %s ALP commander set to SCI\n",sm_p->state_array[sm_p->state].name);
    return(CMD_NORMAL);
  }
  sprintf(cmd,"alp commander wat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->state_array[sm_p->state].alp_commander = WATID;
    printf("CMD: State %s ALP commander set to WAT\n",sm_p->state_array[sm_p->state].name);
    return(CMD_NORMAL);
  }
  sprintf(cmd,"tgt commander shk");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->state_array[sm_p->state].tgt_commander = SHKID;
    printf("CMD: State %s TGT commander set to SHK\n",sm_p->state_array[sm_p->state].name);
    return(CMD_NORMAL);
  }
  sprintf(cmd,"tgt commander lyt");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->state_array[sm_p->state].tgt_commander = LYTID;
    printf("CMD: State %s TGT commander set to LYT\n",sm_p->state_array[sm_p->state].name);
    return(CMD_NORMAL);
  }
  sprintf(cmd,"tgt commander sci");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->state_array[sm_p->state].tgt_commander = SCIID;
    printf("CMD: State %s TGT commander set to SCI\n",sm_p->state_array[sm_p->state].name);
    return(CMD_NORMAL);
  }
  sprintf(cmd,"tgt commander wat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->state_array[sm_p->state].tgt_commander = WATID;
    printf("CMD: State %s TGT commander set to WAT\n",sm_p->state_array[sm_p->state].name);
    return(CMD_NORMAL);
  }
  
  /****************************************
   * DATA RECORDING
   ***************************************/

  //Start Manual SHK Data Recording
  sprintf(cmd,"shk start rec");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Starting SHK data recording\n");
    //Setup filename
    timestamp(stemp);
    sprintf((char *)sm_p->calfile,SHK_OUTFILE,stemp);
    //Start data recording
    printf("  -- File: %s\n",sm_p->calfile);
    sm_p->w[DIAID].launch = getshk_proc;
    sm_p->w[DIAID].run    = 1;
    return(CMD_NORMAL);
  }

  //Stop Manual SHK Data Recording
  sprintf(cmd,"shk stop rec");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Stopping SHK data recording\n");
    //Stop data recording
    sm_p->w[DIAID].run    = 0;
    printf("  -- Done\n");
    return(CMD_NORMAL);
  }

  //Start Manual LYT Data Recording
  sprintf(cmd,"lyt start rec");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Starting LYT data recording\n");
    //Setup filename
    timestamp(stemp);
    sprintf((char *)sm_p->calfile,LYT_OUTFILE,stemp);
    //Start data recording
    printf("  -- File: %s\n",sm_p->calfile);
    sm_p->w[DIAID].launch = getlyt_proc;
    sm_p->w[DIAID].run    = 1;
    return(CMD_NORMAL);
  }

  //Stop Manual LYT Data Recording
  sprintf(cmd,"lyt stop rec");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Stopping data recording\n");
    //Stop data recording
    sm_p->w[DIAID].run    = 0;
    printf("  -- Done\n");
    return(CMD_NORMAL);
  }

  //Start Manual SCI Data Recording
  sprintf(cmd,"sci start rec");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Starting SCI data recording\n");
    //Setup filename
    timestamp(stemp);
    sprintf((char *)sm_p->calfile,SCI_OUTFILE,stemp);
    //Start data recording
    printf("  -- File: %s\n",sm_p->calfile);
    sm_p->w[DIAID].launch = getsci_proc;
    sm_p->w[DIAID].run    = 1;
    return(CMD_NORMAL);
  }

  //Stop Manual SCI Data Recording
  sprintf(cmd,"sci stop rec");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Stopping data recording\n");
    //Stop data recording
    sm_p->w[DIAID].run    = 0;
    printf("  -- Done\n");
    return(CMD_NORMAL);
  }

  /****************************************
   * HEXAPOD CONTROL
   ***************************************/

  //User Hexapod Commands
  sprintf(cmd,"hex");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    //Toggle spiral search autostop
    sprintf(cmd,"hex spiral autostop on");
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Turning HEX spiral autostop ON\n");
      sm_p->hex_spiral_autostop=1;
      return(CMD_NORMAL);
    }
    sprintf(cmd,"hex spiral autostop off");
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Turning HEX spiral autostop OFF\n");
      sm_p->hex_spiral_autostop=0;
      return(CMD_NORMAL);
    }
    if(sm_p->state_array[sm_p->state].hex_commander == WATID){
      //Get hex error
      sprintf(cmd,"hex get error");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	if(sm_p->hex_ready)
	  hex_get_error(sm_p->hexfd);
	else
	  printf("CMD: HEX not ready\n");
	return(CMD_NORMAL);
      }
      //Initialize hexapod
      sprintf(cmd,"hex init");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	if(HEX_ENABLE){
	  printf("CMD: Opening HEX driver\n");
	  if(hex_init((int *)&sm_p->hexfd)){
	    sm_p->hex_ready = 0;
	    printf("CMD: ERROR: HEX init failed!\n");
	  }
	  else{
	    sm_p->hex_ready = 1;
	    printf("CMD: HEX ready\n");
	  }
	}
	else{
	  printf("CMD: HEX disabled\n");
	}
	return(CMD_NORMAL);
      }
      //Increase step size
      sprintf(cmd,"hex inc step");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	rot_poke *= 2.0;
	trl_poke *= 2.0;
	printf("CMD: Increasing HEX movement to %f mm and %f deg\n", trl_poke, rot_poke);
	return(CMD_NORMAL);
      }
      //Decrease step size
      sprintf(cmd,"hex dec step");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	rot_poke /= 2.0;
	trl_poke /= 2.0;
	printf("CMD: Decreasing HEX movement to %f mm and %f deg\n", trl_poke, rot_poke);
	return(CMD_NORMAL);
      }
      //Reset step size
      sprintf(cmd,"hex rst step");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	rot_poke = HEX_ROT_POKE;
	trl_poke = HEX_TRL_POKE;
	printf("CMD: Resetting HEX movement to %f mm and %f deg\n", trl_poke, rot_poke);
	return(CMD_NORMAL);
      }
      //Query current position
      sprintf(cmd,"hex getpos");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	if(!sm_p->hex_ready){
	  printf("CMD: HEX not ready\n");
	  return(CMD_NORMAL);
	}
	printf("CMD: Getting hexapod position\n");
	hex_printpos(sm_p->hexfd);
	return(CMD_NORMAL);
      }
      //Set 6-axis position
      sprintf(cmd,"hex gopos");
      if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
	pch = strtok(line+strlen(cmd),",");
	i=0;
	while(pch != NULL){
	  ftemp  = atof(pch);
	  hexcmd.acmd[i++] = ftemp;
	  pch = strtok(NULL,",");
	}
	if(i==6){
	  printf("CMD: Moving HEX to: %f,%f,%f,%f,%f,%f\n",hexcmd.acmd[0],hexcmd.acmd[1],hexcmd.acmd[2],hexcmd.acmd[3],hexcmd.acmd[4],hexcmd.acmd[5]);
	  if(hex_send_command(sm_p,&hexcmd,WATID))
	    printf("CMD: hex_send_command failed\n");
	}
	return CMD_NORMAL;
      }
      //Go to HOME position
      sprintf(cmd,"hex gohome");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	if(!sm_p->hex_ready){
	  printf("CMD: HEX not ready\n");
	  return(CMD_NORMAL);
	}
	printf("CMD: Moving hexapod to home positon\n");
	memcpy(hexcmd.acmd,hexhome,sizeof(hexhome));
	if(hex_send_command(sm_p,&hexcmd,WATID))
	  printf("CMD: hex_send_command failed\n");
	return(CMD_NORMAL);
      }
      //Go to DEFAULT position
      sprintf(cmd,"hex godef");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	if(!sm_p->hex_ready){
	  printf("CMD: HEX not ready\n");
	  return(CMD_NORMAL);
	}
	printf("CMD: Moving hexapod to default positon\n");
	memcpy(hexcmd.acmd,hexdef,sizeof(hexdef));
	if(hex_send_command(sm_p,&hexcmd,WATID))
	  printf("CMD: hex_send_command failed\n");
	return(CMD_NORMAL);
      }
      //Save current position
      sprintf(cmd,"hex savepos");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	if(!sm_p->hex_ready){
	  printf("CMD: HEX not ready\n");
	  return(CMD_NORMAL);
	}
	printf("CMD: Saving hexapod position\n");
	hex_savepos(sm_p);
	return(CMD_NORMAL);
      }
      //Load hex position from file
      sprintf(cmd,"hex loadpos");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	if(!sm_p->hex_ready){
	  printf("CMD: HEX not ready\n");
	  return(CMD_NORMAL);
	}
	printf("CMD: Loading hexapod position\n");
	hex_loadpos(sm_p,WATID);
	return(CMD_NORMAL);
      }
      //Move in a single axis
      sprintf(cmd,"hex move");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	if(!strncasecmp(line+strlen(cmd)+1,"+x",2)){
	  hex_poke = trl_poke;
	  hex_axis = HEX_AXIS_X;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"-x",2)){
	  hex_poke = -trl_poke;
	  hex_axis = HEX_AXIS_X;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"+y",2)){
	  hex_poke = trl_poke;
	  hex_axis = HEX_AXIS_Y;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"-y",2)){
	  hex_poke = -trl_poke;
	  hex_axis = HEX_AXIS_Y;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"+z",2)){
	  hex_poke = trl_poke;
	  hex_axis = HEX_AXIS_Z;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"-z",2)){
	  hex_poke = -trl_poke;
	  hex_axis = HEX_AXIS_Z;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"+u",2)){
	  hex_poke = rot_poke;
	  hex_axis = HEX_AXIS_U;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"-u",2)){
	  hex_poke = -rot_poke;
	  hex_axis = HEX_AXIS_U;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"+v",2)){
	  hex_poke = rot_poke;
	  hex_axis = HEX_AXIS_V;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"-v",2)){
	  hex_poke = -rot_poke;
	  hex_axis = HEX_AXIS_V;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"+w",2)){
	  hex_poke = rot_poke;
	  hex_axis = HEX_AXIS_W;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"-w",2)){
	  hex_poke = -rot_poke;
	  hex_axis = HEX_AXIS_W;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"dx",2)){
	  pch = strtok(line+strlen(cmd)+3," ");
	  if(pch == NULL){
	    printf("CMD: Bad command format\n");
	    return CMD_NORMAL;
	  }
	  ftemp  = atof(pch);
	  hex_poke = ftemp;
	  hex_axis = HEX_AXIS_X;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"dy",2)){
	  pch = strtok(line+strlen(cmd)+3," ");
	  if(pch == NULL){
	    printf("CMD: Bad command format\n");
	    return CMD_NORMAL;
	  }
	  ftemp  = atof(pch);
	  hex_poke = ftemp;
	  hex_axis = HEX_AXIS_Y;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"dz",2)){
	  pch = strtok(line+strlen(cmd)+3," ");
	  if(pch == NULL){
	    printf("CMD: Bad command format\n");
	    return CMD_NORMAL;
	  }
	  ftemp  = atof(pch);
	  hex_poke = ftemp;
	  hex_axis = HEX_AXIS_Z;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"du",2)){
	  pch = strtok(line+strlen(cmd)+3," ");
	  if(pch == NULL){
	    printf("CMD: Bad command format\n");
	    return CMD_NORMAL;
	  }
	  ftemp  = atof(pch);
	  hex_poke = ftemp;
	  hex_axis = HEX_AXIS_U;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"dv",2)){
	  pch = strtok(line+strlen(cmd)+3," ");
	  if(pch == NULL){
	    printf("CMD: Bad command format\n");
	    return CMD_NORMAL;
	  }
	  ftemp  = atof(pch);
	  hex_poke = ftemp;
	  hex_axis = HEX_AXIS_V;
	  goto move_hex;
	}
	if(!strncasecmp(line+strlen(cmd)+1,"dw",2)){
	  pch = strtok(line+strlen(cmd)+3," ");
	  if(pch == NULL){
	    printf("CMD: Bad command format\n");
	    return CMD_NORMAL;
	  }
	  ftemp  = atof(pch);
	  hex_poke = ftemp;
	  hex_axis = HEX_AXIS_W;
	  goto move_hex;
	}
	printf("CMD: Bad hex move command\n");
	return(CMD_NORMAL);
      move_hex:
	if(!sm_p->hex_ready){
	  printf("CMD: HEX not ready\n");
	  return(CMD_NORMAL);
	}
	printf("CMD: Moving hexapod axis %s by %f %s\n",hex_str_axes[hex_axis],hex_poke,hex_str_unit[hex_axis]);
	if(hex_get_command(sm_p,&hexcmd)){
	  printf("CMD: hex_get_command failed\n");
	  return(CMD_NORMAL);
  	}
	hexcmd.acmd[hex_axis] += hex_poke;
	if(hex_send_command(sm_p,&hexcmd,WATID))
	  printf("CMD: hex_send_command failed\n");
	return(CMD_NORMAL);
      }
    }
    else{
      printf("CMD: Manual hexapod control disabled in this state\n");
      return(CMD_NORMAL);
    }
    //No hex command found
    printf("CMD: Bad hex command format\n");
    return(CMD_NORMAL);
  }
  
  /****************************************
   * ALP DM CONTROL
   ***************************************/

  //Set all ALP actuators to the same value
  sprintf(cmd,"alp bias");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      ftemp = atof(line+strlen(cmd)+1);
      if((ftemp >= ALP_MIN_BIAS) && (ftemp <= ALP_MAX_BIAS)){
	printf("CMD: Setting ALP bias = %f\n",ftemp);
	if(sm_p->alp_ready)
	  if(alp_set_bias(sm_p,ftemp,WATID))
	    printf("CMD: ERROR: alp_set_bias failed!\n");
      }
      else
	printf("CMD: ALP bias must be between %f and %f \n",ALP_MIN_BIAS,ALP_MAX_BIAS);
    }
    else
      printf("CMD: Manual ALP DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Set ALP to all ZEROS
  sprintf(cmd,"alp zero flat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Setting ALP to all zeros\n");
      if(sm_p->alp_ready)
	if(alp_zero_flat(sm_p,WATID))
	  printf("CMD: ERROR: alp_zero_flat failed!\n");
    }
    else
      printf("CMD: Manual ALP DM control disabled in this state\n");
    return(CMD_NORMAL);
  }
  
  //Set ALP to #defined flat
  sprintf(cmd,"alp revert flat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Reverting ALP flat to default\n");
      if(sm_p->alp_ready)
	if(alp_revert_flat(sm_p,WATID))
	  printf("CMD: ERROR: alp_revert_flat failed!\n");
    }
    else
      printf("CMD: Manual ALP DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Save current ALP command to disk
  sprintf(cmd,"alp save flat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Saving current ALP command as flat\n");
      if(sm_p->alp_ready)
	if(alp_save_flat(sm_p))
	  printf("CMD: ERROR: alp_save_flat failed!\n");
    }
    else
      printf("CMD: Manual ALP DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Load ALP command from disk
  sprintf(cmd,"alp load flat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Loading ALP flat from file\n");
      if(sm_p->alp_ready)
	if(alp_load_flat(sm_p,WATID))
	  printf("CMD: ERROR: alp_load_flat failed!\n");
    }
    else
      printf("CMD: Manual ALP DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Perturb ALP command with random actuator values
  sprintf(cmd,"alp random");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Sending random actuator pattern to ALP DM\n");
      if(sm_p->alp_ready)
	if(alp_set_random(sm_p,WATID))
	  printf("CMD: ERROR: alp_set_random failed!\n");
    }
    else
      printf("CMD: Manual ALP DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Perturb ALP command with random zernikes
  sprintf(cmd,"alp zrandom");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Sending random Zernike pattern to ALP DM\n");
      if(sm_p->alp_ready)
	if(alp_set_zrandom(sm_p,WATID))
	  printf("CMD: ERROR: alp_set_zrandom failed!\n");
    }
    else
      printf("CMD: Manual ALP DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Reset ALP functions
  sprintf(cmd,"alp reset");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Resetting ALP functions\n");
    alp_function_reset(sm_p);
    return(CMD_NORMAL);
  }

  //Add a zernike to the current ALP command
  sprintf(cmd,"alp set zernike");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      pch = strtok(line+strlen(cmd)," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      itemp  = atoi(pch);
      pch = strtok(NULL," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      ftemp  = atof(pch);
      printf("CMD: Trying to change Z[%d] by %f microns\n",itemp,ftemp);
      if(itemp >= 0 && itemp < LOWFS_N_ZERNIKE && ftemp >= ALP_DZERNIKE_MIN && ftemp <= ALP_DZERNIKE_MAX){
	//Get current command
	if(alp_get_command(sm_p,&alp)){
	  printf("CMD: alp_get_command failed!\n");
	  return CMD_NORMAL;
	}

	//Set zernike perturbation 
	dz[itemp] = ftemp;

	//Convert to actuators deltas
	alp_zern2alp(dz,da,FUNCTION_NO_RESET);

	//Add to current command
	for(i=0;i<LOWFS_N_ZERNIKE;i++)
	  alp.zcmd[i] += dz[i]; 
	for(i=0;i<ALP_NACT;i++)
	  alp.acmd[i] += da[i]; 
	
	//Send command
	if(alp_send_command(sm_p,&alp,WATID,1)){
	  printf("CMD: alp_send_command failed\n");
	  return CMD_NORMAL;
	}
	else{
	  printf("CMD: Changed Z[%d] by %f microns\n",itemp,ftemp);
	  return CMD_NORMAL;
	}	  
      }
      else{
	printf("CMD: Zernike cmd out of bounds\n");
	return CMD_NORMAL;
      }
    }
    else
      printf("CMD: Manual ALP DM control disabled in this state\n");
    return CMD_NORMAL;
  }

  //Add a single actuator value to the current command
  sprintf(cmd,"alp set actuator");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      pch = strtok(line+strlen(cmd)," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      itemp  = atoi(pch);
      pch = strtok(NULL," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      ftemp  = atof(pch);
      printf("CMD: Trying to change A[%d] by %f units\n",itemp,ftemp);
      if(itemp >= 0 && itemp < ALP_NACT && ftemp >= -1 && ftemp <= 1){
	//Get current command
	if(alp_get_command(sm_p,&alp)){
	  printf("CMD: alp_get_command failed!\n");
	  return CMD_NORMAL;
	}

	//Add to current command
	alp.acmd[itemp] += ftemp;

	//Send command
	if(alp_send_command(sm_p,&alp,WATID,1)){
	  printf("CMD: alp_send_command failed\n");
	  return CMD_NORMAL;
	}
	else{
	  printf("CMD: Changed A[%d] by %f microns\n",itemp,ftemp);
	  return CMD_NORMAL;
	}	  
      }
      else{
	printf("CMD: Zernike cmd out of bounds\n");
	return CMD_NORMAL;
      }
    }
    else
      printf("CMD: Manual ALP DM control disabled in this state\n");
    return CMD_NORMAL;
  }


  /****************************************
   * BMC DM CONTROL
   ***************************************/
  //Reset BMC functions
  sprintf(cmd,"bmc reset");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Resetting BMC functions\n");
    bmc_function_reset(sm_p);
    return(CMD_NORMAL);
  }

  //HV control
  sprintf(cmd,"bmc hv enable");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Enabling BMC HV\n");
    printf("CMD: -- WARNING -- Only operate HV below 30 percent humidity\n");
    sm_p->bmc_hv_enable=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"bmc hv disable");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->bmc_hv_on){
      printf("CMD: Turning OFF BMC HV\n");
      // Set all actuators to Zero
      if(bmc_zero_flat(sm_p,WATID))
	printf("CMD: ERROR: bmc_zero_flat failed!\n");
      // Stop BMC controller, turn OFF HV
      if((ret=libbmc_hv_off((libbmc_device_t *)&sm_p->libbmc_device)) < 0)
	printf("CMD: Failed to stop BMC controller : %s - %s \n", libbmc_error_name(ret), libbmc_strerror(ret));
      else
	sm_p->bmc_hv_on = 0;
    }
    printf("CMD: Disabling BMC HV\n");
    sm_p->bmc_hv_enable=0;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"bmc hv on");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->bmc_ready){
      if(sm_p->bmc_hv_enable){
	printf("CMD: Turning ON BMC HV\n");
	printf("CMD: -- WARNING -- Only operate HV below 30 percent humidity\n");
	// Start BMC controller, turn ON HV
	if((ret=libbmc_hv_on((libbmc_device_t *)&sm_p->libbmc_device,BMC_RANGE)) < 0)
	  printf("CMD: Failed to start BMC controller : %s - %s \n", libbmc_error_name(ret), libbmc_strerror(ret));
	else
	  sm_p->bmc_hv_on = 1;
      }
      else{
	printf("CMD: BMC HV not enabled\n");
      }
    }
    else{
      printf("CMD: BMC controller not ready\n");
    }
    return(CMD_NORMAL);
  }

  sprintf(cmd,"bmc hv off");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->bmc_ready){
      if(sm_p->bmc_hv_enable){
	printf("CMD: Turning OFF BMC HV\n");
	// Set all actuators to Zero -- NOTE: This will fail in states where WATID is not the BMC commander
	if(bmc_zero_flat(sm_p,WATID))
	  printf("CMD: ERROR: bmc_zero_flat failed!\n");
 	// Stop BMC controller, turn OFF HV
	if((ret=libbmc_hv_off((libbmc_device_t *)&sm_p->libbmc_device)) < 0)
	  printf("CMD: Failed to stop BMC controller : %s - %s \n", libbmc_error_name(ret), libbmc_strerror(ret));
	else
	  sm_p->bmc_hv_on = 0;
      }
      else{
	printf("CMD: BMC HV not enabled\n");
      }
    }
    else{
      printf("CMD: BMC controller not ready\n");
    }
    return(CMD_NORMAL);
  }

  //Toggle BMC LEDs
  sprintf(cmd,"bmc leds on");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Turning ON BMC LEDs\n");
    if(sm_p->state_array[sm_p->state].bmc_commander == WATID){
      if(libbmc_toggle_leds_on((libbmc_device_t *)&sm_p->libbmc_device))
	printf("CMD: ERROR (libbmc_toggle_leds_on)\n");
      usleep(LIBBMC_LONG_USLEEP);
    }
    return(CMD_NORMAL);
  }
  sprintf(cmd,"bmc leds off");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Turning OFF BMC LEDs\n");
    if(sm_p->state_array[sm_p->state].bmc_commander == WATID){
      if(libbmc_toggle_leds_off((libbmc_device_t *)&sm_p->libbmc_device))
	printf("CMD: ERROR (libbmc_toggle_leds_off)\n");
      usleep(LIBBMC_LONG_USLEEP);
    }
    return(CMD_NORMAL);
  }
 
  
  //Set all BMC actuators to the same value
  sprintf(cmd,"bmc bias");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].bmc_commander == WATID){
      ftemp = atof(line+strlen(cmd)+1);
      if((ftemp >= BMC_VMIN) && (ftemp <= BMC_VMAX)){
	printf("CMD: Setting BMC bias = %f\n",ftemp);
	if(sm_p->bmc_ready && sm_p->bmc_hv_on){
	  if(bmc_set_bias(sm_p,ftemp,WATID))
	    printf("CMD: ERROR: bmc_set_random failed!\n");
	}
	else
	  printf("CMD: BMC not ready. INIT[%d]  HV[%d]\n",sm_p->bmc_ready,sm_p->bmc_hv_on);
      }
      else
	printf("CMD: BMC bias must be between %d and %d \n",BMC_VMIN,BMC_VMAX);
    }
    else
      printf("CMD: Manual BMC DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Set BMC to all ZEROS
  sprintf(cmd,"bmc zero flat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].bmc_commander == WATID){
      printf("CMD: Setting BMC to all zeros\n");
      if(sm_p->bmc_ready && sm_p->bmc_hv_on){
	if(bmc_zero_flat(sm_p,WATID))
	  printf("CMD: ERROR: bmc_zero_flat failed!\n");
      }
      else
	printf("CMD: BMC not ready. INIT[%d]  HV[%d]\n",sm_p->bmc_ready,sm_p->bmc_hv_on);
    }
    else
      printf("CMD: Manual BMC DM control disabled in this state\n");
    return(CMD_NORMAL);
  }
  
  //Revert BMC to default flat
  sprintf(cmd,"bmc revert flat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].bmc_commander == WATID){
      printf("CMD: Reverting BMC flat to default\n");
      if(sm_p->bmc_ready && sm_p->bmc_hv_on){
	if(bmc_revert_flat(sm_p,WATID))
	  printf("CMD: ERROR: bmc_revert_flat failed!\n");
      }
      else
	printf("CMD: BMC not ready. INIT[%d]  HV[%d]\n",sm_p->bmc_ready,sm_p->bmc_hv_on);
    }
    else
      printf("CMD: Manual BMC DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Save current BMC command to disk
  sprintf(cmd,"bmc save flat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Saving current BMC command to disk as flat\n");
    if(bmc_save_flat(sm_p))
      printf("CMD: ERROR: bmc_save_flat failed!\n");
    return(CMD_NORMAL);
  }

  //Load BMC command from disk
  sprintf(cmd,"bmc load flat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].bmc_commander == WATID){
      printf("CMD: Loading BMC flat from file\n");
      if(sm_p->bmc_ready && sm_p->bmc_hv_on){
	if(bmc_load_flat(sm_p,WATID))
	  printf("CMD: ERROR: bmc_load_flat failed!\n");
      }
      else
	printf("CMD: BMC not ready. INIT[%d]  HV[%d]\n",sm_p->bmc_ready,sm_p->bmc_hv_on);
    }
    else
      printf("CMD: Manual BMC DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Recall flat
  sprintf(cmd,"bmc recall flat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].bmc_commander == WATID){
      pch = strtok(line+strlen(cmd)," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      itemp  = atoi(pch);
      if((itemp >= BMC_NFLAT) || (itemp < 0)){
	printf("CMD: BMC flat position must be within: [%d,%d]\n",0,BMC_NFLAT-1);
	return(CMD_NORMAL);
      }
      printf("CMD: Setting BMC flat to position %d\n",itemp);
      //Set flat
      if(bmc_recall_flat(sm_p,WATID,itemp)){
	printf("CMD: bmc_recall_flat failed\n");
	return CMD_NORMAL;
      }
    }
    else
      printf("CMD: Manual BMC DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Set current position as flat
  sprintf(cmd,"bmc set flat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].bmc_commander == WATID){
      //Get current command
      if(bmc_get_command(sm_p,&bmc)){
	printf("CMD: bmc_get_command failed!\n");
	return CMD_NORMAL;
      }
      //Send command
      if(bmc_send_command(sm_p,&bmc,WATID,BMC_SET_FLAT)){
	printf("CMD: bmc_send_command failed\n");
	return CMD_NORMAL;
      }
      else{
	printf("CMD: current BMC command set as flat\n");
	return CMD_NORMAL;
      }	  
    }
    else
      printf("CMD: Manual BMC DM control disabled in this state\n");
    return CMD_NORMAL;
  }

  //Perturb BMC command with random actuator values
  sprintf(cmd,"bmc random");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].bmc_commander == WATID){
      printf("CMD: Sending random actuator pattern to BMC DM\n");
      if(sm_p->bmc_ready && sm_p->bmc_hv_on){
	if(bmc_set_random(sm_p,WATID))
	  printf("CMD: ERROR: bmc_set_random failed!\n");
      }
      else
	printf("CMD: BMC not ready. INIT[%d]  HV[%d]\n",sm_p->bmc_ready,sm_p->bmc_hv_on);
    }
    else
      printf("CMD: Manual BMC DM control disabled in this state\n");
    return(CMD_NORMAL);
  }
  

  //Add a single actuator value to the current command
  sprintf(cmd,"bmc set actuator");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].bmc_commander == WATID){
      pch = strtok(line+strlen(cmd)," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      itemp  = atoi(pch);
      pch = strtok(NULL," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      ftemp  = atof(pch);
      printf("CMD: Trying to change A[%d] by %f volts\n",itemp,ftemp);
      if(itemp >= 0 && itemp < BMC_NACT && ftemp >= -50  && ftemp <= 50){
	//Get current command
	if(bmc_get_command(sm_p,&bmc)){
	  printf("CMD: bmc_get_command failed!\n");
	  return CMD_NORMAL;
	}

	//Add to current command
	bmc.acmd[itemp] += ftemp;

	//Send command
	if(bmc_send_command(sm_p,&bmc,WATID,BMC_NOSET_FLAT)){
	  printf("CMD: bmc_send_command failed\n");
	  return CMD_NORMAL;
	}
	else{
	  printf("CMD: Changed A[%d] by %f volts\n",itemp,ftemp);
	  return CMD_NORMAL;
	}	  
      }
      else{
	printf("CMD: Actuator delta out of bounds [-50,50]\n");
	return CMD_NORMAL;
      }
    }
    else
      printf("CMD: Manual BMC DM control disabled in this state\n");
    return CMD_NORMAL;
  }

  //Add a HOWFS probe pattern to BMC
  sprintf(cmd,"bmc probe");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].bmc_commander == WATID){
      pch = strtok(line+strlen(cmd)," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      itemp  = atoi(pch);
      
      if(itemp >= 0 && itemp < SCI_HOWFS_NPROBE){
	//Get current command
	if(bmc_get_command(sm_p,&bmc)){
	  printf("CMD: bmc_get_command failed!\n");
	  return CMD_NORMAL;
	}

	//Add probe pattern
	bmc_add_probe(bmc.acmd,bmc.acmd,sm_p->efc_dhrot,itemp,sm_p->efc_probe_amp);
      
	//Send command
	if(bmc_send_command(sm_p,&bmc,WATID,BMC_NOSET_FLAT)){
	  printf("CMD: bmc_send_command failed\n");
	  return CMD_NORMAL;
	}
	else{
	  printf("CMD: set BMC probe %d\n",itemp);
	  return CMD_NORMAL;
	}	  
      }
      else{
	printf("CMD: Invalid probe index [0,1,2,3]\n");
	return CMD_NORMAL;
      }
    }
    else
      printf("CMD: Manual BMC DM control disabled in this state\n");
    return CMD_NORMAL;
  }

  //Add a TEST pattern to BMC
  sprintf(cmd,"bmc test");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].bmc_commander == WATID){
      pch = strtok(line+strlen(cmd)," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      itemp  = atoi(pch);
      
      //Get current command
      if(bmc_get_command(sm_p,&bmc)){
	printf("CMD: bmc_get_command failed!\n");
	return CMD_NORMAL;
      }
      
      //Add test pattern
      bmc_add_test(bmc.acmd,bmc.acmd,itemp,sm_p->bmc_cal_scale);
      
      //Send command
      if(bmc_send_command(sm_p,&bmc,WATID,BMC_NOSET_FLAT)){
	printf("CMD: bmc_send_command failed\n");
	return CMD_NORMAL;
      }
      else{
	printf("CMD: set BMC test pattern %d\n",itemp);
	return CMD_NORMAL;
      }	  
    }
    else
      printf("CMD: Manual BMC DM control disabled in this state\n");
    return CMD_NORMAL;
  }

  //Add a SINE pattern to BMC
  sprintf(cmd,"bmc sine");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].bmc_commander == WATID){
      pch = strtok(line+strlen(cmd)," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      itemp  = atoi(pch);
      
      //Get current command
      if(bmc_get_command(sm_p,&bmc)){
	printf("CMD: bmc_get_command failed!\n");
	return CMD_NORMAL;
      }
      
      //Add sine pattern
      bmc_add_sine(bmc.acmd,bmc.acmd,itemp,sm_p->bmc_cal_scale);
      
      //Send command
      if(bmc_send_command(sm_p,&bmc,WATID,BMC_NOSET_FLAT)){
	printf("CMD: bmc_send_command failed\n");
	return CMD_NORMAL;
      }
      else{
	printf("CMD: set BMC sine pattern %d\n",itemp);
	return CMD_NORMAL;
      }	  
    }
    else
      printf("CMD: Manual BMC DM control disabled in this state\n");
    return CMD_NORMAL;
  }

  
  //Set EFC SCI threshold
  sprintf(cmd,"efc sci thresh");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    pch = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: Bad command format\n");
      return CMD_NORMAL;
    }
    ftemp  = atof(pch);
    sm_p->efc_sci_thresh = ftemp;
    printf("CMD: EFC SCI threshold set to %f\n",sm_p->efc_sci_thresh);
    return CMD_NORMAL;
  }

  //Set EFC BMC max
  sprintf(cmd,"efc bmc max");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    pch = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: Bad command format\n");
      return CMD_NORMAL;
    }
    ftemp  = atof(pch);
    sm_p->efc_bmc_max = ftemp;
    printf("CMD: EFC BMC max set to %f\n",sm_p->efc_bmc_max);
    return CMD_NORMAL;
  }

  //Set EFC Gain
  sprintf(cmd,"efc gain");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    pch = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: Bad command format\n");
      return CMD_NORMAL;
    }
    ftemp  = atof(pch);
    if(ftemp <= 0 && ftemp >= -1){
      sm_p->efc_gain = ftemp;
      printf("CMD: EFC gain set to %f\n",sm_p->efc_gain);
    }
    else{
      printf("CMD: EFC gain %f out of bounds [-1,0]\n",ftemp);
    }
    return CMD_NORMAL;
  }

  //Set EFC Probe Amplitude
  sprintf(cmd,"efc probe amp");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    pch = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: Bad command format\n");
      return CMD_NORMAL;
    }
    itemp  = atoi(pch);
    sm_p->efc_probe_amp = itemp;
    printf("CMD: EFC probe amp set to %d\n",sm_p->efc_probe_amp);
    //Reset SCI to load new matrix
    sm_p->sci_reset=1;
    return CMD_NORMAL;
  }

  //Set EFC matrix index
  sprintf(cmd,"efc matrix");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    pch = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: Bad command format\n");
      return CMD_NORMAL;
    }
    itemp  = atoi(pch);
    sm_p->efc_matrix = itemp;
    printf("CMD: EFC matrix index set to %d\n",sm_p->efc_matrix);
    //Reset SCI to load new matrix
    sm_p->sci_reset=1;
    return CMD_NORMAL;
  }

  //Set EFC darkhole rotation
  sprintf(cmd,"efc dhrot");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    pch = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: Bad command format\n");
      return CMD_NORMAL;
    }
    itemp  = atoi(pch);
    sm_p->efc_dhrot = itemp;
    printf("CMD: EFC darkhole rotation set to %d\n",sm_p->efc_dhrot);
    //Reset SCI to load new matrix
    sm_p->sci_reset=1;
    return CMD_NORMAL;
  }
  
  //Set EFC Relative
  sprintf(cmd,"efc relative");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    pch = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: Bad command format\n");
      return CMD_NORMAL;
    }
    ftemp  = atof(pch);
    if(ftemp >= 0 && ftemp <= 1){
      sm_p->efc_relative = ftemp;
      printf("CMD: EFC relative scale factor set to %f\n",sm_p->efc_relative);
    }
    else{
      printf("CMD: EFC relative scale factor %f out of bounds [0,1]\n",ftemp);
    }
    return CMD_NORMAL;
  }

  //Set Speckle nulling scale factor
  sprintf(cmd,"speckle scale");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    pch = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: Bad command format\n");
      return CMD_NORMAL;
    }
    ftemp  = atof(pch);
    sm_p->speckle_scale = ftemp;
    printf("CMD: Speckle scale set to %f\n",sm_p->speckle_scale);
    return CMD_NORMAL;
  }
  /****************************************
   * SENSOR CALIBRATION
   **************************************/
  
  //SHK HEX Calibration
  sprintf(cmd,"shk calibrate hex");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].hex_commander == SHKID){
      //Get calmode
      cmdfound = 0;
      for(i=0;i<HEX_NCALMODES;i++){
	if(!strncasecmp(line+strlen(cmd)+1,hexcalmodes[i].cmd,strlen(hexcalmodes[i].cmd))){
	  calmode  = i;
	  cmdfound = 1;
	}
      }
      if(!cmdfound){
	printf("CMD: Could not find hex calmode\n");
	print_hex_calmodes(hexcalmodes,sm_p->hex_calmode);
	return(CMD_NORMAL);
      }
      printf("CMD: Running SHK HEX calibration\n");
      //Change calibration output filename
      timestamp(stemp);
      sprintf((char *)sm_p->calfile,SHK_HEX_CALFILE,hexcalmodes[calmode].cmd,sm_p->state_array[sm_p->state].cmd,stemp);
      //Start data recording
      printf("  -- Starting SHK data recording\n");
      printf("  -- File: %s\n",sm_p->calfile);
      sm_p->w[DIAID].launch = getshk_proc;
      sm_p->w[DIAID].run    = 1;
      sleep(3);
      //Start probe pattern
      sm_p->hex_calmode = calmode;
      printf("  -- Changing HEX calibration mode to %s\n",hexcalmodes[sm_p->hex_calmode].name);
      while(sm_p->hex_calmode == calmode)
	sleep(1);
      printf("  -- Stopping data recording\n");
      //Stop data recording
      sm_p->w[DIAID].run    = 0;
      printf("  -- Done\n");
      return(CMD_NORMAL);
    }
    else{
      printf("CMD: Failed: SHK not HEX commander\n");
      return(CMD_NORMAL);
    }
  }

  //SHK ALP Calibration
  sprintf(cmd,"shk calibrate alp");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == SHKID){
      //Get calmode
      cmdfound = 0;
      for(i=0;i<ALP_NCALMODES;i++){
	if(!strncasecmp(line+strlen(cmd)+1,alpcalmodes[i].cmd,strlen(alpcalmodes[i].cmd))){
	  calmode  = i;
	  cmdfound = 1;
	}
      }
      if(!cmdfound){
	printf("CMD: Could not find alp calmode\n");
	print_alp_calmodes(alpcalmodes,sm_p->alp_calmode);
	return(CMD_NORMAL);
      }
      printf("CMD: Running SHK ALP calibration\n");
      //Change calibration output filename
      timestamp(stemp);
      sprintf((char *)sm_p->calfile,SHK_ALP_CALFILE,alpcalmodes[calmode].cmd,sm_p->state_array[sm_p->state].cmd,stemp);
      //Start data recording
      printf("  -- Starting SHK data recording\n");
      printf("  -- File: %s\n",sm_p->calfile);
      sm_p->w[DIAID].launch = getshk_proc;
      sm_p->w[DIAID].run    = 1;
      sleep(3);
      //Start calmode
      sm_p->alp_calmode = calmode;
      printf("  -- Changing ALP calibration mode to %s\n",alpcalmodes[sm_p->alp_calmode].name);
      while(sm_p->alp_calmode == calmode)
	sleep(1);
      printf("  -- Stopping data recording\n");
      //Stop data recording
      sm_p->w[DIAID].run    = 0;
      printf("  -- Done\n");
      return(CMD_NORMAL);
    }
    else{
      printf("CMD: Failed: SHK not ALP commander\n");
      return(CMD_NORMAL);
    }
  }

  //SHK TGT Calibration
  sprintf(cmd,"shk calibrate tgt");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == SHKID){
      //Get calmode
      cmdfound = 0;
      for(i=0;i<TGT_NCALMODES;i++){
	if(!strncasecmp(line+strlen(cmd)+1,tgtcalmodes[i].cmd,strlen(tgtcalmodes[i].cmd))){
	  calmode  = i;
	  cmdfound = 1;
	}
      }
      if(!cmdfound){
	printf("CMD: Could not find tgt calmode\n");
	print_tgt_calmodes(tgtcalmodes,sm_p->tgt_calmode);
	return(CMD_NORMAL);
      }
      printf("CMD: Running SHK TGT calibration\n");
      //Change calibration output filename
      timestamp(stemp);
      sprintf((char *)sm_p->calfile,SHK_TGT_CALFILE,tgtcalmodes[calmode].cmd,sm_p->state_array[sm_p->state].cmd,stemp);
      //Start data recording
      printf("  -- Starting SHK data recording\n");
      printf("  -- File: %s\n",sm_p->calfile);
      sm_p->w[DIAID].launch = getshk_proc;
      sm_p->w[DIAID].run    = 1;
      sleep(3);
      //Start probe pattern
      sm_p->tgt_calmode = calmode;
      printf("  -- Changing TGT calibration mode to %s\n",tgtcalmodes[sm_p->tgt_calmode].name);
      while(sm_p->tgt_calmode == calmode)
	sleep(1);
      printf("  -- Stopping data recording\n");
      //Stop data recording
      sm_p->w[DIAID].run    = 0;
      printf("  -- Done\n");
      return(CMD_NORMAL);
    }
    else{
      printf("CMD: Failed: SHK not ALP commander\n");
      return(CMD_NORMAL);
    }
  }

  //LYT ALP Calibration
  sprintf(cmd,"lyt calibrate alp");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == LYTID){
      //Get calmode
      cmdfound = 0;
      for(i=0;i<ALP_NCALMODES;i++){
	if(!strncasecmp(line+strlen(cmd)+1,alpcalmodes[i].cmd,strlen(alpcalmodes[i].cmd))){
	  calmode  = i;
	  cmdfound = 1;
	}
      }
      if(!cmdfound){
	printf("CMD: Could not find alp calmode\n");
	print_alp_calmodes(alpcalmodes,sm_p->alp_calmode);
	return(CMD_NORMAL);
      }
      printf("CMD: Running LYT ALP calibration\n");
      //Change calibration output filename
      timestamp(stemp);
      sprintf((char *)sm_p->calfile,LYT_ALP_CALFILE,alpcalmodes[calmode].cmd,sm_p->state_array[sm_p->state].cmd,stemp);
      //Start data recording
      printf("  -- Starting LYT data recording\n");
      printf("  -- File: %s\n",sm_p->calfile);
      sm_p->w[DIAID].launch = getlyt_proc;
      sm_p->w[DIAID].run    = 1;
      sleep(3);
      //Start probe pattern
      sm_p->alp_calmode = calmode;
      printf("  -- Changing ALP calibration mode to %s\n",alpcalmodes[sm_p->alp_calmode].name);
      while(sm_p->alp_calmode == calmode)
	sleep(1);
      printf("  -- Stopping data recording\n");
      //Stop data recording
      sm_p->w[DIAID].run    = 0;
      printf("  -- Done\n");
      return(CMD_NORMAL);
    }
    else{
      printf("CMD: Failed: LYT not ALP commander\n");
      return(CMD_NORMAL);
    }
  }
  
  //LYT TGT Calibration
  sprintf(cmd,"lyt calibrate tgt");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == LYTID){
      //Get calmode
      cmdfound = 0;
      for(i=0;i<TGT_NCALMODES;i++){
	if(!strncasecmp(line+strlen(cmd)+1,tgtcalmodes[i].cmd,strlen(tgtcalmodes[i].cmd))){
	  calmode  = i;
	  cmdfound = 1;
	}
      }
      if(!cmdfound){
	printf("CMD: Could not find tgt calmode\n");
	print_tgt_calmodes(tgtcalmodes,sm_p->tgt_calmode);
	return(CMD_NORMAL);
      }
      printf("CMD: Running LYT TGT calibration\n");
      //Change calibration output filename
      timestamp(stemp);
      sprintf((char *)sm_p->calfile,LYT_TGT_CALFILE,tgtcalmodes[calmode].cmd,sm_p->state_array[sm_p->state].cmd,stemp);
      //Start data recording
      printf("  -- Starting LYT data recording\n");
      printf("  -- File: %s\n",sm_p->calfile);
      sm_p->w[DIAID].launch = getlyt_proc;
      sm_p->w[DIAID].run    = 1;
      sleep(3);
      //Start probe pattern
      sm_p->tgt_calmode = calmode;
      printf("  -- Changing TGT calibration mode to %s\n",tgtcalmodes[sm_p->tgt_calmode].name);
      while(sm_p->tgt_calmode == calmode)
	sleep(1);
      printf("  -- Stopping data recording\n");
      //Stop data recording
      sm_p->w[DIAID].run    = 0;
      printf("  -- Done\n");
      return(CMD_NORMAL);
    }
    else{
      printf("CMD: Failed: LYT not ALP commander\n");
      return(CMD_NORMAL);
    }
  }

  //SCI BMC Calibration
  sprintf(cmd,"sci calibrate bmc");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].bmc_commander == SCIID){
      //Get calmode
      cmdfound = 0;
      for(i=0;i<BMC_NCALMODES;i++){
	if(!strncasecmp(line+strlen(cmd)+1,bmccalmodes[i].cmd,strlen(bmccalmodes[i].cmd))){
	  calmode  = i;
	  cmdfound = 1;
	}
      }
      if(!cmdfound){
	printf("CMD: Could not find bmc calmode\n");
	print_bmc_calmodes(bmccalmodes,sm_p->bmc_calmode);
	return(CMD_NORMAL);
      }
      printf("CMD: Running SCI BMC calibration\n");
      //Change calibration output filename
      timestamp(stemp);
      sprintf((char *)sm_p->calfile,SCI_BMC_CALFILE,bmccalmodes[calmode].cmd,sm_p->state_array[sm_p->state].cmd,stemp);
      //Start data recording
      printf("  -- Starting SCI data recording\n");
      printf("  -- File: %s\n",sm_p->calfile);
      sm_p->w[DIAID].launch = getsci_proc;
      sm_p->w[DIAID].run    = 1;
      sleep(3);
      //Start probe pattern
      sm_p->bmc_calmode = calmode;
      printf("  -- Changing BMC calibration mode to %s\n",bmccalmodes[sm_p->bmc_calmode].name);
      while(sm_p->bmc_calmode == calmode)
	sleep(1);
      printf("  -- Stopping data recording\n");
      //Stop data recording
      sm_p->w[DIAID].run    = 0;
      printf("  -- Done\n");
      return(CMD_NORMAL);
    }
    else{
      printf("CMD: Failed: SCI not BMC commander\n");
      return(CMD_NORMAL);
    }
  }
  
  //SCI TGT Calibration
  sprintf(cmd,"sci calibrate tgt");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].tgt_commander == SCIID){
      //Get calmode
      cmdfound = 0;
      for(i=0;i<TGT_NCALMODES;i++){
	if(!strncasecmp(line+strlen(cmd)+1,tgtcalmodes[i].cmd,strlen(tgtcalmodes[i].cmd))){
	  calmode  = i;
	  cmdfound = 1;
	}
      }
      if(!cmdfound){
	printf("CMD: Could not find tgt calmode\n");
	print_tgt_calmodes(tgtcalmodes,sm_p->tgt_calmode);
	return(CMD_NORMAL);
      }
      printf("CMD: Running SCI TGT calibration\n");
      //Change calibration output filename
      timestamp(stemp);
      sprintf((char *)sm_p->calfile,SCI_TGT_CALFILE,tgtcalmodes[calmode].cmd,sm_p->state_array[sm_p->state].cmd,stemp);
      //Start data recording
      printf("  -- Starting SCI data recording\n");
      printf("  -- File: %s\n",sm_p->calfile);
      sm_p->w[DIAID].launch = getsci_proc;
      sm_p->w[DIAID].run    = 1;
      sleep(3);
      //Start probe pattern
      sm_p->tgt_calmode = calmode;
      printf("  -- Changing TGT calibration mode to %s\n",tgtcalmodes[sm_p->tgt_calmode].name);
      while(sm_p->tgt_calmode == calmode)
	sleep(1);
      printf("  -- Stopping data recording\n");
      //Stop data recording
      sm_p->w[DIAID].run    = 0;
      printf("  -- Done\n");
      return(CMD_NORMAL);
    }
    else{
      printf("CMD: Failed: SCI not TGT commander\n");
      return(CMD_NORMAL);
    }
  }

  //SCI Phase Flattening
  sprintf(cmd,"sci runphase");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == SCIID){
      //Get phasemode
      cmdfound = 0;
      for(i=0;i<SCI_NPHASEMODES;i++){
	if(!strncasecmp(line+strlen(cmd)+1,sciphasemodes[i].cmd,strlen(sciphasemodes[i].cmd))){
	  phasemode  = i;
	  cmdfound = 1;
	}
      }
      if(!cmdfound){
	printf("CMD: Could not find sci phasemode\n");
	print_sci_phasemodes(sciphasemodes,sm_p->sci_phasemode);
	return(CMD_NORMAL);
      }
      printf("CMD: Running SCI Phase Flattening\n");
      //Change calibration output filename
      timestamp(stemp);
      sprintf((char *)sm_p->calfile,SCI_PHASE_CALFILE,sciphasemodes[phasemode].cmd,sm_p->state_array[sm_p->state].cmd,stemp);
      //Start data recording
      printf("  -- Starting SCI data recording\n");
      printf("  -- File: %s\n",sm_p->calfile);
      sm_p->w[DIAID].launch = getsci_proc;
      sm_p->w[DIAID].run    = 1;
      sleep(3);
      //Start phase mode
      sm_p->sci_phasemode = phasemode;
      printf("  -- Changing SCI phasemode to %s\n",sciphasemodes[sm_p->sci_phasemode].name);
      while(sm_p->sci_phasemode == phasemode)
	sleep(1);
      printf("  -- Stopping data recording\n");
      //Stop data recording
      sm_p->w[DIAID].run    = 0;
      printf("  -- Done\n");
      return(CMD_NORMAL);
    }
    else{
      printf("CMD: Failed: SCI not ALP commander\n");
      return(CMD_NORMAL);
    }
  }

  /****************************************
   * CAMERA CONTROL
   **************************************/
  
  //Exposure & Frame Time Commands
  sprintf(cmd,"sci exptime");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= SCI_EXPTIME_MIN && ftemp <= SCI_EXPTIME_MAX){
      sm_p->sci_exptime = ftemp;
      sm_p->sci_reset_camera = 1;
      printf("CMD: Setting SCI exptime to %f seconds\n",sm_p->sci_exptime);
    }
    else
      printf("CMD: SCI exptime must be between %f and %f seconds\n",SCI_EXPTIME_MIN,SCI_EXPTIME_MAX);
    return(CMD_NORMAL);
  }
  
  sprintf(cmd,"sci next exptime");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= SCI_EXPTIME_MIN && ftemp <= SCI_EXPTIME_MAX){
      sm_p->sci_next_exptime = ftemp;
      printf("CMD: Setting next SCI exptime to %f seconds\n",sm_p->sci_next_exptime);
    }
    else
      printf("CMD: SCI exptime must be between %f and %f seconds\n",SCI_EXPTIME_MIN,SCI_EXPTIME_MAX);
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci frmtime"); //NOTE this command has no function. frmtime is measured, not set.
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= SCI_FRMTIME_MIN && ftemp <= SCI_FRMTIME_MAX){
      sm_p->sci_frmtime = ftemp;
      sm_p->sci_exptime = ftemp;
      sm_p->sci_reset_camera = 1;
      printf("CMD: Setting SCI frmtime and exptime to %f seconds\n",sm_p->sci_frmtime);
    }
    else
      printf("CMD: SCI frmtime must be between %f and %f seconds\n",SCI_FRMTIME_MIN,SCI_FRMTIME_MAX);
    return(CMD_NORMAL);
  }
  
  sprintf(cmd,"shk exptime");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= SHK_EXPTIME_MIN && ftemp <= SHK_EXPTIME_MAX){
      sm_p->shk_exptime = ftemp;
      sm_p->shk_reset_camera = 1;
      printf("CMD: Setting SHK exptime to %f seconds\n",sm_p->shk_exptime);
    }
    else
      printf("CMD: SHK exptime must be between %f and %f seconds\n",SHK_EXPTIME_MIN,SHK_EXPTIME_MAX);
    return(CMD_NORMAL);
  }

  sprintf(cmd,"shk frmtime");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= SHK_FRMTIME_MIN && ftemp <= SHK_FRMTIME_MAX){
      sm_p->shk_frmtime = ftemp;
      sm_p->shk_exptime = ftemp;
      sm_p->shk_reset_camera = 1;
      printf("CMD: Setting SHK frmtime and exptime to %f seconds\n",sm_p->shk_frmtime);
    }
    else
      printf("CMD: SHK frmtime must be between %f and %f seconds\n",SHK_FRMTIME_MIN,SHK_FRMTIME_MAX);
    return(CMD_NORMAL);
  }

  sprintf(cmd,"lyt exptime");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= LYT_EXPTIME_MIN && ftemp <= LYT_EXPTIME_MAX){
      sm_p->lyt_exptime = ftemp;
      sm_p->lyt_reset_camera = 1;
      printf("CMD: Setting LYT exptime to %f seconds\n",sm_p->lyt_exptime);
    }
    else
      printf("CMD: LYT exptime must be between %f and %f seconds\n",LYT_EXPTIME_MIN,LYT_EXPTIME_MAX);
    return(CMD_NORMAL);
  }

  sprintf(cmd,"lyt frmtime");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= LYT_FRMTIME_MIN && ftemp <= LYT_FRMTIME_MAX){
      sm_p->lyt_frmtime = ftemp;
      sm_p->lyt_exptime = ftemp;
      sm_p->lyt_reset_camera = 1;
      printf("CMD: Setting LYT frmtime and exptime to %f seconds\n",sm_p->lyt_frmtime);
    }
    else
      printf("CMD: LYT frmtime must be between %f and %f seconds\n",LYT_FRMTIME_MIN,LYT_FRMTIME_MAX);
    return(CMD_NORMAL);
  }

  sprintf(cmd,"acq exptime");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= ACQ_EXPTIME_MIN && ftemp <= ACQ_EXPTIME_MAX){
      sm_p->acq_exptime = ftemp;
      sm_p->acq_reset_camera = 1;
      printf("CMD: Setting ACQ exptime to %f seconds\n",sm_p->acq_exptime);
    }
    else
      printf("CMD: ACQ exptime must be between %f and %f seconds\n",ACQ_EXPTIME_MIN,ACQ_EXPTIME_MAX);
    return(CMD_NORMAL);
  }

  sprintf(cmd,"acq frmtime");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= ACQ_FRMTIME_MIN && ftemp <= ACQ_FRMTIME_MAX){
      sm_p->acq_frmtime = ftemp;
      sm_p->acq_exptime = ftemp;
      sm_p->acq_reset_camera = 1;
      printf("CMD: Setting ACQ frmtime and exptime to %f seconds\n",sm_p->acq_frmtime);
    }
    else
      printf("CMD: ACQ frmtime must be between %f and %f seconds\n",ACQ_FRMTIME_MIN,ACQ_FRMTIME_MAX);
    return(CMD_NORMAL);
  }

  /****************************************
   * CAMERA RESET
   **************************************/
  
  sprintf(cmd,"shk reset");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Resetting SHK\n");
    sm_p->shk_reset=1;
    return(CMD_NORMAL);
  }
  sprintf(cmd,"lyt reset");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Resetting LYT\n");
    sm_p->lyt_reset=1;
    return(CMD_NORMAL);
  }
  sprintf(cmd,"sci reset");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Resetting SCI\n");
    sm_p->sci_reset=1;
    return(CMD_NORMAL);
  }
  sprintf(cmd,"acq reset");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Resetting ACQ\n");
    sm_p->acq_reset=1;
    return(CMD_NORMAL);
  }
  
  /****************************************
   * SHACK-HARTMANN LOWFS SETTINGS
   **************************************/
  
  //SHK Origin
  sprintf(cmd,"shk set origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Setting SHK origin\n");
    sm_p->shk_setorigin=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"shk revert origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Reverting SHK origin\n");
    sm_p->shk_revertorigin=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"shk save origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Saving SHK origin\n");
    sm_p->shk_saveorigin=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"shk load origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Loading SHK origin\n");
    sm_p->shk_loadorigin=1;
    return(CMD_NORMAL);
  }
  
  sprintf(cmd,"shk shift origin +x");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: SHK shift origin +1px in X\n");
    sm_p->shk_xshiftorigin = 1;
    return CMD_NORMAL;
  }

  sprintf(cmd,"shk shift origin -x");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: SHK shift origin -1px in X\n");
    sm_p->shk_xshiftorigin = -1;
    return CMD_NORMAL;
  }

  sprintf(cmd,"shk shift origin +y");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: SHK shift origin +1px in Y\n");
    sm_p->shk_yshiftorigin = 1;
    return CMD_NORMAL;
  }

  sprintf(cmd,"shk shift origin -y");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: SHK shift origin -1px in Y\n");
    sm_p->shk_yshiftorigin = -1;
    return CMD_NORMAL;
  }
  
  /****************************************
   * LYOT LOWFS SETTINGS
   **************************************/

  //LYT Origin
  sprintf(cmd,"lyt shift origin +x");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    itemp = sm_p->lyt_xorigin + 1;
    if(itemp < LYT_XORIGIN_MIN || itemp > LYT_XORIGIN_MAX){
      printf("CMD: LYT xorigin %d out of range [%d,%d]\n",itemp,LYT_XORIGIN_MIN,LYT_XORIGIN_MAX);
      return CMD_NORMAL;
    }
    sm_p->lyt_xorigin = itemp;
    printf("CMD: LYT shifted xorigin +1px to %d\n",sm_p->lyt_xorigin);
    return CMD_NORMAL;
  }
  sprintf(cmd,"lyt shift origin -x");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    itemp = sm_p->lyt_xorigin - 1;
    if(itemp < LYT_XORIGIN_MIN || itemp > LYT_XORIGIN_MAX){
      printf("CMD: LYT xorigin %d out of range [%d,%d]\n",itemp,LYT_XORIGIN_MIN,LYT_XORIGIN_MAX);
      return CMD_NORMAL;
    }
    sm_p->lyt_xorigin = itemp;
    printf("CMD: LYT shifted xorigin -1px to %d\n",sm_p->lyt_xorigin);
    return CMD_NORMAL;
  }
  sprintf(cmd,"lyt shift origin +y");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    itemp = sm_p->lyt_yorigin + 1;
    if(itemp < LYT_YORIGIN_MIN || itemp > LYT_YORIGIN_MAX){
      printf("CMD: LYT yorigin %d out of range [%d,%d]\n",itemp,LYT_YORIGIN_MIN,LYT_YORIGIN_MAX);
      return CMD_NORMAL;
    }
    sm_p->lyt_yorigin = itemp;
    printf("CMD: LYT shifted yorigin +1px to %d\n",sm_p->lyt_yorigin);
    return CMD_NORMAL;
  }
  sprintf(cmd,"lyt shift origin -y");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    itemp = sm_p->lyt_yorigin - 1;
    if(itemp < LYT_YORIGIN_MIN || itemp > LYT_YORIGIN_MAX){
      printf("CMD: LYT yorigin %d out of range [%d,%d]\n",itemp,LYT_YORIGIN_MIN,LYT_YORIGIN_MAX);
      return CMD_NORMAL;
    }
    sm_p->lyt_yorigin = itemp;
    printf("CMD: LYT shifted yorigin -1px to %d\n",sm_p->lyt_yorigin);
    return CMD_NORMAL;
  }

  //LYT ROI
  sprintf(cmd,"lyt roi");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    pch = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: Bad command format\n");
      return CMD_NORMAL;
    }
    lyt_roi[0] = atoi(pch);
    pch = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: Bad command format\n");
      return CMD_NORMAL;
    }
    lyt_roi[1] = atoi(pch);
    //Check values
    if((lyt_roi[0] < LYT_ROI_MIN) || (lyt_roi[0] > LYT_ROI_MAX)){
      printf("CMD: LYT ROI out of bounds\n");
      return CMD_NORMAL;
    }
    if((lyt_roi[1] < LYT_ROI_MIN) || (lyt_roi[1] > LYT_ROI_MAX)){
      printf("CMD: LYT ROI out of bounds\n");
      return CMD_NORMAL;
    }
    //Set values & trigger reset
    sm_p->lyt_roi[0] = lyt_roi[0];
    sm_p->lyt_roi[1] = lyt_roi[1];
    sm_p->lyt_roi[2] = LYTREADXS;
    sm_p->lyt_roi[3] = LYTREADYS;
    sm_p->lyt_reset_camera = 1;
    printf("CMD: Set LYT ROI to %d %d %d %d\n",sm_p->lyt_roi[0],sm_p->lyt_roi[1],sm_p->lyt_roi[2],sm_p->lyt_roi[3]);
    return CMD_NORMAL;
  }

  //LYT Image Magnification
  sprintf(cmd,"lyt enable mag");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Enabling LYT Magnification\n");
    sm_p->lyt_mag_enable=1;
    return(CMD_NORMAL);
  }
  
  sprintf(cmd,"lyt disable mag");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Disabling LYT Magnification\n");
    sm_p->lyt_mag_enable=0;
    return(CMD_NORMAL);
  }
  
  sprintf(cmd,"lyt mag");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= LYT_MAG_MIN && ftemp <= LYT_MAG_MAX){
      sm_p->lyt_mag = ftemp;
      printf("CMD: Setting LYT magnification to %f\n",sm_p->lyt_mag);
    }
    else
      printf("CMD: LYT mag must be between %f and %f\n",LYT_MAG_MIN,LYT_MAG_MAX);
    return(CMD_NORMAL);
  }

  sprintf(cmd,"lyt xoff");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    sm_p->lyt_mag_xoff = ftemp;
    printf("CMD: Setting LYT magnification X-offset to %f\n",sm_p->lyt_mag_xoff);
    return(CMD_NORMAL);
  }

  sprintf(cmd,"lyt yoff");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    sm_p->lyt_mag_yoff = ftemp;
    printf("CMD: Setting LYT magnification Y-offset to %f\n",sm_p->lyt_mag_yoff);
    return(CMD_NORMAL);
  }

  //LYT Reset Zernike Commands
  sprintf(cmd,"lyt zcmd reset");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Resetting LYT Zernike commands\n");
    sm_p->lyt_zcmd_reset=1;
    return(CMD_NORMAL);
  }

  //Zernike command limits
  sprintf(cmd,"lyt zcmd limit");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    pch = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      print_lyt_zlimit(sm_p);
      return CMD_NORMAL;
    }
    itemp  = atoi(pch);
    pch = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: Bad command format\n");
      return CMD_NORMAL;
    }
    ftemp  = atof(pch);
    if(itemp >= 0 && itemp < LOWFS_N_ZERNIKE && ftemp >= 0){
	sm_p->lyt_zcmd_limit[itemp] = ftemp;
	printf("CMD: Setting LYT Z[%d] limit = %f microns\n",itemp,sm_p->lyt_zcmd_limit[itemp]);
	print_lyt_zlimit(sm_p);
    }
    else{
      printf("CMD: Zernike limit out of bounds #[%d,%d] limit >= 0\n",0,LOWFS_N_ZERNIKE);
      return CMD_NORMAL;
    }
    return CMD_NORMAL;
  }
  
  
  //LYT Centroid Control
  sprintf(cmd,"lyt enable cen");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Enabling LYT centroid control\n");
    sm_p->lyt_cen_enable=1;
    return(CMD_NORMAL);
  }
  
  sprintf(cmd,"lyt disable cen");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Disabling LYT centroid control\n");
    sm_p->lyt_cen_enable=0;
    return(CMD_NORMAL);
  }

  //LYT Reference Image Commands
  sprintf(cmd,"lyt set ref");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Setting current LYT image as reference\n");
    sm_p->lyt_setref=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"lyt mod ref");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Setting model LYT image as reference\n");
    sm_p->lyt_modref=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"lyt def ref");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Setting default LYT image as reference\n");
    sm_p->lyt_defref=1;
    return(CMD_NORMAL);
  }
  
  sprintf(cmd,"lyt save ref");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Saving LYT reference image to file\n");
    sm_p->lyt_saveref=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"lyt load ref");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Loading LYT reference image from file\n");
    sm_p->lyt_loadref=1;
    return(CMD_NORMAL);
  }

  //LYT Dark Image Commands
  sprintf(cmd,"lyt sub dark enable");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Enabling LYT dark subtraction\n");
    sm_p->lyt_subdark=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"lyt sub dark disable");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Disabling LYT dark subtraction\n");
    sm_p->lyt_subdark=0;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"lyt set dark");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Setting current LYT image as dark\n");
    sm_p->lyt_setdark=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"lyt zero dark");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Setting LYT dark image to zero\n");
    sm_p->lyt_zerodark=1;
    return(CMD_NORMAL);
  }
  
  sprintf(cmd,"lyt save dark");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Saving LYT dark image to file\n");
    sm_p->lyt_savedark=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"lyt load dark");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Loading LYT dark image from file\n");
    sm_p->lyt_loaddark=1;
    return(CMD_NORMAL);
  }

  /****************************************
   * ZERNIKE TARGETS
   **************************************/

  //SHK Zernike Targets
  sprintf(cmd,"shk target reset");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].tgt_commander == WATID){
      printf("CMD: Setting SHK Zernike targets to zero\n");
      for(i=0;i<LOWFS_N_ZERNIKE;i++) sm_p->shk_zernike_target[i]=0;
    }
    else
      printf("CMD: Manual TGT control disabled in this state\n");

    return CMD_NORMAL;
  }
  
  sprintf(cmd,"shk target");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].tgt_commander == WATID){
      pch = strtok(line+strlen(cmd)," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      itemp  = atoi(pch);
      pch = strtok(NULL," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      ftemp  = atof(pch);
      if(itemp >= 0 && itemp < LOWFS_N_ZERNIKE && ftemp >= ALP_ZERNIKE_MIN && ftemp <= ALP_ZERNIKE_MAX){
	sm_p->shk_zernike_target[itemp] = ftemp;
	printf("CMD: Setting SHK target Z[%d] = %f microns\n",itemp,sm_p->shk_zernike_target[itemp]);
      }
      else{
	printf("CMD: Zernike target out of bounds #[%d,%d] C[%f,%f]\n",0,LOWFS_N_ZERNIKE,ALP_ZERNIKE_MIN,ALP_ZERNIKE_MAX);
	return CMD_NORMAL;
      }
    }
    else
      printf("CMD: Manual TGT control disabled in this state\n");
    return CMD_NORMAL;
  }
  
  sprintf(cmd,"shk inc target");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].tgt_commander == WATID){
      pch = strtok(line+strlen(cmd)," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      itemp  = atoi(pch);
      pch = strtok(NULL," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      ftemp  = atof(pch);
      if(itemp >= 0 && itemp < LOWFS_N_ZERNIKE && ftemp >= ALP_DZERNIKE_MIN && ftemp <= ALP_DZERNIKE_MAX){
	sm_p->shk_zernike_target[itemp] += ftemp;
	printf("CMD: Setting SHK target Z[%d] = %f microns\n",itemp,sm_p->shk_zernike_target[itemp]);
      }
      else{
	printf("CMD: Zernike target out of bounds #[%d,%d] C[%f,%f]\n",0,LOWFS_N_ZERNIKE,ALP_DZERNIKE_MIN,ALP_DZERNIKE_MAX);
	return CMD_NORMAL;
      }
    }
    else
      printf("CMD: Manual TGT control disabled in this state\n");
    return CMD_NORMAL;
  }

  //LYT Zernike Targets
  sprintf(cmd,"lyt target reset");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].tgt_commander == WATID){
      printf("CMD: Setting LYT Zernike targets to zero\n");
      for(i=0;i<LOWFS_N_ZERNIKE;i++) sm_p->lyt_zernike_target[i]=0;
    }
    else
      printf("CMD: Manual TGT control disabled in this state\n");
    return CMD_NORMAL;
  }

  sprintf(cmd,"lyt target");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].tgt_commander == WATID){
      pch = strtok(line+strlen(cmd)," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      itemp  = atoi(pch);
      pch = strtok(NULL," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      ftemp  = atof(pch);
      if(itemp >= 0 && itemp < LOWFS_N_ZERNIKE && ftemp >= ALP_ZERNIKE_MIN && ftemp <= ALP_ZERNIKE_MAX){
	sm_p->lyt_zernike_target[itemp] = ftemp;
	printf("CMD: Setting LYT target Z[%d] = %f microns\n",itemp,sm_p->lyt_zernike_target[itemp]);
      }
      else{
	printf("CMD: Zernike target out of bounds #[%d,%d] C[%f,%f]\n",0,LOWFS_N_ZERNIKE,ALP_ZERNIKE_MIN,ALP_ZERNIKE_MAX);
	return CMD_NORMAL;
      }
    }
    else
      printf("CMD: Manual TGT control disabled in this state\n");
    return CMD_NORMAL;
  }

  sprintf(cmd,"lyt inc target");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].tgt_commander == WATID){
      pch = strtok(line+strlen(cmd)," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      itemp  = atoi(pch);
      pch = strtok(NULL," ");
      if(pch == NULL){
	printf("CMD: Bad command format\n");
	return CMD_NORMAL;
      }
      ftemp  = atof(pch);
      if(itemp >= 0 && itemp < LOWFS_N_ZERNIKE && ftemp >= ALP_DZERNIKE_MIN && ftemp <= ALP_DZERNIKE_MAX){
	sm_p->lyt_zernike_target[itemp] += ftemp;
	printf("CMD: Setting LYT target Z[%d] = %f microns\n",itemp,sm_p->lyt_zernike_target[itemp]);
      }
      else{
	printf("CMD: Zernike target out of bounds #[%d,%d] C[%f,%f]\n",0,LOWFS_N_ZERNIKE,ALP_DZERNIKE_MIN,ALP_DZERNIKE_MAX);
	return CMD_NORMAL;
      }
    }
    else
      printf("CMD: Manual TGT control disabled in this state\n");
    return CMD_NORMAL;
  }
  
  /****************************************
   * ZERNIKE CONTROL SWITCHES
   **************************************/
  
  //SHK Zernike control commands
  sprintf(cmd,"shk zernike status");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    print_shk_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"shk zernike disable all");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Disabling SHK control of ALL Zernikes\n\n");
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      sm_p->shk_zernike_control[i]=0;
    print_shk_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"shk zernike enable all");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Enabling SHK control of ALL Zernikes\n\n");
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      sm_p->shk_zernike_control[i]=1;
    print_shk_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"shk zernike disable");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    pch = strtok(line+strlen(cmd)," ");
    while(pch != NULL){
      itemp  = atoi(pch);
      if(itemp >= 0 && itemp < LOWFS_N_ZERNIKE){
	sm_p->shk_zernike_control[itemp]=0;
	printf("CMD: Disabling SHK control of Zernike %d\n",itemp);
      }
      else{
	printf("CMD: Invalid Zernike %d\n",itemp);
      }
      pch = strtok(NULL," ");
    }
    print_shk_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"shk zernike enable");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    pch = strtok(line+strlen(cmd)," ");
    while(pch != NULL){
      itemp  = atoi(pch);
      if(itemp >= 0 && itemp < LOWFS_N_ZERNIKE){
	sm_p->shk_zernike_control[itemp]=1;
	printf("CMD: Enabling SHK control of Zernike %d\n",itemp);
      }
      else{
	printf("CMD: Invalid Zernike %d\n",itemp);
      }
      pch = strtok(NULL," ");
    }
    print_shk_zernikes(sm_p);
    return CMD_NORMAL;
  }

  //LYT Zernike control commands
  sprintf(cmd,"lyt zernike status");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    print_lyt_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"lyt zernike disable all");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Disabling LYT control of ALL Zernikes\n\n");
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      sm_p->lyt_zernike_control[i]=0;
    print_lyt_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"lyt zernike enable all");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Enabling LYT control of ALL Zernikes\n\n");
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      sm_p->lyt_zernike_control[i]=1;
    print_lyt_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"lyt zernike disable");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    pch = strtok(line+strlen(cmd)," ");
    while(pch != NULL){
      itemp  = atoi(pch);
      if(itemp >= 0 && itemp < LOWFS_N_ZERNIKE){
	sm_p->lyt_zernike_control[itemp]=0;
	printf("CMD: Disabling LYT control of Zernike %d\n",itemp);
      }
      else{
	printf("CMD: Invalid Zernike %d\n",itemp);
      }
      pch = strtok(NULL," ");
    }
    print_lyt_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"lyt zernike enable");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    pch = strtok(line+strlen(cmd)," ");
    while(pch != NULL){
      itemp  = atoi(pch);
      if(itemp >= 0 && itemp < LOWFS_N_ZERNIKE){
	sm_p->lyt_zernike_control[itemp]=1;
	printf("CMD: Enabling LYT control of Zernike %d\n",itemp);
      }
      else{
	printf("CMD: Invalid Zernike %d\n",itemp);
      }
      pch = strtok(NULL," ");
    }
    print_lyt_zernikes(sm_p);
    return CMD_NORMAL;
  }
  
  //ALP Zernike control commands
  sprintf(cmd,"alp zernike status");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    print_alp_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"alp zernike disable all");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Disabling ALP control of ALL Zernikes\n\n");
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      sm_p->alp_zernike_control[i]=0;
    print_alp_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"alp zernike enable all");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Enabling ALP control of ALL Zernikes\n\n");
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      sm_p->alp_zernike_control[i]=1;
    print_alp_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"alp zernike disable");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    pch = strtok(line+strlen(cmd)," ");
    while(pch != NULL){
      itemp  = atoi(pch);
      if(itemp >= 0 && itemp < LOWFS_N_ZERNIKE){
	sm_p->alp_zernike_control[itemp]=0;
	printf("CMD: Disabling ALP control of Zernike %d\n",itemp);
      }
      else{
	printf("CMD: Invalid Zernike %d\n",itemp);
      }
      pch = strtok(NULL," ");
    }
    print_alp_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"alp zernike enable");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    pch = strtok(line+strlen(cmd)," ");
    while(pch != NULL){
      itemp  = atoi(pch);
      if(itemp >= 0 && itemp < LOWFS_N_ZERNIKE){
	sm_p->alp_zernike_control[itemp]=1;
	printf("CMD: Enabling ALP control of Zernike %d\n",itemp);
      }
      else{
	printf("CMD: Invalid Zernike %d\n",itemp);
      }
      pch = strtok(NULL," ");
    }
    print_alp_zernikes(sm_p);
    return CMD_NORMAL;
  }
  
  /****************************************
   * GAIN SETTINGS
   **************************************/
  
  //Scale SHK ALP Cell Gain
  sprintf(cmd,"shk alp scale cgain");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= 0){
      for(i=0;i<LOWFS_N_PID;i++){
	sm_p->shk_gain_alp_cell[i] = shk_gain_alp_cell[i]*ftemp;
      }
      printf("CMD: Changing SHK-->ALP cell gain multiplier to %f\n",ftemp);
      printf("CMD: SHK-->ALP Cell Gain: %10.6f | %10.6f | %10.6f\n", sm_p->shk_gain_alp_cell[0],sm_p->shk_gain_alp_cell[1],sm_p->shk_gain_alp_cell[2]);
    }else printf("CMD: Gain multiplier must be >= 0\n");
    return CMD_NORMAL;
  }

  //User input SHK ALP Cell Gain
  sprintf(cmd,"shk alp cgain");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    pch  = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: SHK gain bad format: P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    pgain = atof(pch);
    pch  = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: SHK gain bad format: P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    igain = atof(pch);
    pch  = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: SHK gain bad format: P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    dgain = atof(pch);
    if(pgain <= 0 && pgain >= -1 && igain <= 0 && igain >= -1 && dgain <= 0 && dgain >= -1){
      sm_p->shk_gain_alp_cell[0] = pgain;
      sm_p->shk_gain_alp_cell[1] = igain;
      sm_p->shk_gain_alp_cell[2] = dgain;
      printf("CMD: SHK-->ALP Cell Gain: %10.6f | %10.6f | %10.6f\n", sm_p->shk_gain_alp_cell[0],sm_p->shk_gain_alp_cell[1],sm_p->shk_gain_alp_cell[2]);
    }else printf("CMD: Gain out of bounds: [-1,0]\n");
    return CMD_NORMAL;
  }

  //Scale SHK ALP Zernike Gain
  sprintf(cmd,"shk alp scale zgain");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= 0){
      for(i=0;i<LOWFS_N_ZERNIKE;i++){ 
	for(j=0;j<LOWFS_N_PID;j++){
	  sm_p->shk_gain_alp_zern[i][j] = shk_gain_alp_zern[i][j]*ftemp;
	}
      }
      printf("CMD: Changing SHK-->ALP Zernike gain multiplier to %f\n",ftemp);
      for(i=0;i<LOWFS_N_ZERNIKE;i++) 
	printf("CMD: SHK-->ALP Z[%2.2d] Gain: %10.6f | %10.6f | %10.6f\n", i, sm_p->shk_gain_alp_zern[i][0],sm_p->shk_gain_alp_zern[i][1],sm_p->shk_gain_alp_zern[i][2]);
    }else printf("CMD: Gain multiplier must be >= 0\n");
    return CMD_NORMAL;
  }
  
  //User input SHK ALP Zernike Gain
  sprintf(cmd,"shk alp zgain");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    pch  = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: SHK gain bad format: Z P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    itemp = atoi(pch);
    pch  = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: SHK gain bad format: Z P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    pgain = atof(pch);
    pch  = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: SHK gain bad format: Z P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    igain = atof(pch);
    pch  = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: SHK gain bad format: Z P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    dgain = atof(pch);
    if(itemp >=0 && itemp < LOWFS_N_ZERNIKE && pgain <= 0 && pgain >= -1 && igain <= 0 && igain >= -1 && dgain <= 0 && dgain >= -1){
      sm_p->shk_gain_alp_zern[itemp][0] = pgain;
      sm_p->shk_gain_alp_zern[itemp][1] = igain;
      sm_p->shk_gain_alp_zern[itemp][2] = dgain;
      for(i=0;i<LOWFS_N_ZERNIKE;i++) 
	printf("CMD: SHK-->ALP Z[%2.2d] Gain: %10.6f | %10.6f | %10.6f\n", i, sm_p->shk_gain_alp_zern[i][0],sm_p->shk_gain_alp_zern[i][1],sm_p->shk_gain_alp_zern[i][2]);
    }else printf("CMD: Bad Zernike index [0,%d] or gain [-1,0]\n",LOWFS_N_ZERNIKE-1);
    return CMD_NORMAL;
  }

  //Scale SHK HEX Zernike Gain
  sprintf(cmd,"shk hex scale zgain");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= 0){
      double shk_gain_hex_zern[LOWFS_N_PID] = SHK_GAIN_HEX_ZERN_DEFAULT;
      for(i=0;i<LOWFS_N_PID;i++)
	sm_p->shk_gain_hex_zern[i] = shk_gain_hex_zern[i]*ftemp;
      printf("CMD: Changing SHK-->HEX Zernike gain multiplier to %f\n",ftemp);
      printf("CMD: SHK-->HEX Gain: %10.6f | %10.6f | %10.6f\n", sm_p->shk_gain_hex_zern[0],sm_p->shk_gain_hex_zern[1],sm_p->shk_gain_hex_zern[2]);
    }else printf("CMD: Gain multiplier must be >= 0\n");
    return CMD_NORMAL;
  }

  //User Input SHK HEX Zernike Gain
  sprintf(cmd,"shk hex zgain");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    pch  = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: SHK gain bad format: P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    pgain = atof(pch);
    pch  = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: SHK gain bad format: P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    igain = atof(pch);
    pch  = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: SHK gain bad format: P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    dgain = atof(pch);
    if(pgain <= 0 && pgain >= -1 && igain <= 0 && igain >= -1 && dgain <= 0 && dgain >= -1){
      sm_p->shk_gain_hex_zern[0] = pgain;
      sm_p->shk_gain_hex_zern[1] = igain;
      sm_p->shk_gain_hex_zern[2] = dgain;
      printf("CMD: SHK-->HEX Gain: %10.6f | %10.6f | %10.6f\n", sm_p->shk_gain_hex_zern[0],sm_p->shk_gain_hex_zern[1],sm_p->shk_gain_hex_zern[2]);
    }else printf("CMD: Gain out of bounds: [-1,0]\n");
    return CMD_NORMAL;
  }

  //LYT ALP Gain
  sprintf(cmd,"lyt alp scale zgain");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= 0){
      // -- zernike gain
      for(i=0;i<LOWFS_N_ZERNIKE;i++){ 
	for(j=0;j<LOWFS_N_PID;j++){
	  sm_p->lyt_gain_alp_zern[i][j] = lyt_gain_alp_zern[i][j]*ftemp;
	}
      }
      printf("CMD: Changing LYT-->ALP gain multiplier to %f\n",ftemp);
      for(i=0;i<LOWFS_N_ZERNIKE;i++) 
	printf("CMD: LYT-->ALP Z[%2.2d] Gain: %10.6f | %10.6f | %10.6f\n", i, sm_p->lyt_gain_alp_zern[i][0],sm_p->lyt_gain_alp_zern[i][1],sm_p->lyt_gain_alp_zern[i][2]);
    }else printf("CMD: Gain multiplier must be >= 0\n");
    return CMD_NORMAL;
  }
  
  //User input LYT ALP Zernike Gain
  sprintf(cmd,"lyt alp zgain");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    pch  = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: LYT gain bad format: Z P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    itemp = atoi(pch);
    pch  = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: LYT gain bad format: Z P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    pgain = atof(pch);
    pch  = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: LYT gain bad format: Z P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    igain = atof(pch);
    pch  = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: LYT gain bad format: Z P.P I.I D.D\n");
      return CMD_NORMAL;
    }
    dgain = atof(pch);
    if(itemp >=0 && itemp < LOWFS_N_ZERNIKE && pgain <= 0 && pgain >= -1 && igain <= 0 && igain >= -1 && dgain <= 0 && dgain >= -1){
      sm_p->lyt_gain_alp_zern[itemp][0] = pgain;
      sm_p->lyt_gain_alp_zern[itemp][1] = igain;
      sm_p->lyt_gain_alp_zern[itemp][2] = dgain;
      for(i=0;i<LOWFS_N_ZERNIKE;i++) 
	printf("CMD: LYT-->ALP Z[%2.2d] Gain: %10.6f | %10.6f | %10.6f\n", i, sm_p->lyt_gain_alp_zern[i][0],sm_p->lyt_gain_alp_zern[i][1],sm_p->lyt_gain_alp_zern[i][2]);
    }else printf("CMD: Bad Zernike index [0,%d] or gain [-1,0]\n",LOWFS_N_ZERNIKE-1);
    return CMD_NORMAL;
  }

  /****************************************
   * SCI CAMERA SETTINGS
   **************************************/

  //SCI Origin Commands
  sprintf(cmd,"sci set origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->sci_fastmode){
      printf("CMD: %s disabled in SCI fast mode\n",cmd);
      return(CMD_NORMAL);
    }
    printf("CMD: Setting SCI origin\n");
    sm_p->sci_setorigin=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci find origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->sci_fastmode){
      printf("CMD: %s disabled in SCI fast mode\n",cmd);
      return(CMD_NORMAL);
    }
    printf("CMD: Finding SCI origin\n");
    sm_p->sci_findorigin=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci track origin on");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->sci_fastmode){
      printf("CMD: %s disabled in SCI fast mode\n",cmd);
      return(CMD_NORMAL);
    }
    printf("CMD: Turning SCI origin tracking ON\n");
    sm_p->sci_trackorigin=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci track origin off");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->sci_fastmode){
      printf("CMD: %s disabled in SCI fast mode\n",cmd);
      return(CMD_NORMAL);
    }
    printf("CMD: Turning SCI origin tracking OFF\n");
    sm_p->sci_trackorigin=0;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci revert origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->sci_fastmode){
      printf("CMD: %s disabled in SCI fast mode\n",cmd);
      return(CMD_NORMAL);
    }
    printf("CMD: Reverting SCI origin\n");
    sm_p->sci_revertorigin=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci save origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->sci_fastmode){
      printf("CMD: %s disabled in SCI fast mode\n",cmd);
      return(CMD_NORMAL);
    }
    printf("CMD: Saving SCI origin\n");
    sm_p->sci_saveorigin=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci load origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->sci_fastmode){
      printf("CMD: %s disabled in SCI fast mode\n",cmd);
      return(CMD_NORMAL);
    }
    printf("CMD: Loading SCI origin\n");
    sm_p->sci_loadorigin=1;
    return(CMD_NORMAL);
  }
  
  sprintf(cmd,"sci shift origin");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    pch  = strtok(line+strlen(cmd)," ");
    if(pch == NULL){
      printf("CMD: SCI origin bad format: BAND DIMENSION NPIX\n");
      return CMD_NORMAL;
    }
    iband = atoi(pch);
    if(iband < 0 || iband >= SCI_NBANDS){
      printf("CMD: SCI origin band %d invalid, must be [0-%d]\n",iband,SCI_NBANDS-1);
      return CMD_NORMAL;
    }
    pch  = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: SCI origin bad format: BAND DIMENSION NPIX\n");
      return CMD_NORMAL;
    }
    strncpy(dim,pch,1);
    pch  = strtok(NULL," ");
    if(pch == NULL){
      printf("CMD: SCI origin bad format: BAND DIMENSION NPIX\n");
      return CMD_NORMAL;
    }
    npix = atoi(pch);
    if(!strncasecmp(dim,"x",1)){
      itemp = sm_p->sci_xorigin[iband] + npix;
      if(itemp < SCI_XORIGIN_MIN || itemp > SCI_XORIGIN_MAX){
	printf("CMD: SCI X origin %d out of range [%d,%d]\n",itemp,SCI_XORIGIN_MIN,SCI_XORIGIN_MAX);
	return CMD_NORMAL;
      }
      sm_p->sci_xorigin[iband] = itemp;
      printf("CMD: SCI shifted X origin %d px to %d\n",npix,sm_p->sci_xorigin[iband]);
      if(sm_p->sci_fastmode) sm_p->sci_reset_camera=1;
      return CMD_NORMAL;
    }
    if(!strncasecmp(dim,"y",1)){
      itemp = sm_p->sci_yorigin[iband] + npix;
      if(itemp < SCI_YORIGIN_MIN || itemp > SCI_YORIGIN_MAX){
	printf("CMD: SCI Y origin %d out of range [%d,%d]\n",itemp,SCI_YORIGIN_MIN,SCI_YORIGIN_MAX);
	return CMD_NORMAL;
      }
      sm_p->sci_yorigin[iband] = itemp;
      printf("CMD: SCI shifted Y origin %d px to %d\n",npix,sm_p->sci_yorigin[iband]);
      if(sm_p->sci_fastmode) sm_p->sci_reset_camera=1;
      return CMD_NORMAL;
    }
    printf("CMD: SCI origin bad format: BAND DIMENSION NPIX\n");
    return CMD_NORMAL;
  }

  //SCI Phase Flattening
  sprintf(cmd,"sci phasemode");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    cmdfound = 0;
    for(i=0;i<SCI_NPHASEMODES;i++){
      if(!strncasecmp(line+strlen(cmd)+1,sciphasemodes[i].cmd,strlen(sciphasemodes[i].cmd))){
	sm_p->sci_phasemode=i;
	printf("CMD: Changed SCI phase mode to %s\n",sciphasemodes[i].name);
	cmdfound = 1;
      }
    }
    if(!cmdfound)
      print_sci_phasemodes(sciphasemodes,sm_p->sci_phasemode);
    
    return(CMD_NORMAL);
  }
  
  sprintf(cmd,"sci phase nzern");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    itemp = atoi(line+strlen(cmd)+1);
    if(itemp >= 0 && itemp <= LOWFS_N_ZERNIKE){
      sm_p->sci_phase_n_zernike = itemp;
      sm_p->sci_reset_camera=1;
      printf("CMD: Changed SCI phase N zernike to %d\n",sm_p->sci_phase_n_zernike);
    }else{
      printf("CMD: SCI phase N zernike out of bounds [%d,%d]\n",0,LOWFS_N_ZERNIKE);
    }
    return(CMD_NORMAL);
  }
  sprintf(cmd,"sci phase expscale");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp > 0){
      sm_p->sci_phase_expscale = ftemp;
      printf("CMD: Changed SCI phase expscale to %f\n",sm_p->sci_phase_expscale);
    }else{
      printf("CMD: SCI phase expscale must be greater than 0\n");
    }
    return(CMD_NORMAL);
  }
  sprintf(cmd,"sci optmode");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    cmdfound = 0;
    for(i=0;i<SCI_NOPTMODES;i++){
      if(!strncasecmp(line+strlen(cmd)+1,scioptmodes[i].cmd,strlen(scioptmodes[i].cmd))){
	sm_p->sci_optmode=i;
	printf("CMD: Changed SCI opt mode to %s\n",scioptmodes[i].name);
	cmdfound = 1;
      }
    }
    if(!cmdfound)
      print_sci_optmodes(scioptmodes,sm_p->sci_optmode);
    
    return(CMD_NORMAL);
  }
  sprintf(cmd,"sci phase testgrad on");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->sci_phase_testgrad = 1;
    printf("CMD: Turned SCI testgrad on\n");
    return(CMD_NORMAL);
  }
  sprintf(cmd,"sci phase testgrad off");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->sci_phase_testgrad = 0;
    printf("CMD: Turned SCI testgrad on\n");
    return(CMD_NORMAL);
  }

  //SCI Reference Image
  sprintf(cmd,"sci set ref");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Setting SCI reference image\n");
    sm_p->sci_setref=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci man ref");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    sm_p->sci_manref = ftemp;
    printf("CMD: Changed SCI manref to %f\n",sm_p->sci_manref);
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci ref scale");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    sm_p->sci_refscale = ftemp;
    printf("CMD: Changed SCI ref scale to %f\n",sm_p->sci_refscale);
    return(CMD_NORMAL);
  }

  //SCI Fast Mode
  sprintf(cmd,"sci fastmode on");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(SCI_NBANDS==1){
      printf("CMD: Turning SCI fast mode ON\n");
      sm_p->sci_fastmode=1;
      sm_p->sci_reset_camera=1;
    }else printf("CMD: SCI fast mode only possible when SCI_NBANDS==1\n");
    return(CMD_NORMAL);
  }
  sprintf(cmd,"sci fastmode off");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(SCI_NBANDS==1){
      printf("CMD: Turning SCI fast mode OFF\n");
      sm_p->sci_fastmode=0;
      sm_p->sci_reset_camera=1;
    }else printf("CMD: SCI fast mode only possible when SCI_NBANDS==1\n");
    return(CMD_NORMAL);
  }

  
  //TEC control
  sprintf(cmd,"sci tec enable");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Enabling SCI TEC\n");
    printf("CMD: -- WARNING -- Only operate TEC at vacuum\n");
    sm_p->sci_tec_enable=1;
    sm_p->sci_reset_camera=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci tec disable");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Disabling SCI TEC\n");
    sm_p->sci_tec_setpoint=SCI_TEC_SETPOINT_MAX;
    sm_p->sci_tec_enable=0;
    sm_p->sci_reset_camera=1;
    return(CMD_NORMAL);
  }
  
  sprintf(cmd,"sci tec setpoint");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->sci_tec_enable){
      itemp = atoi(line+strlen(cmd)+1);
      if(itemp >= SCI_TEC_SETPOINT_MIN && itemp <= SCI_TEC_SETPOINT_MAX){
	sm_p->sci_tec_setpoint = itemp;
	sm_p->sci_reset_camera=1;
	printf("CMD: Changed SCI TEC setpoint to %dC\n",sm_p->sci_tec_setpoint);
      }else{
	printf("CMD: SCI TEC setpoint out of bounds [%d,%d]\n",SCI_TEC_SETPOINT_MIN,SCI_TEC_SETPOINT_MAX);
      }
    }else{
      printf("CMD: SCI TEC disabled\n");
    }
    return(CMD_NORMAL);
  }
  
  /****************************************
   * ACQ CAMERA SETTINGS
   **************************************/
  sprintf(cmd,"acq thresh");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    itemp = atoi(line+strlen(cmd)+1);
    if(itemp >= 0 && itemp <= ACQ_THRESH_MAX){
      sm_p->acq_thresh = itemp;
      printf("CMD: Changed ACQ threshold to %d\n",sm_p->acq_thresh);
    }else{
      printf("CMD: ACQ threshold out of bounds [%d,%d]\n",0,ACQ_THRESH_MAX);
    }
    return(CMD_NORMAL);
  }

  /****************************************
   * DOOR CONTROL
   **************************************/
  
  //Open/Close/Stop Doors
  sprintf(cmd,"open door");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    itemp = atoi(line+strlen(cmd)+1);
    if(itemp >= 1 && itemp <= MTR_NDOORS){
      printf("CMD: Opening door %d\n",itemp);
      sm_p->open_door[itemp-1] = 1;
    }
    else{
      printf("CMD: Door index out of range: [1,%d]\n",MTR_NDOORS);
    }
    return CMD_NORMAL;
  }
  sprintf(cmd,"close door");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    itemp = atoi(line+strlen(cmd)+1);
    if(itemp >= 1 && itemp <= MTR_NDOORS){
      printf("CMD: Closing door %d\n",itemp);
      sm_p->close_door[itemp-1] = 1;
    }
    else{
      printf("CMD: Door index out of range: [1,%d]\n",MTR_NDOORS);
    }
    return CMD_NORMAL;
  }
  sprintf(cmd,"stop door");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    itemp = atoi(line+strlen(cmd)+1);
    if(itemp >= 1 && itemp <= MTR_NDOORS){
      printf("CMD: Stopping door %d\n",itemp);
      sm_p->stop_door[itemp-1] = 1;
    }
    else{
      printf("CMD: Door index out of range: [1,%d]\n",MTR_NDOORS);
    }
    return CMD_NORMAL;
  }
  
  /****************************************
   * HEATER CONTROL
   **************************************/

  //Heater Settings
  sprintf(cmd,"htr");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    //Print heater status
    sprintf(cmd,"htr status");
    if(!strncasecmp(line,cmd,strlen(cmd))){
      print_htr_status(sm_p);
      return CMD_NORMAL;
    }
    //Commands for All heaters
    sprintf(cmd,"htr all override");
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Enabling ALL heaters manual override\n");
      for(i=0;i<SSR_NCHAN;i++){
	sm_p->htr[i].override  = 1;
	sm_p->htr[i].power     = 0; //zero power to be safe
	sm_p->htr[i].overpower = 0; //zero power to be safe
      }
      print_htr_status(sm_p);
      return CMD_NORMAL;
    }
    sprintf(cmd,"htr all release");
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Disabling ALL heaters manual override\n");
      for(i=0;i<SSR_NCHAN;i++){
	sm_p->htr[i].override  = 0;
	sm_p->htr[i].power     = 0; //zero power to be safe
	sm_p->htr[i].overpower = 0; //zero power to be safe
      }
      print_htr_status(sm_p);
      return CMD_NORMAL;
    }
    sprintf(cmd,"htr all enable");
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Enabling ALL heaters\n");
      for(i=0;i<SSR_NCHAN;i++) sm_p->htr[i].enable = 1;
      print_htr_status(sm_p);
      return CMD_NORMAL;
    }
    sprintf(cmd,"htr all disable");
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Disabling ALL heaters\n");
      for(i=0;i<SSR_NCHAN;i++) sm_p->htr[i].enable = 0;
      print_htr_status(sm_p);
      return CMD_NORMAL;
    }
    sprintf(cmd,"htr all pid enable");
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Enabling PID control for ALL heaters\n");
      for(i=0;i<SSR_NCHAN;i++) sm_p->htr[i].usepid = 1;
      print_htr_status(sm_p);
      return CMD_NORMAL;
    }
    sprintf(cmd,"htr all pid disable");
    if(!strncasecmp(line,cmd,strlen(cmd))){
      printf("CMD: Disabling PID control for ALL heaters\n");
      for(i=0;i<SSR_NCHAN;i++) sm_p->htr[i].usepid = 0;
      print_htr_status(sm_p);
      return CMD_NORMAL;
    }
    //Commands for individual heaters
    for(i=0;i<SSR_NCHAN;i++){
      sprintf(cmd,"htr %d override",i);
      if(!strncasecmp(line,cmd,strlen(cmd))){
        printf("CMD: Enabling heater %d manual override\n",i);
	sm_p->htr[i].override  = 1;
	sm_p->htr[i].power     = 0; //zero power to be safe
	sm_p->htr[i].overpower = 0; //zero power to be safe
	print_htr_status(sm_p);
	return CMD_NORMAL;
      }
      sprintf(cmd,"htr %d release",i);
      if(!strncasecmp(line,cmd,strlen(cmd))){
        printf("CMD: Disabling heater %d manual override\n",i);
	sm_p->htr[i].override  = 0;
	sm_p->htr[i].power     = 0; //zero power to be safe
	sm_p->htr[i].overpower = 0; //zero power to be safe
	print_htr_status(sm_p);
	return CMD_NORMAL;
      }
      sprintf(cmd,"htr %d enable",i);
      if(!strncasecmp(line,cmd,strlen(cmd))){
        printf("CMD: Enabling heater %d\n",i);
	sm_p->htr[i].enable = 1;
	print_htr_status(sm_p);
	return CMD_NORMAL;
      }
      sprintf(cmd,"htr %d disable",i);
      if(!strncasecmp(line,cmd,strlen(cmd))){
        printf("CMD: Disabling heater %d\n",i);
	sm_p->htr[i].enable = 0;
	print_htr_status(sm_p);
	return CMD_NORMAL;
      }
      sprintf(cmd,"htr %d pid enable",i);
      if(!strncasecmp(line,cmd,strlen(cmd))){
        printf("CMD: Enabling PID control for heater %d\n",i);
	sm_p->htr[i].usepid = 1;
	print_htr_status(sm_p);
	return CMD_NORMAL;
      }
      sprintf(cmd,"htr %d pid disable",i);
      if(!strncasecmp(line,cmd,strlen(cmd))){
        printf("CMD: Disabling PID control for heater %d\n",i);
	sm_p->htr[i].usepid = 0;
	print_htr_status(sm_p);
	return CMD_NORMAL;
      }
      sprintf(cmd,"htr %d power",i);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	if(sm_p->htr[i].enable){
	  if(sm_p->htr[i].override){
	    itemp = atoi(line+strlen(cmd)+1);
	    if(itemp >= HTR_POWER_MIN && itemp <= sm_p->htr[i].maxpower){
	      sm_p->htr[i].overpower = itemp;
	      //we set the power as well so that the printout will show the correct value
	      //until it is updated by thm_proc
	      sm_p->htr[i].power = itemp;
	      printf("CMD: Setting heater %d power to %d percent\n",i,sm_p->htr[i].overpower);
	      print_htr_status(sm_p);
	      return CMD_NORMAL;
	    }
	    else{
	      printf("CMD: Heater power out of bounds [%d-%d]\n",HTR_POWER_MIN,sm_p->htr[i].maxpower);
	      return CMD_NORMAL;
	    }
	  }
	  else{
	    printf("CMD: Heater %d manual override is disabled\n",i);
	    return CMD_NORMAL;
	  }
	}
	else{
	  printf("CMD: Heater %d disabled\n",i);
	  return CMD_NORMAL;
	}
      }
      sprintf(cmd,"htr %d maxpower",i);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	itemp = atoi(line+strlen(cmd)+1);
	if(itemp >= HTR_POWER_MIN && itemp <= HTR_POWER_MAX){
	  sm_p->htr[i].maxpower = itemp;
	  printf("CMD: Setting heater %d max power to %d percent\n",i,sm_p->htr[i].maxpower);
	  print_htr_status(sm_p);
	  return CMD_NORMAL;
	}
	else{
	  printf("CMD: Heater max power out of bounds [%d-%d]\n",HTR_POWER_MIN,HTR_POWER_MAX);
	  return CMD_NORMAL;
	}
      }
      sprintf(cmd,"htr %d setpoint",i);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	ftemp = atof(line+strlen(cmd)+1);
	if(ftemp >= HTR_SETPOINT_MIN && ftemp <= HTR_SETPOINT_MAX){
	  sm_p->htr[i].setpoint = ftemp;
	  printf("CMD: Setting heater %d setpoint to %fC\n",i,sm_p->htr[i].setpoint);
	  print_htr_status(sm_p);
	  return CMD_NORMAL;
	}
	else{
	  printf("CMD: Heater setpoint out of bounds [%d-%d]\n",HTR_SETPOINT_MIN,HTR_SETPOINT_MAX);
	  return CMD_NORMAL;
	}
      }
      sprintf(cmd,"htr %d deadband",i);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	ftemp = atof(line+strlen(cmd)+1);
	if(ftemp >= HTR_DEADBAND_MIN && ftemp <= HTR_DEADBAND_MAX){
	  sm_p->htr[i].deadband = ftemp;
	  printf("CMD: Setting heater %d deadband to %fC\n",i,sm_p->htr[i].deadband);
	  print_htr_status(sm_p);
	  return CMD_NORMAL;
	}
	else{
	  printf("CMD: Heater deadband out of bounds [%d-%d]\n",HTR_DEADBAND_MIN,HTR_DEADBAND_MAX);
	  return CMD_NORMAL;
	}
      }
      sprintf(cmd,"htr %d gain",i);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	ftemp = atof(line+strlen(cmd)+1);
	if(ftemp >= HTR_GAIN_MIN && ftemp <= HTR_GAIN_MAX){
	  sm_p->htr[i].gain = ftemp;
	  printf("CMD: Setting heater %d gain to %f\n",i,sm_p->htr[i].gain);
	  print_htr_status(sm_p);
	  return CMD_NORMAL;
	}
	else{
	  printf("CMD: Heater gain out of bounds [%d-%d]\n",HTR_GAIN_MIN,HTR_GAIN_MAX);
	  return CMD_NORMAL;
	}
      }
      sprintf(cmd,"htr %d intmax",i);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	ftemp = atof(line+strlen(cmd)+1);
	sm_p->htr[i].intmax = ftemp;
	printf("CMD: Setting heater %d intmax to %f\n",i,sm_p->htr[i].intmax);
	print_htr_status(sm_p);
	return CMD_NORMAL;
      }
      sprintf(cmd,"htr %d pid gain",i);
      if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
	pch  = strtok(line+strlen(cmd)," ");
	if(pch == NULL){
	  printf("CMD: HTR gain bad format: P.P I.I D.D\n");
	  return CMD_NORMAL;
	}
	pgain = atof(pch);
	pch  = strtok(NULL," ");
	if(pch == NULL){
	  printf("CMD: HTR gain bad format: P.P I.I D.D\n");
	  return CMD_NORMAL;
	}
	igain = atof(pch);
	pch  = strtok(NULL," ");
	if(pch == NULL){
	  printf("CMD: HTR gain bad format: P.P I.I D.D\n");
	  return CMD_NORMAL;
	}
	dgain = atof(pch);
	if(pgain <= HTR_GAIN_MAX && pgain >= HTR_GAIN_MIN && igain <= HTR_GAIN_MAX && igain >= HTR_GAIN_MIN && dgain <= HTR_GAIN_MAX && dgain >= HTR_GAIN_MIN){
	  sm_p->htr[i].kP = pgain;
	  sm_p->htr[i].kI = igain;
	  sm_p->htr[i].kD = dgain;
	  printf("CMD: Setting heater %d PID gain to %f %f %f\n",i,sm_p->htr[i].kP,sm_p->htr[i].kI,sm_p->htr[i].kD);
	  print_htr_status(sm_p);
	  return CMD_NORMAL;
	}
	else{
	  printf("CMD: Heater PID gain out of bounds [%d-%d]\n",HTR_GAIN_MIN,HTR_GAIN_MAX);
	  return CMD_NORMAL;
	}
      }
      sprintf(cmd,"htr %d sensor",i);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	pch  = strtok(line+strlen(cmd)," ");
	if(pch == NULL){
	  printf("CMD: Heater sensor command bad format\n");
	  return CMD_NORMAL;
	}
	iadc = atoi(pch);
	pch  = strtok(NULL," ");
	if(pch == NULL){
	  printf("CMD: Heater sensor command bad format\n");
	  return CMD_NORMAL;
	}
	ich  = atoi(pch);
	if(iadc < HTR_ADC_MIN || iadc > HTR_ADC_MAX){
	  printf("CMD: Heater ADC out of bounds [%d-%d]\n",HTR_ADC_MIN,HTR_ADC_MAX);
	  return CMD_NORMAL;
	}
	if(iadc == 1){
	  if(ich >= 0 && iadc < ADC1_NCHAN){
	    sm_p->htr[i].adc = iadc;
	    sm_p->htr[i].ch  = ich;
	    printf("CMD: Setting heater %d sensor to ADC %d CH %d\n",i,sm_p->htr[i].adc,sm_p->htr[i].ch);
	    print_htr_status(sm_p);
	    return CMD_NORMAL;
	  }
	  else{
	    printf("CMD: Heater CH out of bounds [%d-%d]\n",0,ADC1_NCHAN-1);
	    return CMD_NORMAL;
	  }
	}
	if(iadc == 2){
	  if(ich >= 0 && iadc < ADC2_NCHAN){
	    sm_p->htr[i].adc = iadc;
	    sm_p->htr[i].ch  = ich;
	    printf("CMD: Setting heater %d sensor to ADC %d CH %d\n",i,sm_p->htr[i].adc,sm_p->htr[i].ch);
	    print_htr_status(sm_p);
	    return CMD_NORMAL;
	  }
	  else{
	    printf("CMD: Heater CH out of bounds [%d-%d]\n",0,ADC2_NCHAN-1);
	    return CMD_NORMAL;
	  }
	}
	if(iadc == 3){
	  if(ich >= 0 && iadc < ADC3_NCHAN){
	    sm_p->htr[i].adc = iadc;
	    sm_p->htr[i].ch  = ich;
	    printf("CMD: Setting heater %d sensor to ADC %d CH %d\n",i,sm_p->htr[i].adc,sm_p->htr[i].ch);
	    print_htr_status(sm_p);
	    return CMD_NORMAL;
	  }
	  else{
	    printf("CMD: Heater CH out of bounds [%d-%d]\n",0,ADC3_NCHAN-1);
	    return CMD_NORMAL;
	  }
	}
      }
    }
    //No heater command found
    printf("CMD: Bad heater command format\n");
    return CMD_NORMAL;
  }

  /****************************************
   * THERMAL CONTROL 
   **************************************/
  sprintf(cmd,"thm vref enable");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Enabling THM VREF calibration\n");
    sm_p->thm_enable_vref = 1;
    return(CMD_NORMAL);
  }
  sprintf(cmd,"thm vref disable");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Disabling THM VREF calibration\n");
    sm_p->thm_enable_vref = 0;
    return(CMD_NORMAL);
  }


  /****************************************
   * LED CONTROL
   ***************************************/
  
  //LED commands
  sprintf(cmd,"led on");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(LED_ENABLE){
      ftemp = atof(line+strlen(cmd)+1);
      if(ftemp >= 0 && ftemp <= 5){
	printf("CMD: Turning LED ON %f volts \n",ftemp);
	led = (ftemp/5.0) * 0x0FFF;
 	outb((led & 0x00FF),ADC1_BASE+4);
	outb(((led & 0xFF00) >> 8),ADC1_BASE+5);
      }else{
	printf("CMD: LED voltage range = [0,5]\n");
      }
    }else{
      printf("CMD: LED Disabled\n");
    }
    return CMD_NORMAL;
  }
  sprintf(cmd,"led off");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(LED_ENABLE){
      printf("CMD: Turning LED OFF\n");
      outb(0x00,ADC1_BASE+4);
      outb(0x00,ADC1_BASE+5);
    }else{
      printf("CMD: LED Disabled\n");
    }
    return CMD_NORMAL;
  }
  
  /****************************************
   * BLANK COMMAND
   ***************************************/
  if(strlen(line) == 1)
    return CMD_NORMAL;

  //return with command not found
  return(CMD_NOT_FOUND);
}
