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

/**************************************************************/
/* PRINT_PROCSTATUS                                           */
/*  - Print out current process status                        */
/**************************************************************/
void print_procstatus(sm_t *sm_p){
  int i;

  printf("************ Process Status ************\n");
  printf("Process      Running\n");
  for(i=0;i<NCLIENTS;i++)
    if(sm_p->w[i].run)
      printf("%s          YES\n",sm_p->w[i].name);
    else
      printf("%s          NO\n",sm_p->w[i].name);
  printf("******************************************\n");

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
      printf("%02d   %-6s    *%s\n",i,state[i].cmd,state[i].name);
    else
      printf("%02d   %-6s     %s\n",i,state[i].cmd,state[i].name);
  }
  printf("******************************************\n");

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
      printf("%02d   %-6s    *%s\n",i,alp[i].cmd,alp[i].name);
    else
      printf("%02d   %-6s     %s\n",i,alp[i].cmd,alp[i].name);
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
      printf("%02d   %-6s    *%s\n",i,hex[i].cmd,hex[i].name);
    else
      printf("%02d   %-6s     %s\n",i,hex[i].cmd,hex[i].name);
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
      printf("%02d   %-6s    *%s\n",i,bmc[i].cmd,bmc[i].name);
    else
      printf("%02d   %-6s     %s\n",i,bmc[i].cmd,bmc[i].name);
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
      printf("%02d   %-6s    *%s\n",i,tgt[i].cmd,tgt[i].name);
    else
      printf("%02d   %-6s     %s\n",i,tgt[i].cmd,tgt[i].name);
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
/* PRINT_HTR_STATUS                                           */
/*  - Print out current heater status                         */
/**************************************************************/
void print_htr_status(sm_t *sm_p){
  int i;

  printf("******************************** Heater Status *********************************\n");
  printf("%3s%7s%7s%7s%7s%7s%7s%7s%7s%7s%7s\n","#","Name","Enable","Over","Power","Max","ADC","CH","Temp","SetP","DeadB");
  for(i=0;i<SSR_NCHAN;i++)
    printf("%3d%7s%7d%7d%7d%7d%7d%7d%7.2f%7.2f%7.2f\n",i,sm_p->htr[i].name,sm_p->htr[i].enable,sm_p->htr[i].override,sm_p->htr[i].power,sm_p->htr[i].maxpower,sm_p->htr[i].adc,sm_p->htr[i].ch,sm_p->htr[i].temp,sm_p->htr[i].setpoint,sm_p->htr[i].deadband);
  printf("********************************************************************************\n");

}

/**************************************************************/
/* HANDLE_COMMAND                                             */
/*  - Handle user commands                                    */
/**************************************************************/
int handle_command(char *line, sm_t *sm_p){
  double ftemp;
  int    itemp,ich,iadc;
  char   stemp[CMD_MAX_LENGTH];
  char   cmd[CMD_MAX_LENGTH];
  int    cmdfound=0;
  int    i=0,j=0,hex_axis=0;
  int hexfd;
  double hex_poke=0;
  int    calmode=0;
  uint16_t led;
  static double trl_poke = HEX_TRL_POKE;//0.01;
  static double rot_poke = HEX_ROT_POKE;///0.01;
  static calmode_t alpcalmodes[ALP_NCALMODES];
  static calmode_t hexcalmodes[HEX_NCALMODES];
  static calmode_t bmccalmodes[BMC_NCALMODES];
  static calmode_t tgtcalmodes[TGT_NCALMODES];
  static calmode_t fakemodes[NFAKEMODES];
  static int init=0;
  const char hex_str_axes[HEX_NAXES][5] = {"X","Y","Z","U","V","W"};
  const char hex_str_unit[HEX_NAXES][5] = {"mm","mm","mm","deg","deg","deg"};
  const double hexhome[HEX_NAXES] = HEX_POS_HOME;
  const double hexdef[HEX_NAXES]  = HEX_POS_DEFAULT;
  hex_t hexcmd;
  memset(&hexcmd,0,sizeof(hex_t));
  char *pch;
  alp_t alp;
  double dz[LOWFS_N_ZERNIKE] = {0};
  double da[ALP_NACT] = {0};
    
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
    printf("Flight data erased\n");
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
    printf("Calibration data erased\n");
    return(CMD_NORMAL);
  }

  //Sleep
  sprintf(cmd,"sleep");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    itemp = atoi(line+strlen(cmd)+1);
    if(itemp >= 1 && itemp <= 5){
      printf("CMD: Input console sleeping for %d seconds\n",itemp);
      sleep(itemp);
    }
    else
      printf("CMD: sleep range = [1,5] seconds\n");
    return(CMD_NORMAL);
  }
  

  /****************************************
   * PROCESS CONTROL
   ***************************************/

  //Process ON/OFF/RESTART
  for(i=0;i<NCLIENTS;i++){
    if(i != WATID){
      //ON command
      sprintf(cmd,"%s_proc on",sm_p->w[i].name);
      for(j=0;j<strlen(cmd);j++) cmd[j]=tolower(cmd[j]);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	printf("CMD: Turning %s ON\n",sm_p->w[i].name);
	sm_p->w[i].run = 1;
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
    print_procstatus(sm_p);
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
   * CALIBRATION MODES
   ***************************************/

  //ALP timer length
  sprintf(cmd,"alp timer length");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp > 0 && ftemp <= CALMODE_TIMER_MAX){
      sm_p->alpcal.timer_length = ftemp;
      printf("CMD: Changed ALP calibration timer length to %.1f seconds\n",sm_p->alpcal.timer_length);
    }else{
      printf("CMD: ALP calibration timer out of bounds [%d,%d]\n",0,CALMODE_TIMER_MAX);
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
    if(!cmdfound) print_states(sm_p->state_array,sm_p->state);
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
    printf("  -- Recording data to: %s\n",sm_p->calfile);
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
    printf("  -- Recording data to: %s\n",sm_p->calfile);
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
    printf("  -- Recording data to: %s\n",sm_p->calfile);
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
	  if(hex_init(&hexfd)){
	    printf("CMD: ERROR: HEX init failed!\n");
	  }
	  else{
	    sm_p->hexfd = hexfd;
	    sm_p->hex_ready = 1;
	    printf("CMD: HEX ready\n");
	  }
	}
	else{
	  printf("CMD: HEX disabled\n");
	}
	return(CMD_NORMAL);
      }

      //Turn ON tilt correction
      sprintf(cmd,"hex tcor on");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	printf("CMD: Turning HEX tilt correction ON\n");
	sm_p->hex_tilt_correct = 1;
 	return(CMD_NORMAL);
      }
      //Turn OFF tilt correction
      sprintf(cmd,"hex tcor off");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	printf("CMD: Turning HEX tilt correction OFF\n");
	sm_p->hex_tilt_correct = 0;
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
      //Go to HOME position
      sprintf(cmd,"hex gohome");
      if(!strncasecmp(line,cmd,strlen(cmd))){
	if(!sm_p->hex_ready){
	  printf("CMD: HEX not ready\n");
	  return(CMD_NORMAL);
	}
	printf("CMD: Moving hexapod to home positon\n");
	memcpy(hexcmd.acmd,hexhome,sizeof(hexhome));
	if(!hex_send_command(sm_p,&hexcmd,WATID))
	  printf("CMD: Hexapod command failed\n");
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
	if(!hex_send_command(sm_p,&hexcmd,WATID))
	  printf("CMD: Hexapod command failed\n");
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
	printf("CMD: Bad hex move command\n");
	return(CMD_NORMAL);
      move_hex:
	if(!sm_p->hex_ready){
	  printf("CMD: HEX not ready\n");
	  return(CMD_NORMAL);
	}
	printf("CMD: Moving hexapod axis %s by %f %s\n",hex_str_axes[hex_axis],hex_poke,hex_str_unit[hex_axis]);
	hex_get_command(sm_p,&hexcmd);
	hexcmd.acmd[hex_axis] += hex_poke;
	if(!hex_send_command(sm_p,&hexcmd,WATID))
	  printf("CMD: Hexapod command failed\n");
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
   * ALPAO DM CONTROL
   ***************************************/

  //Set all ALP actuators to the same value
  sprintf(cmd,"alp bias");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      ftemp = atof(line+strlen(cmd)+1);
      if((ftemp >= ALP_MIN_BIAS) && (ftemp <= ALP_MAX_BIAS)){
	printf("CMD: Setting ALP bias = %f\n",ftemp);
	if(sm_p->alp_ready)
	  if(alp_set_bias(sm_p,ftemp,WATID)==0)
	    printf("CMD: ERROR: alp_set_random failed!\n");
      }
      else
	printf("CMD: ALP bias must be between %f and %f \n",ALP_MIN_BIAS,ALP_MAX_BIAS);
    }
    else
      printf("CMD: Manual ALPAO DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Set ALP to all ZEROS
  sprintf(cmd,"alp zero flat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Setting ALP to all zeros\n");
      if(sm_p->alp_ready)
	if(alp_zero_flat(sm_p,WATID)==0)
	  printf("CMD: ERROR: alp_revert_flat failed!\n");
    }
    else
      printf("CMD: Manual ALPAO DM control disabled in this state\n");
    return(CMD_NORMAL);
  }
  
  //Set ALP to #defined flat
  sprintf(cmd,"alp revert flat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Reverting ALP flat to default\n");
      if(sm_p->alp_ready)
	if(alp_revert_flat(sm_p,WATID)==0)
	  printf("CMD: ERROR: alp_revert_flat failed!\n");
    }
    else
      printf("CMD: Manual ALPAO DM control disabled in this state\n");
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
      printf("CMD: Manual ALPAO DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Load ALP command from disk
  sprintf(cmd,"alp load flat");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Loading ALP flat from file\n");
      if(sm_p->alp_ready)
	if(alp_load_flat(sm_p,WATID)==0)
	  printf("CMD: ERROR: alp_load_flat failed!\n");
    }
    else
      printf("CMD: Manual ALPAO DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Perturb ALP command with random actuator values
  sprintf(cmd,"alp random");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Sending random actuator pattern to ALP DM\n");
      if(sm_p->alp_ready)
	if(alp_set_random(sm_p,WATID)==0)
	  printf("CMD: ERROR: alp_set_random failed!\n");
    }
    else
      printf("CMD: Manual ALPAO DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Perturb ALP command with random zernikes
  sprintf(cmd,"alp zrandom");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Sending random Zernike pattern to ALP DM\n");
      if(sm_p->alp_ready)
	if(alp_set_zrandom(sm_p,WATID)==0)
	  printf("CMD: ERROR: alp_set_zrandom failed!\n");
    }
    else
      printf("CMD: Manual ALPAO DM control disabled in this state\n");
    return(CMD_NORMAL);
  }

  //Reset user ALP zernike matrix
  sprintf(cmd,"alp reset");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    alp_zern2alp(NULL,NULL,FUNCTION_RESET);
    return(CMD_NORMAL);
  }

  //Add a zernike to the current ALP command
  sprintf(cmd,"alp zernike");
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
	alp_get_command(sm_p,&alp);

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
	  printf("CMD: Changed Z[%d] by %f microns\n",itemp,ftemp);
	  return CMD_NORMAL;
	}
	else{
	  printf("CMD: alp_send_command failed\n");
	  return CMD_NORMAL;
	}	  
      }
      else{
	printf("CMD: Zernike cmd out of bounds\n");
	return CMD_NORMAL;
      }
    }
    else
      printf("CMD: Manual ALPAO DM control disabled in this state\n");
    return CMD_NORMAL;
  }

  //Add a single actuator value to the current command
  sprintf(cmd,"alp actuator");
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
	alp_get_command(sm_p,&alp);

	//Add to current command
	alp.acmd[itemp] += ftemp;

	//Send command
	if(alp_send_command(sm_p,&alp,WATID,1)){
	  printf("CMD: Changed A[%d] by %f microns\n",itemp,ftemp);
	  return CMD_NORMAL;
	}
	else{
	  printf("CMD: alp_send_command failed\n");
	  return CMD_NORMAL;
	}	  
      }
      else{
	printf("CMD: Zernike cmd out of bounds\n");
	return CMD_NORMAL;
      }
    }
    else
      printf("CMD: Manual ALPAO DM control disabled in this state\n");
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
      printf("  -- Starting data recording to file: %s\n",sm_p->calfile);
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
      printf("  -- Starting data recording to file: %s\n",sm_p->calfile);
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
      printf("  -- Starting data recording to file: %s\n",sm_p->calfile);
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
      printf("  -- Starting data recording to file: %s\n",sm_p->calfile);
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
      printf("  -- Starting data recording to file: %s\n",sm_p->calfile);
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
      printf("  -- Starting data recording to file: %s\n",sm_p->calfile);
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
  
  sprintf(cmd,"sci frmtime");
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

  /****************************************
   * ZERNIKE TARGETS
   **************************************/

  //SHK Zernike Targets
  sprintf(cmd,"shk target reset");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Setting SHK Zernike targets to zero\n");
    for(i=0;i<LOWFS_N_ZERNIKE;i++) sm_p->shk_zernike_target[i]=0;
    return CMD_NORMAL;
  }
  
  sprintf(cmd,"shk target");
  if(!strncasecmp(line,cmd,strlen(cmd))){
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
      printf("CMD: Setting SHK target Z[%d] = %f microns\n",itemp,ftemp);
    }
    else{
      printf("CMD: Zernike target out of bounds #[%d,%d] C[%f,%f]\n",0,LOWFS_N_ZERNIKE,ALP_ZERNIKE_MIN,ALP_ZERNIKE_MAX);
      return CMD_NORMAL;
    }
    return CMD_NORMAL;
  }
  
  //LYT Zernike Targets
  sprintf(cmd,"lyt target reset");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Setting LYT Zernike targets to zero\n");
    for(i=0;i<LOWFS_N_ZERNIKE;i++) sm_p->lyt_zernike_target[i]=0;
    return CMD_NORMAL;
  }

  sprintf(cmd,"lyt target");
  if(!strncasecmp(line,cmd,strlen(cmd))){
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
      printf("CMD: Setting LYT target Z[%d] = %f microns\n",itemp,ftemp);
    }
    else{
      printf("CMD: Zernike target out of bounds #[%d,%d] C[%f,%f]\n",0,LOWFS_N_ZERNIKE,ALP_ZERNIKE_MIN,ALP_ZERNIKE_MAX);
      return CMD_NORMAL;
    }
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
  
  /****************************************
   * GAIN SETTINGS
   **************************************/
  
  //SHK ALP Gain
  sprintf(cmd,"shk alp gain");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= 0){
      double shk_gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID] = SHK_GAIN_ALP_ZERN_DEFAULT;
      double shk_gain_alp_cell[LOWFS_N_PID] = SHK_GAIN_ALP_CELL_DEFAULT;
      for(i=0;i<LOWFS_N_PID;i++)
	sm_p->shk_gain_alp_cell[i] = shk_gain_alp_cell[i]*ftemp;
      for(i=0;i<LOWFS_N_ZERNIKE;i++) 
	for(j=0;j<LOWFS_N_PID;j++)
	  sm_p->shk_gain_alp_zern[i][j];
      printf("CMD: Changing SHK-->ALP gain multiplier to %f\n",ftemp);
      for(i=0;i<LOWFS_N_ZERNIKE;i++) 
	printf("CMD: SHK Z[%2.2d] Gain: %10.6f | %10.6f | %10.6f\n", i, sm_p->shk_gain_alp_zern[i][0],sm_p->shk_gain_alp_zern[i][1],sm_p->shk_gain_alp_zern[i][2]);
    }else printf("CMD: Gain multiplier must be >= 0\n");
    return CMD_NORMAL;
  }

  //SHK HEX Gain
  sprintf(cmd,"shk hex gain");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= 0){
      double shk_gain_hex_zern[LOWFS_N_PID] = SHK_GAIN_HEX_ZERN_DEFAULT;
      for(i=0;i<LOWFS_N_PID;i++)
	sm_p->shk_gain_hex_zern[i] = shk_gain_hex_zern[i]*ftemp;
      printf("CMD: Changing SHK-->HEX gain multiplier to %f\n",ftemp);
    }else printf("CMD: Gain multiplier must be >= 0\n");
    return CMD_NORMAL;
  }

  //LYT ALP Gain
  sprintf(cmd,"lyt alp gain");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    ftemp = atof(line+strlen(cmd)+1);
    if(ftemp >= 0){
      double lyt_gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID] = LYT_GAIN_ALP_ZERN_DEFAULT;
      // -- zernike gain
      for(i=0;i<LOWFS_N_ZERNIKE;i++) 
	for(j=0;j<LOWFS_N_PID;j++)
	  sm_p->lyt_gain_alp_zern[i][j] = ftemp * lyt_gain_alp_zern[i][j];
      printf("CMD: Changing LYT-->ALP gain multiplier to %f\n",ftemp);
      for(i=0;i<LOWFS_N_ZERNIKE;i++) 
	printf("CMD: LYT Z[%2.2d] Gain: %10.6f | %10.6f | %10.6f\n", i, sm_p->lyt_gain_alp_zern[i][0],sm_p->lyt_gain_alp_zern[i][1],sm_p->lyt_gain_alp_zern[i][2]);
    }else printf("CMD: Gain multiplier must be >= 0\n");
    return CMD_NORMAL;
  }
  
  /****************************************
   * SCI CAMERA SETTINGS
   **************************************/

  //SCI Origin Commands
  sprintf(cmd,"sci set origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Setting SCI origin\n");
    sm_p->sci_setorigin=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci find origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Finding SCI origin\n");
    sm_p->sci_findorigin=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci track origin on");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Turning SCI origin tracking ON\n");
    sm_p->sci_trackorigin=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci track origin off");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Turning SCI origin tracking OFF\n");
    sm_p->sci_trackorigin=0;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci revert origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Reverting SCI origin\n");
    sm_p->sci_revertorigin=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci save origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Saving SCI origin\n");
    sm_p->sci_saveorigin=1;
    return(CMD_NORMAL);
  }

  sprintf(cmd,"sci load origin");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Loading SCI origin\n");
    sm_p->sci_loadorigin=1;
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
    sm_p->sci_tec_enable=0;
    sm_p->sci_reset_camera=1;
    return(CMD_NORMAL);
  }
  
  sprintf(cmd,"sci tec setpoint");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->sci_tec_enable){
      itemp = atoi(line+strlen(cmd)+1);
      if(itemp > SCI_TEC_SETPOINT_MIN && itemp <= SCI_TEC_SETPOINT_MAX){
	sm_p->sci_tec_setpoint = itemp;
	sm_p->sci_reset_camera=1;
	printf("CMD: Changed SCI TEC setpoint to %d C\n",sm_p->sci_tec_setpoint);
      }else{
	printf("CMD: SCI TEC setpoint out of bounds [%d,%d]\n",SCI_TEC_SETPOINT_MIN,SCI_TEC_SETPOINT_MAX);
      }
    }else{
      printf("CMD: SCI TEC disabled\n");
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
	  printf("CMD: Setting heater %d setpoint to %f C\n",i,sm_p->htr[i].setpoint);
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
	  printf("CMD: Setting heater %d deadband to %f C\n",i,sm_p->htr[i].deadband);
	  print_htr_status(sm_p);
	  return CMD_NORMAL;
	}
	else{
	  printf("CMD: Heater deadband out of bounds [%d-%d]\n",HTR_DEADBAND_MIN,HTR_DEADBAND_MAX);
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
   * LED CONTROL
   ***************************************/
  
  //LED commands
  sprintf(cmd,"led on");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(LED_ENABLE){
      ftemp = atof(line+strlen(cmd)+1);
      if(ftemp >= 0 && ftemp <= 5){
	printf("CMD: Turning LED ON %f volts \n",ftemp);
	led = (ftemp/5.0) * 2048 + 2048;
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
      led = (0.0/5.0) * 2048 + 2048;
      outb((led & 0x00FF),ADC1_BASE+4);
      outb(((led & 0xFF00) >> 8),ADC1_BASE+5);
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
