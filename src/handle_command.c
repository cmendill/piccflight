#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>

/* piccflight headers */
#include "watchdog.h"
#include "handle_command.h"
#include "controller.h"
#include "hex_functions.h"
#include "alp_functions.h"
#include "common_functions.h"
#include "fakemodes.h"

/* Prototypes */
void getshk_proc(void); //get shkevents
void getlyt_proc(void); //get lytevents
void init_fakemode(int fakemode, calmode_t *fake);

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
void print_states(sm_t *sm_p){
  int i;
  printf("************ Available States ************\n");
  printf("#    Command    Name\n");
  for(i=0;i<NSTATES;i++)
    printf("%02d   %-6s     %s\n",i,sm_p->state_array[i].cmd,sm_p->state_array[i].name);
  printf("******************************************\n");

}

/**************************************************************/
/* PRINT_ALP_CALMODES                                         */
/*  - Print available ALP calmodes                          */
/**************************************************************/
void print_alp_calmodes(calmode_t *alp){
  int i;
  printf("************ Available ALP Calmodes  ************\n");
  printf("#    Command    Name\n");
  for(i=0;i<ALP_NCALMODES;i++)
    printf("%02d   %-6s     %s\n",i,alp[i].cmd,alp[i].name);
  printf("*************************************************\n");
}

/**************************************************************/
/* PRINT_HEX_CALMODES                                         */
/*  - Print available HEX calmodes                            */
/**************************************************************/
void print_hex_calmodes(calmode_t *hex){
  int i;
  printf("************ Available HEX Calmodes  ************\n");
  printf("#    Command    Name\n");
  for(i=0;i<HEX_NCALMODES;i++)
    printf("%02d   %-6s     %s\n",i,hex[i].cmd,hex[i].name);
  printf("*************************************************\n");
}

/**************************************************************/
/* PRINT_FAKEMODES                                            */
/*  - Print available fake data modes                         */
/**************************************************************/
void print_fakemodes(calmode_t *fake){
  int i;
  printf("******************* Available Fake Modes  *******************\n");
  printf("#    Command                Name\n");
  for(i=0;i<NFAKEMODES;i++)
    printf("%02d   %-18s     %s\n",i,fake[i].cmd,fake[i].name);
  printf("*************************************************************\n");
}

/**************************************************************/
/* PRINT_ZERNIKES                                             */
/*  - Print out zernike control status                        */
/**************************************************************/
void print_zernikes(sm_t *sm_p){
  int i;
  printf("************ Zernike Control ************\n");
  printf("Zernike   Controlled\n");
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    if(sm_p->zernike_control[i] == 1) printf("%02d        YES\n",i);        
    if(sm_p->zernike_control[i] == 0) printf("%02d        NO\n",i);
  }
  printf("******************************************\n");

}

/**************************************************************/
/* HANDLE_COMMAND                                             */
/*  - Handle user commands                                    */
/**************************************************************/
int handle_command(char *line, sm_t *sm_p){
  float ftemp;
  int   itemp;
  char  stemp[CMD_MAX_LENGTH];
  char  cmd[CMD_MAX_LENGTH];
  int   cmdfound=0;
  int   i=0,j=0,hex_axis=0;
  double hex_poke=0;
  int    calmode=0;
  static double trl_poke = HEX_TRL_POKE;//0.01;
  static double rot_poke = HEX_ROT_POKE;///0.01;
  static calmode_t alpcalmodes[ALP_NCALMODES];
  static calmode_t hexcalmodes[HEX_NCALMODES];
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
    
  /****************************************
   * INITIALIZE
   ***************************************/
  if(!init){
    //Init ALP calmodes
    for(i=0;i<ALP_NCALMODES;i++)
      alp_init_calmode(i,&alpcalmodes[i]);
    //Init HEX calmodes
    for(i=0;i<HEX_NCALMODES;i++)
      hex_init_calmode(i,&hexcalmodes[i]);
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
  
  //print packet info
  sprintf(cmd,"packet info");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: pktheadr = %lu bytes\n",sizeof(pkthed_t));
    printf("CMD: scievent = %lu bytes\n",sizeof(scievent_t));
    printf("CMD: shkevent = %lu bytes\n",sizeof(shkevent_t));
    printf("CMD: lytevent = %lu bytes\n",sizeof(lytevent_t));
    printf("CMD: acqevent = %lu bytes\n",sizeof(acqevent_t));
    return(CMD_NORMAL);
  }
  

  /****************************************
   * NORMAL COMMANDS
   ***************************************/
  //Process Control
  for(i=0;i<NCLIENTS;i++){
    if(i != WATID){
      //ON command
      sprintf(cmd,"%s_proc on",sm_p->w[i].name);
      for(j=0;j<strlen(cmd);j++) cmd[j]=tolower(cmd[j]);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	printf("CMD: Turning %s ON\n",sm_p->w[i].name);
	sm_p->w[i].run    = 1;
	return(CMD_NORMAL);
      }
      //OFF command
      sprintf(cmd,"%s_proc off",sm_p->w[i].name);
      for(j=0;j<strlen(cmd);j++) cmd[j]=tolower(cmd[j]);
      if(!strncasecmp(line,cmd,strlen(cmd))){
	printf("CMD: Turning %s OFF\n",sm_p->w[i].name);
	sm_p->w[i].run    = 0;
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
	  print_fakemodes(fakemodes);
	return(CMD_NORMAL);
      }
    }
  }
  
  
  //ALP Calmodes
  if(!strncasecmp(line,"alp calmode",11)){
    cmdfound = 0;
    for(i=0;i<ALP_NCALMODES;i++){
      if(!strncasecmp(line+12,alpcalmodes[i].cmd,strlen(alpcalmodes[i].cmd))){
	sm_p->alp_calmode=i;
	printf("CMD: Changed ALP calibration mode to %s\n",alpcalmodes[i].name);
	cmdfound = 1;
      }
    }
    if(!cmdfound)
      print_alp_calmodes(alpcalmodes);

    return(CMD_NORMAL);
  }

  //HEX Calmodes
  if(!strncasecmp(line,"hex calmode",11)){
    cmdfound = 0;
    for(i=0;i<HEX_NCALMODES;i++){
      if(!strncasecmp(line+12,hexcalmodes[i].cmd,strlen(hexcalmodes[i].cmd))){
	sm_p->hex_calmode=i;
	printf("CMD: Changed HEX calibration mode to %s\n",hexcalmodes[i].name);
	cmdfound = 1;
      }
    }
    if(!cmdfound)
      print_hex_calmodes(hexcalmodes);

    return(CMD_NORMAL);
  }


  //States
  if(!strncasecmp(line,"state",5)){
    cmdfound=0;
    for(i=0;i<NSTATES;i++){
      if(!strncasecmp(line+6,(char *)sm_p->state_array[i].cmd,strlen((char *)sm_p->state_array[i].cmd))){
	sm_p->state = i;
	printf("CMD: Changing state to %s\n",sm_p->state_array[i].name);
	cmdfound = 1;
      }
    }
    if(!cmdfound) print_states(sm_p);
    return(CMD_NORMAL);
  }


  //Start Manual SHK Data Recording
  if(!strncasecmp(line, "shk start rec", 13)){
    printf("CMD: Starting SHK data recording\n");
    //Setup filename
    sprintf((char *)sm_p->calfile,SHK_OUTFILE);
    //Start data recording
    printf("  -- Recording data to: %s\n",sm_p->calfile);
    sm_p->w[DIAID].launch = getshk_proc;
    sm_p->w[DIAID].run    = 1;
    return(CMD_NORMAL);
  }

  //Stop Manual SHK Data Recording
  if(!strncasecmp(line, "shk stop rec", 12)){
    printf("CMD: Stopping SHK data recording\n");
    //Stop data recording
    sm_p->w[DIAID].run    = 0;
    printf("  -- Done\n");
    return(CMD_NORMAL);
  }

  //Start Manual LYT Data Recording
  if(!strncasecmp(line, "lyt start rec", 13)){
    printf("CMD: Starting LYT data recording\n");
    //Setup filename
    sprintf((char *)sm_p->calfile,LYT_OUTFILE);
    //Start data recording
    printf("  -- Recording data to: %s\n",sm_p->calfile);
    sm_p->w[DIAID].launch = getlyt_proc;
    sm_p->w[DIAID].run    = 1;
    return(CMD_NORMAL);
  }

  //Stop Manual LYT Data Recording
  if(!strncasecmp(line, "lyt stop rec", 12)){
    printf("CMD: Stopping data recording\n");
    //Stop data recording
    sm_p->w[DIAID].run    = 0;
    printf("  -- Done\n");
    return(CMD_NORMAL);
  }

  //User Hexapod Control
  if(!strncasecmp(line,"hex",3)){
    if(sm_p->state_array[sm_p->state].hex_commander == WATID){
      //Turn ON tilt correction
      if(!strncasecmp(line,"hex tcor on",11)){
	printf("CMD: Turning HEX tilt correction ON\n");
	sm_p->hex_tilt_correct = 1;
      }
      //Turn OFF tilt correction
      if(!strncasecmp(line,"hex tcor off",11)){
	printf("CMD: Turning HEX tilt correction OFF\n");
	sm_p->hex_tilt_correct = 0;
      }
      //Reset step size
      if(!strncasecmp(line,"hex rst step",12)){
	rot_poke = HEX_ROT_POKE;
	trl_poke = HEX_TRL_POKE;
	printf("CMD: Resetting HEX movement to %f mm and %f deg\n", trl_poke, rot_poke);
	return(CMD_NORMAL);
      }
      //Increase step size
      if(!strncasecmp(line,"hex inc step",12)){
	rot_poke *= 2.0;
	trl_poke *= 2.0;
	printf("CMD: Increasing HEX movement to %f mm and %f deg\n", trl_poke, rot_poke);
	return(CMD_NORMAL);
      }
      //Decrease step size
      if(!strncasecmp(line,"hex dec step",12)){
	rot_poke /= 2.0;
	trl_poke /= 2.0;
	printf("CMD: Decreasing HEX movement to %f mm and %f deg\n", trl_poke, rot_poke);
	return(CMD_NORMAL);
      }
      //Query current position
      if(!strncasecmp(line,"hex getpos",10)){
	printf("CMD: Getting hexapod position\n");
	hex_printpos(sm_p->hexfd);
	return(CMD_NORMAL);
      }
      //Commands to Move Hexapod
      if(!strncasecmp(line,"hex gohome",10)){
	printf("CMD: Moving hexapod to home positon\n");
	memcpy(hexcmd.axis_cmd,hexhome,sizeof(hexhome));
	if(!hex_send_command(sm_p,&hexcmd,WATID))
	  printf("CMD: Hexapod command failed\n");
	return(CMD_NORMAL);
      }
      if(!strncasecmp(line,"hex godef",9)){
	printf("CMD: Moving hexapod to default positon\n");
	memcpy(hexcmd.axis_cmd,hexdef,sizeof(hexdef));
	if(!hex_send_command(sm_p,&hexcmd,WATID))
	  printf("CMD: Hexapod command failed\n");
	return(CMD_NORMAL);
      }
      if(!strncasecmp(line,"hex move",8)){
	if(!strncasecmp(line+9,"+x",2)){
	  hex_poke = trl_poke;
	  hex_axis = HEX_AXIS_X;
	  goto move_hex;
	}
	if(!strncasecmp(line+9,"-x",2)){
	  hex_poke = -trl_poke;
	  hex_axis = HEX_AXIS_X;
	  goto move_hex;
	}
	if(!strncasecmp(line+9,"+y",2)){
	  hex_poke = trl_poke;
	  hex_axis = HEX_AXIS_Y;
	  goto move_hex;
	}
	if(!strncasecmp(line+9,"-y",2)){
	  hex_poke = -trl_poke;
	  hex_axis = HEX_AXIS_Y;
	  goto move_hex;
	}
	if(!strncasecmp(line+9,"+z",2)){
	  hex_poke = trl_poke;
	  hex_axis = HEX_AXIS_Z;
	  goto move_hex;
	}
	if(!strncasecmp(line+9,"-z",2)){
	  hex_poke = -trl_poke;
	  hex_axis = HEX_AXIS_Z;
	  goto move_hex;
	}
	if(!strncasecmp(line+9,"+u",2)){
	  hex_poke = rot_poke;
	  hex_axis = HEX_AXIS_U;
	  goto move_hex;
	}
	if(!strncasecmp(line+9,"-u",2)){
	  hex_poke = -rot_poke;
	  hex_axis = HEX_AXIS_U;
	  goto move_hex;
	}
	if(!strncasecmp(line+9,"+v",2)){
	  hex_poke = rot_poke;
	  hex_axis = HEX_AXIS_V;
	  goto move_hex;
	}
	if(!strncasecmp(line+9,"-v",2)){
	  hex_poke = -rot_poke;
	  hex_axis = HEX_AXIS_V;
	  goto move_hex;
	}
	if(!strncasecmp(line+9,"+w",2)){
	  hex_poke = rot_poke;
	  hex_axis = HEX_AXIS_W;
	  goto move_hex;
	}
	if(!strncasecmp(line+9,"-w",2)){
	  hex_poke = -rot_poke;
	  hex_axis = HEX_AXIS_W;
	  goto move_hex;
	}
	printf("CMD: Bad hex move command.\n");
	return(CMD_NORMAL);
      move_hex:
	printf("CMD: Moving hexapod axis %s by %f %s\n",hex_str_axes[hex_axis],hex_poke,hex_str_unit[hex_axis]);
	hex_get_command(sm_p,&hexcmd);
	hexcmd.axis_cmd[hex_axis] += hex_poke;
	if(!hex_send_command(sm_p,&hexcmd,WATID))
	  printf("CMD: Hexapod command failed\n");
      }
      return(CMD_NORMAL);

    }
    else{
      printf("CMD: Manual hexapod control disabled in this state.\n");
      return(CMD_NORMAL);
    }
  }

  //User ALP control
  if(!strncasecmp(line,"alp revert flat",15)){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Reverting ALP flat to default\n");
      if(sm_p->alp_ready)
	if(alp_revert_flat(sm_p,WATID)==0)
	  printf("CMD: ERROR: alp_revert_flat failed!\n");
    }
    else
      printf("CMD: Manual ALPAO DM control disabled in this state.\n");
    return(CMD_NORMAL);
  }
  
  if(!strncasecmp(line,"alp save flat",13)){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Saving current ALP command as flat\n");
      if(sm_p->alp_ready)
	if(alp_save_flat(sm_p))
	  printf("CMD: ERROR: alp_save_flat failed!\n");
    }
    else
      printf("CMD: Manual ALPAO DM control disabled in this state.\n");
    return(CMD_NORMAL);
  }
  
  if(!strncasecmp(line,"alp load flat",13)){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Loading ALP flat from file\n");
      if(sm_p->alp_ready)
	if(alp_load_flat(sm_p,WATID)==0)
	  printf("CMD: ERROR: alp_load_flat failed!\n");
    }
    else
      printf("CMD: Manual ALPAO DM control disabled in this state.\n");
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"alp random",10)){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Sending random actuator pattern to ALP DM\n");
      if(sm_p->alp_ready)
	if(alp_set_random(sm_p,WATID)==0)
	  printf("CMD: ERROR: alp_set_random failed!\n");
    }
    else
      printf("CMD: Manual ALPAO DM control disabled in this state.\n");
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"alp zrandom",11)){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
      printf("CMD: Sending random Zernike pattern to ALP DM\n");
      if(sm_p->alp_ready)
	if(alp_set_zrandom(sm_p,WATID)==0)
	  printf("CMD: ERROR: alp_set_zrandom failed!\n");
    }
    else
      printf("CMD: Manual ALPAO DM control disabled in this state.\n");
    return(CMD_NORMAL);
  }

  sprintf(cmd,"alp zernike");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
     pch = strtok(line+strlen(cmd)," ");
      if(pch == NULL){
	printf("CMD: Bad command format.\n");
	return CMD_NORMAL;
      }
      itemp  = atoi(pch);
      pch = strtok(NULL," ");
      if(pch == NULL){
	printf("CMD: Bad command format.\n");
	return CMD_NORMAL;
      }
      ftemp  = atof(pch);
      printf("CMD: Trying to change Z[%d] by %f microns\n",itemp,ftemp);
      if(itemp >= 0 && itemp < LOWFS_N_ZERNIKE && ftemp >= -1 && ftemp <= 1){
	//Get current command
	alp_get_command(sm_p,&alp);

	//Set zernike perturbation 
	dz[itemp] = ftemp;

	//Convert to actuators deltas
	alp_zern2alp(dz,da,FUNCTION_NO_RESET);

	//Add to current command
	for(i=0;i<LOWFS_N_ZERNIKE;i++)
	  alp.zernike_cmd[i] += dz[i]; 
	for(i=0;i<ALP_NACT;i++)
	  alp.act_cmd[i] += da[i]; 

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
      printf("CMD: Manual ALPAO DM control disabled in this state.\n");
    return CMD_NORMAL;
  }
  
  sprintf(cmd,"alp actuator");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    if(sm_p->state_array[sm_p->state].alp_commander == WATID){
     pch = strtok(line+strlen(cmd)," ");
      if(pch == NULL){
	printf("CMD: Bad command format.\n");
	return CMD_NORMAL;
      }
      itemp  = atoi(pch);
      pch = strtok(NULL," ");
      if(pch == NULL){
	printf("CMD: Bad command format.\n");
	return CMD_NORMAL;
      }
      ftemp  = atof(pch);
      printf("CMD: Trying to change A[%d] by %f units\n",itemp,ftemp);
      if(itemp >= 0 && itemp < ALP_NACT && ftemp >= -1 && ftemp <= 1){
	//Get current command
	alp_get_command(sm_p,&alp);

	//Add to current command
	alp.act_cmd[itemp] += ftemp;

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
      printf("CMD: Manual ALPAO DM control disabled in this state.\n");
    return CMD_NORMAL;
  }
  
  
  //SHK HEX Calibration
  if(!strncasecmp(line,"shk calibrate hex",17)){
    if(sm_p->state == STATE_SHK_HEX_CALIBRATE){
      //Get calmode
      cmdfound = 0;
      for(i=0;i<HEX_NCALMODES;i++){
	if(!strncasecmp(line+18,hexcalmodes[i].cmd,strlen(hexcalmodes[i].cmd))){
	  calmode  = i;
	  cmdfound = 1;
	}
      }
      if(!cmdfound){
	printf("CMD: Could not find hex calmode\n");
	print_hex_calmodes(hexcalmodes);
	return(CMD_NORMAL);
      }
      printf("CMD: Running SHK HEX calibration\n");
      //Change calibration output filename
      sprintf((char *)sm_p->calfile,SHK_HEX_CALFILE,(char *)hexcalmodes[calmode].cmd);
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
      printf("CMD: Must be in STATE_SHK_HEX_CALIBRATE\n");
      return(CMD_NORMAL);
    }
  }

  //SHK ALP Calibration
  if(!strncasecmp(line,"shk calibrate alp",17)){
    if(sm_p->state == STATE_SHK_ALP_CALIBRATE){
      //Get calmode
      cmdfound = 0;
      for(i=0;i<ALP_NCALMODES;i++){
	if(!strncasecmp(line+18,alpcalmodes[i].cmd,strlen(alpcalmodes[i].cmd))){
	  calmode  = i;
	  cmdfound = 1;
	}
      }
      if(!cmdfound){
	printf("CMD: Could not find alp calmode\n");
	print_alp_calmodes(alpcalmodes);
	return(CMD_NORMAL);
      }
      printf("CMD: Running SHK ALP calibration\n");
      //Change calibration output filename
      sprintf((char *)sm_p->calfile,SHK_ALP_CALFILE,(char *)alpcalmodes[calmode].cmd);
      //Start data recording
      printf("  -- Starting data recording to file: %s\n",sm_p->calfile);
      sm_p->w[DIAID].launch = getshk_proc;
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
      printf("CMD: Must be in STATE_SHK_ALP_CALIBRATE\n");
      return(CMD_NORMAL);
    }
  }

  //LYT ALP Calibration
  if(!strncasecmp(line,"lyt calibrate alp",17)){
    if(sm_p->state == STATE_LYT_ALP_CALIBRATE){
      //Get calmode
      cmdfound = 0;
      for(i=0;i<ALP_NCALMODES;i++){
	if(!strncasecmp(line+18,alpcalmodes[i].cmd,strlen(alpcalmodes[i].cmd))){
	  calmode  = i;
	  cmdfound = 1;
	}
      }
      if(!cmdfound){
	printf("CMD: Could not find alp calmode\n");
	print_alp_calmodes(alpcalmodes);
	return(CMD_NORMAL);
      }
      printf("CMD: Running LYT ALP calibration\n");
      //Change calibration output filename
      sprintf((char *)sm_p->calfile,LYT_ALP_CALFILE,(char *)alpcalmodes[calmode].cmd);
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
      printf("CMD: Must be in STATE_LYT_ALP_CALIBRATE\n");
      return(CMD_NORMAL);
    }
  }
  //LYT reference image (debugging)
  sprintf(cmd,"lyt refimg on");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->lyt_send_refimg=1;
    printf("CMD: Starting LYT sending reference image\n"); 
    return(CMD_NORMAL);
  }
  sprintf(cmd,"lyt refimg off");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    sm_p->lyt_send_refimg=0;
    printf("CMD: Stopping LYT sending reference image\n");
    return(CMD_NORMAL);
  }
  
  //Reset Commands
  if(!strncasecmp(line,"shk reset",9)){
    sm_p->shk_reset=1;
    printf("CMD: Resetting SHK\n");
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"lyt reset",9)){
    sm_p->lyt_reset=1;
    printf("CMD: Resetting LYT\n");
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"acq reset",9)){
    sm_p->acq_reset=1;
    printf("CMD: Resetting ACQ\n");
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"sci reset",9)){
    sm_p->sci_reset=1;
    printf("CMD: Resetting SCI\n");
    return(CMD_NORMAL);
  }

  //Exposure Time Commands
  sprintf(cmd,"sci exptime");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd));
    if(ftemp >= SCI_EXPTIME_MIN && ftemp <= SCI_EXPTIME_MAX){
      sm_p->sci_exptime = ftemp;
      sm_p->sci_reset_camera = 1;
      printf("CMD: Setting SCI exptime to %f seconds\n",sm_p->sci_exptime);
    }
    else
      printf("CMD: SCI exptime must be between %f and %f seconds\n",SCI_EXPTIME_MIN,SCI_EXPTIME_MAX);
    return(CMD_NORMAL);
  }
  
  sprintf(cmd,"shk exptime");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd));
    if(ftemp >= SHK_EXPTIME_MIN && ftemp <= SHK_EXPTIME_MAX){
      sm_p->shk_exptime = ftemp;
      sm_p->shk_reset_camera = 1;
      printf("CMD: Setting SHK exptime to %f seconds\n",sm_p->shk_exptime);
    }
    else
      printf("CMD: SHK exptime must be between %f and %f seconds\n",SHK_EXPTIME_MIN,SHK_EXPTIME_MAX);
    return(CMD_NORMAL);
  }

  sprintf(cmd,"lyt exptime");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd));
    if(ftemp >= LYT_EXPTIME_MIN && ftemp <= LYT_EXPTIME_MAX){
      sm_p->lyt_exptime = ftemp;
      sm_p->lyt_reset_camera = 1;
      printf("CMD: Setting LYT exptime to %f seconds\n",sm_p->lyt_exptime);
    }
    else
      printf("CMD: LYT exptime must be between %f and %f seconds\n",LYT_EXPTIME_MIN,LYT_EXPTIME_MAX);
    return(CMD_NORMAL);
  }

  sprintf(cmd,"acq exptime");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    ftemp = atof(line+strlen(cmd));
    if(ftemp >= ACQ_EXPTIME_MIN && ftemp <= ACQ_EXPTIME_MAX){
      sm_p->acq_exptime = ftemp;
      sm_p->acq_reset_camera = 1;
      printf("CMD: Setting ACQ exptime to %f seconds\n",sm_p->acq_exptime);
    }
    else
      printf("CMD: ACQ exptime must be between %f and %f seconds\n",ACQ_EXPTIME_MIN,ACQ_EXPTIME_MAX);
    return(CMD_NORMAL);
  }

  //SHK Commands
  if(!strncasecmp(line,"shk set cenbox origin",21)){
    printf("CMD: Setting SHK centroid box origin\n");
    sm_p->shk_setcenboxorigin=1;
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"shk set origin",14)){
    printf("CMD: Setting SHK origin\n");
    sm_p->shk_setorigin=1;
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"shk revert origin",17)){
    printf("CMD: Reverting SHK origin\n");
    sm_p->shk_revertorigin=1;
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"shk save origin",15)){
    printf("CMD: Saving SHK origin\n");
    sm_p->shk_saveorigin=1;
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"shk load origin",15)){
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
  
  //SHK ALP Gain
  if(!strncasecmp(line,"shk alp gain ",13) && strlen(line)>14){
    ftemp = atof(line+13);
    if(ftemp > 0){
      sm_p->shk_kP_alp_cell = SHK_KP_ALP_CELL_DEFAULT*ftemp;
      sm_p->shk_kI_alp_cell = SHK_KI_ALP_CELL_DEFAULT*ftemp;
      sm_p->shk_kD_alp_cell = SHK_KD_ALP_CELL_DEFAULT*ftemp;
      sm_p->shk_kP_alp_zern = SHK_KP_ALP_ZERN_DEFAULT*ftemp;
      sm_p->shk_kI_alp_zern = SHK_KI_ALP_ZERN_DEFAULT*ftemp;
      sm_p->shk_kD_alp_zern = SHK_KD_ALP_ZERN_DEFAULT*ftemp;
      printf("SHK switching to ALP cell gain: %f, %f, %f\n",sm_p->shk_kP_alp_cell,sm_p->shk_kI_alp_cell,sm_p->shk_kD_alp_cell);
      printf("SHK switching to ALP zern gain: %f, %f, %f\n",sm_p->shk_kP_alp_zern,sm_p->shk_kI_alp_zern,sm_p->shk_kD_alp_zern);
    }else printf("CMD: Gain multiplier must be > 0\n");
    return CMD_NORMAL;
  }

  //SHK HEX Gain
  if(!strncasecmp(line,"shk hex gain ",13) && strlen(line)>14){
    ftemp = atof(line+13);
    if(ftemp > 0){
      sm_p->shk_kP_hex_zern = SHK_KP_HEX_ZERN_DEFAULT*ftemp;
      sm_p->shk_kI_hex_zern = SHK_KI_HEX_ZERN_DEFAULT*ftemp;
      sm_p->shk_kD_hex_zern = SHK_KD_HEX_ZERN_DEFAULT*ftemp;
      printf("SHK switching to HEX zern gain: %f, %f, %f\n",sm_p->shk_kP_hex_zern,sm_p->shk_kI_hex_zern,sm_p->shk_kD_hex_zern);
    }else printf("CMD: Gain multiplier must be > 0\n");
    return CMD_NORMAL;
  }

  //LYT ALP Gain
  if(!strncasecmp(line,"lyt alp gain ",13) && strlen(line)>14){
    ftemp = atof(line+13);
    if(ftemp > 0){
      double lyt_gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID] = LYT_GAIN_ALP_ZERN_DEFAULT;
      double lyt_gain_alp_act[LOWFS_N_PID] = LYT_GAIN_ALP_ACT_DEFAULT;
      // -- zernike gain
      for(i=0;i<LOWFS_N_ZERNIKE;i++) 
	for(j=0;j<LOWFS_N_PID;j++)
	  sm_p->lyt_gain_alp_zern[i][j] = ftemp * lyt_gain_alp_zern[i][j];
      // -- actuator gain
      for(j=0;j<LOWFS_N_PID;j++)
	sm_p->lyt_gain_alp_act[j] = ftemp * lyt_gain_alp_act[j];

      printf("LYT changing gain multiplier to %f\n",ftemp);
    }else printf("CMD: Gain multiplier must be > 0\n");
    return CMD_NORMAL;
  }

  //Zernike control commands
  sprintf(cmd,"zernike info");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    print_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"zernike disable all");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Disabling control of ALL Zernikes\n\n");
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      sm_p->zernike_control[i]=0;
    print_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"zernike enable all");
  if(!strncasecmp(line,cmd,strlen(cmd))){
    printf("CMD: Enabling control of ALL Zernikes\n\n");
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      sm_p->zernike_control[i]=1;
    print_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"zernike disable");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    itemp = atoi(line+strlen(cmd));
    sm_p->zernike_control[itemp]=0;
    printf("CMD: Disabling control of Zernike %d\n\n",itemp);
    print_zernikes(sm_p);
    return CMD_NORMAL;
  }
  sprintf(cmd,"zernike enable");
  if(!strncasecmp(line,cmd,strlen(cmd)) && strlen(line) > strlen(cmd)){
    itemp = atoi(line+strlen(cmd));
    sm_p->zernike_control[itemp]=1;
    printf("CMD: Enabling control of Zernike %d\n\n",itemp);
    print_zernikes(sm_p);
    return CMD_NORMAL;
  }

  //SCI Commands
  if(!strncasecmp(line,"sci set origin",14)){
    printf("CMD: Setting SCI origin\n");
    sm_p->sci_setorigin=1;
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"sci revert origin",17)){
    printf("CMD: Reverting SCI origin\n");
    sm_p->sci_revertorigin=1;
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"sci save origin",15)){
    printf("CMD: Saving SCI origin\n");
    sm_p->sci_saveorigin=1;
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"sci load origin",15)){
    printf("CMD: Loading SCI origin\n");
    sm_p->sci_loadorigin=1;
    return(CMD_NORMAL);
  }
  
  /****************************************
   * BLANK COMMAND
   ***************************************/
  if(strlen(line) == 1)
    return CMD_NORMAL;

  //return with command not found
  return(CMD_NOT_FOUND);
}
