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

/* Prototypes */
void getshk_proc(void); //get shkevents

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
/* HANDLE_COMMAND                                             */
/*  - Handle user commands                                    */
/**************************************************************/
int handle_command(char *line, sm_t *sm_p){
  float ftemp;
  int   itemp;
  char  stemp[CMD_MAX_LENGTH];
  int   cmdfound=0;
  int   i=0,hex_axis=0;
  double hex_poke=0;
  int    calmode=0;
  static double trl_poke = HEX_TRL_POKE;//0.01;
  static double rot_poke = HEX_ROT_POKE;///0.01;
  static calmode_t alpcalmodes[ALP_NCALMODES];
  static calmode_t hexcalmodes[HEX_NCALMODES];
  static int init=0;
  static hexevent_t hexrecv = {0},hexsend = {0};
  const char hex_str_axes[HEX_NAXES][5] = {"X","Y","Z","U","V","W"};
  const char hex_str_unit[HEX_NAXES][5] = {"mm","mm","mm","deg","deg","deg"};
  const double hexhome[6] = HEX_POS_HOME;
  const double hexdef[6]  = HEX_POS_DEFAULT;

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
    //Set init flag
    init=1;
  }
  
  /****************************************
   * SYSTEM COMMANDS
   ***************************************/
  //exit: exit watchdog
  if(!strncasecmp(line,"exit",4)){
    return(CMD_EXIT_WATCHDOG);
  }

  /****************************************
   * SHARED MEMORY COMMANDS
   ***************************************/
  //Fake Data
  if(!strncasecmp(line,"shk fake",8)){
    itemp = atoi(line+8);
    sm_p->shk_fake_mode = itemp;
    printf("CMD: Changed SHK fake mode to %d\n",sm_p->shk_fake_mode);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"lyt fake",8)){
    itemp = atoi(line+8);
    sm_p->lyt_fake_mode = itemp;
    printf("CMD: Changed LYT fake mode to %d\n",sm_p->lyt_fake_mode);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"sci fake",8)){
    itemp = atoi(line+8);
    sm_p->sci_fake_mode = itemp;
    printf("CMD: Changed SCI fake mode to %d\n",sm_p->sci_fake_mode);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"acq fake",8)){
    itemp = atoi(line+8);
    sm_p->acq_fake_mode = itemp;
    printf("CMD: Changed ACQ fake mode to %d\n",sm_p->acq_fake_mode);
    return(CMD_NORMAL);
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

  
  //SHK Flight Test
  if(!strncasecmp(line,"shk flight test",15)){
    printf("CMD: Running SHK Flight Test\n");
    //Start data recording
    printf("  -- Starting data recording\n");
    sm_p->w[DIAID].launch = getshk_proc;
    sm_p->w[DIAID].run    = 1;
    sleep(3);
    //Start probe pattern
    sm_p->alp_calmode=ALP_CALMODE_FLIGHT;
    printf("  -- Changing ALP calibration mode to %d\n",sm_p->alp_calmode);
    while(sm_p->alp_calmode == ALP_CALMODE_FLIGHT)
      sleep(1);
    printf("  -- Stopping data recording\n");
    //Stop data recording
    sm_p->w[DIAID].run    = 0;
    printf("  -- Done\n");

    return(CMD_NORMAL);
  }

  //Start Manual Data Recording
  if(!strncasecmp(line, "shk start rec", 13)){
    printf("CMD: Start Manual Data Recording\n");
    //Setup filename
    sprintf((char *)sm_p->calfile,SHK_OUTFILE);
    //Start data recording
    printf("  --Starting data recording to: %s\n",sm_p->calfile);
    sm_p->w[DIAID].launch = getshk_proc;
    sm_p->w[DIAID].run    = 1;
    sleep(3);
    return(CMD_NORMAL);
  }

  //Stop Manual Data Recording
  if(!strncasecmp(line, "shk stop rec", 12)){
    printf("CMD: Stopping Manual Data Recording\n");
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
	sm_p->hex_getpos=1;
	printf("CMD: Getting hexapod position\n");
	return(CMD_NORMAL);
      }
      //Read current hexapod position
      while(read_from_buffer(sm_p,&hexrecv,HEXRECV,WATID)){
	//Copy accepted command to current position
	memcpy(&hexsend,&hexrecv,sizeof(hexevent_t));
      }
      //Set commander ID
      hexsend.clientid = WATID;
      //Commands to Move Hexapod
      if(!strncasecmp(line,"hex gohome",10)){
	memcpy(hexsend.hex.axis_cmd,hexhome,sizeof(hexhome));
	printf("CMD: Moving hexapod to home positon\n");
	write_to_buffer(sm_p,&hexsend,WAT_HEXSEND);
	return(CMD_NORMAL);
      }
      if(!strncasecmp(line,"hex godef",9)){
	memcpy(hexsend.hex.axis_cmd,hexdef,sizeof(hexdef));
	printf("CMD: Moving hexapod to default positon\n");
	write_to_buffer(sm_p,&hexsend,WAT_HEXSEND);
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
	hexsend.hex.axis_cmd[hex_axis] += hex_poke;
	write_to_buffer(sm_p,&hexsend,WAT_HEXSEND);
      }
      return(CMD_NORMAL);
      
    }
    else{
      printf("CMD: Manual hexapod control disabled in this state.\n");
      return(CMD_NORMAL);
    }
  }
  
  //HEX Calibration
  if(!strncasecmp(line,"shk calibrate hex",17)){
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
    printf("CMD: Running HEX calibration\n");
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

  //ALP Calibration
  if(!strncasecmp(line,"shk calibrate alp",17)){
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
    printf("CMD: Running ALP calibration\n");
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

  //SHK Commands
  if(!strncasecmp(line,"shk set origin",14)){
    printf("CMD: Setting SHK origin\n");
    sm_p->shk_setorigin=1;
    return(CMD_NORMAL);
  }

  //SHK ALP Gain
  if(!strncasecmp(line,"shk alp gain ",13) && strlen(line)>14){
    ftemp = atof(line+13);
    if(ftemp >= 0 && ftemp <= 5)
      ftemp /= 5.0;
    sm_p->shk_kP_alp_cell = SHK_KP_ALP_CELL_DEFAULT*ftemp;
    sm_p->shk_kI_alp_cell = SHK_KI_ALP_CELL_DEFAULT*ftemp;
    sm_p->shk_kD_alp_cell = SHK_KD_ALP_CELL_DEFAULT*ftemp;
    sm_p->shk_kP_alp_zern = SHK_KP_ALP_ZERN_DEFAULT*ftemp;
    sm_p->shk_kI_alp_zern = SHK_KI_ALP_ZERN_DEFAULT*ftemp;
    sm_p->shk_kD_alp_zern = SHK_KD_ALP_ZERN_DEFAULT*ftemp;
    printf("SHK switching to ALP cell gain 5: %f, %f, %f\n",sm_p->shk_kP_alp_cell,sm_p->shk_kI_alp_cell,sm_p->shk_kD_alp_cell);
    printf("SHK switching to ALP zern gain 5: %f, %f, %f\n",sm_p->shk_kP_alp_zern,sm_p->shk_kI_alp_zern,sm_p->shk_kD_alp_zern);
    return CMD_NORMAL;
  }

  //SHK HEX Gain
  if(!strncasecmp(line,"shk hex gain ",13) && strlen(line)>14){
    ftemp = atof(line+13);
    if(ftemp >= 0 && ftemp <= 5)
      ftemp /= 5.0;
    sm_p->shk_kP_hex_zern = SHK_KP_HEX_ZERN_DEFAULT*ftemp;
    sm_p->shk_kI_hex_zern = SHK_KI_HEX_ZERN_DEFAULT*ftemp;
    sm_p->shk_kD_hex_zern = SHK_KD_HEX_ZERN_DEFAULT*ftemp;
    printf("SHK switching to HEX zern gain 5: %f, %f, %f\n",sm_p->shk_kP_hex_zern,sm_p->shk_kI_hex_zern,sm_p->shk_kD_hex_zern);
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
