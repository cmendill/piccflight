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
#include "states.h"

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
    printf("%02d   %s        %s\n",i,sm_p->state_array[i].cmd,sm_p->state_array[i].name);
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
  int   found_state=0;
  int   i;
  static double trl_poke = HEX_TRL_POKE;//0.01;
  static double rot_poke = HEX_ROT_POKE;///0.01;
  
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

  //ALP Calmodes
  if(!strncasecmp(line,"alp calmode",11)){
    itemp = atoi(line+11);
    if(itemp >= 0 && itemp < ALP_NCALMODE){
      sm_p->alp_calmode=itemp;
      printf("CMD: Changed ALP calibration mode to %d\n",sm_p->alp_calmode);
    }else printf("CMD: Invalid ALP calmode %d\n",itemp);
    return(CMD_NORMAL);
  }

  //ALP Calibration
  if(!strncasecmp(line,"shk calibrate alp",17)){
    printf("CMD: Running SHK ALP calibration\n");
    printf("  -- Disabling PID\n");
    //Turn off gains
    sm_p->shk_kP_cell = 0;
    sm_p->shk_kI_cell = 0;
    sm_p->shk_kD_cell = 0;
    sm_p->shk_kP_zern = 0;
    sm_p->shk_kI_zern = 0;
    sm_p->shk_kD_zern = 0;
    sleep(1);
    //Start data recording
    printf("  -- Starting data recording\n");
    sm_p->w[DIAID].launch = getshk_proc;
    sm_p->w[DIAID].run    = 1;
    sleep(3);
    //Start probe pattern
    sm_p->alp_calmode = ALP_CALMODE_POKE;
    printf("  -- Changing ALP calibration mode to %d\n",sm_p->alp_calmode);
    while(sm_p->alp_calmode == ALP_CALMODE_POKE)
      sleep(1);
    //Stop data recording
    printf("  -- Stopping data recording\n");
    sm_p->w[DIAID].run    = 0;
    printf("  -- Done\n");

    return(CMD_NORMAL);
  }

  //States
  if(!strncasecmp(line,"state",5)){
    found_state=0;
    for(i=0;i<NSTATES;i++){
      if(!strncasecmp(line+6,(char *)sm_p->state_array[i].cmd,3)){
	sm_p->state = i;
	printf("CMD: Changing state to %s\n",sm_p->state_array[i].name);
	found_state = 1;
      }
    }
    if(!found_state) print_states(sm_p);
    return(CMD_NORMAL);
  }

  

  //SHK Zernike Calibration
  if(!strncasecmp(line,"shk calibrate zern",18)){
    printf("CMD: Running SHK Zern calibration\n");
    printf("  -- Disabling PID\n");
    //Turn off gains
    sm_p->shk_kP_cell = 0;
    sm_p->shk_kI_cell = 0;
    sm_p->shk_kD_cell = 0;
    sm_p->shk_kP_zern = 0;
    sm_p->shk_kI_zern = 0;
    sm_p->shk_kD_zern = 0;
    sleep(1);
    //Start data recording
    printf("  -- Starting data recording\n");
    sm_p->w[DIAID].launch = getshk_proc;
    sm_p->w[DIAID].run    = 1;
    sleep(3);
    //Start probe pattern
    sm_p->alp_calmode=ALP_CALMODE_ZPOKE;
    printf("  -- Changing ALP calibration mode to %d\n",sm_p->alp_calmode);
    while(sm_p->alp_calmode == ALP_CALMODE_ZPOKE)
      sleep(1);
    //Stop data recording
    printf("  -- Stopping data recording\n");
    sm_p->w[DIAID].run    = 0;
    printf("  -- Done\n");
    
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
    //Start data recording
    printf("  --Starting data recording\n");
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
  if(sm_p->state_array[sm_p->state].usr.control_hex){
    if(!strncasecmp(line,"hex getpos",10)){
      sm_p->hex_getpos=1;
      printf("CMD: Getting hexapod position\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex gohome",10)){
      sm_p->hex_gohome=1;
      printf("CMD: Moving hexapod to home positon\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex godef",9)){
      sm_p->hex_godef=1;
      printf("CMD: Moving hexapod to default positon\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex move x",10)){
      if(sm_p->hex_last_recv == sm_p->hex_last_sent){
	sm_p->hex_command[HEX_AXIS_X] += trl_poke;
	sm_p->hex_last_sent++;
	printf("CMD: Moving hexapod axis X by %f mm\n", trl_poke);
      } else printf("CMD: Hexapod busy...\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex move -x",11)){
      if(sm_p->hex_last_recv == sm_p->hex_last_sent){
	sm_p->hex_command[HEX_AXIS_X] -= trl_poke;
	sm_p->hex_last_sent++;
	printf("CMD: Moving hexapod axis X by %f mm\n", -trl_poke);
      } else printf("CMD: Hexapod busy...\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex move y",10)){
      if(sm_p->hex_last_recv == sm_p->hex_last_sent){
	sm_p->hex_command[HEX_AXIS_Y] += trl_poke;
	sm_p->hex_last_sent++;
	printf("CMD: Moving hexapod axis Y by %f mm\n", trl_poke);
      } else printf("CMD: Hexapod busy...\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex move -y",11)){
      if(sm_p->hex_last_recv == sm_p->hex_last_sent){
	sm_p->hex_command[HEX_AXIS_Y] -= trl_poke;
	sm_p->hex_last_sent++;
	printf("CMD: Moving hexapod axis Y by %f mm\n", -trl_poke);
      } else printf("CMD: Hexapod busy...\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex move z",10)){
      if(sm_p->hex_last_recv == sm_p->hex_last_sent){
	sm_p->hex_command[HEX_AXIS_Z] += trl_poke;
	sm_p->hex_last_sent++;
	printf("CMD: Moving hexapod axis Z by %f mm\n", trl_poke);
      } else printf("CMD: Hexapod busy...\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex move -z",11)){
      if(sm_p->hex_last_recv == sm_p->hex_last_sent){
	sm_p->hex_command[HEX_AXIS_Z] -= trl_poke;
	sm_p->hex_last_sent++;
	printf("CMD: Moving hexapod axis Z by %f mm\n", -trl_poke);
      } else printf("CMD: Hexapod busy...\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex move u",10)){
      if(sm_p->hex_last_recv == sm_p->hex_last_sent){
	sm_p->hex_command[HEX_AXIS_U] += rot_poke;
	sm_p->hex_last_sent++;
	printf("CMD: Moving hexapod axis U by %f deg\n", rot_poke);
      } else printf("CMD: Hexapod busy...\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex move -u",11)){
      if(sm_p->hex_last_recv == sm_p->hex_last_sent){
	sm_p->hex_command[HEX_AXIS_U] -= rot_poke;
	sm_p->hex_last_sent++;
	printf("CMD: Moving hexapod axis U by %f deg\n", -rot_poke);
      } else printf("CMD: Hexapod busy...\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex move v",10)){
      if(sm_p->hex_last_recv == sm_p->hex_last_sent){
	sm_p->hex_command[HEX_AXIS_V] += rot_poke;
	sm_p->hex_last_sent++;
	printf("CMD: Moving hexapod axis V by %f deg\n", rot_poke);
      } else printf("CMD: Hexapod busy...\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex move -v",11)){
      if(sm_p->hex_last_recv == sm_p->hex_last_sent){
	sm_p->hex_command[HEX_AXIS_V] -= rot_poke;
	sm_p->hex_last_sent++;
	printf("CMD: Moving hexapod axis V by %f deg\n", -rot_poke);
      } else printf("CMD: Hexapod busy...\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex move w",10)){
      if(sm_p->hex_last_recv == sm_p->hex_last_sent){
	sm_p->hex_command[HEX_AXIS_W] += rot_poke;
	sm_p->hex_last_sent++;
	printf("CMD: Moving hexapod axis W by %f deg\n", rot_poke);
      } else printf("CMD: Hexapod busy...\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex move -w",11)){
      if(sm_p->hex_last_recv == sm_p->hex_last_sent){
	sm_p->hex_command[HEX_AXIS_W] -= rot_poke;
	sm_p->hex_last_sent++;
	printf("CMD: Moving hexapod axis W by %f deg\n", -rot_poke);
      } else printf("CMD: Hexapod busy...\n");
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex inc step",12)){
      rot_poke *= 2.0;
      trl_poke *= 2.0;
      printf("CMD: Increasing HEX movement to %f mm and %f deg\n", trl_poke, rot_poke);
      return(CMD_NORMAL);
    }
    if(!strncasecmp(line,"hex dec step",12)){
      rot_poke /= 2.0;
      trl_poke /= 2.0;
      printf("CMD: Decreasing HEX movement to %f mm and %f deg\n", trl_poke, rot_poke);
      return(CMD_NORMAL);
    }
  }
  
  //HEX Calmodes
  if(!strncasecmp(line,"hex calmode",11)){
    itemp = atoi(line+11);
    if(itemp >= 0 && itemp < HEX_NCALMODE){
      sm_p->hex_calmode=itemp;
      printf("CMD: Changed HEX calibration mode to %d\n",sm_p->hex_calmode);
    }else printf("CMD: Invalid HEX calmode %d\n",itemp);
    return(CMD_NORMAL);
  }

  //HEX Calibration
  if(!strncasecmp(line,"shk calibrate hex",17)){
    printf("CMD: Running HEX AXS calibration\n");
    printf("  -- Disabling PID\n");
    //Turn off gains
    sm_p->shk_kP_cell = 0;
    sm_p->shk_kI_cell = 0;
    sm_p->shk_kD_cell = 0;
    sm_p->shk_kP_zern = 0;
    sm_p->shk_kI_zern = 0;
    sm_p->shk_kD_zern = 0;
    sleep(3);

    //Start data recording
    printf("  -- Starting data recording\n");
    sm_p->w[DIAID].launch = getshk_proc;
    sm_p->w[DIAID].run    = 1;
    sleep(3);
    //Start probe pattern
    sm_p->hex_calmode = HEX_CALMODE_POKE;
    printf("  -- Changing HEX calibration mode to %d\n",sm_p->hex_calmode);
    while(sm_p->hex_calmode == HEX_CALMODE_POKE)
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
    //Turn off gains
    printf("  -- Disabling PID\n");
    sm_p->shk_kP_cell = 0;
    sm_p->shk_kI_cell = 0;
    sm_p->shk_kD_cell = 0;
    sm_p->shk_kP_zern = 0;
    sm_p->shk_kI_zern = 0;
    sm_p->shk_kD_zern = 0;
    sleep(3);

    //Start data recording
    printf("  -- Starting data recording\n");
    sm_p->w[DIAID].launch = getshk_proc;
    sm_p->w[DIAID].run    = 1;
    sleep(3);
    //Start probe pattern
    sm_p->hex_calmode = HEX_CALMODE_POKE;
    printf("  -- Changing HEX calibration mode to %d\n",sm_p->hex_calmode);
    while(sm_p->hex_calmode == HEX_CALMODE_POKE)
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
    //Turn off gains
    printf("  -- Disabling PID\n");
    sm_p->shk_kP_cell = 0;
    sm_p->shk_kI_cell = 0;
    sm_p->shk_kD_cell = 0;
    sm_p->shk_kP_zern = 0;
    sm_p->shk_kI_zern = 0;
    sm_p->shk_kD_zern = 0;
    printf("  -- Resetting SHK\n");
    sm_p->shk_reset = 1;
    sleep(1);
    sm_p->shk_setorigin=1;
    return(CMD_NORMAL);
  }
  
  //SHK Gain
  if(!strncasecmp(line,"shk gain ",9) && strlen(line)>10){
    ftemp = atof(line+9);
    if(ftemp >= 0 && ftemp <= 5)
      ftemp /= 5.0;
    sm_p->shk_kP_cell = SHK_KP_CELL_DEFAULT*ftemp;
    sm_p->shk_kI_cell = SHK_KI_CELL_DEFAULT*ftemp;
    sm_p->shk_kD_cell = SHK_KD_CELL_DEFAULT*ftemp;
    sm_p->shk_kP_zern = SHK_KP_ZERN_DEFAULT*ftemp;
    sm_p->shk_kI_zern = SHK_KI_ZERN_DEFAULT*ftemp;
    sm_p->shk_kD_zern = SHK_KD_ZERN_DEFAULT*ftemp;
    printf("SHK switching to cell gain 5: %f, %f, %f\n",sm_p->shk_kP_cell,sm_p->shk_kI_cell,sm_p->shk_kD_cell);
    printf("SHK switching to zerm gain 5: %f, %f, %f\n",sm_p->shk_kP_zern,sm_p->shk_kI_zern,sm_p->shk_kD_zern);
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
