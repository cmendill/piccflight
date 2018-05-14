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

/* prototypes */
void getshk_proc(void); //get shkevents


/* Handle user commands*/
int handle_command(char *line, sm_t *sm_p){
  float ftemp;
  int   itemp;
  char  stemp[CMD_MAX_LENGTH];
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
    strncpy(stemp,line+8,4);
    itemp = atoi(stemp);
    sm_p->shk_fake_mode = itemp;
    printf("CMD: Changed SHK fake mode to %d\n",sm_p->shk_fake_mode);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"lyt fake",8)){
    strncpy(stemp,line+8,4);
    itemp = atoi(stemp);
    sm_p->lyt_fake_mode = itemp;
    printf("CMD: Changed LYT fake mode to %d\n",sm_p->lyt_fake_mode);
    return(CMD_NORMAL);
  }

  //ALP Calmodes
  if(!strncasecmp(line,"alp calmode 0",13)){
    sm_p->alp_calmode=0;
    printf("CMD: Changed ALP calibration mode to %d\n",sm_p->alp_calmode);
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"alp calmode 1",13)){
    sm_p->alp_calmode=1;
    printf("CMD: Changed ALP calibration mode to %d\n",sm_p->alp_calmode);
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"alp calmode 2",13)){
    sm_p->alp_calmode=2;
    printf("CMD: Changed ALP calibration mode to %d\n",sm_p->alp_calmode);
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"alp calmode 3",13)){
    sm_p->alp_calmode=3;
    printf("CMD: Changed ALP calibration mode to %d\n",sm_p->alp_calmode);
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"alp calmode 4",13)){
    sm_p->alp_calmode=4;
    printf("CMD: Changed ALP calibration mode to %d\n",sm_p->alp_calmode);
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"alp calmode 5",13)){
    sm_p->alp_calmode=5;
    printf("CMD: Changed ALP calibration mode to %d\n",sm_p->alp_calmode);
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"alp calmode 6",13)){
    sm_p->alp_calmode=6;
    printf("CMD: Changed ALP calibration mode to %d\n",sm_p->alp_calmode);
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"alp calmode 7",13)){
    sm_p->alp_calmode=7;
    printf("CMD: Changed ALP calibration mode to %d\n",sm_p->alp_calmode);
    return(CMD_NORMAL);
  }

  // if(!strncasecmp(line,"zern",4)){
  //   char buff1[2];
  //   printf("Please enter zern mode:\n");
  //   scanf("%s", buff1);
  //   printf("You entered: %s\r\n", buff1);
  //   int zern = atoi(buff1);
  //
  //   char buff2[2];
  //   printf("Please enter value:\n");
  //   scanf("%s", buff2);
  //   printf("You entered: %s\r\n", buff2);
  //   double val = atof(buff2);
  //   printf("zern: %i, val: %f\r\n", zern, val);
  //   sm_p->zern_targ[zern] = val;
  //   return(0);
  // }

  //ALP Calibration
  if(!strncasecmp(line,"shk calibrate alp",17)){
    printf("CMD: Running SHK ALP calibration\n");
    printf("  -- Disabling PID\n");
    //Turn off gains
    sm_p->shk_kP = 0;
    sm_p->shk_kI = 0;
    sm_p->shk_kD = 0;
    //printf("  -- Resetting SHK\n");
    //sm_p->shk_reset = 1;
    sleep(3);

    //Start data recording
    printf("  -- Starting data recording\n");
    sm_p->w[DIAID].launch = getshk_proc;
    sm_p->w[DIAID].run    = 1;
    sleep(3);
    //Start probe pattern
    sm_p->alp_calmode=2;
    printf("  -- Changing ALP calibration mode to %d\n",sm_p->alp_calmode);
    while(sm_p->alp_calmode == 2)
      sleep(1);
    printf("  -- Stopping data recording\n");
    //Stop data recording
    sm_p->w[DIAID].run    = 0;
    printf("  -- Done\n");

    return(CMD_NORMAL);
  }




    //Zern Calibration
    if(!strncasecmp(line,"shk calibrate zern",18)){
      printf("CMD: Running SHK Zern calibration\n");
      sleep(3);

      //Start data recording
      printf("  -- Starting data recording\n");
      sm_p->w[DIAID].launch = getshk_proc;
      sm_p->w[DIAID].run    = 1;
      sleep(3);
      //Start probe pattern
      sm_p->alp_calmode=7;
      printf("  -- Changing ALP calibration mode to %d\n",sm_p->alp_calmode);
      while(sm_p->alp_calmode == 7)
        sleep(1);
      printf("  -- Stopping data recording\n");
      //Stop data recording
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
    sm_p->alp_calmode=4;
    printf("  -- Changing ALP calibration mode to %d\n",sm_p->alp_calmode);
    while(sm_p->alp_calmode == 4)
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

  //SHK Zernike Fitting
  if(!strncasecmp(line,"shk zfit on",11)){
    sm_p->shk_fit_zernike=1;
    printf("CMD: Turned SHK Zernike fitting ON\n");
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"shk zfit off",12)){
    sm_p->shk_fit_zernike=0;
    printf("CMD: Turned SHK Zernike fitting OFF\n");
    return(CMD_NORMAL);
  }

  //Hexapod control
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
    sm_p->hex[0] += trl_poke;
    printf("CMD: Moving hexapod axis x by %f mm\n", trl_poke);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"hex move -x",11)){
    sm_p->hex[0] -= trl_poke;
    printf("CMD: Moving hexapod axis -x by %f mm\n", trl_poke);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"hex move y",10)){
    sm_p->hex[1] += trl_poke;
    printf("CMD: Moving hexapod axis y by %f mm\n", trl_poke);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"hex move -y",11)){
    sm_p->hex[1] -= trl_poke;
    printf("CMD: Moving hexapod axis -y by %f mm\n", trl_poke);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"hex move z",10)){
    sm_p->hex[2] += trl_poke;
    printf("CMD: Moving hexapod axis z by %f mm\n", trl_poke);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"hex move -z",11)){
    sm_p->hex[2] -= trl_poke;
    printf("CMD: Moving hexapod axis -z by %f mm\n", trl_poke);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"hex move u",10)){
    sm_p->hex[3] += rot_poke;
    printf("CMD: Moving hexapod axis u by %f deg\n", rot_poke);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"hex move -u",11)){
    sm_p->hex[3] -= rot_poke;
    printf("CMD: Moving hexapod axis -u by %f deg\n", rot_poke);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"hex move v",10)){
    sm_p->hex[4] += rot_poke;
    printf("CMD: Moving hexapod axis v by %f deg\n", rot_poke);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"hex move -v",11)){
    sm_p->hex[4] -= rot_poke;
    printf("CMD: Moving hexapod axis -v by %f deg\n", rot_poke);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"hex move w",10)){
    sm_p->hex[5] += rot_poke;
    printf("CMD: Moving hexapod axis w by %f deg\n", rot_poke);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"hex move -w",11)){
    sm_p->hex[5] -= rot_poke;
    printf("CMD: Moving hexapod axis -w by %f deg\n", rot_poke);
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

  // HEX Calmodes
  if(!strncasecmp(line,"hex calmode 0",13)){
    sm_p->hex_calmode=0;
    printf("CMD: Changed HEX calibration mode to %d\n",sm_p->hex_calmode);
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"hex calmode 1",13)){
    sm_p->hex_calmode=1;
    printf("CMD: Changed HEX calibration mode to %d\n",sm_p->hex_calmode);
    return(CMD_NORMAL);
  }

  if(!strncasecmp(line,"hex calmode 2",13)){
    sm_p->hex_calmode=2;
    printf("CMD: Changed HEX calibration mode to %d\n",sm_p->hex_calmode);
    return(CMD_NORMAL);
  }

  //HEX Calibration
  if(!strncasecmp(line,"shk calibrate hex",17)){
    printf("CMD: Running HEX AXS calibration\n");
    printf("  -- Disabling PID\n");
    //Turn off gains
    sm_p->shk_kP = 0;
    sm_p->shk_kI = 0;
    sm_p->shk_kD = 0;
    //printf("  -- Resetting SHK\n");
    //sm_p->shk_reset = 1;
    sleep(3);

    //Start data recording
    printf("  -- Starting data recording\n");
    sm_p->w[DIAID].launch = getshk_proc;
    sm_p->w[DIAID].run    = 1;
    sleep(3);
    //Start probe pattern
    sm_p->hex_calmode=2;
    printf("  -- Changing HEX calibration mode to %d\n",sm_p->hex_calmode);
    while(sm_p->hex_calmode == 2)
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

  //SHK Commands
  if(!strncasecmp(line,"shk set origin",14)){
    printf("CMD: Setting SHK origin\n");
    //Turn off gains
    //printf("  -- Disabling PID\n");
    //sm_p->shk_kP = 0;
    //sm_p->shk_kI = 0;
    //sm_p->shk_kD = 0;
    //printf("  -- Resetting SHK\n");
    //sm_p->shk_reset = 1;
    sleep(1);
     sm_p->shk_setorigin=1;
    return(CMD_NORMAL);
  }

  //SHK Gain
  if(!strncasecmp(line,"shk gain ",9) && strlen(line)>10){
    if(!strncasecmp(line+9,"5",1)){
      sm_p->shk_kP = SHK_KP_DEFAULT*1.0;
      sm_p->shk_kI = SHK_KI_DEFAULT*1.0;
      sm_p->shk_kD = SHK_KD_DEFAULT*1.0;
      printf("SHK switching to gain 5: %f, %f, %f\n",sm_p->shk_kP,sm_p->shk_kI,sm_p->shk_kD);
      return CMD_NORMAL;
    }
    if(!strncasecmp(line+9,"4",1)){
      sm_p->shk_kP = SHK_KP_DEFAULT*0.8;
      sm_p->shk_kI = SHK_KI_DEFAULT*0.8;
      sm_p->shk_kD = SHK_KD_DEFAULT*0.8;
      printf("SHK switching to gain 4: %f, %f, %f\n",sm_p->shk_kP,sm_p->shk_kI,sm_p->shk_kD);
      return CMD_NORMAL;
    }
    if(!strncasecmp(line+9,"3",1)){
      sm_p->shk_kP = SHK_KP_DEFAULT*0.6;
      sm_p->shk_kI = SHK_KI_DEFAULT*0.6;
      sm_p->shk_kD = SHK_KD_DEFAULT*0.6;
      printf("SHK switching to gain 3: %f, %f, %f\n",sm_p->shk_kP,sm_p->shk_kI,sm_p->shk_kD);
      return CMD_NORMAL;
    }
    if(!strncasecmp(line+9,"2",1)){
      sm_p->shk_kP = SHK_KP_DEFAULT*0.4;
      sm_p->shk_kI = SHK_KI_DEFAULT*0.4;
      sm_p->shk_kD = SHK_KD_DEFAULT*0.4;
      printf("SHK switching to gain 2: %f, %f, %f\n",sm_p->shk_kP,sm_p->shk_kI,sm_p->shk_kD);
      return CMD_NORMAL;
    }
    if(!strncasecmp(line+9,"1",1)){
      sm_p->shk_kP = SHK_KP_DEFAULT*0.2;
      sm_p->shk_kI = SHK_KI_DEFAULT*0.2;
      sm_p->shk_kD = SHK_KD_DEFAULT*0.2;
      printf("SHK switching to gain 1: %f, %f, %f\n",sm_p->shk_kP,sm_p->shk_kI,sm_p->shk_kD);
      return CMD_NORMAL;
    }
    if(!strncasecmp(line+9,"0",1)){
      sm_p->shk_kP = 0;
      sm_p->shk_kI = 0;
      sm_p->shk_kD = 0;
      printf("SHK switching to gain 0: %f, %f, %f\n",sm_p->shk_kP,sm_p->shk_kI,sm_p->shk_kD);
      return CMD_NORMAL;
    }
  }

  //HEX Gain
  if(!strncasecmp(line,"hex gain ",9) && strlen(line)>10){
    if(!strncasecmp(line+9,"5",1)){
      sm_p->hex_kP = HEX_KP_DEFAULT/1;
      printf("HEX switching to gain 5: %f\n",sm_p->hex_kP);
      return CMD_NORMAL;
    }
    if(!strncasecmp(line+9,"4",1)){
      sm_p->hex_kP = HEX_KP_DEFAULT*0.8;
      printf("HEX switching to gain 4: %f\n",sm_p->hex_kP);
      return CMD_NORMAL;
    }
    if(!strncasecmp(line+9,"3",1)){
      sm_p->hex_kP = HEX_KP_DEFAULT*0.6;
      printf("HEX switching to gain 3: %f\n",sm_p->hex_kP);
      return CMD_NORMAL;
    }
    if(!strncasecmp(line+9,"2",1)){
      sm_p->hex_kP = HEX_KP_DEFAULT*0.4;
      printf("HEX switching to gain 2: %f\n",sm_p->hex_kP);
      return CMD_NORMAL;
    }
    if(!strncasecmp(line+9,"1",1)){
      sm_p->hex_kP = HEX_KP_DEFAULT*0.2;
      printf("HEX switching to gain 1: %f\n",sm_p->hex_kP);
      return CMD_NORMAL;
    }
    if(!strncasecmp(line+9,"0",1)){
      sm_p->hex_kP = 0;
      printf("HEX switching to gain 0: %f\n",sm_p->hex_kP);
      return CMD_NORMAL;
    }
  }


  //return with command not found
  return(CMD_NOT_FOUND);
}
