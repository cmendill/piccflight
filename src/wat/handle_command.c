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
#include "handle_command.h"
#include "../common/controller.h"

/* Report Fake Modes */
void report_fake_modes(void){
  printf("************************************************\n");
  printf("*                   FAKE MODES                 *\n");
  printf("************************************************\n");
  printf("0: Disabled\n");
  printf("1: TM test pattern\n");
  printf("2: Generate fake Shack-Hartmann images\n");
  printf("\n");
}

/* Handle user commands*/
int handle_command(char *line, sm_t *sm_p){
  float ftemp;
  int   itemp;
  char  stemp[CMD_MAX_LENGTH];

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

  //IWC Calibration
  if(!strncasecmp(line,"iwc calmode 0",13)){
    sm_p->iwc_calmode=0;
    printf("CMD: Changed IWC calibration mode to %d\n",sm_p->iwc_calmode);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"iwc calmode 1",13)){
    sm_p->iwc_calmode=1;
    printf("CMD: Changed IWC calibration mode to %d\n",sm_p->iwc_calmode);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"iwc calmode 2",13)){
    sm_p->iwc_calmode=2;
    printf("CMD: Changed IWC calibration mode to %d\n",sm_p->iwc_calmode);
    return(CMD_NORMAL);
  }
  
  //return with command not found
  return(CMD_NOT_FOUND);
}

