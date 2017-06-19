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

  //SHK Zernike Fitting
  if(!strncasecmp(line,"shk zfit on",11)){
    sm_p->shk_fit_zernike=1;
    printf("CMD: Turned SHK Zernike fitting ON [%d]\n",sm_p->shk_fit_zernike);
    return(CMD_NORMAL);
  }
  if(!strncasecmp(line,"shk zfit off",12)){
    sm_p->shk_fit_zernike=0;
    printf("CMD: Turned SHK Zernike fitting OFF [%d]\n",sm_p->shk_fit_zernike);
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
  
  
  //return with command not found
  return(CMD_NOT_FOUND);
}

