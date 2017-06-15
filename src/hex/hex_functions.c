#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>

/* piccflight headers */
#include "PI_GCS2_DLL.h"
#include "../common/controller.h"

//Connect to hexapod
int hex_connect(void){
  return PI_ConnectRS232ByDevName(HEX_DEVICE,HEX_BAUD);
}

//Move hexapod
int hex_move(int id, double *pos){
  const char axes_all[13] = HEX_AXES_ALL;
  char *chkaxis=" "; //will check all axes
  
  if(!PI_MOV(id, axes_all, pos)){
    printf("HEX: PI_MOV error!\n");
    return 1;
  }
  int bIsMoving = 1;
  while(bIsMoving){
    if(!PI_IsMoving(id,chkaxis , &bIsMoving)){
      printf("HEX: PI_IsMoving error!\n");
      return 1;
    }
    usleep(1000);
  }
  return 0;
}

//Check to see if the hexapod needs referencing, and do so if needed
int hex_reference(int id,int force){
  int bReferenced;
  char axis[] = "X";
  if(!PI_qFRF(id, axis, &bReferenced)){
    printf("HEX: PI_qFRF error!\n");
    return 1;
  }
  if(!bReferenced || force){
    printf("HEX: Referencing axis %s...\n\r",axis);
    if(!PI_FRF(id, axis)){
      printf("HEX: PI_FRF error!\n");
      return 1;
    }
  }
  return 0;
}
