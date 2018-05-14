#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <time.h>
#include <math.h>
#include <PI_GCS2_DLL.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "numeric.h"

/**************************************************************/
/*                      HEX_INIT                              */
/**************************************************************/
void hex_init(hex_t *hex){
  int i;
  double hexdef[HEX_NAXES] = HEX_POS_DEFAULT;
  for(i=0;i<HEX_NAXES;i++)
    hex->axs[i] = hexdef[i];
}

//Connect to hexapod
int hex_connect(void){
  return PI_ConnectRS232ByDevName(HEX_DEVICE,HEX_BAUD);
}

//Disconnect from hexapod
int hex_disconnect(hex_t *hex){
  PI_CloseConnection(0);
    printf("[hex]: Hexapod Disconnected.\r\n");
    return 0;
}

//Convert from Hexapod coords to scope coords
int hex2scope(double *position, double *result){
  int i,j,k;
  double rotation_matrix[9] = { COS_Y*COS_Z,                     -COS_Y*SIN_Z,                      SIN_Y,
                                SIN_X*SIN_Y*COS_Z + COS_X*SIN_Z, -SIN_X*SIN_Y*SIN_Z + COS_X*COS_Z, -SIN_X*COS_Y,
                               -COS_X*SIN_Y*COS_Z + SIN_X*SIN_Z,  COS_X*SIN_Y*SIN_Z + SIN_X*COS_Z,  COS_X*COS_Y  };

  for(i=0; i<6; i++){
    if(i<3){
      for(j=0; j<3; j++){
        result[i] += rotation_matrix[j+i*3] * position[j+0];
      }
    }else{
      k=i-3;
      for(j=0; j<3; j++){
        result[i] += rotation_matrix[j+k*3] * position[j+3];
      }
    }
  }
  return 0;
}

//Convert from scope to hexapod coords
int scope2hex(double *position, double *result){
  int i,j,k;
  double rotation_matrix[9] = { COS_Y*COS_Z,  SIN_X*SIN_Y*COS_Z + COS_X*SIN_Z, -COS_X*SIN_Y*COS_Z + SIN_X*SIN_Z,
                               -COS_Y*SIN_Z, -SIN_X*SIN_Y*SIN_Z + COS_X*COS_Z,  COS_X*SIN_Y*SIN_Z + SIN_X*COS_Z,
                                SIN_Y,       -SIN_X*COS_Y ,                     COS_X*COS_Y                      };

  for(i=0; i<6; i++){
    if(i<3){
      for(j=0; j<3; j++){
        result[i] += rotation_matrix[j+i*3] * position[j+0];
      }
    }else{
      k=i-3;
      for(j=0; j<3; j++){
        result[i] += rotation_matrix[j+k*3] * position[j+3];
      }
    }
  }
  return 0;
}

//Move hexapod
int hex_move(int id, double *pos){
  double result[6] = {0};
  scope2hex(pos, result);
  const char axes_all[13] = HEX_AXES_ALL;
  char *chkaxis=""; //will check all axes

  if(!PI_MOV(id, axes_all, result)){
    printf("HEX: PI_MOV error!\n");

    int err;
    err=PI_GetError(0);
    printf("Error: %i\r\n", err);
    return 1;
  }
  int bIsMoving = 1;
  while(bIsMoving){
    if(!PI_IsMoving(id,chkaxis, &bIsMoving)){
      printf("HEX: PI_IsMoving error!\n");
      return 1;
    }
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

//Obtain positions of all axes
int hex_getpos(int id, double *pos){
  const char axes_all[13] = HEX_AXES_ALL;
  if (!PI_qPOS(id,axes_all, pos))
    {
      printf("HEX: PI_qPOS error!\n");
      return 1;
    }
  return 0;
}

//Set pivot point
int hex_setpivot(int id, double *pivot){
  const char axes_piv[7] = "R S T";

  if(!PI_SPI(id, axes_piv, pivot)){
    int err;
    err=PI_GetError(0);
    printf("HEX: PI_SPI error!\n");
    printf("Error: %i\r\n", err);
    return 1;
  }
  return 0;
}

/**************************************************************/
/*                      HEX_CALIBRATE                         */
/**************************************************************/

int hex_calibrate(int calmode, hex_t *hex, int reset){
  int i;
  double hexdef[6]  = HEX_POS_DEFAULT;
  static struct timespec start,this,last,delta;
  static unsigned long int countA=0, countB=0;
  static int init=0;
  time_t t;
  double dt=0;
  double axs[HEX_NAXES];

  /* Reset */
  if(reset){
    countA=0;
    countB=0;
    init=0;
    return calmode;
  }

  /* Initialize */
  if(!init){
    countA=0;
    countB=0;
    clock_gettime(CLOCK_REALTIME, &start);
    init=1;
  }
    /* Calculate times */
    clock_gettime(CLOCK_REALTIME, &this);
    if(timespec_subtract(&delta,&this,&start))
      printf("SHK: shk_process_image --> timespec_subtract error!\n");
    ts2double(&delta,&dt);

  /* CALMODE 1: Be not zero.*/


  /* CALMODE 2: Move through axes one at a time.
  *              Go home in between each move.
  */
  if(calmode == 2){
    if((countA >= 0 && countA < (2*HEX_NAXES*HEX_NCALIM)) || (countA/HEX_NCALIM) % 2 == 0){
      //move hexapod to default position.
      for(i=0;i<HEX_NAXES;i++){
        hex->axs[i]=hexdef[i];
      }
      //move one axis
      if((countA/HEX_NCALIM) % 2 == 1){
        if((countB/HEX_NCALIM) % HEX_NAXES <=2){
          hex->axs[(countB/HEX_NCALIM) % HEX_NAXES] += HEX_TRL_POKE;
        }else{
          hex->axs[(countB/HEX_NCALIM) % HEX_NAXES] += HEX_ROT_POKE;
        }
        countB++;
      }
      countA++;
    }
    else{
      //Turn off calibration
      printf("HEX: Stopping HEX calmode 2\n");
      for(i=0;i<HEX_NAXES;i++)
        hex->axs[i] = hexdef[i];
      calmode = 0;
      init = 0;
    }
  return calmode;
  }
  return calmode;
}
