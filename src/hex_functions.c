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
#include "hex_functions.h"

/* Error messages */
#define PI_ERR_LENGTH 128

/**************************************************************/
/* HEX_INIT_CALMODE                                           */
/*  - Initialize HEX calmode structure                        */
/**************************************************************/
void hex_init_calmode(int calmode, calmode_t *hex){
  //DEFAULTS
  hex->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD;

  //HEX_CALMODE_NONE
  if(calmode == HEX_CALMODE_NONE){
    sprintf(hex->name,"HEX_CALMODE_NONE");
    sprintf(hex->cmd,"none");
    hex->shk_boxsize_cmd = SHK_BOXSIZE_CMD_STD; 
  }
  //HEX_CALMODE_POKE
  if(calmode == HEX_CALMODE_POKE){
    sprintf(hex->name,"HEX_CALMODE_POKE");
    sprintf(hex->cmd,"poke");
    hex->shk_boxsize_cmd = SHK_BOXSIZE_CMD_MAX; 
  }
  //HEX_CALMODE_SPIRAL
  if(calmode == HEX_CALMODE_SPIRAL){
    sprintf(hex->name,"HEX_CALMODE_SPIRAL");
    sprintf(hex->cmd,"spiral");
    hex->shk_boxsize_cmd = SHK_BOXSIZE_CMD_MAX; 
  }
}

/**************************************************************/
/* HEX_CONNECT                                                */
/*  - Open connection to hexapod                              */
/**************************************************************/
int hex_connect(void){
  return PI_ConnectRS232ByDevName(HEX_DEVICE,HEX_BAUD);
}

/**************************************************************/
/* HEX_DISCONNECT                                             */
/*  - Close connection to hexapod                             */
/**************************************************************/
void hex_disconnect(int id){
  PI_CloseConnection(id);
}

/**************************************************************/
/* HEX_GET_ERROR                                              */
/*  - Get PI error code and statement                         */
/**************************************************************/
void hex_get_error(int id){
  char msg[PI_ERR_LENGTH];
  PI_TranslateError(PI_GetError(id),msg,PI_ERR_LENGTH);
  printf("HEX: Error: %s\n",msg);
}

/**************************************************************/
/* HEX_INIT                                                   */
/*  - Initialize Hexapod                                      */
/**************************************************************/
int hex_init(int *hexfd){
  int bFlag,i;
  char msg[PI_ERR_LENGTH];
  double hexzero[HEX_NAXES] = {0,0,0,0,0,0};
  double pivot[3]  = {HEX_PIVOT_X,HEX_PIVOT_Y,HEX_PIVOT_Z};
  char *chkaxis=""; //will check all axes
  int bIsMoving=0;

  /* Connect to Hexapod */
  if((*hexfd=hex_connect()) < 0){
    printf("HEX: hex_connect error!\n");
    return 1;
  }

  /* Clear Error State */
  PI_TranslateError(PI_GetError(*hexfd),msg,PI_ERR_LENGTH);
  printf("HEX: Initial error state: %s\n",msg);
   
  /* Reference Hexapod */
  if(hex_reference(*hexfd, 0)){
    printf("HEX: hex_reference error!\n");
    return 0;
  }
  
  /* Wait for Referencing to Finish */
  for(i=0;i<HEX_REF_TIMEOUT;i++){
    bFlag = 0;
    if(!PI_IsControllerReady(*hexfd, &bFlag)){
      PI_TranslateError(PI_GetError(*hexfd),msg,PI_ERR_LENGTH);
      printf("HEX: PI_IsControllerReady error: %s\n",msg);
      return 0;
    }
    if(bFlag) break;
    //Sleep 1 second
    sleep(1);
  }
  if(i==HEX_REF_TIMEOUT){
    printf("HEX: Referencing timeout!!\n");
    return 0;
  }else{
    printf("HEX: Referencing complete after %d seconds\n",i);
  }

  /* Go to home (all zeros) position */
  if(hex_move(*hexfd, hexzero)){
    PI_TranslateError(PI_GetError(*hexfd),msg,PI_ERR_LENGTH);
    printf("HEX: hex_move error: %s\n",msg);
    return 0;
  }

  /* Wait for hexapod to stop moving */
  for(i=0;i<HEX_MOVE_TIMEOUT;i++){
    printf("HEX: Waiting for hexapod to stop moving...\n");
    if(!PI_IsMoving(*hexfd,chkaxis, &bIsMoving)){
      PI_TranslateError(PI_GetError(*hexfd),msg,PI_ERR_LENGTH);
      printf("HEX: PI_IsMoving error: %s\n",msg);
      return 0;
    }
    if(!bIsMoving) break;
    sleep(1);
  }
  if(i==HEX_MOVE_TIMEOUT)
    printf("HEX: Wait for hexapod to stop moving timeout\n");
  printf("HEX: Stopped\n");
  
  /* Set Pivot Point*/
  if(hex_setpivot(*hexfd, pivot)){
    printf("HEX: hex_setpivot error!\n");
    return 0;
  }
  printf("HEX: Pivot point set\n");
  
  return 0;
}

/**************************************************************/
/* HEX_GET_COMMAND                                            */
/* - Function to get the last command sent to the HEX         */
/* - Use atomic operations to prevent two processes from      */
/*   accessing the commands at the same time                  */
/**************************************************************/
int hex_get_command(sm_t *sm_p, hex_t *cmd){
  int retval = 1;
  
  //Atomically test and set HEX command lock using GCC built-in function
  if(__sync_lock_test_and_set(&sm_p->hex_command_lock,1)==0){
    //Copy command
    memcpy(cmd,(hex_t *)&sm_p->hex_command,sizeof(hex_t));
    //Release lock
    __sync_lock_release(&sm_p->hex_command_lock);
    //Return 0 on success
    retval = 0;
  }

  //Return
  return retval;
}

/**************************************************************/
/* HEX_SEND_COMMAND                                           */
/* - Function to command the HEX                              */
/* - Use atomic operations to prevent two processes from      */
/*   sending commands at the same time                        */
/* - Return 0 if the command was sent and 1 if it wasn't      */
/**************************************************************/
int hex_send_command(sm_t *sm_p, hex_t *cmd, int proc_id){
  int retval = 1;
  
  //Atomically test and set HEX command lock using GCC built-in function
  if(__sync_lock_test_and_set(&sm_p->hex_command_lock,1)==0){
    
    //Check if the commanding process is the HEX commander
    if(proc_id == sm_p->state_array[sm_p->state].hex_commander){
      
      //Send the command
      if(!hex_move(sm_p->hexfd,cmd->acmd)){
	//Copy command to current position
	memcpy((hex_t *)&sm_p->hex_command,cmd,sizeof(hex_t));
	//Set retval for good command
	retval = 0;
      }
    }  
    
    //Release lock
    __sync_lock_release(&sm_p->hex_command_lock);
  }

  //Return
  return retval;
}

/**************************************************************/
/* HEX_HEX2SCOPE                                              */
/*  - Convert from hexapod coords to scope coords             */
/**************************************************************/
int hex_hex2scope(double *position, double *result){
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

/**************************************************************/
/* HEX_SCOPE2HEX                                              */
/*  - Convert from scope coords to hexapod coords             */
/**************************************************************/
int hex_scope2hex(double *position, double *result){
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

/**************************************************************/
/* HEX_MOVE                                                   */
/*  - Move hexapod                                            */
/**************************************************************/
int hex_move(int id, double *pos){
  double result[6] = {0};
  const char axes_all[13] = HEX_AXES_ALL;
  char *chkaxis=""; //will check all axes
  int bIsMoving=0;
  char msg[PI_ERR_LENGTH];
  
  //Check if hexapod is moving
  if(!PI_IsMoving(id,chkaxis, &bIsMoving)){
    PI_TranslateError(PI_GetError(id),msg,PI_ERR_LENGTH);
    printf("HEX: PI_IsMoving error: %s\n",msg);
    return 1;
  }
  if(bIsMoving){
    printf("HEX: Hexapod is moving\n");
    return 1;
  }
  
  //Convert telescope to hexapod coordinates
  hex_scope2hex(pos, result);
  
  //Send command to move hexapod
  if(!PI_MOV(id, axes_all, result)){
    PI_TranslateError(PI_GetError(id),msg,PI_ERR_LENGTH);
    printf("HEX: PI_MOV error: %s\n",msg);
    return 1;
  }

  //Return 0 for successful command
  return 0;
}

/**************************************************************/
/* HEX_REFERENCE                                              */
/*  - Reference hexapod, if needed                            */
/**************************************************************/
int hex_reference(int id, int force){
  int bReferenced=0;
  char axis[] = "X";
  char msg[PI_ERR_LENGTH];
  
  //Check if HEX is already referenced
  if(!PI_qFRF(id, axis, &bReferenced)){
    PI_TranslateError(PI_GetError(id),msg,PI_ERR_LENGTH);
    printf("HEX: PI_qFRF error: %s\n",msg);
    return 1;
  }
  
  //Reference
  if(!bReferenced || force){
    printf("HEX: Referencing axis %s...\n\r",axis);
    if(!PI_FRF(id, axis)){
      PI_TranslateError(PI_GetError(id),msg,PI_ERR_LENGTH);
      printf("HEX: PI_FRF error: %s\n",msg);
      return 1;
    }
  }
  
  return 0;
}

/**************************************************************/
/* HEX_GETPOS                                                 */
/*  - Get position of all hexapod axes                        */
/**************************************************************/
int hex_getpos(int id, double *pos){
  const char axes_all[13] = HEX_AXES_ALL;
  char msg[PI_ERR_LENGTH];
  
  if (!PI_qPOS(id,axes_all, pos))
    {
      PI_TranslateError(PI_GetError(id),msg,PI_ERR_LENGTH);
      printf("HEX: PI_qPOS error: %s\n",msg);
      return 1;
    }
  
  return 0;
}

/**************************************************************/
/* HEX_PRINTPOS                                               */
/*  - Get position of all hexapod axes and print them out     */
/**************************************************************/
int hex_printpos(int id){
  double hexpos[6]={0};
  double scopepos[6]={0};
  if(hex_getpos(id,hexpos)){
    printf("HEX: hex_getpos error!\n");
    return 1;
  }
  //Convert to telescope coordinates
  hex_hex2scope(hexpos, scopepos);
  //Print position
  printf("HEX: X = %f \n",scopepos[0]);
  printf("HEX: Y = %f \n",scopepos[1]);
  printf("HEX: Z = %f \n",scopepos[2]);
  printf("HEX: U = %f \n",scopepos[3]);
  printf("HEX: V = %f \n",scopepos[4]);
  printf("HEX: W = %f \n",scopepos[5]);
  printf("HEX: {%f,%f,%f,%f,%f,%f}\n",scopepos[0],scopepos[1],scopepos[2],scopepos[3],scopepos[4],scopepos[5]);

  return 0;
}


/**************************************************************/
/* HEX_SETPIVOT                                               */
/*  - Set pivot point for hexapod rotation                    */
/**************************************************************/
int hex_setpivot(int id, double *pivot){
  const char axes_piv[7] = "R S T";
  char msg[PI_ERR_LENGTH];
  
  if(!PI_SPI(id, axes_piv, pivot)){
    PI_TranslateError(PI_GetError(id),msg,PI_ERR_LENGTH);
    printf("HEX: PI_SPI error %s\n",msg);
    return 1;
  }
  return 0;
}

/**************************************************************/
/* HEX_ZERN2HEX                                               */
/*  - Convert from zernike commands to hexapod commands       */
/**************************************************************/
int hex_zern2hex(double *zernikes, double *axes){
  FILE *fp=NULL;
  char matrix_file[MAX_FILENAME];
  static int init=0;
  static double zern2hex_matrix[LOWFS_N_HEX_ZERNIKE*HEX_NAXES]={0};
  uint64 fsize,rsize;
  int c,i;
  double hex_zernikes[LOWFS_N_HEX_ZERNIKE]={0};


  if(!init){
    /* Open zern2hex matrix file */
    //--setup filename
    sprintf(matrix_file,SHKZER2HEXACT_FILE);
    //--open file
    if((fp = fopen(matrix_file,"r")) == NULL){
      printf("zern2hex file\n");
      perror("fopen");
      return 1;
    }
    //--check file size
    fseek(fp, 0L, SEEK_END);
    fsize = ftell(fp);
    rewind(fp);
    rsize = (LOWFS_N_HEX_ZERNIKE)*HEX_NAXES*sizeof(double);
    if(fsize != rsize){
      printf("HEX: incorrect HEX matrix file size %lu != %lu\n",fsize,rsize);
      return 1;
    }
    //--read matrix
    if(fread(zern2hex_matrix,(LOWFS_N_HEX_ZERNIKE)*HEX_NAXES*sizeof(double),1,fp) != 1){
      printf("zern2hex file\r");
      perror("fread");
      return 1;
    }
    //--close file
    fclose(fp);

    //--set init flag
    init=1;
  }

  //Select HEX zernike commands
  for(i=0;i<LOWFS_N_HEX_ZERNIKE;i++)
    hex_zernikes[i] = zernikes[i];
      
  //Do Matrix Multiply
  num_dgemv(zern2hex_matrix, hex_zernikes, axes, HEX_NAXES, LOWFS_N_HEX_ZERNIKE);
  
  return 0;
}

/**************************************************************/
/* HEX_ZERN2HEX_ALT                                           */
/*  - Convert from zernike commands to hexapod commands       */
/*  - Alternate Version                                       */
/**************************************************************/
void hex_zern2hex_alt(double *zernikes, double *axes){
  double dX=0,dY=0,dZ=0,dU=0,dV=0,dW=0;

  //Calibration
  double dVdZ0 = 0.00097870;    //+V --> +Z0
  double dUdZ1 = 0.00095131;    //+U --> +Z1
  double dXdZ4 = 2*(-0.64/0.3); //+X --> -Z4 (after correct with -V)
  double dYdZ3 = 2*(-0.64/0.2); //+Y --> -Z3 (after correct with +U)
  double dZdZ2 = 0.19390708;    //+Z --> +Z2
  double dZdZ1 = 0.05027623;    //+Z --> +Z1
  double dVdX  = -0.1; //+X == +V, Move V opposite X to fix
  double dUdY  =  0.1; //+Y == -U, Move U same as Y to fix
  double dUdZ  = -dUdZ1 / dZdZ1;
  
  //Convert astig to HEX dX & dY
  dX = dXdZ4 * zernikes[4];
  dY = dYdZ3 * zernikes[3];
  //--correct tilt for dX & dY
  dV = dVdX * dX;
  dU = dUdY * dY;

  //Convert focus to HEX dZ
  dZ = dZdZ2 * zernikes[2];
  //--tilt correction
  dU += dUdZ * dZ;
  
  //Convert tilt to dU & dV
  dV += dVdZ0 * zernikes[0];
  dU += dUdZ1 * zernikes[1];

  //Set final command
  axes[HEX_AXIS_X] = dX;
  axes[HEX_AXIS_Y] = dY;
  axes[HEX_AXIS_Z] = dZ;
  axes[HEX_AXIS_U] = dU;
  axes[HEX_AXIS_V] = dV;
  axes[HEX_AXIS_W] = dW;
  
}

/**************************************************************/
/* HEX_CALIBRATE                                              */
/*  - Run hexapod calibration routines                        */
/**************************************************************/
int hex_calibrate(int calmode, hex_t *hex, uint32_t *step, int procid, int reset){
  int i;
  const double hexdef[HEX_NAXES] = HEX_POS_DEFAULT;
  static struct timespec start,this,last,delta;
  static uint64 countA[HEX_NCALMODES] = {0};
  static uint64 countB[HEX_NCALMODES] = {0};
  static int init=0;
  static int mode_init[HEX_NCALMODES]={0};
  static hex_t hex_start[HEX_NCALMODES];
  time_t t;
  double dt=0;
  const double poke[HEX_NAXES]={HEX_X_CAL_POKE,HEX_Y_CAL_POKE,HEX_Z_CAL_POKE,HEX_U_CAL_POKE,HEX_V_CAL_POKE,HEX_W_CAL_POKE};
  int iax;
  int ncalim=0;
  static int leg,nu,nv,s,u,v;
  const double spiral_step = 0.03;//degrees

  /* Reset */
  if(reset){
    memset(countA,0,sizeof(countA));
    memset(countB,0,sizeof(countB));
    memset(mode_init,0,sizeof(mode_init));
    memset(hex_start,0,sizeof(hex_start));
    init=0;
    return calmode;
  }
  
  /* Initialize */
  if(!init){
    memset(countA,0,sizeof(countA));
    memset(countB,0,sizeof(countB));
    memset(mode_init,0,sizeof(mode_init));
    memset(hex_start,0,sizeof(hex_start));
    clock_gettime(CLOCK_REALTIME, &start);
    init=1;
  }

  /* Set calibration parameters */
  if(procid == SHKID){
    ncalim = HEX_SHK_NCALIM;
  }
  if(procid == LYTID){
    ncalim = HEX_LYT_NCALIM;
  }


  /* Calculate times */
  clock_gettime(CLOCK_REALTIME, &this);
  if(timespec_subtract(&delta,&this,&start))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);

  /* HEX_CALMODE_NONE: Do nothing. */
  if(calmode == HEX_CALMODE_NONE){
    return calmode;
  }
  
  /* HEX_CALMODE_POKE: Move through axes one at a time. */
  /*                   Go home in between each move.    */
  if(calmode == HEX_CALMODE_POKE){
    //Save hex starting position
    if(!mode_init[calmode]){
      memcpy(&hex_start[calmode],hex,sizeof(hex_t));
      mode_init[calmode]=1;
    }
    //Proceed with calibration
    if(countA[calmode] >= 0 && countA[calmode] < (2*HEX_NAXES*ncalim)){
      //Set calibration step
      *step = countA[calmode]/ncalim;
      //Set hex to starting position
      memcpy(hex,&hex_start[HEX_CALMODE_POKE],sizeof(hex_t));
	
      if((countA[calmode]/ncalim) % 2 == 1){
	//move one axis
	iax = (countB[calmode]/ncalim) % HEX_NAXES;
	hex->acmd[iax] += poke[iax];
	countB[calmode]++;
      }
      countA[calmode]++;
    }
    else{
      //Set hex back to starting position
      memcpy(hex,&hex_start[calmode],sizeof(hex_t));
      mode_init[calmode]=0;
      //Turn off calibration
      printf("HEX: Stopping HEX calmode HEX_CALMODE_POKE\n");
      calmode = HEX_CALMODE_NONE;
      init = 0;
    }
    return calmode;
  }

  /* HEX_CALMODE_SPIRAL: Spiral Search. Tip/tilt hexapod axes in a spiral. */
  if(calmode == HEX_CALMODE_SPIRAL){
    //Save hex starting position
    if(!mode_init[calmode]){
      memcpy(&hex_start[calmode],hex,sizeof(hex_t));
      //reset spiral
      u=0;
      v=0;
      nu=1;
      nv=1;
      s=0;
      leg=0;
      mode_init[calmode]=1;
    }
    
    //Set counter
    *step = countA[calmode];
    
    //Increment spiral search
    /*
      
       -4 -3 -2 -1  0  1  2  3  4 
     4 64 63 62 61 60 59 58 57 56  4
     3 65 36 35 34 33 32 31 30 55  3
     2 66 37 16 15 14 13 12 29 54  2
     1 67 38 17  4  3  2 11 28 53  1
  V  0 68 39 18  5  0  1 10 27 52  0
    -1 69 40 19  6  7  8  9 26 51 -1
    -2 70 41 20 21 22 23 24 25 50 -2
    -3 71 42 43 44 45 46 47 48 49 -3
    -4 72 73 74 75 76 77 78 79 80 -4
       -4 -3 -2 -1  0  1  2  3  4 
                    U
    */

    
    if(leg%4 == 0){
      //BL to BR
      u++;
      if(++s == nu){
	nu++;
	leg++;
	s=0;
      }
    }
    else if(leg%4 == 1){
      //BL to TR
      v++;
      if(++s == nv){
	nv++;
	leg++;
	s=0;
      }
    }
    else if(leg%4 == 2){
      //TR to TL
      u--;
      if(++s == nu){
	nu++;
	leg++;
	s=0;
      }
    }
    else if(leg%4 == 3){
      //TL to BL
      v--;
      if(++s == nv){
	nv++;
	leg++;
	s=0;
      }
    }

    //Set new position
    hex->acmd[HEX_AXIS_U] = hex_start[calmode].acmd[HEX_AXIS_U] + u*spiral_step;
    hex->acmd[HEX_AXIS_V] = hex_start[calmode].acmd[HEX_AXIS_V] + v*spiral_step;
    printf("HEX: Spiral %d: %d %d\n",countA[calmode]+1,u,v);

    //Increment counter
    countA[calmode]++;
   
    return calmode;
  }
  
  //Return calmode
  return calmode;
}




