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

/* TCOR Calibration */
#define HEX_TCOR_DVDX -0.1
#define HEX_TCOR_DUDY  0.1
#define HEX_TCOR_DUDZ -0.021214635

/**************************************************************/
/* HEX_INIT_CALMODE                                           */
/*  - Initialize HEX calmode structure                        */
/**************************************************************/
void hex_init_calmode(int calmode, calmode_t *hex){
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
  //HEX_CALMODE_TCOR
  if(calmode == HEX_CALMODE_TCOR){
    sprintf(hex->name,"HEX_CALMODE_TCOR");
    sprintf(hex->cmd,"tcor");
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
/* HEX_INIT                                                   */
/*  - Initialize Hexapod                                      */
/**************************************************************/
int hex_init(int *hexfd){
  int bFlag,i;
  double pivot[3]   = {HEX_PIVOT_X,HEX_PIVOT_Y,HEX_PIVOT_Z};
  
  /* Connect to Hexapod */
  if((*hexfd=hex_connect()) < 0){
    printf("HEX: hex_connect error!\n");
    return 1;
  }
  
  /* Reference Hexapod */
  if(hex_reference(*hexfd, 0)){
    printf("HEX: hex_reference error!\n");
    return 1;
  }
  
  /* Wait for Referencing to Finish */
  for(i=0;i<HEX_REF_TIMEOUT;i++){
    bFlag = 0;
    if(!PI_IsControllerReady(*hexfd, &bFlag)){
      printf("HEX: PI_IsControllerReady error!\n");
      return 1;
    }
    if(bFlag) break;
    //Sleep 1 second
    sleep(1);
  }
  if(i==HEX_REF_TIMEOUT){
    printf("HEX: Referencing timeout!!\n");
    return 1;
  }else{
    printf("HEX: Referencing complete after %d seconds\n",i);
  }
  
  /* Set Pivot Point*/
  if(hex_setpivot(*hexfd, pivot)){
    printf("HEX: hex_setpivot error!\n");
    return 1;
  }
  
  return 0;
}

/**************************************************************/
/* HEX_GET_COMMAND                                            */
/* - Function to get the last command sent to the HEX         */
/* - Use atomic operations to prevent two processes from      */
/*   accessing the commands at the same time                  */
/**************************************************************/
void hex_get_command(sm_t *sm_p, hex_t *cmd){
  //Atomically test and set HEX command lock using GCC built-in function
  while(__sync_lock_test_and_set(&sm_p->hex_command_lock,1));
  //Copy command
  memcpy(cmd,(hex_t *)&sm_p->hex_command,sizeof(hex_t));
  //Release lock
  __sync_lock_release(&sm_p->hex_command_lock);
}

/**************************************************************/
/* HEX_SEND_COMMAND                                           */
/* - Function to command the HEX                              */
/* - Use atomic operations to prevent two processes from      */
/*   sending commands at the same time                        */
/* - Return 1 if the command was sent and 0 if it wasn't      */
/**************************************************************/
int hex_send_command(sm_t *sm_p, hex_t *cmd, int proc_id){
  int retval=0;

  //Atomically test and set HEX command lock using GCC built-in function
  if(__sync_lock_test_and_set(&sm_p->hex_command_lock,1)==0){

    //Check if the commanding process is the HEX commander
    if(proc_id == sm_p->state_array[sm_p->state].hex_commander){
      
      //Send the command
      if(hex_move(sm_p->hexfd,cmd->axis_cmd) == 0){
	//Copy command to current position
	memcpy((hex_t *)&sm_p->hex_command,cmd,sizeof(hex_t));
	//Set return value
	retval = 1;
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

  //Check if hexapod is moving
  if(!PI_IsMoving(id,chkaxis, &bIsMoving)){
    printf("HEX: PI_IsMoving error!\n");
    return 1;
  }
  if(bIsMoving)
    return 1;

  //Convert telescope to hexapod coordinates
  hex_scope2hex(pos, result);

  //Send command to move hexapod
  if(!PI_MOV(id, axes_all, result)){
    printf("HEX: (hex_move) Error: %i\r\n", PI_GetError(0));
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

/**************************************************************/
/* HEX_GETPOS                                                 */
/*  - Get position of all hexapod axes                        */
/**************************************************************/
int hex_getpos(int id, double *pos){
  const char axes_all[13] = HEX_AXES_ALL;
  if (!PI_qPOS(id,axes_all, pos))
    {
      printf("HEX: PI_qPOS error!\n");
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

  if(!PI_SPI(id, axes_piv, pivot)){
    printf("HEX: PI_SPI error %d\n", PI_GetError(0));
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
  double dVdZ0 = -0.000532325;
  double dUdZ1 =  0.000566154;
  double dXdZ4 =  0.64/0.3;
  double dYdZ3 = -0.64/0.2;
  double dZdZ2 =  0.1;
  double dVdX  = -0.1;
  double dUdY  =  0.1; 
  double dZdZ1 = 0.0277399; //mm/zern micron
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
  const double tcor[HEX_NAXES]={HEX_X_CAL_TCOR,HEX_Y_CAL_TCOR,HEX_Z_CAL_TCOR,HEX_U_CAL_TCOR,HEX_V_CAL_TCOR,HEX_W_CAL_TCOR};
  int iax;
  int ncalim=0;

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
	hex->axis_cmd[iax] += poke[iax];
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

  /* HEX_CALMODE_TCOR:      Move through axes one at a time. */
  /*                        Use tilt correction.             */
  /*                        Go home in between each move.    */
  if(calmode == HEX_CALMODE_TCOR){
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
      memcpy(hex,&hex_start[HEX_CALMODE_TCOR],sizeof(hex_t));
	
      if((countA[calmode]/ncalim) % 2 == 1){
	//move one axis
	iax = (countB[calmode]/ncalim) % HEX_NAXES;
	hex->axis_cmd[iax] += tcor[iax];
	//correct tilt
	if(iax == HEX_AXIS_X)
	  hex->axis_cmd[HEX_AXIS_V] += HEX_TCOR_DVDX * tcor[iax];
	if(iax == HEX_AXIS_Y)
	  hex->axis_cmd[HEX_AXIS_U] += HEX_TCOR_DUDY * tcor[iax];
	if(iax == HEX_AXIS_Z)
	  hex->axis_cmd[HEX_AXIS_U] += HEX_TCOR_DUDZ * tcor[iax];
	countB[calmode]++;
      }
      countA[calmode]++;
    }
    else{
      //Set hex back to starting position
      memcpy(hex,&hex_start[calmode],sizeof(hex_t));
      mode_init[calmode]=0;
      //Turn off calibration
      printf("HEX: Stopping HEX calmode HEX_CALMODE_TCOR\n");
      calmode = HEX_CALMODE_NONE;
      init = 0;
    }
    return calmode;
  }

  /* HEX_CALMODE_SPIRAL: Spiral Search. Tip/tilt hexapod axes in a spiral. */

  if(calmode == HEX_CALMODE_SPIRAL){
    double u_step;
    double v_step;
    int max_step = 5000;
    double spiral_radius = 0.00001;
    //Save hex starting position
    if(!mode_init[calmode]){
      memcpy(&hex_start[calmode],hex,sizeof(hex_t));
      mode_init[calmode]=1;
    }
    //Set counter
    *step = countA[calmode];

    //Run spiral search
    if((countA[calmode]) < max_step){
      u_step = countA[calmode] * spiral_radius * cos(countA[calmode] * (PI/180.0));
      v_step = countA[calmode] * spiral_radius * sin(countA[calmode] * (PI/180.0));
      hex->axis_cmd[HEX_AXIS_U] = hex_start[HEX_CALMODE_SPIRAL].axis_cmd[HEX_AXIS_U] + 0.005 + u_step;
      hex->axis_cmd[HEX_AXIS_V] = hex_start[HEX_CALMODE_SPIRAL].axis_cmd[HEX_AXIS_V] + 0.005 + v_step;
      if((countA[calmode] % 20)==0)
	printf("HEX: Searching... %lu\n", countA[calmode]);
	
      countA[calmode]++;
    }else{
      //Set hex back to starting position
      memcpy(hex,&hex_start[calmode],sizeof(hex_t));
      mode_init[calmode]=0;
      //Turn off calibration
      printf("HEX: Stopping HEX calmode HEX_CALMODE_SPIRAL\n");
      calmode = HEX_CALMODE_NONE;
      init=0;
    }
    return calmode;
  }
  
  //Return calmode
  return calmode;
}




