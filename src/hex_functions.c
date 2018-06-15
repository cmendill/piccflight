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

/**************************************************************/
/* HEX_INIT_CALMODE                                           */
/*  - Initialize HEX calmode structure                        */
/**************************************************************/
void hex_init_calmode(int calmode, calmode_t *hex){
  //HEX_CALMODE_NONE
  if(calmode == HEX_CALMODE_NONE){
    sprintf(hex->name,"HEX_CALMODE_NONE");
    sprintf(hex->cmd,"none");
  }
  //HEX_CALMODE_POKE
  if(calmode == HEX_CALMODE_POKE){
    sprintf(hex->name,"HEX_CALMODE_POKE");
    sprintf(hex->cmd,"poke");
  }
  //HEX_CALMODE_SPIRAL
  if(calmode == HEX_CALMODE_SPIRAL){
    sprintf(hex->name,"HEX_CALMODE_SPIRAL");
    sprintf(hex->cmd,"spiral");
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
/*  - Close connection to hexapod                              */
/**************************************************************/
void hex_disconnect(int id){
  PI_CloseConnection(id);
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

/**************************************************************/
/* HEX_MOVE                                                   */
/*  - Move hexapod                                            */
/**************************************************************/
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

/**************************************************************/
/* HEX_REFERENCE                                              */
/*  - Reference hexapod, if needed                            */
/**************************************************************/
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
/* HEX_SETPIVOT                                               */
/*  - Set pivot point for hexapod rotation                    */
/**************************************************************/
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
    sprintf(matrix_file,ZERNIKE2HEX_FILE);
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
int hex_zern2hex_alt(double *zernikes, double *axes){
  FILE *fp=NULL;
  char matrix_file[MAX_FILENAME];
  static int init=0;
  static double zern2hex_matrix[LOWFS_N_HEX_ZERNIKE*HEX_NAXES]={0};
  static double hex2zern_matrix[LOWFS_N_HEX_ZERNIKE*HEX_NAXES]={0};
  uint64 fsize,rsize;
  int c,i;
  double hex_zernikes[LOWFS_N_HEX_ZERNIKE]={0};
  double input_zernikes[LOWFS_N_HEX_ZERNIKE]={0};
  double dX=0,dY=0,dZ=0,dU=0,dV=0,dW=0;
  double hex_axes[HEX_NAXES];
  
  if(!init){
    /* Open zern2hex matrix file */
    //--setup filename
    sprintf(matrix_file,ZERNIKE2HEX_FILE);
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

    /* Open hex2zern matrix file */
    //--setup filename
    sprintf(matrix_file,ZERNIKE2HEX_FILE);
    //--open file
    if((fp = fopen(matrix_file,"r")) == NULL){
      printf("hex2zern file\n");
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
    if(fread(hex2zern_matrix,(LOWFS_N_HEX_ZERNIKE)*HEX_NAXES*sizeof(double),1,fp) != 1){
      printf("hex2zern file\r");
      perror("fread");
      return 1;
    }
    //--close file
    fclose(fp);

    //--set init flag
    init=1;
  }

  //Copy input zernikes
  for(i=0;i<LOWFS_N_HEX_ZERNIKE;i++)
    input_zernikes[i] = zernikes[i];

  //Convert astig to HEX dX & dY
  memset(hex_zernikes,0,sizeof(hex_zernikes));
  memset(hex_axes,0,sizeof(hex_axes));
  hex_zernikes[3] = input_zernikes[3];
  hex_zernikes[4] = input_zernikes[4];
  num_dgemv(zern2hex_matrix, hex_zernikes, hex_axes, HEX_NAXES, LOWFS_N_HEX_ZERNIKE);
  dX = hex_axes[HEX_AXIS_X];
  dY = hex_axes[HEX_AXIS_Y];
 
  //Subtract the tilt from dX and dY to the input error
  memset(hex_zernikes,0,sizeof(hex_zernikes));
  memset(hex_axes,0,sizeof(hex_axes));
  hex_axes[HEX_AXIS_X] = dX;
  hex_axes[HEX_AXIS_Y] = dY;
  num_dgemv(hex2zern_matrix, hex_axes, hex_zernikes, LOWFS_N_HEX_ZERNIKE, HEX_NAXES);
  input_zernikes[0] -= hex_zernikes[0];
  input_zernikes[1] -= hex_zernikes[1];
  
  //Convert input tip and tilt to HEX dU & dV
  memset(hex_zernikes,0,sizeof(hex_zernikes));
  memset(hex_axes,0,sizeof(hex_axes));
  hex_zernikes[0] = input_zernikes[0];
  hex_zernikes[1] = input_zernikes[1];
  num_dgemv(zern2hex_matrix, hex_zernikes, hex_axes, HEX_NAXES, LOWFS_N_HEX_ZERNIKE);
  dU = hex_axes[HEX_AXIS_U];
  dV = hex_axes[HEX_AXIS_V];

  //Convert power to HEX dZ
  memset(hex_zernikes,0,sizeof(hex_zernikes));
  memset(hex_axes,0,sizeof(hex_axes));
  hex_zernikes[2] = input_zernikes[2];
  num_dgemv(zern2hex_matrix, hex_zernikes, hex_axes, HEX_NAXES, LOWFS_N_HEX_ZERNIKE);
  dZ = hex_axes[HEX_AXIS_Z];

  //Set final command
  axes[HEX_AXIS_X] = dX;
  axes[HEX_AXIS_Y] = dY;
  axes[HEX_AXIS_Z] = dZ;
  axes[HEX_AXIS_U] = dU;
  axes[HEX_AXIS_V] = dV;
  axes[HEX_AXIS_W] = dW;
  
  return 0;
}

/**************************************************************/
/* HEX_CALIBRATE                                              */
/*  - Run hexapod calibration routines                        */
/**************************************************************/
int hex_calibrate(int calmode, hex_t *hex, uint64 *step, int reset){
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
    if(!mode_init[HEX_CALMODE_POKE]){
      memcpy(&hex_start[HEX_CALMODE_POKE],hex,sizeof(hex_t));
      mode_init[HEX_CALMODE_POKE]=1;
    }
    //Proceed with calibration
    if(countA[calmode] >= 0 && countA[calmode] < (2*HEX_NAXES*HEX_NCALIM)){
      //Set calibration step
      *step = countA[calmode]/HEX_NCALIM;
      //Set hex to starting position
      memcpy(hex,&hex_start[HEX_CALMODE_POKE],sizeof(hex_t));
	
      if((countA[calmode]/HEX_NCALIM) % 2 == 1){
	//move one axis
	iax = (countB[calmode]/HEX_NCALIM) % HEX_NAXES;
	hex->axis_cmd[iax] += poke[iax];
	countB[calmode]++;
      }
      countA[calmode]++;
    }
    else{
      //Turn off calibration
      printf("HEX: Stopping HEX calmode HEX_CALMODE_POKE\n");
      calmode = HEX_CALMODE_NONE;
      init = 0;
      //Set hex back to starting position
      memcpy(hex,&hex_start[HEX_CALMODE_POKE],sizeof(hex_t));
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
    if(!mode_init[HEX_CALMODE_SPIRAL]){
      memcpy(&hex_start[HEX_CALMODE_SPIRAL],hex,sizeof(hex_t));
      mode_init[HEX_CALMODE_SPIRAL]=1;
    }
      
    if((countA[calmode]) < max_step){
      u_step = countA[calmode] * spiral_radius * cos(countA[calmode] * (PI/180.0));
      v_step = countA[calmode] * spiral_radius * sin(countA[calmode] * (PI/180.0));
      hex->axis_cmd[HEX_AXIS_U] = hex_start[HEX_CALMODE_SPIRAL].axis_cmd[HEX_AXIS_U] + 0.005 + u_step;
      hex->axis_cmd[HEX_AXIS_V] = hex_start[HEX_CALMODE_SPIRAL].axis_cmd[HEX_AXIS_V] + 0.005 + v_step;
      if((countA[calmode] % 20)==0)
	printf("HEX: Searching... %lu\n", countA[calmode]);
	
      countA[calmode]++;
    }else{
      //Turn off calibration
      printf("HEX: Stopping HEX calmode HEX_CALMODE_SPIRAL\n");
      calmode = HEX_CALMODE_NONE;
      init=0;
      //Set hex back to starting position
      memcpy(hex,&hex_start[HEX_CALMODE_SPIRAL],sizeof(hex_t));
    }
    return calmode;
  }
  
  //Return calmode
  return calmode;
}
