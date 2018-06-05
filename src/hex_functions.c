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
/* HEX_INIT                                                   */
/*  - Set hex command to default                              */
/**************************************************************/
void hex_init(hex_t *hex){
  int i;
  const double hexdef[HEX_NAXES] = HEX_POS_DEFAULT;
  for(i=0;i<HEX_NAXES;i++)
    hex->axis_cmd[i] = hexdef[i];
}

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
int hex_zern2hex(hex_t *hex){
  FILE *fp=NULL;
  char matrix_file[MAX_FILENAME];
  static int init=0;
  static double zern2hex_matrix[LOWFS_N_HEX_ZERNIKE*HEX_NAXES]={0};
  static double astig2tilt_matrix[LOWFS_N_HEX_ZERNIKE*HEX_NAXES]={0};
  double zernike_cmd[LOWFS_N_HEX_ZERNIKE];
  uint64 fsize,rsize;
  int c,i;


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

    /* Open astig2tilt matrix file */
    //--setup filename
    sprintf(matrix_file,ASTIG2TILT_FILE);
    //--open file
    if((fp = fopen(matrix_file,"r")) == NULL){
      printf("astig2tilt file\n");
      perror("fopen");
      return 1;
    }
    //--check file size
    fseek(fp, 0L, SEEK_END);
    fsize = ftell(fp);
    rewind(fp);
    rsize = (LOWFS_N_HEX_ZERNIKE)*HEX_NAXES*sizeof(double);
    if(fsize != rsize){
      printf("HEX: incorrect astig2tilt matrix file size %lu != %lu\n",fsize,rsize);
      return 1;
    }
    //--read matrix
    if(fread(astig2tilt_matrix,(LOWFS_N_HEX_ZERNIKE)*HEX_NAXES*sizeof(double),1,fp) != 1){
      printf("astig2tilt file\r");
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
    zernike_cmd[i] = hex->zernike_cmd[i];
      
  //Do Matrix Multiply
  num_dgemv(zern2hex_matrix, zernike_cmd, hex->axis_cmd, HEX_NAXES, LOWFS_N_HEX_ZERNIKE);
  
  return 0;
}

/**************************************************************/
/* HEX_CALIBRATE                                              */
/*  - Run hexapod calibration routines                        */
/**************************************************************/
int hex_calibrate(int calmode, hex_t *hex, int reset){
  int i;
  double hexdef[6]  = HEX_POS_DEFAULT;
  static struct timespec start,this,last,delta;
  static unsigned long int countA=0, countB=0, countC=0;
  static int init=0;
  time_t t;
  double dt=0;
  double axes[HEX_NAXES];

  /* Reset */
  if(reset){
    countA=0;
    countB=0;
    countC=0;
    init=0;
    return calmode;
  }

  /* Initialize */
  if(!init){
    countA=0;
    countB=0;
    countC=0;
    clock_gettime(CLOCK_REALTIME, &start);
    init=1;
  }
  /* Calculate times */
  clock_gettime(CLOCK_REALTIME, &this);
  if(timespec_subtract(&delta,&this,&start))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
    
  /* HEX_CALMODE_NONE: Do nothing. Just reset counters  */
  if(calmode == HEX_CALMODE_NONE){
    countA=0;
    countB=0;
    countC=0;
    return calmode;
  }
  
  /* HEX_CALMODE_POKE: Move through axes one at a time. */
  /*                   Go home in between each move.    */
  if(calmode == HEX_CALMODE_POKE){
    if((countA >= 0 && countA < (2*HEX_NAXES*HEX_NCALIM)) || (countA/HEX_NCALIM) % 2 == 0){
      //move hexapod to default position.
      for(i=0;i<HEX_NAXES;i++){
        hex->axis_cmd[i]=hexdef[i];
      }
      //move one axis
      if((countA/HEX_NCALIM) % 2 == 1){
        if((countB/HEX_NCALIM) % HEX_NAXES <=2){
          hex->axis_cmd[(countB/HEX_NCALIM) % HEX_NAXES] += HEX_TRL_POKE;
        }else{
          hex->axis_cmd[(countB/HEX_NCALIM) % HEX_NAXES] += HEX_ROT_POKE;
        }
        countB++;
      }
      countA++;
    }
    else{
      //Turn off calibration
      printf("HEX: Stopping HEX calmode %d\n",HEX_CALMODE_POKE);
      for(i=0;i<HEX_NAXES;i++)
        hex->axis_cmd[i] = hexdef[i];
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
    double start[HEX_NAXES] = HEX_POS_DEFAULT;

    if((countC) <= max_step){
      u_step = countC * spiral_radius * cos(countC * (PI/180.0));
      v_step = countC * spiral_radius * sin(countC * (PI/180.0));
      hex->axis_cmd[3] = start[3] + 0.005 + u_step;
      hex->axis_cmd[4] = start[4] + 0.005 + v_step;
      if(countC == 0){
        sleep(1);
      }
      if((countC % 20)==0){
        printf("Searching... %lu\n", countC/20);
      }
      countC++;
    }else{
      //Turn off calibration
      printf("HEX: Stopping HEX calmode %d\n",HEX_CALMODE_SPIRAL);
      calmode = HEX_CALMODE_SPIRAL;
      countC = 0;
    }
  }
  return calmode;
}
