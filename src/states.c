#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>

/* piccflight headers */
#include "controller.h"
#include "watchdog.h"

/*************************************************
 * CHANGE_STATE
 *  - Change state and setup system settings
 *************************************************/
void change_state(sm_t *sm_p, int state){
  int i;
  uint8 procrun[NCLIENTS] = PROCRUN;
  
  //Set process run flags -- do not override defaults
  for(i=0;i<NCLIENTS;i++)
    sm_p->w[i].run = procrun[i] && sm_p->state_array[state].proc_run[i];
  
  //Change state
  sm_p->state = state;
}

/*************************************************
 * INIT_STATE
 *  - Define state settings
 *************************************************/
void init_state(int state_number, state_t *state){
  int i;
   
  //Clear state
  memset(state,0,sizeof(state_t));

  //Set defaults
  state->hex_commander = -1;
  state->alp_commander = -1;
  state->bmc_commander = -1;
  state->shk.fit_zernikes = 1;
  state->lyt.fit_zernikes = 1;
  
  //Enable all processes by default
  for(i=0;i<NCLIENTS;i++)
    state->proc_run[i] = 1;

  //Turn off ACQ camera by default
  state->proc_run[ACQID] = 0;

  //STATE_STANDBY
  if(state_number == STATE_STANDBY){
    //Set name
    sprintf(state->name,"STATE_STANDBY");
    //Set cmd
    sprintf(state->cmd,"stb");
    //SHK Settings
    state->shk.fit_zernikes = 1;
    //LYT Settings
    state->lyt.fit_zernikes = 1;
    //HEX Commander
    state->hex_commander = WATID;
    //ALP Commander
    state->alp_commander = WATID;
    return;
  }

  //STATE_LOW_POWER
  if(state_number == STATE_LOW_POWER){
    //Set name
    sprintf(state->name,"STATE_LOW_POWER");
    //Set cmd
    sprintf(state->cmd,"lpw");
    //Disable camera procs
    state->proc_run[SHKID] = 0;
    state->proc_run[LYTID] = 0;
    state->proc_run[SCIID] = 0;
    state->proc_run[ACQID] = 0;
    //HEX Commander
    state->hex_commander = WATID;
    //ALP Commander
    state->alp_commander = WATID;
    return;
  }
  
  //STATE_ACQUIRE_TARGET
  if(state_number == STATE_ACQUIRE_TARGET){
    //Set name
    sprintf(state->name,"STATE_ACQUIRE_TARGET");
    //Set cmd
    sprintf(state->cmd,"acq");
    //HEX Commander
    state->hex_commander = WATID;
    //ALP Commander
    state->alp_commander = WATID;
    //Configure Cameras
    state->proc_run[SHKID] = 0;
    state->proc_run[LYTID] = 0;
    state->proc_run[SCIID] = 0;
    state->proc_run[ACQID] = 1;
    return;
  }

  
  //STATE_SHK_HEX_ALIGN
  if(state_number == STATE_SHK_HEX_ALIGN){
    //Set name
    sprintf(state->name,"STATE_SHK_HEX_ALIGN");
    //Set cmd
    sprintf(state->cmd,"sha");
    //SHK Settings
    state->shk.fit_zernikes = 1;
    state->shk.zernike_control[0] = ACTUATOR_HEX;
    state->shk.zernike_control[1] = ACTUATOR_HEX;
    state->shk.zernike_control[2] = ACTUATOR_HEX;
    state->shk.zernike_control[3] = ACTUATOR_HEX;
    state->shk.zernike_control[4] = ACTUATOR_HEX;
    //Set SHKID as hex commander
    state->hex_commander = SHKID;
    return;
  }

  //STATE_M2_ALIGN
  if(state_number == STATE_M2_ALIGN){
    //Set name
    sprintf(state->name,"STATE_M2_ALIGN");
    //Set cmd
    sprintf(state->cmd,"m2a");
    //SHK Settings
    state->shk.fit_zernikes = 1;
    state->shk.zernike_control[0] = ACTUATOR_ALP;
    state->shk.zernike_control[1] = ACTUATOR_ALP;
    state->shk.zernike_control[2] = ACTUATOR_HEX;
    state->shk.zernike_control[3] = ACTUATOR_HEX;
    state->shk.zernike_control[4] = ACTUATOR_HEX;
    //Set SHKID as HEX and ALP commander
    state->hex_commander = SHKID;
    state->alp_commander = SHKID;
    //Offload ALP tilt to hexapod
    state->shk.alp_zernike_offload[0] = ACTUATOR_HEX;
    state->shk.alp_zernike_offload[1] = ACTUATOR_HEX;
    return;
  }

  //STATE_SHK_HEX_CALIBRATE
  if(state_number == STATE_SHK_HEX_CALIBRATE){
    //Set name
    sprintf(state->name,"STATE_SHK_HEX_CALIBRATE");
    //Set cmd
    sprintf(state->cmd,"shc");
    //SHK Settings
    state->shk.fit_zernikes = 1;
    //Set SHKID as hex commander
    state->hex_commander = SHKID;
    return;
  }

  //STATE_SHK_ALP_CALIBRATE
  if(state_number == STATE_SHK_ALP_CALIBRATE){
    //Set name
    sprintf(state->name,"STATE_SHK_ALP_CALIBRATE");
    //Set cmd
    sprintf(state->cmd,"sac");
    //SHK Settings
    state->shk.fit_zernikes = 1;
    //Set SHKID as alp commander
    state->alp_commander = SHKID;
    return;
  }

  //STATE_SHK_ZERN_LOWFC
  if(state_number == STATE_SHK_ZERN_LOWFC){
    //Set name
    sprintf(state->name,"STATE_SHK_ZERN_LOWFC");
    //Set cmd
    sprintf(state->cmd,"szc");
    //Set SHKID as alp commander
    state->alp_commander = SHKID;
    //SHK Settings
    state->shk.fit_zernikes = 1;
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      state->shk.zernike_control[i] = ACTUATOR_ALP;

    return;
  }

  //STATE_SHK_CELL_LOWFC
  if(state_number == STATE_SHK_CELL_LOWFC){
    //Set name
    sprintf(state->name,"STATE_SHK_CELL_LOWFC");
    //Set cmd
    sprintf(state->cmd,"scc");
    //Set SHKID as alp commander
    state->alp_commander = SHKID;
    //SHK Settings
    state->shk.fit_zernikes = 1;
    state->shk.cell_control = ACTUATOR_ALP;
    return;
  }

  //STATE_LYT_ALP_CALIBRATE
  if(state_number == STATE_LYT_ALP_CALIBRATE){
    //Set name
    sprintf(state->name,"STATE_LYT_ALP_CALIBRATE");
    //Set cmd
    sprintf(state->cmd,"lac");
    //LYT Settings
    state->lyt.fit_zernikes = 1;
    //Set LYTID as alp commander
    state->alp_commander = LYTID;
    return;
  }

  //STATE_LYT_ZERN_LOWFC
  if(state_number == STATE_LYT_ZERN_LOWFC){
    //Set name
    sprintf(state->name,"STATE_LYT_ZERN_LOWFC");
    //Set cmd
    sprintf(state->cmd,"lzc");
    //Set LYTID as alp commander
    state->alp_commander = LYTID;
    //LYT Settings
    state->lyt.fit_zernikes = 1;
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      state->lyt.zernike_control[i] = ACTUATOR_ALP;
    return;
  }
  
  //STATE_LYT_TT_LOWFC
  if(state_number == STATE_LYT_TT_LOWFC){
    //Set name
    sprintf(state->name,"STATE_LYT_TT_LOWFC");
    //Set cmd
    sprintf(state->cmd,"ltt");
    //Set LYTID as alp commander
    state->alp_commander = LYTID;
    //LYT Settings
    state->lyt.fit_zernikes = 1;
    state->lyt.zernike_control[0] = ACTUATOR_ALP;
    state->lyt.zernike_control[1] = ACTUATOR_ALP;
    return;
  }

  //STATE_SCI_BMC_CALIBRATE
  if(state_number == STATE_SCI_BMC_CALIBRATE){
    //Set name
    sprintf(state->name,"STATE_SCI_BMC_CALIBRATE");
    //Set cmd
    sprintf(state->cmd,"sbc");
    //Set SCIID as bmc commander
    state->bmc_commander = SCIID;
    return;
  }

  //STATE_SCI_DARK_HOLE
  if(state_number == STATE_SCI_DARK_HOLE){
    //Set name
    sprintf(state->name,"STATE_SCI_DARK_HOLE");
    //Set cmd
    sprintf(state->cmd,"sdh");
    return;
  }
}
