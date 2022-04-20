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
#include "bmc_functions.h"

/*************************************************
 * CHANGE_STATE
 *  - Change state and setup system settings
 *************************************************/
void change_state(sm_t *sm_p, int state){
  int i,ret;
  uint8 procrun[NCLIENTS] = PROCRUN;
  
  //Set process run flags -- do not override defaults or enable flags
  for(i=0;i<NCLIENTS;i++)
    sm_p->w[i].run = procrun[i] && sm_p->state_array[state].proc_run[i] && sm_p->w[i].ena;
  
  //Change state
  sm_p->state = state;

  /* State specific actions */

  //Turn off BMC HV in STATE_LOW_POWER
  if(state == STATE_LOW_POWER){
    if(sm_p->bmc_ready){
      if(sm_p->bmc_hv_enable){
	printf("WAT: Turning OFF BMC HV\n");
	// Set all actuators to Zero -- NOTE: This will fail in states where WATID is not the BMC commander
	if(bmc_zero_flat(sm_p,WATID))
	  printf("WAT: ERROR: bmc_zero_flat failed!\n");
 	// Stop BMC controller, turn OFF HV
	if((ret=libbmc_hv_off((libbmc_device_t *)&sm_p->libbmc_device)) < 0)
	  printf("WAT: Failed to stop BMC controller : %s - %s \n", libbmc_error_name(ret), libbmc_strerror(ret));
	else
	  sm_p->bmc_hv_on = 0;
      }
      else{
	printf("WAT: BMC HV not enabled\n");
      }
    }
    else{
      printf("WAT: BMC controller not ready\n");
    }
  }
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
  state->hex_commander = WATID;
  state->alp_commander = WATID;
  state->bmc_commander = WATID;
  state->tgt_commander = WATID;
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
    return;
  }
  
  //STATE_ACQUIRE_TARGET
  if(state_number == STATE_ACQUIRE_TARGET){
    //Set name
    sprintf(state->name,"STATE_ACQUIRE_TARGET");
    //Set cmd
    sprintf(state->cmd,"acq");
    //Configure Cameras
    state->proc_run[SHKID] = 0;
    state->proc_run[LYTID] = 0;
    state->proc_run[SCIID] = 1;
    state->proc_run[ACQID] = 1;
    return;
  }

  //STATE_SPIRAL_SEARCH
  if(state_number == STATE_SPIRAL_SEARCH){
    //Set name
    sprintf(state->name,"STATE_SPIRAL_SEARCH");
    //Set cmd
    sprintf(state->cmd,"sps");
    //HEX Commander
    state->hex_commander = ACQID;
    //Configure Cameras
    state->proc_run[SHKID] = 0;
    state->proc_run[LYTID] = 0;
    state->proc_run[SCIID] = 1;
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
    state->shk.cell_control = ACTUATOR_ALP;
    return;
  }

  //STATE_LYT_ALP_CALIBRATE
  if(state_number == STATE_LYT_ALP_CALIBRATE){
    //Set name
    sprintf(state->name,"STATE_LYT_ALP_CALIBRATE");
    //Set cmd
    sprintf(state->cmd,"lac");
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
    state->lyt.zernike_control[0] = ACTUATOR_ALP;
    state->lyt.zernike_control[1] = ACTUATOR_ALP;
    return;
  }

  //STATE_HYB_ZERN_LOWFC
  if(state_number == STATE_HYB_ZERN_LOWFC){
    //Set name
    sprintf(state->name,"STATE_HYB_ZERN_LOWFC");
    //Set cmd
    sprintf(state->cmd,"hzc");
    //Set LYTID as alp commander
    state->alp_commander = LYTID;
    //LYT Settings
    state->lyt.zernike_control[0] = ACTUATOR_ALP;
    state->lyt.zernike_control[1] = ACTUATOR_ALP;
    //SHK Settings
    for(i=2;i<LOWFS_N_ZERNIKE;i++)
      state->shk.zernike_control[i] = ACTUATOR_ALP;
    //Set SHK2LYT
    state->shk.shk2lyt=1;
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

  //STATE_HOWFS
  if(state_number == STATE_HOWFS){
    //Set name
    sprintf(state->name,"STATE_HOWFS");
    //Set cmd
    sprintf(state->cmd,"how");
    //Set options
    state->bmc_commander = SCIID;
    state->sci.run_howfs = 1;
    state->sci.run_efc   = 0;
    //Set LYTID as alp commander
    state->alp_commander = LYTID;
    //LYT Settings
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      state->lyt.zernike_control[i] = ACTUATOR_ALP;
    return;
  }

  //STATE_EFC
  if(state_number == STATE_EFC){
    //Set name
    sprintf(state->name,"STATE_EFC");
    //Set cmd
    sprintf(state->cmd,"efc");
    //Set options
    state->bmc_commander = SCIID;
    state->sci.run_howfs = 1;
    state->sci.run_efc   = 1;
    //Set LYTID as alp commander
    state->alp_commander = LYTID;
    //LYT Settings
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      state->lyt.zernike_control[i] = ACTUATOR_ALP;
    return;
  }

  //STATE_SHK_EFC
  if(state_number == STATE_SHK_EFC){
    //Set name
    sprintf(state->name,"STATE_SHK_EFC");
    //Set cmd
    sprintf(state->cmd,"sfc");
    //Set SCIID as bmc commander
    state->bmc_commander = SCIID;
    state->sci.run_howfs = 1;
    state->sci.run_efc   = 1;
    //Set SHKID as alp commander
    state->alp_commander = SHKID;
    //SHK Settings
    state->shk.cell_control = ACTUATOR_ALP;
    return;
  }


  //STATE_HYB_EFC
  if(state_number == STATE_HYB_EFC){
    //Set name
    sprintf(state->name,"STATE_HYB_EFC");
    //Set cmd
    sprintf(state->cmd,"hfc");
    //Set SCIID as bmc commander
    state->bmc_commander = SCIID;
    state->sci.run_howfs = 1;
    state->sci.run_efc   = 1;
    //Set LYTID as alp commander
    state->alp_commander = LYTID;
    //LYT Settings
    state->lyt.zernike_control[0] = ACTUATOR_ALP;
    state->lyt.zernike_control[1] = ACTUATOR_ALP;
    //SHK Settings
    for(i=2;i<LOWFS_N_ZERNIKE;i++)
      state->shk.zernike_control[i] = ACTUATOR_ALP;
    //Set SHK2LYT
    state->shk.shk2lyt=1;
    return;
  }

}
