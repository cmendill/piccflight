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
  state->wsp_commander = -1;

  //STATE_STANDBY
  if(state_number == STATE_STANDBY){
    //Set name
    sprintf(state->name,"STATE_STANDBY");
    //Set cmd
    sprintf(state->cmd,"stb");
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
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
    //Config Cameras
    state->shk.run_camera = 0;
    state->lyt.run_camera = 0;
    state->sci.run_camera = 0;
    state->acq.run_camera = 0;
    return;
  }

  //STATE_LED_LOCATE
  if(state_number == STATE_LED_LOCATE){
    //Set name
    sprintf(state->name,"STATE_LED_LOCATE");
    //Set cmd
    sprintf(state->cmd,"led");
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
    //Tell ACQ to locate LED
    state->acq.locate_led = 1;
    return;
  }

  //STATE_HEX_MANUAL_CONTROL
  if(state_number == STATE_HEX_MANUAL_CONTROL){
    //Set name
    sprintf(state->name,"STATE_HEX_MANUAL_CONTROL");
    //Set cmd
    sprintf(state->cmd,"hmc");
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
    //Set WATID as hex commander
    state->hex_commander = WATID;
    //Enable SHK zernike fitting
    state->shk.fit_zernikes = 1;
    return;
  }

  //STATE_HEX_DEFAULT_HOME
  if(state_number == STATE_HEX_DEFAULT_HOME){
    //Set name
    sprintf(state->name,"STATE_HEX_DEFAULT_HOME");
    //Set cmd
    sprintf(state->cmd,"hdh");
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
    //Set ACQID as hex commander
    state->hex_commander = ACQID;
    //Tell ACQ to set hex to default home
    state->acq.hex_default_home = 1;

    return;
  }

  //STATE_HEX_THERMAL_HOME
  if(state_number == STATE_HEX_THERMAL_HOME){
    //Set name
    sprintf(state->name,"STATE_HEX_THERMAL_HOME");
    //Set cmd
    sprintf(state->cmd,"hth");
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
    //Set ACQID as hex commander
    state->hex_commander = ACQID;
    //Tell ACQ to set hex to thermal home
    state->acq.hex_thermal_home = 1;
    return;
  }

  //STATE_HEX_SPIRAL_SEARCH
  if(state_number == STATE_HEX_SPIRAL_SEARCH){
    //Set name
    sprintf(state->name,"STATE_HEX_SPIRAL_SEARCH");
    //Set cmd
    sprintf(state->cmd,"hss");
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
    //Set ACQID as hex commander
    state->hex_commander = ACQID;
    //Tell ACQ to run hex spiral search
    state->acq.hex_spiral_search = 1;
    return;
  }

  //STATE_HEX_CAPTURE_TARGET
  if(state_number == STATE_HEX_CAPTURE_TARGET){
    //Set name
    sprintf(state->name,"STATE_HEX_CAPTURE_TARGET");
    //Set cmd
    sprintf(state->cmd,"hct");
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
    //Define ACQID as hex controller
    state->hex_commander = ACQID;
    //Tell ACQ to capture target
    state->acq.hex_capture_target = 1;
    return;
  }

  //STATE_M2_ALIGN
  if(state_number == STATE_M2_ALIGN){
    //Set name
    sprintf(state->name,"STATE_M2_ALIGN");
    //Set cmd
    sprintf(state->cmd,"m2a");
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
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

  //STATE_SHK_HEX_CALIBRATE
  if(state_number == STATE_SHK_HEX_CALIBRATE){
    //Set name
    sprintf(state->name,"STATE_SHK_HEX_CALIBRATE");
    //Set cmd
    sprintf(state->cmd,"shc");
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
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
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
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
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
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
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
    //Set SHKID as alp commander
    state->alp_commander = SHKID;
    //SHK Settings
    state->shk.fit_zernikes = 1;
    state->shk.cell_control = 1;
    return;
  }

  //STATE_LYT_ALP_CALIBRATE
  if(state_number == STATE_LYT_ALP_CALIBRATE){
    //Set name
    sprintf(state->name,"STATE_LYT_ALP_CALIBRATE");
    //Set cmd
    sprintf(state->cmd,"lac");
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
    //LYT Settings
    state->lyt.fit_zernikes = 1;
    //Set LYTID as alp commander
    state->alp_commander = LYTID;
    return;
  }

  //STATE_LYT_LOWFC
  if(state_number == STATE_LYT_LOWFC){
    //Set name
    sprintf(state->name,"STATE_LYT_LOWFC");
    //Set cmd
    sprintf(state->cmd,"lwc");
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
    //Set LYTID as alp commander
    state->alp_commander = LYTID;
    //LYT Settings
    state->lyt.fit_zernikes = 1;
    for(i=0;i<9;i++)
      state->lyt.zernike_control[i] = ACTUATOR_ALP;
    return;
  }

  //STATE_SCI_DARK_HOLE
  if(state_number == STATE_SCI_DARK_HOLE){
    //Set name
    sprintf(state->name,"STATE_SCI_DARK_HOLE");
    //Set cmd
    sprintf(state->cmd,"sdh");
    //Config Cameras
    state->shk.run_camera = 1;
    state->lyt.run_camera = 1;
    state->sci.run_camera = 1;
    state->acq.run_camera = 1;
    return;
  }
}
