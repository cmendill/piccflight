#ifndef _STATES
#define _STATES

/*************************************************
 * States
 *************************************************/
enum states { STATE_STANDBY,
	      STATE_LOW_POWER,
	      STATE_LED_LOCATE,
	      STATE_HEX_MANUAL_CONTROL,
	      STATE_HEX_DEFAULT_HOME,
	      STATE_HEX_THERMAL_HOME,
	      STATE_HEX_SPIRAL_SEARCH,
	      STATE_HEX_CAPTURE_TARGET,
	      STATE_M2_ALIGN,
	      STATE_SHK_LOWFC,
	      STATE_LYT_LOWFC,
	      STATE_SCI_DARK_HOLE,
	      NSTATES};
  
/*************************************************
 * State Control Structures
 *************************************************/
// Shack-Hartmann Control (shk_proc.c)
typedef struct shkctrl_struct{
  int run_camera;
  int fit_zernikes;
  int pid_cells;
  int pid_zernikes;
  int all_zernikes_to_alp;
  int all_cells_to_alp;
  int all_zernikes_to_hex;
  int all_cells_to_hex;
  int only_tilt_to_alp;
  int only_tilt_to_hex;
  int only_astig_and_focus_to_hex;
  int offload_tilt_to_hex;
  int offload_tilt_to_wasp;
} shkctrl_t;
// Lyot Sensor Control (lyt_proc.c)
typedef struct lytctrl_struct{
  int run_camera;
  int all_zernikes_to_alp;
  int offload_tilt_to_hex;
  int offload_tilt_to_wasp;
} lytctrl_t;
// Science Camera Control (sci_proc.c)
typedef struct scictrl_struct{
  int run_camera;
  int sensing_bmc;
  int dig_dark_hole;
} scictrl_t;
// Acquisition Camera Control (acq_proc.c)
typedef struct acqctrl_struct{
  int run_camera;
  int locate_led;
  int hex_spiral_search;
  int hex_capture_target;
  int hex_thermal_home;
  int hex_default_home;
} acqctrl_t;
// User Command Control (handle_command.c)
typedef struct usrctrl_struct{
  int control_hex;
} usrctrl_t;
//State Structure
typedef struct state_struct{
  char      name[128]; 
  char      cmd[128];  
  shkctrl_t shk;        
  lytctrl_t lyt;
  scictrl_t sci;
  acqctrl_t acq;
  usrctrl_t usr;
} state_t;


/*************************************************
 * Function Prototypes
 *************************************************/
void init_state(int state_number, state_t *state);

#endif
