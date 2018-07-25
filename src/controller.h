/***********************************************
 * Main Header for PICTURE-C Flight Software
 ***********************************************/
#include <stdint.h>
#include <time.h>
#include <dm7820_library.h>

#ifndef _CONTROLLER
#define _CONTROLLER

/*************************************************
 * Macros
 *************************************************/
#define MAX(A,B)   ((A) > (B) ? (A) : (B))
#define MIN(A,B)   ((A) < (B) ? (A) : (B))
#define ABS(A)     ((A) >= (0) ? (A) : (-A))
#define SIGN(A)    ((A) > (0) ? (1) : (-1))
#define FLOOR(A)   ((A) > (0) ? (int)(A) : (int)(A)-1)
#define CEIL(A)    ((A) < (0) ? (int)(A) : (int)(A)+1)
#define swap_s(A)  ((((uint16)(A) & 0xff00) >> 8) | \
                   (((uint16)(A) & 0x00ff) << 8))
#define swap_l(A)  ((((uint32)(A) & 0xff000000) >> 24) | \
                   (((uint32)(A) & 0x00ff0000) >> 8)  | \
                   (((uint32)(A) & 0x0000ff00) << 8)  | \
                   (((uint32)(A) & 0x000000ff) << 24))
#define SQ(A)      ((A)*(A))
#define istrue(e)  ((e) != 0)
#ifndef FALSE
#define FALSE      0
#endif
#ifndef TRUE
#define TRUE       1
#endif
#define packf(f) (pack754((f), 32, 8))
#define packd(f) (pack754((f), 64, 11))
#define unpackf(i) (unpack754((i), 32, 8))
#define unpackd(i) (unpack754((i), 64, 11))


/*************************************************
 * Number Typdef
 *************************************************/
typedef uint64_t uint64;
typedef int64_t int64;
typedef uint32_t uint32;
typedef int32_t int32;
typedef uint16_t uint16;
typedef int16_t int16;
typedef uint8_t uint8;
typedef int8_t int8;

/*************************************************
 * Numbers
 *************************************************/
#define ONE_BILLION  1000000000
#define ONE_MILLION  1000000
#define ONE_THOUSAND 1000
#define PI           3.141592653589793
#define TWOPI        6.28318530717959
#define FOURPI       (4.0*PI)
#define FIFTYONEPI   (51.0*PI)
#define RAD2AS       206264.81
#define LAMBDA       0.600         // Central Wavelength [microns]
#define PHASE_RAD2NM (LAMBDA*1000./TWOPI)
#define MAX_FILENAME 128

/*************************************************
 * Process ID Numbers
 *************************************************/
enum procids {WATID, SCIID, SHKID, LYTID, TLMID, ACQID, MOTID, THMID, SRVID, HEXID, DIAID, NCLIENTS};

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
	      STATE_SHK_HEX_CALIBRATE,
	      STATE_SHK_ALP_CALIBRATE,
	      STATE_SHK_ZERN_LOWFC,
	      STATE_SHK_CELL_LOWFC,
	      STATE_LYT_LOWFC,
	      STATE_SCI_DARK_HOLE,
	      NSTATES};

/*************************************************
 * Commands
 *************************************************/
#define CMD_SENDDATA  0x0ABACABB

/*************************************************
* Actuator Enable Switches
*************************************************/
#define ALP_ENABLE      1 // ALPAO DM
#define BMC_ENABLE      0 // BMC DM
#define HEX_ENABLE      1 // Hexapod
#define WSP_ENABLE      0 // WASP
#define LED_ENABLE      0 // LED
#define HTR_ENABLE      0 // Heaters
#define MOT_ENABLE      0 // Motors

/*************************************************
 * Actuator IDs
 *************************************************/
#define ACTUATOR_ALP 1
#define ACTUATOR_HEX 2
#define ACTUATOR_BMC 3
#define ACTUATOR_WSP 4
#define ACTUATOR_LED 5
#define ACTUATOR_HTR 6
#define ACTUATOR_MOT 7

/*************************************************
* Software Switches
*************************************************/
#define PUT_SOMETHING_HERE 4

/*************************************************
 * Fake Mode Bits
 *************************************************/
#define FAKE_IMAGES          1<<0 // Fake images
#define FAKE_TM_TEST_PATTERN 1<<1 // Send TM test pattern
#define FAKE_GEN             1<<2 // 1:Generate fake images, 0:Read from disk
#define FAKE_TIMER           1<<3 // 1:Sync to timer, 0:Sync to Interrupts

/*************************************************
 * LOWFS Status
 *************************************************/
#define LOWFS_NOT_LOCKED  0
#define LOWFS_LOCKED      1
#define LOWFS_NO_DATA     2
#define LOWFS_OVERRIDE    3
#define LOWFS_OFF         4
#define LOWFS_FAKE        5

/*************************************************
 * Files
 *************************************************/
#define DARKFILE_SCI      "config/darkframe.sci.%3.3d.%3.3d.%2.2dC.dat"
#define FLATFILE_SCI      "config/flatframe.sci.%3.3d.dat"
#define FAKEFILE_SCI      "config/fakeframe.sci.%3.3d.%s.dat"
#define CELLS2ALP_FILE    "config/shk2alp.dat"
#define CELLS2HEX_FILE    "config/shk2hex.dat"
#define ZERNIKE2HEX_FILE  "config/zern2hex.dat"
#define HEX2ZERNIKE_FILE  "config/hex2zern.dat"
#define ZERNIKE2ALP_FILE  "config/zern2alp.dat"
#define SHK2ZERNIKE_FILE  "config/shk2zern.dat"
#define SHK_CONFIG_FILE   "config/shk.cfg"
#define LYT_CONFIG_FILE   "config/lyt.cfg"
#define DATAPATH          "output/flight_data/folder_%5.5d/"
#define DATANAME          "output/flight_data/folder_%5.5d/picture.%s.%8.8d.dat"
#define SHK_HEX_CALFILE   "output/calibration/shk_hex_%s_caldata.dat"
#define SHK_ALP_CALFILE   "output/calibration/shk_alp_%s_caldata.dat"
#define SHK2ZERN_OUTFILE  "output/calibration/shk2zern_flight_output.dat"
#define ZERN2SHK_OUTFILE  "output/calibration/zern2shk_flight_output.dat"
#define SHK_OUTFILE       "output/calibraiton/shk_output.dat"


/*************************************************
 * Network Addresses & Ports
 *************************************************/
#define GSE_ADDR  "192.168.0.6"
#define GSE_PORT  "1337"
#define HOSTPORT  "ANY:24924"

/*************************************************
 * Circular Buffer Info
 *************************************************/
enum bufids {SCIEVENT, SCIFULL,
	     SHKEVENT, SHKFULL,
	     LYTEVENT, LYTFULL,
	     ACQEVENT, ACQFULL,
	     SHK_HEXSEND,  LYT_HEXSEND,
	     ACQ_HEXSEND,  WAT_HEXSEND,
	     SHK_HEXRECV,  LYT_HEXRECV,
	     ACQ_HEXRECV,  WAT_HEXRECV,
	     HEXRECV, NCIRCBUF};
#define SCIEVENTSIZE     3
#define SHKEVENTSIZE     3
#define LYTEVENTSIZE     3
#define ACQEVENTSIZE     3
#define SCIFULLSIZE      3
#define SHKFULLSIZE      3
#define LYTFULLSIZE      3
#define ACQFULLSIZE      3
#define HEXSENDSIZE      3
#define HEXRECVSIZE      3

/*************************************************
 * Define Errors
 *************************************************/
#define _ERROR	       -1
#define _NO_ERROR	0

/*************************************************
 * LOWFS Settings
 *************************************************/
#define LOWFS_N_ZERNIKE         23 //no piston
#define LOWFS_N_HEX_ZERNIKE     5  //no piston

/*************************************************
 * Zernike Errors
 *************************************************/
#define ZERNIKE_ERRORS_FILE   "data/zernike/zernike_errors.dat"
#define ZERNIKE_ERRORS_NUMBER 15000
#define ZERNIKE_ERRORS_PERIOD 0.00200000
#define ZERNIKE_ERRORS_NZERN  24

/*************************************************
 * Camera Settings -- Keep sizes divisible by 4 (packets)
 *************************************************/
#define SCIXS           128
#define SCIYS           128
#define SHKXS           1024
#define SHKYS           1024
#define LYTXS           16
#define LYTYS           16
#define ACQXS           1280
#define ACQYS           960


/*************************************************
 * Camera Full Image Times
 *************************************************/
#define SHK_FULL_IMAGE_TIME   0.5    //[seconds] period that full images are written to circbuf
#define LYT_FULL_IMAGE_TIME   0.5    //[seconds] period that full images are written to circbuf
#define SCI_FULL_IMAGE_TIME   0.5    //[seconds] period that full images are written to circbuf
#define ACQ_FULL_IMAGE_TIME   0.5    //[seconds] period that full images are written to circbuf

/*************************************************
 * Debug Messaging
 *************************************************/
#define WAT_DEBUG       0 // print wat messages
#define SCI_DEBUG       0 // print sci messages
#define SHK_DEBUG       0 // print shk messages
#define LYT_DEBUG       0 // print lyt messages
#define TLM_DEBUG       0 // print tlm messages
#define ACQ_DEBUG       0 // print acq messages
#define MOT_DEBUG       0 // print mot messages
#define THM_DEBUG       0 // print thm messages
#define SRV_DEBUG       0 // print srv messages
#define HEX_DEBUG       1 // print hex messages
#define DIA_DEBUG       0 // print dia messages

/*************************************************
 * Other Messaging
 *************************************************/
#define MSG_SAVEDATA    0 // print data saving messages
#define MSG_CTRLC       0 // print SIGINT messages

/*************************************************
 * Limits
 *************************************************/
#define GAIN_P_MIN     -1        //(.)   Minimum P gain
#define GAIN_P_MAX      0        //(.)   Maximum P gain
#define GAIN_I_MIN     -1        //(.)   Minimum I gain
#define GAIN_I_MAX      0        //(.)   Maximum I gain
#define GAIN_D_MIN     -1        //(.)   Minimum D gain
#define GAIN_D_MAX      0        //(.)   Maximum D gain

/*************************************************
 * BMC DM Parameters
 *************************************************/
#define BMC_NACT    952
#define BMC_STROKE  1.5
#define BMCXS        34
#define BMCYS        34
#define BMC_DMAX     ((1<<14) - 1)
#define BMC_DMIN     0
#define BMC_DMID     ((DM_DMIN+DM_DMAX)/2)

/*************************************************
 * ALPAO DM Parameters
 *************************************************/
#define ALP_NAME     "BAX197"
#define ALP_NACT     97
#define ALP_STROKE   2.0
#define ALPXS        10
#define ALPYS        10
#define ALP_DMAX     0.9
#define ALP_DMIN     -0.9
#define ALP_DMID     ((ALP_DMIN+ALP_DMAX)/2)
#define ALP_BIAS     0.0
#define ALP_POKE     0.05
#define ALP_ZPOKE    0.1 //zernike microns RMS
#define ALP_NCALIM   25  //number of calibration images to take per step

/*************************************************
 * HEXAPOD Parameters
 *************************************************/
#define HEX_DEVICE       "/dev/ttyS0"
#define HEX_BAUD         115200
#define HEX_NAXES        6
#define HEX_AXES_ALL     "X Y Z U V W"
#define HEX_AXES_PIV     "R S T"
#define HEX_AXIS_X       0
#define HEX_AXIS_Y       1
#define HEX_AXIS_Z       2
#define HEX_AXIS_U       3
#define HEX_AXIS_V       4
#define HEX_AXIS_W       5
#define HEX_POS_HOME     {0,0,0,0,0,0}
#define HEX_POS_DEFAULT  {-1.213673,3.527778,-0.157447,0.228300,0.437629,0.001699}
#define HEX_TRL_POKE      0.01
#define HEX_ROT_POKE      0.001
#define HEX_X_CAL_POKE    0.01
#define HEX_Y_CAL_POKE    0.01
#define HEX_Z_CAL_POKE    0.05
#define HEX_U_CAL_POKE    0.001
#define HEX_V_CAL_POKE    0.001
#define HEX_W_CAL_POKE    0.005
#define HEX_X_CAL_TCOR    0.5
#define HEX_Y_CAL_TCOR    0.5
#define HEX_Z_CAL_TCOR    0.05
#define HEX_U_CAL_TCOR    0.001
#define HEX_V_CAL_TCOR    0.001
#define HEX_W_CAL_TCOR    0.005
#define HEX_NCALIM        10
#define HEX_PIVOT_X       0//122.32031250
#define HEX_PIVOT_Y       0//206.61012268
#define HEX_PIVOT_Z       0//74.0
#define DEG_ROT_X         0.0 //deg
#define DEG_ROT_Y         0.0 //deg
#define DEG_ROT_Z         30.0 // deg
#define DEG2RAD           3.14159265 / 180.0000000
#define THETA_X           DEG_ROT_X * DEG2RAD
#define THETA_Y           DEG_ROT_Y * DEG2RAD
#define THETA_Z           DEG_ROT_Z * DEG2RAD
#define COS_X             cos(THETA_X)
#define SIN_X             sin(THETA_X)
#define COS_Y             cos(THETA_Y)
#define SIN_Y             sin(THETA_Y)
#define COS_Z             cos(THETA_Z)
#define SIN_Z             sin(THETA_Z)
#define HEX_REF_TIMEOUT   20 //seconds
#define HEX_PER_SHKEVENT  5  //number of shk images per HEX update
#define HEX_CMD_PER_SEC   2  //commands per second by hex_proc

/*************************************************
 * Shack-Hartmann (SHK) Settings
 *************************************************/
#define SHK_PX_PITCH_UM       5.5
#define SHK_XCELLS            16
#define SHK_YCELLS            16
#define SHK_NCELLS            256
#define SHK_LENSLET_PITCH_UM  300.0
#define SHK_FOCAL_LENGTH_UM   18600.0
#define SHK_BOX_DEADBAND      3      //[pixels] deadband radius for switching to smaller boxsize
#define SHK_MIN_BOXSIZE       (SHK_BOX_DEADBAND+1)
#define SHK_MAX_BOXSIZE       27     //[pixels] gives a 5 pixel buffer around edges
#define SHK_SPOT_UPPER_THRESH 200
#define SHK_SPOT_LOWER_THRESH 100
#define SHK_CELL_XOFF         67 //1px = 0.24 microns tilt
#define SHK_CELL_YOFF         74 
#define SHK_CELL_ROTATION     0.0
#define SHK_CELL_XSCALE       1.0
#define SHK_CELL_YSCALE       1.0
#define SHK_ORIGIN_NAVG       25
#define SHK_XMIN              0
#define SHK_XMAX              (SHKXS-1)
#define SHK_YMIN              0
#define SHK_YMAX              (SHKYS-1)
#define SHK_READ_MATRIX       1      //Read Zernike fitting matrix instead of building it

/*************************************************
 * Config Structure
 *************************************************/
typedef struct procinfo_struct{
  int   pid;
  uint8 run;
  uint8 die;
  uint8 done;
  uint8 res;
  uint32 chk; //# checkins
  uint32 rec; //# recipts of checkin
  uint8 cnt;
  uint8 tmo;
  uint8 ask;
  uint8 per;
  uint8 pri;
  char  *name;
  char  *mod;
  void (*launch)(void);
} procinfo_t;

/*************************************************
 * Calmode Structure
 *************************************************/
typedef struct calmode_struct{
  char name[128];
  char cmd[128];
} calmode_t;

/*************************************************
 * State Control Structures
 *************************************************/
// Shack-Hartmann Control (shk_proc.c)
typedef struct shkctrl_struct{
  int run_camera;
  int fit_zernikes;
  int pid_cells;
  int pid_zernikes;
  int zernike_control[LOWFS_N_ZERNIKE];
  int cell_control;
  int offload_tilt_to_hex;
  int offload_tilt_to_wasp;
} shkctrl_t;

// Lyot Sensor Control (lyt_proc.c)
typedef struct lytctrl_struct{
  int run_camera;
  int zernike_control[LOWFS_N_ZERNIKE];
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

//State Structure
typedef struct state_struct{
  char      name[128];
  char      cmd[128];
  int       hex_commander;
  int       alp_commander;
  int       bmc_commander;
  int       wsp_commander;
  shkctrl_t shk;
  lytctrl_t lyt;
  scictrl_t sci;
  acqctrl_t acq;
} state_t;

/*************************************************
 * Data Structures
 *************************************************/
typedef struct sci_struct{
  uint16 data[SCIXS][SCIYS];
} sci_t;

typedef struct shk_struct{
  uint16 data[SHKXS][SHKYS];
} shk_t;

typedef struct lyt_struct{
  uint16 data[LYTXS][LYTYS];
} lyt_t;

typedef struct acq_struct{
  uint16 data[ACQXS][ACQYS];
} acq_t;

typedef struct {
  //keep aligned on 8 byte boundary
  uint16 index;
  uint16 beam_select;
  uint16 spot_found;
  uint16 spot_captured;
  //-----
  uint32 maxpix;
  uint32 maxval;
  //-----
  uint16 blx;
  uint16 bly;
  uint16 trx;
  uint16 try;
  //-----
  double intensity;
  double background;
  double origin[2];
  double cenbox_origin[2];
  double centroid[2];
  double deviation[2];
  double command[2];
} shkcell_t;

/*************************************************
 * Device Command Structures
 *************************************************/
typedef struct alp_struct{
  double act_cmd[ALP_NACT];
  double zernike_cmd[LOWFS_N_ZERNIKE];
} alp_t;

typedef struct hex_struct{
  double axis_cmd[HEX_NAXES];
  double zernike_cmd[LOWFS_N_ZERNIKE];
} hex_t;

typedef struct bmc_struct{
  uint16 act_cmd[BMC_NACT];
} bmc_t;

typedef struct wsp_struct{
  double pitch_cmd;
  double yaw_cmd;
} wsp_t;


/*************************************************
 * Packet Header
 *************************************************/
typedef struct pktheader_struct{
  uint32  packet_type;
  uint32  frame_number;
  float   exptime;      //commanded exposure time
  float   ontime;       //measured exposure time
  float   temp;         //sensor temperature, if available
  uint32  imxsize;      //image x size [px]
  uint32  imysize;      //image y size [px]
  uint32  mode;         //camera mode
  uint32  state;        //system state
  uint32  dummy;        //8-byte alignment
  int64   start_sec;    //event start time
  int64   start_nsec;   //event start time
  int64   end_sec;      //event end time
  int64   end_nsec;     //event end time
} pkthed_t;

/*************************************************
 * Event Structures
 *************************************************/
typedef struct scievent_struct{
  pkthed_t hed;
  sci_t    image;
} scievent_t;

typedef struct shkevent_struct{
  pkthed_t  hed;
  uint32    beam_ncells;
  uint32    boxsize;
  uint32    hex_calmode;
  uint32    alp_calmode;
  double    xtilt;
  double    ytilt;
  double    kP_alp_cell;
  double    kI_alp_cell;
  double    kD_alp_cell;
  double    kP_alp_zern;
  double    kI_alp_zern;
  double    kD_alp_zern;
  double    kP_hex_zern;
  double    kI_hex_zern;
  double    kD_hex_zern;
  shkcell_t cells[SHK_NCELLS];
  double    zernike_measured[LOWFS_N_ZERNIKE];
  double    zernike_target[LOWFS_N_ZERNIKE];
  double    alp_zernike_delta[LOWFS_N_ZERNIKE];
  double    hex_zernike_delta[LOWFS_N_ZERNIKE];
  uint64    cal_step;
  hex_t     hex;
  alp_t     alp;
  wsp_t     wsp;
} shkevent_t;

typedef struct lytevent_struct{
  pkthed_t  hed;
  double    kP_zernike;
  double    kI_zernike;
  double    kD_zernike;
  double    zernike_measured[LOWFS_N_ZERNIKE];
  double    zernike_target[LOWFS_N_ZERNIKE];
  lyt_t     image;
  hex_t     hex;
  alp_t     alp;
  wsp_t     wsp;
} lytevent_t;

typedef struct acqevent_struct{
  pkthed_t  hed;
  hex_t     hex;
  wsp_t     wsp;
} acqevent_t;

typedef struct hexevent_struct{
  int    clientid;
  uint64 command_number;
  hex_t  hex;
} hexevent_t;

/*************************************************
 * Full Frame Structures
 *************************************************/
typedef struct scifull_struct{
  pkthed_t hed;
  sci_t    image;
} scifull_t;

typedef struct shkfull_struct{
  pkthed_t   hed;
  shk_t      image;
  shkevent_t shkevent;
} shkfull_t;

typedef struct lytfull_struct{
  pkthed_t hed;
  lyt_t    image;
} lytfull_t;

typedef struct acqfull_struct{
  pkthed_t hed;
  acq_t    image;
} acqfull_t;


/*************************************************
 * Circular Buffer Structure
 *************************************************/
typedef struct circbuf_struct{
  volatile void *buffer;
  uint32 read_offsets[NCLIENTS]; //last entry read
  uint32 write_offset; //last entry written
  uint32 nbytes;   //number of bytes in structure
  uint32 bufsize;  //number of structures in circular buffer
  char name[128];  //name of buffer
} circbuf_t;

/*************************************************
 * Shared Memory Layout
 *************************************************/
typedef volatile struct {

  //Runtime switches
  int memlock;        //Shared memory locking bit
  int die;            //Kill all processes

  //Process information
  procinfo_t w[NCLIENTS];

  //RTD board descriptor
  DM7820_Board_Descriptor* p_rtd_board;

  //State
  int state;                    //Current operational state
  state_t state_array[NSTATES]; //Array of states

  //Fake modes
  int tlm_fake_mode;        //Telemetry fake mode
  int sci_fake_mode;        //Science camera fake mode
  int lyt_fake_mode;        //Lyot LOWFS camera fake mode
  int shk_fake_mode;        //Shack-Hartmann camera fake mode
  int acq_fake_mode;        //Acquisition camera fake mode

  //Camera modes
  int sci_mode;        //Science camera mode
  int lyt_mode;        //Lyot LOWFS camera mode
  int shk_mode;        //Shack-Hartmann camera mode
  int acq_mode;        //Acquisition camera mode

  //ALP Calibration Mode
  int alp_calmode;

  //HEX Calibration Mode
  int hex_calmode;

  //Calibration file name
  char calfile[MAX_FILENAME];

  //Shack-Hartmann Settings
  int shk_boxsize;            //SHK centroid boxsize
  double shk_kP_alp_cell;     //SHK ALP cell gains
  double shk_kI_alp_cell;     //SHK ALP cell gains
  double shk_kD_alp_cell;     //SHK ALP cell gains
  double shk_kP_alp_zern;     //SHK ALP zernike gains
  double shk_kI_alp_zern;     //SHK ALP zernike gains
  double shk_kD_alp_zern;     //SHK ALP zernike gains
  double shk_kP_hex_cell;     //SHK HEX cell gains
  double shk_kI_hex_cell;     //SHK HEX cell gains
  double shk_kD_hex_cell;     //SHK HEX cell gains
  double shk_kP_hex_zern;     //SHK HEX zernike gains
  double shk_kI_hex_zern;     //SHK HEX zernike gains
  double shk_kD_hex_zern;     //SHK HEX zernike gains

  //Reset Commands
  int shk_reset;
  int acq_reset;
  int sci_reset;
  int lyt_reset;

  //Other Commands
  int hex_getpos;
  int shk_setorigin;
  int hex_tilt_correct;

  //Zernike Targets
  double zernike_target[LOWFS_N_ZERNIKE];

  //Events circular buffers
  scievent_t scievent[SCIEVENTSIZE];
  shkevent_t shkevent[SHKEVENTSIZE];
  lytevent_t lytevent[LYTEVENTSIZE];
  acqevent_t acqevent[ACQEVENTSIZE];
  hexevent_t hexrecv[HEXRECVSIZE];
  hexevent_t shk_hexsend[HEXSENDSIZE];
  hexevent_t lyt_hexsend[HEXSENDSIZE];
  hexevent_t acq_hexsend[HEXSENDSIZE];
  hexevent_t wat_hexsend[HEXSENDSIZE];
  hexevent_t shk_hexrecv[HEXRECVSIZE];
  hexevent_t lyt_hexrecv[HEXRECVSIZE];
  hexevent_t acq_hexrecv[HEXRECVSIZE];
  hexevent_t wat_hexrecv[HEXRECVSIZE];

  //Full frame circular buffers
  scifull_t scifull[SCIFULLSIZE];
  shkfull_t shkfull[SHKFULLSIZE];
  lytfull_t lytfull[LYTFULLSIZE];
  acqfull_t acqfull[ACQFULLSIZE];

  //Circular buffer package
  circbuf_t circbuf[NCIRCBUF];

} sm_t;


#endif
