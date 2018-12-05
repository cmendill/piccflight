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
#define MAX_COMMAND  32

/*************************************************
 * Process ID Numbers
 *************************************************/
enum procids {WATID, SCIID, SHKID, LYTID, TLMID, ACQID, MTRID, THMID, SRVID, DIAID, NCLIENTS};

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
	      STATE_LYT_ALP_CALIBRATE,
	      STATE_LYT_ZERN_LOWFC,
	      STATE_LYT_FULL_LOWFC,
	      STATE_SCI_BMC_CALIBRATE,
	      STATE_SCI_DARK_HOLE,
	      NSTATES};

/*************************************************
 * Commands
 *************************************************/
#define CMD_SENDDATA  0x0ABACABB

/*************************************************
* Instrument Input Type
*************************************************/
#define INPUT_TYPE_SINGLE_PASS   0 //Normal operation
#define INPUT_TYPE_DOUBLE_PASS   1 //Fed from PSI
#define INSTRUMENT_INPUT_TYPE    INPUT_TYPE_SINGLE_PASS

/*************************************************
* Enable Switches
*************************************************/
#define ALP_ENABLE      1 // ALPAO DM
#define BMC_ENABLE      0 // BMC DM
#define HEX_ENABLE      0 // Hexapod
#define WSP_ENABLE      0 // WASP
#define LED_ENABLE      0 // LED
#define HTR_ENABLE      0 // Heaters
#define MTR_ENABLE      0 // Motors
#define TLM_ENABLE      1 // Telemetry
#define DIO_ENABLE      1 // DIO ports

/*************************************************
 * Actuator IDs
 *************************************************/
#define ACTUATOR_ALP 1
#define ACTUATOR_HEX 2
#define ACTUATOR_BMC 3
#define ACTUATOR_WSP 4
#define ACTUATOR_LED 5
#define ACTUATOR_HTR 6
#define ACTUATOR_MTR 7

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
#define DARKFILE_SCI           "config/darkframe.sci.%3.3d.%3.3d.%2.2dC.dat"
#define FLATFILE_SCI           "config/flatframe.sci.%3.3d.dat"
#define FAKEFILE_SCI           "config/fakeframe.sci.%3.3d.%s.dat"
#define SHKCEL2ALPACT_FILE     "config/shkcel2alpact.dat"
#define SHKCEL2HEXACT_FILE     "config/shkcel2hexact.dat"
#define SHKZER2HEXACT_FILE     "config/shkzer2hexact.dat"
#define HEXACT2SHKZER_FILE     "config/hexact2shkzer.dat"
#define ALPACT2SHKZER_FILE     "config/alpact2shkzer.dat"
#define SHKZER2ALPACT_FILE     "config/shkzer2alpact.dat"
#define ALPZER2LYTPIX_FILE     "config/alpzer2lytpix.dat"
#define LYTPIX2ALPZER_FILE     "config/lytpix2alpzer.dat"
#define LYTPIX2ALPACT_FILE     "config/lytpix2alpact.dat"
#define LYTPIX2ALPZER_REFIMG_FILE "config/lytpix2alpzer_refimg.dat"
#define LYTPIX2ALPZER_PXMASK_FILE "config/lytpix2alpzer_pxmask.dat"
#define LYTPIX2ALPACT_REFIMG_FILE "config/lytpix2alpact_refimg.dat"
#define LYTPIX2ALPACT_PXMASK_FILE "config/lytpix2alpact_pxmask.dat"
#define SHK_CONFIG_FILE        "config/shk.cfg"
#define LYT_CONFIG_FILE        "config/lyt.cfg"
#define DATAPATH               "output/flight_data/folder_%5.5d/"
#define DATANAME               "output/flight_data/folder_%5.5d/picture.%s.%8.8d.dat"
#define SHK_HEX_CALFILE        "output/calibration/shk_hex_%s_caldata.dat"
#define SHK_ALP_CALFILE        "output/calibration/shk_alp_%s_caldata.dat"
#define SHK_TGT_CALFILE        "output/calibration/shk_tgt_%s_caldata.dat"
#define LYT_ALP_CALFILE        "output/calibration/lyt_alp_%s_caldata.dat"
#define SCI_BMC_CALFILE        "output/calibration/sci_bmc_%s_caldata.dat"
#define SHKCEL2SHKZER_OUTFILE  "output/calibration/shkcel2shkzer_flight_output.dat"
#define SHKZER2SHKCEL_OUTFILE  "output/calibration/shkzer2shkcel_flight_output.dat"
#define SHK_OUTFILE            "output/calibration/shk_output.dat"
#define LYT_OUTFILE            "output/calibration/lyt_output.dat"
#define SHK_ORIGIN_FILE        "output/calibration/shk_origin.dat"
#define ALP_FLAT_FILE          "output/calibration/alp_flat.dat"
#define SCI_ORIGIN_FILE        "output/calibration/sci_origin.dat"

/*************************************************
 * Network Addresses & Ports
 *************************************************/
#define TLM_PORT     "1337"
#define SRV_PORT     "14000"
#define CMD_SENDDATA  0x0ABACABB

/*************************************************
 * ISA Board Base Addresses
 *************************************************/
#define ADC1_BASE 0x300 //DMM Differential
#define ADC2_BASE 0x200 //DMM Single-ended
#define ADC3_BASE 0x180 //DMM Single-ended
#define COM1_BASE 0x380 //EMM COM port 1
#define COM2_BASE 0x388 //EMM COM port 2
#define COM3_BASE 0x288 //EMM COM port 3
#define COM4_BASE 0x230 //EMM COM port 4
#define SSR_BASE  0x310 //ACCES IO SSR board
#define REL_BASE  0x320 //RTD relay board
#define ADC_PAGE_OFFSET           8
#define ADC_PORTA_OFFSET         12
#define ADC_PORTA_CONFIG_OFFSET  15
#define ADC_IOPORT_LENGTH        16
#define ADC_PORTA_PAGE           0x01
#define ADC_PORTA_CONFIG         0x80
#define ADC1_NCHAN               16
#define ADC2_NCHAN               32
#define ADC3_NCHAN               32
#define REL_IOPORT_LENGTH        4
#define SSR_IOPORT_LENGTH        8

/*************************************************
 * Circular Buffer Info
 *************************************************/
enum bufids {SCIEVENT, SCIFULL,
	     SHKEVENT, SHKFULL,
	     LYTEVENT, LYTFULL,
	     ACQEVENT, ACQFULL,
	     THMEVENT, MTREVENT,
	     NCIRCBUF};

#define SCIEVENTSIZE     3
#define SHKEVENTSIZE     3
#define LYTEVENTSIZE     3
#define ACQEVENTSIZE     3
#define THMEVENTSIZE     3
#define MTREVENTSIZE     3
#define SCIFULLSIZE      3
#define SHKFULLSIZE      3
#define LYTFULLSIZE      3
#define ACQFULLSIZE      3

/*************************************************
 * LOWFS Settings
 *************************************************/
#define LOWFS_N_ZERNIKE         23 //no piston
#define LOWFS_N_PID             3
#define LOWFS_N_HEX_ZERNIKE     5  //no piston

/*************************************************
 * Function Reset Commands
 *************************************************/
#define FUNCTION_RESET    1
#define FUNCTION_NO_RESET 0

/*************************************************
 * Zernike Errors
 *************************************************/
#define ZERNIKE_ERRORS_FILE   "config/zernike_errors.dat"
#define ZERNIKE_ERRORS_NUMBER 15000
#define ZERNIKE_ERRORS_PERIOD 0.00200000

/*************************************************
 * Camera Settings -- Keep sizes divisible by 4 (packets)
 *************************************************/
#define SCIXS           300
#define SCIYS           300
#define SHKXS           1024
#define SHKYS           1024
#define LYTXS           32
#define LYTYS           32
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
 * Camera Exposure Time Limits
 *************************************************/
#define SHK_EXPTIME_MIN  0.010
#define SHK_EXPTIME_MAX  16.00
#define LYT_EXPTIME_MIN  0.001
#define LYT_EXPTIME_MAX  16.00
#define SCI_EXPTIME_MIN  0.001
#define SCI_EXPTIME_MAX  600.0
#define ACQ_EXPTIME_MIN  0.001
#define ACQ_EXPTIME_MAX  1.000

/*************************************************
 * Debug Messaging
 *************************************************/
#define WAT_DEBUG       0 // print wat messages
#define SCI_DEBUG       0 // print sci messages
#define SHK_DEBUG       0 // print shk messages
#define LYT_DEBUG       0 // print lyt messages
#define TLM_DEBUG       0 // print tlm messages
#define ACQ_DEBUG       0 // print acq messages
#define MTR_DEBUG       0 // print mtr messages
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
#define ALP_NAME              "BAX197"
#define ALP_NACT              97
#define ALP_HIDDEN_NACT       12
#define ALP_STROKE            2.0
#define ALPXS                 11
#define ALPYS                 11
#define ALP_AMIN             -1.0
#define ALP_AMAX              1.0
#define ALP_AMID              0.0
#define ALP_DMAX              0x3FFF
#define ALP_DMIN              0x0000
#define ALP_DMID              0x2000
#define ALP_MAX_POWER         40
#define ALP_MAX_BIAS          0.642161299 //sqrt(max_power/97)
#define ALP_MIN_BIAS         -0.642161299 //sqrt(max_power/97)
#define ALP_N_CHANNEL         128 // data size for the ALPAO DM controller
#define ALP_HEADER_LENGTH     2   // leading uint16_ts for the header
#define ALP_CHECKSUM_LENGTH   1   // trailing uint16_t for the checksum
#define ALP_PAD_LENGTH        1   // pad end with an empty word
#define ALP_FRAME_LENGTH      (ALP_HEADER_LENGTH+ALP_N_CHANNEL+ALP_CHECKSUM_LENGTH) //[uint16_t]
#define ALP_FRAME_SIZE        (2*ALP_FRAME_LENGTH) //[bytes]
#define ALP_DATA_LENGTH       (ALP_FRAME_LENGTH+ALP_PAD_LENGTH) //[uint16_t]
#define ALP_DATA_SIZE         (2*ALP_DATA_LENGTH) //[bytes]
#define ALP_MIN_ANALOG_STEP   0.000122070312500 // hardcoded pow(2.0,-13);
#define ALP_START_WORD        0xF800
#define ALP_INIT_COUNTER      0x5C00
#define ALP_END_WORD          0xF100
#define ALP_FRAME_END         0xFEED
#define ALP_BIAS              0.0
#define ALP_SHK_POKE          0.05  //shk alp actuator calibration poke
#define ALP_SHK_ZPOKE         0.02  //shk zernike microns RMS
#define ALP_SHK_NCALIM        35    //shk number of calibration images to take per step
#define ALP_LYT_POKE          0.01  //lyt alp actuator calibration poke
#define ALP_LYT_ZPOKE         0.005 //lyt zernike microns RMS
#define ALP_LYT_NCALIM        110   //lyt number of calibration images to take per step
#define ALP_ZERNIKE_MIN      -5.0   //ALP min zernike command
#define ALP_ZERNIKE_MAX       5.0   //ALP max zernike command
#define ALP_DZERNIKE_MIN     -1.0   //ALP min delta zernike command
#define ALP_DZERNIKE_MAX      1.0   //ALP max delta zernike command


/*************************************************
 * TARGET Parameters
 *************************************************/
#define TGT_SHK_NCALIM        100   //shk number of calibration images to take per step
#define TGT_LYT_NCALIM        100   //lyt number of calibration images to take per step
#define TGT_LYT_ZPOKE         0.005 //lyt tgt zpoke [microns rms]
#define TGT_SHK_ZPOKE         0.02  //shk tgt zpoke [microns rms]

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
#define HEX_REF_TIMEOUT   20  //seconds
#define HEX_PERIOD        0.5 //seconds, time between commands

/*************************************************
 * Shack-Hartmann Parameters
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
#define SHK_CELL_XOFF         78 //+1px = -0.24 microns tip/tilt
#define SHK_CELL_YOFF         89
#define SHK_CELL_ROTATION     0.0
#define SHK_CELL_XSCALE       1.0
#define SHK_CELL_YSCALE       1.0
#define SHK_ORIGIN_NAVG       25
#define SHK_XMIN              0
#define SHK_XMAX              (SHKXS-1)
#define SHK_YMIN              0
#define SHK_YMAX              (SHKYS-1)
#define SHK_BOXSIZE_CMD_STD   0  //use the current runtime boxsize
#define SHK_BOXSIZE_CMD_MAX   1  //use the maximum boxsize

/*************************************************
 * Lyot-LOWFS Parameters
 *************************************************/
#define LYT_CONTROL_NPIX      709 //number of controlled pixels on LLOWFS

/*************************************************
 * SCI Camera Parameters
 *************************************************/
#define SCI_NBANDS              5 //number of bands on a single SCI camera image

/*************************************************
 * RTD Parameters
 *************************************************/
#define RTD_BOARD_MINOR                0 // Minor device number of the RTD board
#define RTD_PRGCLK_0_DIVISOR           8 // Programmable clock frequency = 25/RTD_PRGCLK_0_DIVISOR [MHz]
#define RTD_TIMER_A0_DIVISOR           2 // Output clock frequency = (25/RTD_PRGCLK_0_DIVISOR)/RTD_TIMER_A0_DIVISOR [MHz]
#define RTD_CLK_FREQUENCY              ((25000000.0/RTD_PRGCLK_0_DIVISOR)/RTD_TIMER_A0_DIVISOR) //[Hz]

/*************************************************
 * Telemetry Parameters
 *************************************************/
#define TLM_DATA_RATE      250000                //Words per second = 4Mbps = 4us/word
#define TLM_EMPTY_CODE     0xFADE                //Code to send when there is no data
#define TLM_REPLACE_CODE   0xFFFF                //Code to replace empty codes with in data
#define TLM_BUFFER_LENGTH  (TLM_DATA_RATE/10)    //TLM DMA buffer length (10 updates/sec)
#define TLM_BUFFER_SIZE    (TLM_BUFFER_LENGTH*2) //TLM DMA buffer size

/*************************************************
 * Motor Parameters
 *************************************************/
#define MTR_NDOORS         4


/*************************************************
 * Config Structure
 *************************************************/
typedef struct procinfo_struct{
  int    pid;
  int    run;
  int    die;
  int    done;
  int    res;
  uint32 chk; //# checkins
  uint32 rec; //# recipts of checkin
  int    cnt;
  int    tmo;
  int    ask;
  int    per;
  int    pri;
  int    fakemode;
  int    reset; 
  char   *name;
  char   *mod;
  void (*launch)(void);
} procinfo_t;

/*************************************************
 * Calmode Structure
 *************************************************/
typedef struct calmode_struct{
  char name[MAX_COMMAND];
  char cmd[MAX_COMMAND];
  int  shk_boxsize_cmd;
} calmode_t;

/*************************************************
 * State Control Structures
 *************************************************/
// Shack-Hartmann Control (shk_proc.c)
typedef struct shkctrl_struct{
  int run_camera;
  int fit_zernikes;
  int zernike_control[LOWFS_N_ZERNIKE];
  int cell_control;
  int offload_tilt_to_hex;
  int offload_tilt_to_wasp;
} shkctrl_t;

// Lyot Sensor Control (lyt_proc.c)
typedef struct lytctrl_struct{
  int run_camera;
  int fit_zernikes;
  int zernike_control[LOWFS_N_ZERNIKE];
  int act_control;
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
  char      name[MAX_COMMAND];
  char      cmd[MAX_COMMAND];
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
  double target[2];
  double centroid[2];
  double target_deviation[2];
  double origin_deviation[2];
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
  float   ontime;       //measured frame time
  float   temp;         //sensor temperature, if available
  uint32  imxsize;      //image x size [px]
  uint32  imysize;      //image y size [px]
  uint32  state;        //system state
  uint32  hex_calmode;  //hex calmode
  uint32  alp_calmode;  //alp calmode
  uint32  bmc_calmode;  //bmc calmode
  uint32  tgt_calmode;  //tgt calmode
  uint32  hex_calstep;  //hex calstep
  uint32  alp_calstep;  //alp calstep
  uint32  bmc_calstep;  //bmc calstep
  uint32  tgt_calstep;  //tgt calstep
  char    state_name[MAX_COMMAND]; //string name of state
  char    hex_calmode_name[MAX_COMMAND]; //string name of hex_calmode
  char    alp_calmode_name[MAX_COMMAND]; //string name of alp_calmode
  char    bmc_calmode_name[MAX_COMMAND]; //string name of bmc_calmode
  char    tgt_calmode_name[MAX_COMMAND]; //string name of tgt_calmode
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
  uint32   xorigin[SCI_NBANDS];
  uint32   yorigin[SCI_NBANDS];
  sci_t    image[SCI_NBANDS];
} scievent_t;

typedef struct shkevent_struct{
  pkthed_t  hed;
  uint32    beam_ncells;
  uint32    boxsize;
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
  hex_t     hex;
  alp_t     alp;
  wsp_t     wsp;
} shkevent_t;

typedef struct lytevent_struct{
  pkthed_t  hed;
  double    xtilt;
  double    ytilt;
  double    gain_alp_act[LOWFS_N_PID];
  double    gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID];
  double    zernike_measured[LOWFS_N_ZERNIKE];
  double    zernike_target[LOWFS_N_ZERNIKE];
  double    alp_measured[ALP_NACT];
  alp_t     alp;
  lyt_t     image;
} lytevent_t;

typedef struct acqevent_struct{
  pkthed_t  hed;
  hex_t     hex;
  wsp_t     wsp;
} acqevent_t;

typedef struct thmevent_struct{
  pkthed_t  hed;
  float     adc1[ADC1_NCHAN];
  float     adc2[ADC2_NCHAN];
  float     adc3[ADC3_NCHAN];
} thmevent_t;

typedef struct mtrevent_struct{
  pkthed_t  hed;
  uint16_t  door_status[MTR_NDOORS];
} mtrevent_t;

/*************************************************
 * Full Frame Structures
 *************************************************/
typedef struct scifull_struct{
  pkthed_t hed;
  sci_t    image[SCI_NBANDS];
} scifull_t;

typedef struct shkfull_struct{
  pkthed_t   hed;
  shk_t      image;
  shkevent_t shkevent;
} shkfull_t;

typedef struct lytfull_struct{
  pkthed_t   hed;
  lyt_t      image;
  lytevent_t lytevent;
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

  //Device ready flags
  int alp_ready;
  int bmc_ready;
  int hex_ready;
  int tlm_ready;
  int dio_ready;

  //RTD board descriptor
  DM7820_Board_Descriptor* p_rtd_board;

  //Hexapod file descriptor
  int hexfd;

  //State
  int state;                    //Current operational state
  state_t state_array[NSTATES]; //Array of states

  //Camera exposure times
  double sci_exptime;
  double shk_exptime;
  double lyt_exptime;
  double acq_exptime;
   
  //ALP Command
  int   alp_command_lock;
  alp_t alp_command;
  int   alp_proc_id;
  int   alp_n_dither;

  //BMC Command
  int   bmc_command_lock;
  bmc_t bmc_command;

  //HEX Command
  int   hex_command_lock;
  hex_t hex_command;

  //Calibration Modes
  int alp_calmode;
  int hex_calmode;
  int bmc_calmode;
  int tgt_calmode;

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

  //Lyot LOWFS Settings
  double lyt_gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID];  //LYT ALP zernike PID gains
  double lyt_gain_alp_act[LOWFS_N_PID];                    //LYT ALP actuator PID gains
  
  //Camera Reset Commands
  int shk_reset_camera;
  int acq_reset_camera;
  int sci_reset_camera;
  int lyt_reset_camera;

  //Other Commands
  int hex_getpos;
  int shk_setorigin;
  int shk_revertorigin;
  int shk_saveorigin;
  int shk_loadorigin;
  int shk_xshiftorigin;
  int shk_yshiftorigin;
  int hex_tilt_correct;
  int sci_setorigin;
  int sci_revertorigin;
  int sci_saveorigin;
  int sci_loadorigin;

  //Door Commands
  int open_door[MTR_NDOORS];
  int close_door[MTR_NDOORS];
      
  //Zernike Targets
  double shk_zernike_target[LOWFS_N_ZERNIKE];
  double lyt_zernike_target[LOWFS_N_ZERNIKE];

  //Zernike control switches
  int zernike_control[LOWFS_N_ZERNIKE];
  
  //Events circular buffers
  scievent_t scievent[SCIEVENTSIZE];
  shkevent_t shkevent[SHKEVENTSIZE];
  lytevent_t lytevent[LYTEVENTSIZE];
  acqevent_t acqevent[ACQEVENTSIZE];
  thmevent_t thmevent[THMEVENTSIZE];
  mtrevent_t mtrevent[MTREVENTSIZE];

  //Full frame circular buffers
  scifull_t scifull[SCIFULLSIZE];
  shkfull_t shkfull[SHKFULLSIZE];
  lytfull_t lytfull[LYTFULLSIZE];
  acqfull_t acqfull[ACQFULLSIZE];

  //Circular buffer package
  circbuf_t circbuf[NCIRCBUF];

} sm_t;


#endif
