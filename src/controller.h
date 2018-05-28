/***********************************************
 * Main Header for PICTURE-C Flight Software
 ***********************************************/
//
#include <stdint.h>
#include <time.h>

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

/*************************************************
 * General
 *************************************************/
#define CMD_SENDDATA  0x0ABACABB

/*************************************************
* Hardware Switches
*************************************************/
#define XIN_ENABLE      0 // Xinetics Controller (Master IWC,DM,PEZ)
#define IWC_ENABLE      0 // IWC
#define ALP_ENABLE      0 // ALPAO DM
#define DM_ENABLE       0 // DM
#define PEZ_ENABLE      0 // PIEZO Mirrors
#define HEX_ENABLE      1 // Hexapod

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
 * Base Addresses & Files
 *************************************************/
#define DARKFILE_SCI      "data/darkframe.sci.%3.3d.%3.3d.%2.2dC.dat"
#define FLATFILE_SCI      "data/flatframe.sci.%3.3d.dat"
#define FAKEFILE_SCI      "data/fakeframe.sci.%3.3d.%s.dat"
#define HOSTPORT          "ANY:24924"
#define DATAPATH          "data/flight_data/folder_%5.5d/"
#define DATANAME          "data/flight_data/folder_%5.5d/picture.%s.%8.8d.dat"
#define SHKMATRIX_FILE    "data/shk/shk2alp.dat"
#define ZERNIKE2HEX_FILE  "data/shk/zern2hex.dat"
#define ZERNIKE2SPA_FILE  "data/shk/zern2spa.dat"
#define ZERNIKE2ALP_FILE  "data/shk/zern2alp.dat"
#define SHK2ZERNIKE_FILE  "data/shk/shk2zern.dat"
#define ASTIG2TILT_FILE   "data/shk/astig2tilt.dat"
#define MAX_FILENAME      128


/*************************************************
 * Network Addresses & Ports
 *************************************************/
#define GSE_ADDR  "192.168.0.6"
#define GSE_PORT  "1337"

/*************************************************
 * System Settings & Messages
 *************************************************/
#define WARNING   "WARNING...WARNING...WARNING...WARNING\nWARNING...WARNING...WARNING...WARNING\nWARNING...WARNING...WARNING...WARNING\nWARNING...WARNING...WARNING...WARNING\n"
#define REBOOT   "REBOOT...REBOOT...REBOOT...REBOOT\nREBOOT...REBOOT...REBOOT...REBOOT\nREBOOT...REBOOT...REBOOT...REBOOT\nREBOOT...REBOOT...REBOOT...REBOOT\n"
#define EXIT_TIMEOUT    25  //procwait exit timeout
#define PROC_TIMEOUT    5   //procwait process timeout
#define ERASE_TIMEOUT   25  //Time to wait for TLM to exit on command: erase flight data


/*************************************************
 * Circular Buffer Info
 *************************************************/
enum bufids {SCIEVENT, SCIFULL, SHKEVENT, SHKFULL, LYTEVENT, LYTFULL, ACQEVENT, ACQFULL, NCIRCBUF};
#define SCIEVENTSIZE     3
#define SHKEVENTSIZE     3
#define LYTEVENTSIZE     3
#define ACQEVENTSIZE     3
#define SCIFULLSIZE      3
#define SHKFULLSIZE      3
#define LYTFULLSIZE      3
#define ACQFULLSIZE      3

/*************************************************
 * Define Errors
 *************************************************/
#define _ERROR	       -1
#define _NO_ERROR	0

/*************************************************
 * LOWFS Settings
 *************************************************/
#define LOWFS_N_ZERNIKE         24
#define LOWFS_N_HEX_ZERNIKE     5

/*************************************************
 * Zernike Errors
 *************************************************/
#define ZERNIKE_ERRORS_FILE   "data/zernike/zernike_errors.dat"
#define ZERNIKE_ERRORS_NUMBER 15000
#define ZERNIKE_ERRORS_PERIOD 0.00200000
#define ZERNIKE_ERRORS_SHORT_FILE "data/zernike/zernike_errors_short.dat"
#define ZERNIKE_ERRORS_SHORT_NUMBER 600

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
#define HEX_DEBUG       0 // print hex messages
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
 * Xinetics Controller Parameters
 *************************************************/
#define XIN_NCHANNELS   96 //288 causes actuator glitch

/*************************************************
 * DM Parameters
 *************************************************/
#define DM_NACT     1024
#define DM_STROKE   0.5
#define DMXS        32
#define DMYS        32
#define DM_DMAX     ((1<<14) - 1)
#define DM_DMIN     0
#define DM_DMID     ((DM_DMIN+DM_DMAX)/2)

/*************************************************
 * IWC Parameters
 *************************************************/
#define IWC_NSPA     76
#define IWC_NTTP     3
#define IWC_STROKE   2.0
#define IWCXS        10
#define IWCYS        10
#define IWC_DMAX     ((1<<14) - 1)
#define IWC_DMIN     0
#define IWC_DMID     ((IWC_DMIN+IWC_DMAX)/2)
#define IWC_SPA_BIAS 9502
#define IWC_SPA_POKE 3000
#define IWC_NCALIM   25  //number of calibration images to take per step

/*************************************************
 * ALPAO Parameters
 *************************************************/
#define ALP_NACT     97
#define ALP_STROKE   2.0
#define ALPXS        10
#define ALPYS        10
#define ALP_DMAX     0.9
#define ALP_DMIN     -0.9
#define ALP_DMID     ((ALP_DMIN+ALP_DMAX)/2)
#define ALP_BIAS     0.0
#define ALP_POKE     0.05
#define ALP_NCALIM   25  //number of calibration images to take per step

/*************************************************
 * PIEZO Mirror Parameters
 *************************************************/
#define PEZ_NACT 2

/*************************************************
 * HEXAPOD Parameters
 *************************************************/
#define HEX_DEVICE       "/dev/ttyS0"
#define HEX_BAUD         115200
#define HEX_NAXES        6
#define HEX_AXES_ALL     "X Y Z U V W"
#define HEX_AXES_PIV     "R S T"
#define HEX_POS_HOME     {0,0,0,0,0,0}
// #define HEX_POS_DEFAULT  {-1.213708, 3.527789, -0.203860, 0.238664, 0.438938, 0.001710} // Scope coords  68 deg
#define HEX_POS_DEFAULT  {-1.213708, 3.527789, -0.157457, 0.231722, 0.439093, 0.00171} // Scope coords  68 deg (minimum measured focus zern via SH)
// #define HEX_POS_DEFAULT  {-1.213686, 3.527814 ,-0.071349, 0.233803, 0.441126, 0.001688} // Scope coords  73 deg
#define HEX_TRL_POKE      0.01
#define HEX_ROT_POKE      0.001
#define HEX_NCALIM        50
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

/*************************************************
 * Process ID Numbers
 *************************************************/
enum procids {WATID, SCIID, SHKID, LYTID, TLMID, ACQID, MOTID, THMID, SRVID, HEXID, DIAID, NCLIENTS};

/*************************************************
 * Shack-Hartmann (SHK) Settings
 *************************************************/
#define SHK_CONFIG_FILE       "phx_config/shk.cfg"
#define SHK_FULL_IMAGE_TIME   0.5    //[seconds] period that full images are written to circbuf
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
#define SHK_CELL_XOFF         73
#define SHK_CELL_YOFF         56
#define SHK_CELL_ROTATION     0.0
#define SHK_CELL_XSCALE       1.0
#define SHK_CELL_YSCALE       1.0
#define SHK_ORIGIN_NAVG       25
#define SHK_XMIN              0
#define SHK_XMAX              (SHKXS-1)
#define SHK_YMIN              0
#define SHK_YMAX              (SHKYS-1)


/*************************************************
 * Acquisition Camera (ACQ) Settings
 *************************************************/
#define ACQ_FULL_IMAGE_TIME   0.5  //[seconds] period that full images are written to circbuf

/*************************************************
 * Packets
 *************************************************/
typedef struct tlmheader_struct{
  uint32  packet_type;
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  uint32  imxsize;
  uint32  imysize;
  uint32  state;
  uint32  mode;
  struct timespec time;
} tlmheader_t;


/*************************************************
 * Config Structures
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

typedef struct iwc_struct{
  uint16 spa[IWC_NSPA];
  uint16 ttp[IWC_NTTP];
  uint16 calmode;
} iwc_t;

typedef struct alp_struct{
  double act_cmd[ALP_NACT];
  double act_now[ALP_NACT];
  double zern_now[LOWFS_N_ZERNIKE];
  double zern_cmd[LOWFS_N_ZERNIKE];
  double zern_trg[LOWFS_N_ZERNIKE];
  uint16 calmode;
} alp_t;

typedef struct hex_struct{
  double axs[HEX_NAXES];
  uint64 calmode;
} hex_t;

typedef struct dm_struct{
  uint16 act[DM_NACT];
} dm_t;

typedef struct pez_struct{
  uint16 fm1[PEZ_NACT];
  uint16 fm2[PEZ_NACT];
} pez_t;

typedef struct shk_zmatrix_inv_struct{
  uint32 beam_ncells;
  double matrix_inv[SHK_NCELLS*LOWFS_N_ZERNIKE*2];
} shk_zmatrix_t;

/*************************************************
 * Event Structures
 *************************************************/
typedef struct scievent_struct{
  uint32  packet_type;
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  uint32  imxsize;
  uint32  imysize;
  uint32  mode;
  int64   start_sec;
  int64   start_nsec;
  int64   end_sec;
  int64   end_nsec;
  sci_t   image;
} scievent_t;

typedef struct shkevent_struct{
  uint32    packet_type;
  uint32    frame_number;
  float     exptime;
  float     ontime;
  uint32    beam_ncells;
  uint32    imxsize;
  uint32    imysize;
  uint16    mode;
  uint16    boxsize;
  int64     start_sec;
  int64     start_nsec;
  int64     end_sec;
  int64     end_nsec;
  double    xtilt;
  double    ytilt;
  double    kP;
  double    kI;
  double    kD;
  double    kH;
  shkcell_t cells[SHK_NCELLS];
  double    zernikes[LOWFS_N_ZERNIKE];
  double    iwc_spa_matrix[IWC_NSPA];
  double    alp_act_matrix[ALP_NACT];
  double    hex_axs_matrix[HEX_NAXES];
  iwc_t     iwc;
  hex_t     hex;
  alp_t     alp;
} shkevent_t;

typedef struct lytevent_struct{
  uint32  packet_type;
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  uint32  imxsize;
  uint32  imysize;
  uint32  mode;
  int64   start_sec;
  int64   start_nsec;
  int64   end_sec;
  int64   end_nsec;
  double  zernikes[LOWFS_N_ZERNIKE];
  iwc_t   iwc;
  lyt_t   image;
} lytevent_t;

typedef struct acqevent_struct{
  uint32  packet_type;
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  uint32  imxsize;
  uint32  imysize;
  uint32  mode;
  int64   start_sec;
  int64   start_nsec;
  int64   end_sec;
  int64   end_nsec;
} acqevent_t;

/*************************************************
 * Full Frame Structures
 *************************************************/
typedef struct scifull_struct{
  uint32  packet_type;
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  uint32  imxsize;
  uint32  imysize;
  uint32  mode;
  int64   start_sec;
  int64   start_nsec;
  int64   end_sec;
  int64   end_nsec;
  sci_t   image;
} scifull_t;

typedef struct shkfull_struct{
  uint32  packet_type;
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  uint32  imxsize;
  uint32  imysize;
  uint32  mode;
  int64   start_sec;
  int64   start_nsec;
  int64   end_sec;
  int64   end_nsec;
  shk_t   image;
  shkevent_t shkevent;
} shkfull_t;

typedef struct lytfull_struct{
  uint32  packet_type;
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  uint32  imxsize;
  uint32  imysize;
  uint32  mode;
  int64   start_sec;
  int64   start_nsec;
  int64   end_sec;
  int64   end_nsec;
  lyt_t   image;
} lytfull_t;

typedef struct acqfull_struct{
  uint32  packet_type;
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  uint32  imxsize;
  uint32  imysize;
  uint32  mode;
  int64   start_sec;
  int64   start_nsec;
  int64   end_sec;
  int64   end_nsec;
  acq_t   image;
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

  //Devices
  signed short xin_dev;   //Xinetics Quickusb device handle
  int xin_commander;   //Process in control of the Xinetics controller
  int alp_dev;           //ALPAO DM device handle

  //Actuators
  double hex[HEX_NAXES];

  //ALPCalibration Mode
  int alp_calmode;

  //HEX Calibration Mode
  int hex_calmode;

  //Shack-Hartmann Settings
  int shk_boxsize;        //SHK centroid boxsize
  int shk_fit_zernike;    //Turn SHK Zernike fitting ON/OFF
  double shk_kP;
  double shk_kI;
  double shk_kD;
  double hex_kP;

  //Commands
  int hex_getpos;
  int hex_gohome;
  int hex_godef;
  int shk_reset;
  int shk_setorigin;
  double zern_targ[LOWFS_N_ZERNIKE];
  int acq_reset;

  //Events circular buffers
  scievent_t scievent[SCIEVENTSIZE];
  shkevent_t shkevent[SHKEVENTSIZE];
  lytevent_t lytevent[LYTEVENTSIZE];
  acqevent_t acqevent[ACQEVENTSIZE];

  //Full frame circular buffers
  scifull_t scifull[SCIFULLSIZE];
  shkfull_t shkfull[SHKFULLSIZE];
  lytfull_t lytfull[LYTFULLSIZE];
  acqfull_t acqfull[ACQFULLSIZE];

  //Circular buffer package
  circbuf_t circbuf[NCIRCBUF];

} sm_t;
