/***********************************************
 * Main Header for PICTURE-C Flight Software
 ***********************************************/

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
#define FALSE      0
#define TRUE       1
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
#define IWC_ENABLE      1 // IWC
#define DM_ENABLE       1 // DM

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
#define DARKFILE_SCI    "data/darkframe.sci.%3.3d.%3.3d.%2.2dC.dat"
#define FLATFILE_SCI    "data/flatframe.sci.%3.3d.dat" 
#define FAKEFILE_SCI    "data/fakeframe.sci.%3.3d.%s.dat"
#define FITSHEAD        "com/fits"
#define FITSNAME        "data/fits/data%c.%05d.fits"
#define HOSTPORT        "ANY:24924"
#define DATAPATH        "data/flight_data/folder_%5.5d/"
#define DATANAME        "data/flight_data/folder_%5.5d/picture.%s.%8.8d.dat"
#define MAX_FILENAME    128


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
#define PROC_WAITTIME        5  //Timeout for procwait
#define EXIT_WAITTIME       10  //Timeout for exit
#define ERASE_TIMEOUT       25  //Time to wait for TLM to exit on command: erase flight data


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

/*************************************************
 * Camera Settings -- Keep sizes divisible by 4
 *************************************************/
#define SCIXS           128
#define SCIYS           128
#define SHKXS           1024
#define SHKYS           1024
#define LYTXS           16
#define LYTYS           16
#define ACQXS           512
#define ACQYS           512

/*************************************************
 * Debug Messaging
 *************************************************/
#define TLM_DEBUG       0 // print tlm messages
#define HSK_DEBUG       0 // print hsk messages
#define LYT_DEBUG       0 // print bin messages
#define SHK_DEBUG       0 // print shk messages
#define SCI_DEBUG       0 // print sci messages
#define WAT_DEBUG       0 // print wat messages
#define SRV_DEBUG       1 // print wat messages

/*************************************************
 * Other Messaging
 *************************************************/
#define MSG_SAVEDATA    0 // print data saving messages
#define MSG_CTRLC       1 // print SIGINT messages

/*************************************************
 * Limits
 *************************************************/
#define BOXSIZE_MIN     3        //(npx) Minimum centroid boxsize
#define BOXSIZE_MAX     35       //(npx) Maximum centroid boxsize
#define GAIN_P_MIN     -1        //(.)   Minimum P gain
#define GAIN_P_MAX      0        //(.)   Maximum P gain      
#define GAIN_I_MIN     -1        //(.)   Minimum I gain   
#define GAIN_I_MAX      0        //(.)   Maximum I gain      
#define GAIN_D_MIN     -1        //(.)   Minimum D gain   
#define GAIN_D_MAX      0        //(.)   Maximum D gain      

/*************************************************
 * DM Parameters
 *************************************************/
#define DM_CHANNELS 1024
#define DM_STROKE   0.5
#define DMXS        32
#define DMYS        32
#define DM_DMAX     ((1<<14) - 1)
#define DM_DMIN     0                   
#define DM_DMID     ((DM_DMIN+DM_DMAX)/2)  

/*************************************************
 * IWC Parameters
 *************************************************/
#define IWC_CHANNELS 79
#define IWC_STROKE   0.5
#define IWCXS        10
#define IWCYS        10
#define IWC_DMAX     ((1<<14) - 1)
#define IWC_DMIN     0                   
#define IWC_DMID     ((IWC_DMIN+IWC_DMAX)/2)  

/*************************************************
 * Process ID Numbers
 *************************************************/
enum procids {WATID, SCIID, SHKID, LYTID, TLMID, ACQID, MOTID, THMID, SRVID, TMPID, HSKID,DIAID,NCLIENTS};

/*************************************************
 * Shack-Hartmann (SHK) Settings
 *************************************************/
#define SHK_CONFIG_FILE       "phx_config/shk.cfg"
#define SHK_FULL_IMAGE_TIME   0.5    //seconds
#define SHK_XCELLS            16
#define SHK_YCELLS            16
#define SHK_NCELLS            256
#define SHK_LENSLET_PITCH_UM  300.0  
#define SHK_FOCAL_LENGTH_UM   18600.0
#define SHK_PX_PITCH_UM       4.65
#define SHK_SPOT_UPPER_THRESH 20
#define SHK_SPOT_LOWER_THRESH 15
#define SHK_CELL_XOFF         0.0
#define SHK_CELL_YOFF         0.0
#define SHK_CELL_ROTATION     0.0
#define SHK_CELL_XSCALE       1.0
#define SHK_CELL_YSCALE       1.0


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
  uint16_t index;
  uint16_t beam_select;
  uint16_t maxpix;
  uint16_t maxval;
  double origin[2];
  double centroid[2];
  double deviation[2];
} shkcell_t;

/*************************************************
 * Event Structures
 *************************************************/
typedef struct scievent_struct{
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  uint16  imxsize;
  uint16  imysize;
  uint32  state;
  uint32  mode;
  struct timespec time;
  sci_t   image; 
} scievent_t;

typedef struct shkevent_struct{
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  uint16  imxsize;
  uint16  imysize;
  uint32  state;
  uint32  mode;
  int64   time_sec;
  int64   time_nsec;
  shkcell_t cells[SHK_NCELLS];
} shkevent_t;

typedef struct lytevent_struct{
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  uint16  imxsize;
  uint16  imysize;
  uint32  state;
  uint32  mode;
  int64   time_sec;
  int64   time_nsec;
  double  zernikes[LOWFS_N_ZERNIKE];
  uint16  iwc[IWC_CHANNELS];
  lyt_t   image; 
} lytevent_t;

typedef struct acqevent_struct{
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  uint16  imxsize;
  uint16  imysize;
  uint32  state;
  uint32  mode;
  struct timespec time;
  acq_t   image; 
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
  int64   time_sec;
  int64   time_nsec;
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
  int64   time_sec;
  int64   time_nsec;
  shk_t   image;
  float   zernikes[LOWFS_N_ZERNIKE];
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
  int64   time_sec;
  int64   time_nsec;
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
  int64   time_sec;
  int64   time_nsec;
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
  uint32 tlm_fake_mode;        //Telemetry fake mode
  uint32 sci_fake_mode;        //Science camera fake mode
  uint32 lyt_fake_mode;        //Lyot LOWFS camera fake mode
  uint32 shk_fake_mode;        //Shack-Hartmann camera fake mode
  uint32 acq_fake_mode;        //Acquisition camera fake mode
  
  //Camera modes
  uint32 sci_mode;        //Science camera mode
  uint32 lyt_mode;        //Lyot LOWFS camera mode
  uint32 shk_mode;        //Shack-Hartmann camera mode
  uint32 acq_mode;        //Acquisition camera mode
  
  //DM positions
  int16   dm[DMXS][DMYS];
  int16   iwc[IWCXS][IWCYS];

  //LOWFC Settings
  uint16 iwc_commander;

  //Shack-Hartmann Settings
  uint16 shk_boxsize;     //SHK centroid boxsize
  
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



