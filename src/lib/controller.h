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
typedef unsigned long int uint32;
typedef signed long int int32;
typedef unsigned short int uint16;
typedef signed short int int16;
typedef unsigned char uint8;
typedef signed char int8;
typedef int boolean;

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
#define PUT_SOMETHING_HERE

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
#define SHARED_BASE    0x0D200000 
#define DIO1_ADDRESS   0x110
#define DIO2_ADDRESS   0x100
#define SEMA_FILE       "semaphore.txt"
#define SEMA_ID         5
#define SEMA_RETRY      10
#define DARKFILE_SCI    "data/darkframe.sci.%3.3d.%3.3lu.%2.2dC.dat"
#define FLATFILE_SCI    "data/flatframe.sci.%3.3d.dat" 
#define FAKEFILE_SCI    "data/fakeframe.sci.%3.3d.%s.dat"
#define FITSHEAD        "com/fits"
#define FITSNAME        "data/fits/data%c.%05d.fits"
#define HOSTPORT        "ANY:24924"
#define DATAPATH        "data/flight_data/folder_%5.5lu/"
#define DATANAME        "data/flight_data/folder_%5.5lu/picture.%s.%8.8lu.dat"
#define MAX_FILENAME    128

/*************************************************
 * Network Addresses & Ports
 *************************************************/
#define GSE_ADDR  "192.168.0.6"

/*************************************************
 * System Settings & Messages
 *************************************************/
#define WARNING   "WARNING...WARNING...WARNING...WARNING\nWARNING...WARNING...WARNING...WARNING\nWARNING...WARNING...WARNING...WARNING\nWARNING...WARNING...WARNING...WARNING\n"
#define REBOOT   "REBOOT...REBOOT...REBOOT...REBOOT\nREBOOT...REBOOT...REBOOT...REBOOT\nREBOOT...REBOOT...REBOOT...REBOOT\nREBOOT...REBOOT...REBOOT...REBOOT\n"
#define SHMEM_SLEEP          1  //Shared memory retry rate
#define SHMEM_RETRY          5  //Shared memory timeout
#define PROC_WAITTIME        5  //Timeout for procwait
#define EXIT_WAITTIME       10  //Timeout for exit
#define ERASE_TIMEOUT       25  //Time to wait for TLM to exit on command: erase flight data


/*************************************************
 * Circular Buffer Info
 *************************************************/
#define CIRCBUFSIZE     3

/*************************************************
 * Define Errors
 *************************************************/
#define _ERROR	       -1
#define _NO_ERROR	0

/*************************************************
 * Interrupts
 *************************************************/
#define SCI_IRQ	        7      //IRQ for camera interrupts
#define SHK_IRQ	        8      //IRQ for camera interrupts
#define LYT_IRQ	        9      //IRQ for camera interrupts

/*************************************************
 * Camera Settings
 *************************************************/
#define SCIXS           128
#define SCIYS           128
#define SHKXS           1024
#define SHKYS           1024
#define LYTXS           16
#define LYTYS           16

/*************************************************
 * Messaging
 *************************************************/
#define TLM_DEBUG       0 // print tlm messages
#define HSK_DEBUG       0 // print hsk messages
#define LYT_DEBUG       0 // print bin messages
#define SHK_DEBUG       0 // print shk messages
#define SCI_DEBUG       0 // print sci messages
#define WAT_DEBUG       0 // print wat messages

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
#define DM_PHASE    0.0324278
#define DMXS        32
#define DMYS        32
#define DM_DMAX     ((1<<14) - 1)
#define DM_DMIN     0                   
#define DM_DMID     ((DM_DMIN+DM_DMAX)/2)  
#define DM_RMAX     (1./DM_PHASE)
#define DM_RMIN     0
#define DM_RMID     ((DM_RMIN+DM_RMAX)/2)  

#define DM_I_CEIL    0.5   
#define DM_I_FLOOR  -0.5
#define DM_MWAVE     0.633 //microns
#define DM_DWAVE     ((DM_MWAVE/2)*(DM_DMAX/1.6)) //this is correct
#define DM_RWAVE     TWOPI
#define DM_R2D       (DM_DWAVE/DM_RWAVE)  

/*************************************************
 * States
 *************************************************/
enum states {STATE_LOST,STATE_SENSE,N_STATES};
    
  
/*************************************************
 * Process ID Numbers
 *************************************************/
enum procids {WATID, SCIID, SHKID, LYTID, TLMID, ACQID, MOTID, THMID, SRVID, TMPID, HSKID};


/*************************************************
 * Packets
 *************************************************/
typedef struct tlmheader_struct{
  uint32  packet_type;
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  int32   timestamp;
  uint32  imxsize;
  uint32  imysize;
  uint32  state;
  uint32  mode;
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
  uint8 rtl;
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

/*************************************************
 * Event Structures
 *************************************************/
typedef struct science_struct{
  uint32  frame_number;
  float   exptime;
  float   ontime;
  float   temp;
  int32   timestamp;
  uint16  imxsize;
  uint16  imysize;
  uint32  state;
  uint32  mode;
  sci_t   S; //SCI
} science_t;

/*************************************************
 * Shared Memory Layout
 *************************************************/
typedef volatile struct {

  //Runtime switches
  boolean memlock;        //Shared memory locking bit
  boolean die;            //Kill all processes
  
  //Fake mode
  uint16  fake_mode;      //Fake data mode

  //Process information
  procinfo_t w[NCLIENTS];

  //Camera modes
  uint32 sci_mode;
  double sci_exptime;  
  struct timespec sci_ts;
 
  //DM positions
  int16    dm[DMXS][DMYS];

  //Science circular buffer
  science_t science_cirbuf[CIRCBUFSIZE];
  uint32  science_write_offset;	          //last entry written
  uint32  science_read_offsets[NCLIENTS]; //last entry read
} sm_t;



