/***********************************************
 * Main Header for PICTURE-C Flight Software
 ***********************************************/
#include <stdint.h>
#include <time.h>
#include <dm7820_library.h>
#include <libbmc.h>

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
#define MAX_LINE     128
#define MAX_FILENAME 128
#define MAX_COMMAND  32

/*************************************************
 * System Settings & Messages
 *************************************************/
#define WARNING   "WARNING...WARNING...WARNING...WARNING\n"
#define REBOOT    "REBOOT...REBOOT...REBOOT...REBOOT\n"
#define EXIT_TIMEOUT    25  //procwait exit timeout
#define PROC_TIMEOUT    5   //procwait process timeout
#define ERASE_TIMEOUT   25  //Time to wait for TLM to exit on command: erase flight data

/*************************************************
 * Process ID Numbers
 *************************************************/
enum procids {WATID, SCIID, SHKID, LYTID, TLMID, ACQID, MTRID, THMID, MSGID, DIAID, NCLIENTS};

/*************************************************
 * States
 *************************************************/
enum states { STATE_STANDBY,
	      STATE_LOW_POWER,
	      STATE_ACQUIRE_TARGET,
	      STATE_SPIRAL_SEARCH,
	      STATE_M2_ALIGN,
	      STATE_SHK_HEX_ALIGN,
	      STATE_SHK_HEX_CALIBRATE,
	      STATE_SHK_ALP_CALIBRATE,
	      STATE_SHK_ZERN_LOWFC,
	      STATE_SHK_CELL_LOWFC,
	      STATE_LYT_ALP_CALIBRATE,
	      STATE_LYT_ZERN_LOWFC,
	      STATE_HYB_ZERN_LOWFC,
	      STATE_LYT_TT_LOWFC,
	      STATE_SCI_BMC_CALIBRATE,
	      STATE_HOWFS,
	      STATE_EFC,
	      STATE_SHK_EFC,
	      STATE_HYB_EFC,
	      NSTATES};

/*************************************************
 * Calibration Modes
 *************************************************/
enum alpcalmodes {ALP_CALMODE_NONE,
		  ALP_CALMODE_TIMER,
		  ALP_CALMODE_POKE,
		  ALP_CALMODE_ZPOKE,
		  ALP_CALMODE_FLIGHT,
		  ALP_CALMODE_RAMP,
		  ALP_CALMODE_ZRAMP,
		  ALP_CALMODE_RAND,
		  ALP_CALMODE_ZRAND,
		  ALP_CALMODE_PMZPOKE,
		  ALP_NCALMODES};

enum hexcalmodes {HEX_CALMODE_NONE,
		  HEX_CALMODE_POKE,
		  HEX_CALMODE_SPIRAL,
		  HEX_NCALMODES};

enum tgtcalmodes {TGT_CALMODE_NONE,
		  TGT_CALMODE_ZPOKE,
		  TGT_CALMODE_ZRAMP,
		  TGT_CALMODE_ZRAND,
		  TGT_NCALMODES};

enum bmccalmodes {BMC_CALMODE_NONE,
		  BMC_CALMODE_TIMER,
		  BMC_CALMODE_POKE,
		  BMC_CALMODE_VPOKE,
		  BMC_CALMODE_RAND,
		  BMC_CALMODE_PROBE,
		  BMC_CALMODE_SINE,
		  BMC_CALMODE_SINE_DIFF,
		  BMC_NCALMODES};

/*************************************************
 * Commands
 *************************************************/
#define CMD_SENDDATA  0x0ABACABB

/*************************************************
* Instrument Input Type
*************************************************/
#define INPUT_TYPE_SINGLE_PASS   0 //One reflection off DMs
#define INPUT_TYPE_DOUBLE_PASS   1 //Two reflections off DMs
#define INSTRUMENT_INPUT_TYPE    INPUT_TYPE_SINGLE_PASS

/*************************************************
* Master Enable Switches
*************************************************/
#define ALP_ENABLE      1 // ALPAO DM over RTD
#define BMC_ENABLE      1 // BMC DM
#define HEX_ENABLE      1 // Hexapod
#define LED_ENABLE      1 // LED
#define HTR_ENABLE      1 // Heaters
#define MTR_ENABLE      1 // Motors
#define TLM_ENABLE      0 // Telemetry over RTD

/*************************************************
 * Actuator IDs
 *************************************************/
#define ACTUATOR_ALP 1
#define ACTUATOR_HEX 2
#define ACTUATOR_BMC 3
#define ACTUATOR_LED 4
#define ACTUATOR_HTR 5
#define ACTUATOR_MTR 6

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
#define LYTPIX2ALPZER_REFIMG_FILE "config/lytpix2alpzer_refimg.dat"
#define LYTPIX2ALPZER_REFMOD_FILE "config/lytpix2alpzer_refmod.dat"
#define LYTPIX2ALPZER_PXMASK_FILE "config/lytpix2alpzer_pxmask.dat"
#define SHK_CONFIG_FILE        "config/shk_2bin_2tap_8bit.cfg"
#define LYT_CONFIG_FILE        "config/lyt.cfg"
#define SCI_MASK_FILE          "config/howfs_scimask.dat"
#define BMC_PROBE_FILE         "config/howfs_bmcprobe%d_probe_%dnm.dat"
#define BMC_CAL_A_FILE         "config/bmc_cal_a.dat"
#define BMC_CAL_B_FILE         "config/bmc_cal_b.dat"
#define BMC_DEFAULT_FILE       "config/bmc_default.dat"
#define BMC_ACTIVE2FULL_FILE   "config/bmc_active2full.dat"
#define BMC_ROTATE_FILE        "config/bmc_rotate.dat"
#define BMC_POLARITY_FILE      "config/bmc_polarity.dat"
#define BMC_TEST_FILE          "config/bmc_test_%3.3d.dat"
#define BMC_SINE_FILE          "config/bmc_sine_%3.3d.dat"
#define BMC_FLAT_FILE          "output/settings/bmc_flat.dat"
#define HOWFS_RMATRIX0_FILE    "config/howfs_rmatrix0_probe_%dnm.dat"
#define HOWFS_RMATRIX1_FILE    "config/howfs_rmatrix1_probe_%dnm.dat"
#define HOWFS_IMATRIX0_FILE    "config/howfs_imatrix0_probe_%dnm.dat"
#define HOWFS_IMATRIX1_FILE    "config/howfs_imatrix1_probe_%dnm.dat"
#define EFC_MATRIX_FILE        "config/efc_matrix.dat"
#define SCI_FAKE_PROBE_FILE    "config/sci_fakedata_probe_%d.dat"
#define SCI_DARK_FILE          "config/sci_dark_%d.dat"
#define SCI_BIAS_FILE          "config/sci_bias_%d.dat"
#define DATAPATH               "output/data/flight_data/folder_%5.5d/"
#define DATANAME               "output/data/flight_data/folder_%5.5d/picture.%10.10ld.%s.%8.8d.dat"
#define SHK_HEX_CALFILE        "output/data/calibration/shk_hex_%s_%s_%s_caldata.dat"
#define SHK_ALP_CALFILE        "output/data/calibration/shk_alp_%s_%s_%s_caldata.dat"
#define SHK_TGT_CALFILE        "output/data/calibration/shk_tgt_%s_%s_%s_caldata.dat"
#define LYT_ALP_CALFILE        "output/data/calibration/lyt_alp_%s_%s_%s_caldata.dat"
#define LYT_TGT_CALFILE        "output/data/calibration/lyt_tgt_%s_%s_%s_caldata.dat"
#define SCI_BMC_CALFILE        "output/data/calibration/sci_bmc_%s_%s_%s_caldata.dat"
#define SHKCEL2SHKZER_OUTFILE  "output/data/calibration/shkcel2shkzer_flight_output.dat"
#define SHKZER2SHKCEL_OUTFILE  "output/data/calibration/shkzer2shkcel_flight_output.dat"
#define SHK_OUTFILE            "output/data/calibration/shk_output_%s.dat"
#define LYT_OUTFILE            "output/data/calibration/lyt_output_%s.dat"
#define SCI_OUTFILE            "output/data/calibration/sci_output_%s.dat"
#define SHK_ORIGIN_FILE        "output/settings/shk_origin.dat"
#define ALP_FLAT_FILE          "output/settings/alp_flat.dat"
#define SCI_XORIGIN_FILE       "output/settings/sci_xorigin.dat"
#define SCI_YORIGIN_FILE       "output/settings/sci_yorigin.dat"
#define LYT_REFIMG_FILE        "output/settings/lyt_refimg.dat"
#define LYT_DARKIMG_FILE       "output/settings/lyt_darkimg.dat"
#define HEX_POS_FILE           "output/settings/hex_pos.dat"
#define CMD_ERASE_FLIGHT_DATA  "rm -r output/data/flight_data/*"
#define CMD_ERASE_CAL_DATA     "rm -r output/data/calibration/*"

/*************************************************
 * Network Addresses & Ports
 *************************************************/
#define TLM_PORT     "1337"
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
#define ADC1_NCHAN               16
#define ADC2_NCHAN               32
#define ADC3_NCHAN               32
#define ADC_IOPORT_LENGTH        16
#define REL_IOPORT_LENGTH        4
#define SSR_IOPORT_LENGTH        8
#define SSR_NCHAN                16

/*************************************************
 * Circular Buffer Info
 *************************************************/
enum bufids {BUFFER_SCIEVENT, BUFFER_SHKEVENT,
	     BUFFER_LYTEVENT, BUFFER_ACQEVENT,
	     BUFFER_THMEVENT, BUFFER_MTREVENT,
	     BUFFER_SHKPKT,   BUFFER_LYTPKT,
	     BUFFER_SHKFULL,  BUFFER_ACQFULL,
	     BUFFER_WFSEVENT, BUFFER_MSGEVENT, NCIRCBUF};

#define SCIEVENTSIZE     5
#define SHKEVENTSIZE     20
#define LYTEVENTSIZE     400
#define ACQEVENTSIZE     5
#define THMEVENTSIZE     5
#define MTREVENTSIZE     5
#define SHKPKTSIZE       5
#define LYTPKTSIZE       5
#define SHKFULLSIZE      5
#define ACQFULLSIZE      5
#define WFSEVENTSIZE     5
#define MSGEVENTSIZE     100

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
//#define ZERNIKE_ERRORS_FILE   "config/picture_c_pointing_data_20200727_ansys_stiff_epsUma_window_zernike_errors.dat"
//#define ZERNIKE_ERRORS_NUMBER 174006
//#define ZERNIKE_ERRORS_PERIOD 0.00200000
//#define ZERNIKE_ERRORS_LENGTH 348.012
#define ZERNIKE_ERRORS_FILE   "config/picture_c_pointing_data_requirement_zernike_errors.dat"
#define ZERNIKE_ERRORS_NUMBER 200000
#define ZERNIKE_ERRORS_PERIOD 0.0020000
#define ZERNIKE_ERRORS_LENGTH 400
//#define ZERNIKE_ERRORS_FILE   "config/measured_zernike_errors.dat"
//#define ZERNIKE_ERRORS_NUMBER 12441
//#define ZERNIKE_ERRORS_PERIOD 0.0011000000
//#define ZERNIKE_ERRORS_LENGTH 13.685100

/*************************************************
 * Camera Settings -- Keep sizes divisible by 4 (packets)
 *************************************************/
#define SCIXS           100
#define SCIYS           100
#define SHKXS           512
#define SHKYS           512
#define SHKBIN          2
#define LYTXS           16
#define LYTYS           16
#define LYTREADXS       64   
#define LYTREADYS       64 
#define ACQREADXS       1280
#define ACQREADYS       960
#define ACQXS           640
#define ACQYS           480

/*************************************************
 * Camera Full Image Times
 *************************************************/
#define SHK_FULL_IMAGE_TIME   0.5    //[seconds] period that full images are written to circbuf
#define ACQ_FULL_IMAGE_TIME   0.0    //[seconds] period that full images are written to circbuf

/*************************************************
 * Camera Exposure Time Limits
 *************************************************/
#define SHK_EXPTIME_MIN  0.000001
#define SHK_EXPTIME_MAX  16.00
#define SHK_FRMTIME_MIN  0.001
#define SHK_FRMTIME_MAX  16.00
#define LYT_EXPTIME_MIN  0.000001
#define LYT_EXPTIME_MAX  16.00
#define LYT_FRMTIME_MIN  0.001
#define LYT_FRMTIME_MAX  16.00
#define SCI_EXPTIME_MIN  0.001
#define SCI_EXPTIME_MAX  600.0
#define SCI_FRMTIME_MIN  0.001
#define SCI_FRMTIME_MAX  600.0
#define ACQ_EXPTIME_MIN  0.001
#define ACQ_EXPTIME_MAX  1.000
#define ACQ_FRMTIME_MIN  0.001
#define ACQ_FRMTIME_MAX  1.000

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
#define HEX_DEBUG       0 // print hex messages
#define DIA_DEBUG       0 // print dia messages

/*************************************************
 * Other Messaging
 *************************************************/
#define MSG_SAVEDATA    0 // print data saving messages
#define MSG_CTRLC       1 // print SIGINT messages

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
 * Shack-Hartmann Parameters
 *************************************************/
#define SHK_PX_PITCH_UM       5.5
#define SHK_XCELLS            16
#define SHK_YCELLS            16
#define SHK_NCELLS            256
#define SHK_LENSLET_PITCH_UM  300.0
#define SHK_FOCAL_LENGTH_UM   18600.0
#define SHK_BOX_DEADBAND      2      //[pixels] deadband radius for switching to smaller boxsize
#define SHK_MIN_BOXSIZE       (SHK_BOX_DEADBAND+1)
#define SHK_MAX_BOXSIZE       27     //[pixels] gives a 5 pixel buffer around edges
#define SHK_SPOT_UPPER_THRESH 5  //spot found above this
#define SHK_SPOT_LOWER_THRESH 2  //spot lost below this
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
#define SHK_ALP_CELL_INT_MAX  1
#define SHK_ALP_CELL_INT_MIN -1
#define SHK_ALP_ZERN_INT_MAX  0.1
#define SHK_ALP_ZERN_INT_MIN -0.1
#define SHK_SAVE_ZMATRIX      0
#define SHK_SHKPKT_TIME       1.1 //Maximum time between writing shkpkt
#define SHK_NSAMPLES          40  //Number of samples per shkpkt
#define SHK_BEAM_SELECT	{			\
      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,		\
      0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,		\
      0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,		\
      0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,		\
      0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,		\
      0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,		\
      0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,		\
      0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,		\
      0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,		\
      0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,		\
      0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,		\
      0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,		\
      0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,		\
      0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,		\
      0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,		\
      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define SHK_BEAM_NCELLS 148
#define SHK_ZFIT_MIN_CELLS 4

/*************************************************
 * Lyot-LOWFS Parameters
 *************************************************/
#define LYT_CONTROL_NPIX      709 //number of controlled pixels on LLOWFS
#define LYT_NSAMPLES          400 //number of samples per lytpkt
#define LYT_LYTPKT_TIME       1.1 //Maximum amount of time between writing lytpkt
#define LYT_ALP_ZERN_INT_MAX  0.1
#define LYT_ALP_ZERN_INT_MIN -0.1
#define LYT_ZERNIKE_MIN      -0.05 //min limit for measured zernikes [microns]
#define LYT_ZERNIKE_MAX       0.05 //max limit for measured zernikes [microns]
#define LYT_XORIGIN_MIN       0
#define LYT_XORIGIN_MAX       (LYTREADYS-LYTYS) //origins are transposed
#define LYT_YORIGIN_MIN       0
#define LYT_YORIGIN_MAX       (LYTREADXS-LYTXS) //origins are transposed
#define LYT_PIXEL_THRESH      200 //ADU
#define LYT_MAG_MIN           0.1
#define LYT_MAG_MAX           10.0
#define LYT_ROI_MIN           0
#define LYT_ROI_MAX           400
#define LYT_NDARK             1000 //number of frames to average into the LYT dark frame

/*************************************************
 * SCI Camera Parameters
 *************************************************/
#define SCI_NBANDS              1 //number of bands on a single SCI camera image
#define SCI_NSAMPLES            1 //number of scievents to save in a single packet
#define SCI_NPIX              728 //number of pixels in dark zone
#define SCI_HOWFS_NPROBE        4 //number of HOWFS DM probe steps
#define SCI_HOWFS_NSTEP         5 //number of HOWFS steps
#define SCI_ROI_XSIZE        2840
#define SCI_ROI_YSIZE        2224
#define SCI_HBIN                1 //do not change, will break code below
#define SCI_VBIN                1 //do not change, will break code below
#define SCI_UL_X                0
#define SCI_UL_Y                0
#define SCI_LR_X (SCI_UL_X+(SCI_ROI_XSIZE/SCI_HBIN))
#define SCI_LR_Y (SCI_UL_Y+(SCI_ROI_YSIZE/SCI_VBIN))
#define SCI_XORIGIN_MIN         (SCIXS/2)
#define SCI_XORIGIN_MAX         (SCI_ROI_XSIZE-SCIXS/2)
#define SCI_YORIGIN_MIN         (SCIYS/2)
#define SCI_YORIGIN_MAX         (SCI_ROI_YSIZE-SCIYS/2)
#define SCI_NFLUSHES            4
#define SCI_EXP_RETURN_FAIL     1
#define SCI_EXP_RETURN_KILL     2
#define SCI_EXP_RETURN_ABORT    3
#define SCI_MODE_10MHZ          0
#define SCI_MODE_1_7MHZ         1
#define SCI_SEARCH            400 //px search diameter to find star in each band
#define SCI_TEC_SETPOINT_MIN  -40 //C
#define SCI_TEC_SETPOINT_MAX   35 //C
#define SCI_TEMP_INC            5 //C
#define SCI_SATURATION      65535 //SCI saturation
#define SCI_SIM_MAX         {0.0766} //[SCI_NBANDS] Maximum pixel value of unocculted image simulation (for field normalization)
#define SCI_SCALE_DEFAULT {6.37e-9}//[SCI_NBANDS] Default image normalization for field calculation
	       
/*************************************************
 * ACQ Camera Parameters
 *************************************************/
#define ACQ_MAX_GIF_SIZE       10000 //bytes
#define ACQ_THRESH_MAX         255
#define ACQ_STAR_THRESH        3     //ADU
#define ACQ_NSTAR_THRESH       3     //pixels above star_thresh

/*************************************************
 * BMC DM Parameters
 *************************************************/
#define BMC_NACT       952 //LIBBMC_NACT
#define BMC_NTEST      11  //LIBBMC_NTSTPNT
#define BMC_NACTIVE    856
#define BMC_RANGE      LIBBMC_VOLT_RANGE_150V
#define BMC_VMIN       0
#define BMC_VMAX       150
#define BMC_STROKE     1.5
#define BMCXS          34
#define BMCYS          34
#define BMC_DMAX       ((1<<14) - 1)
#define BMC_DMIN       0
#define BMC_DMID       ((DM_DMIN+DM_DMAX)/2)
#define BMC_SCI_NCALIM 1
#define BMC_SCI_POKE   50 //nm
#define BMC_SCI_VPOKE  10 //volts
#define BMC_OW_FLAT    2
#define BMC_SET_FLAT   1
#define BMC_NOSET_FLAT 0
#define BMC_NFLAT      10
#define BMC_NSINE      108

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
#define ALP_ZERNIKE_MIN      -5.0   //ALP min zernike command
#define ALP_ZERNIKE_MAX       5.0   //ALP max zernike command
#define ALP_DZERNIKE_MIN     -1.5   //ALP min delta zernike command
#define ALP_DZERNIKE_MAX      1.5   //ALP max delta zernike command
#define ALP_SHK_POKE          0.05  //shk alp actuator calibration poke
#define ALP_SHK_ZPOKE         0.02  //shk zernike microns RMS
#define ALP_SHK_NCALIM        40    //shk number of calibration images per alp step
#define ALP_LYT_POKE          0.01  //lyt alp actuator calibration poke
#define ALP_LYT_ZPOKE         0.02  //lyt zernike microns RMS
#define ALP_LYT_NCALIM        400   //lyt number of calibration images per alp step

/*************************************************
 * HEXAPOD Parameters
 *************************************************/
#define HEX_DEVICE       "/dev/ttyS3"
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
#define HEX_POS_DEFAULT  {-1.045104,-0.480952,0.182165,0.104829,0.653859,0.001735}
#define HEX_TRL_POKE      0.01
#define HEX_ROT_POKE      0.001
#define HEX_X_CAL_POKE    0.01
#define HEX_Y_CAL_POKE    0.01
#define HEX_Z_CAL_POKE    0.05
#define HEX_U_CAL_POKE    0.001
#define HEX_V_CAL_POKE    0.001
#define HEX_W_CAL_POKE    0.005
#define HEX_SHK_NCALIM    5
#define HEX_LYT_NCALIM    5
#define HEX_PIVOT_X       0    //122.32031250
#define HEX_PIVOT_Y       0    //206.61012268
#define HEX_PIVOT_Z       0    //74.0
#define DEG_ROT_X         0.0  //deg
#define DEG_ROT_Y         0.0  //deg
#define DEG_ROT_Z         30.0 //deg
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
#define HEX_MOVE_TIMEOUT  5   //seconds
#define HEX_PERIOD        0.5 //seconds, time between commands

/*************************************************
 * Target Parameters
 *************************************************/
#define TGT_SHK_NCALIM        100   //shk number of calibration images per tgt step
#define TGT_LYT_NCALIM        200   //lyt number of calibration images per tgt step
#define TGT_LYT_ZPOKE         0.005 //lyt tgt zpoke [microns rms]
#define TGT_SHK_ZPOKE         0.02  //shk tgt zpoke [microns rms]

/*************************************************
 * RTD Parameters
 *************************************************/
#define RTD_ALP_BOARD_MINOR            0 // Minor device number of the ALP RTD board
#define RTD_TLM_BOARD_MINOR            0 // Minor device number of the TLM RTD board
#define RTD_PRGCLK_0_DIVISOR           8 // Programmable clock frequency = 25/RTD_PRGCLK_0_DIVISOR [MHz]
#define RTD_TIMER_A0_DIVISOR           2 // Output clock frequency = (25/RTD_PRGCLK_0_DIVISOR)/RTD_TIMER_A0_DIVISOR [MHz]
#define RTD_CLK_FREQUENCY              ((25000000.0/RTD_PRGCLK_0_DIVISOR)/RTD_TIMER_A0_DIVISOR) //[Hz]

/*************************************************
 * Telemetry Parameters
 *************************************************/
#define TLM_DATA_RATE      250000                //Words per second = 4Mbps = 4us/word
#define TLM_EMPTY_CODE     0xFADE                //Code to send when there is no data
#define TLM_REPLACE_CODE   0xFFFF                //Code to replace empty codes with in data
#define TLM_PRESYNC        0x12345678            //TLM packet pre sync word
#define TLM_POSTSYNC       0xDEADBEEF            //TLM packet post sync word
#define TLM_BUFFER_LENGTH  (TLM_DATA_RATE/250)   //TLM DMA buffer length (250 updates/sec)
#define TLM_BUFFER_SIZE    (TLM_BUFFER_LENGTH*2) //TLM DMA buffer size
#define TLM_UDP_MULTICAST  1                     //1:multicast, 0:unicast
#define TLM_UDP_UNI_ADDR   "192.168.0.4"         //UDP unicast sendto address
#define TLM_UDP_UNI_PORT   "1337"                //UDP unicast sendto port
#define TLM_UDP_MULTI_ADDR "224.255.0.1"         //UDP multicast sendto address (group)
#define TLM_UDP_MULTI_PORT "20000"               //UDP multicast sendto port
#define TLM_UDP_MAX_SIZE   65000                 //Maximum UDP packet size (bytes)

/*************************************************
 * Motor Parameters
 *************************************************/
#define MTR_NDOORS         4

/*************************************************
 * Heater Parameters
 *************************************************/
#define HTR_POWER_MIN      0
#define HTR_POWER_MAX      100
#define HTR_SETPOINT_MIN  -40
#define HTR_SETPOINT_MAX   80
#define HTR_DEADBAND_MIN   0
#define HTR_DEADBAND_MAX   10
#define HTR_GAIN_MIN       0
#define HTR_GAIN_MAX       100
#define HTR_ADC_MIN        1
#define HTR_ADC_MAX        3

/*************************************************
 * Command Uplink Parameters
 *************************************************/
#define UPLINK_DEVICE   "/dev/ttyS1"
#define UPLINK_BAUD     1200

/*************************************************
 * Humidity Sensors
 *************************************************/
#define HUM_NSENSORS     3
#define HUM1_ADDR        0x40
#define HUM2_ADDR        0x41
#define HUM3_ADDR        0x43

/*************************************************
 * Control System Parameters
 *************************************************/
#define PID_DOUBLE_INTEGRATOR 0
#define PID_SINGLE_INTEGRATOR 1

/*************************************************
 * Other Parameters
 *************************************************/
#define CALMODE_TIMER_SEC       30 //default length of calmode_timer
#define ALP_CAL_SCALE_MAX      100 //max ALP calbration command scale
#define ALP_CAL_TIMER_MAX      400 //max ALP calbration timer length
#define BMC_CAL_SCALE_MAX      100 //max BMC calbration command scale
#define BMC_CAL_TIMER_MAX      400 //max BMC calbration timer length
#define CPU_AFFINITY_PHX0        1 //cpu bit mask
#define CPU_AFFINITY_PHX1        2 //cpu bit mask
#define CPU_AFFINITY_XHCI_HCD    1 //cpu bit mask

/*************************************************
 * Config Structure
 *************************************************/
typedef struct procinfo_struct{
  //Settings (watchdog.h)
  int    run;  //Run switch
  int    ena;  //Enable switch
  int    ask;  //Tell watchdog to ask process to exit with "die" command
  int    tmo;  //# process timeout (seconds)
  char  *name; //Process name
  int    per;  //Process checkin period
  //Runtime
  int    pid; //PID # of process
  int    die; //Ask process to exit switch
  int    res; //Restart process
  uint32 chk; //# checkins
  uint32 rec; //# recipts of checkin
  int    cnt; //# missed checkins 
  int    fakemode; //Process fake mode
  void (*launch)(void);
} procinfo_t;

/*************************************************
 * State Control Structures
 *************************************************/
// Shack-Hartmann Control (shk_proc.c)
typedef struct shkctrl_struct{
  int fit_zernikes;
  int zernike_control[LOWFS_N_ZERNIKE];
  int cell_control;
  int shk2lyt;
  int alp_zernike_offload[LOWFS_N_ZERNIKE];
} shkctrl_t;

// Lyot Sensor Control (lyt_proc.c)
typedef struct lytctrl_struct{
  int fit_zernikes;
  int zernike_control[LOWFS_N_ZERNIKE];
  int act_control;
  int alp_zernike_offload[LOWFS_N_ZERNIKE];
} lytctrl_t;

// Science Camera Control (sci_proc.c)
typedef struct scictrl_struct{
  int run_howfs;
  int run_efc;
} scictrl_t;

// Acquisition Camera Control (acq_proc.c)
typedef struct acqctrl_struct{
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
  int       proc_run[NCLIENTS];
  int       hex_commander;
  int       alp_commander;
  int       bmc_commander;
  shkctrl_t shk;
  lytctrl_t lyt;
  scictrl_t sci;
  acqctrl_t acq;
} state_t;

/*************************************************
 * Image Structures
 *************************************************/
typedef struct sci_struct{
  uint16 data[SCIXS][SCIYS];
} sci_t;

typedef struct sci_bands_struct{
  sci_t band[SCI_NBANDS];
} sci_bands_t;

typedef struct sci_howfs_struct{
  sci_bands_t step[SCI_HOWFS_NSTEP];
} sci_howfs_t;

typedef struct sci_field_struct{
  double r[SCI_NPIX];
  double i[SCI_NPIX];
} sci_field_t;

typedef struct sci_cal{
  double *dark;
  double *bias;
} sci_cal_t;

typedef struct shk_struct{
  uint8 data[SHKXS][SHKYS];
} shk_t;

typedef struct lyt_struct{
  double data[LYTXS][LYTYS];
} lyt_t;

typedef struct lytread_struct{
  uint16 data[LYTREADXS][LYTREADYS];
} lytread_t;

typedef struct lytdark_struct{
  double data[LYTREADXS][LYTREADYS];
} lytdark_t;

typedef struct lytref_struct{
  double refimg[LYTXS][LYTYS]; //current reference image
  double refmod[LYTXS][LYTYS]; //model reference image
  double refdef[LYTXS][LYTYS]; //default reference image
  uint16 pxmask[LYTXS][LYTYS]; //pixel mask
} lytref_t;

typedef struct acq_struct{
  uint8 data[ACQXS][ACQYS];
} acq_t;

typedef struct acqread_struct{
  uint8 data[ACQREADXS][ACQREADYS];
} acqread_t;


/*************************************************
 * Device Command Structures
 *************************************************/
typedef struct alp_struct{
  double acmd[ALP_NACT];
  double zcmd[LOWFS_N_ZERNIKE];
} alp_t;

typedef struct hex_struct{
  double acmd[HEX_NAXES];
  double zcmd[LOWFS_N_ZERNIKE];
} hex_t;

typedef struct bmc_struct{
  float acmd[BMC_NACT];
  float tcmd[BMC_NTEST];
  float pad;
} bmc_t;

typedef struct htr_struct{
  char   name[MAX_COMMAND]; //Heater name
  uint8  adc;       //ADC number {1,2,3}
  uint8  ch;        //ADC channel index
  uint8  power;     //Heater power [0-100%]
  uint8  maxpower;  //Heater max power [0-100%]
  uint8  override;  //User override flag
  uint8  overpower; //User override power
  uint8  enable;    //Heater enable flag
  uint8  pad1;   
  float  gain;     //Control gain
  float  temp;     //Sensor temperature [C]
  float  setpoint; //Sensor setpoint [C]
  float  deadband; //Control deadband [C]
} htr_t;

typedef struct hum_struct{
  float humidity;
  float temp;
} hum_t;

//BMC Status Struct: NOTE this is a copy from libbmc.h DO NOT CHANGE
//                   It is here so that the GSE software can parse the packet.
typedef struct bmc_status_struct {
  uint8_t power;
  uint8_t supply_5va;
  uint8_t supply_5vd;
  uint8_t supply_hv; 
  uint8_t leds;
  uint8_t tec;
  uint8_t fan;
  uint8_t over_temp;
  //----
  uint8_t fw_major;
  uint8_t fw_minor;
  uint8_t range;
  uint8_t volt_3v3;
  uint8_t volt_5;
  uint8_t pad1;
  uint8_t pad2;
  uint8_t pad3;
  //----
  uint32_t serial;
  float voltage_input_v;
  //----
  float rail_3v1_v;
  float rail_3v2_v;
  //----
  float rail_5v1_v;
  float rail_5v2_v;
  //----
  float current_ma;
  float main_brd_temp_c;
  //----
  float top_brd_temp_c;
  float mid_brd_temp_c;
  //----
  float bot_brd_temp_c;
  float heatsink_temp_c;
  //----
  float ambient_temp_c;
  float sock1_temp_c;
  //----
  float sock2_temp_c;
  float hv_ref_v;
  //----
  float testpoint_v[BMC_NTEST];
  float ic_temp_c[BMC_NTEST];
  //----
  float hv_supp_v[2];
} bmc_status_t;

/*************************************************
 * Calmode Structures
 *************************************************/
typedef struct calmode_struct{
  char   name[MAX_COMMAND];
  char   cmd[MAX_COMMAND];
  int    shk_boxsize_cmd;
  int    shk_ncalim;
  int    lyt_ncalim;
  int    sci_ncalim;
  double shk_poke;
  double shk_zpoke[LOWFS_N_ZERNIKE];
  double lyt_poke;
  double lyt_zpoke[LOWFS_N_ZERNIKE];
  double sci_poke;
  double sci_vpoke;
} calmode_t;

typedef struct alpcal_struct{
  uint64 countA[ALP_NCALMODES];
  uint64 countB[ALP_NCALMODES];
  alp_t  alp_start[ALP_NCALMODES];
  struct timespec start[ALP_NCALMODES];
  double last_zernike[LOWFS_N_ZERNIKE];
  double zernike_errors[LOWFS_N_ZERNIKE][ZERNIKE_ERRORS_NUMBER];
  double timer_length;
} alpcal_t;

typedef struct bmccal_struct{
  uint64 countA[BMC_NCALMODES];
  uint64 countB[BMC_NCALMODES];
  bmc_t  bmc_start[BMC_NCALMODES];
  struct timespec start[BMC_NCALMODES];
  double timer_length;
} bmccal_t;

/*************************************************
 * Packet Header
 *************************************************/
#define PICC_PKT_VERSION     41  //packet version number
typedef struct pkthed_struct{
  uint16  version;       //packet version number
  uint16  type;          //packet ID word
  uint32  frame_number;  //image counter

  uint8   state;         //system state
  uint8   alp_commander; //alp commander client id
  uint8   hex_commander; //hex commander client id
  uint8   bmc_commander; //bmc commander client id
  float   exptime;       //commanded exposure time
  float   frmtime;       //commanded frame time
  float   ontime;        //measured frame time
  
  uint16  hex_calmode;   //hex calmode
  uint16  alp_calmode;   //alp calmode
  uint16  bmc_calmode;   //bmc calmode
  uint16  tgt_calmode;   //tgt calmode
  
  uint32  hex_calstep;   //hex calstep
  uint32  alp_calstep;   //alp calstep
  uint32  bmc_calstep;   //bmc calstep
  uint32  tgt_calstep;   //tgt calstep

  int64   start_sec;     //event start time
  int64   start_nsec;    //event start time
  int64   end_sec;       //event end time
  int64   end_nsec;      //event end time
} pkthed_t;

/*************************************************
 * Event Structures
 *************************************************/
typedef struct shkcell_struct{
  uint16    spot_found;
  uint16    spot_captured;
  uint16    maxval;
  uint16    boxsize;
  uint64    intensity;
  double    background;
  double    xorigin;
  double    yorigin;
  double    xtarget;
  double    ytarget;
  double    xcentroid;
  double    ycentroid;
  double    xorigin_deviation;
  double    yorigin_deviation;
  double    xtarget_deviation;
  double    ytarget_deviation;
  double    xcommand;
  double    ycommand;
} shkcell_t;

typedef struct shkevent_struct{
  pkthed_t  hed;
  shkcell_t cells[SHK_BEAM_NCELLS];
  uint32    boxsize;
  float     ccd_temp;
  uint32    nspot_found;
  uint32    nspot_captured;
  double    gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID];
  double    gain_alp_cell[LOWFS_N_PID];
  double    gain_hex_zern[LOWFS_N_PID];
  double    zernike_target[LOWFS_N_ZERNIKE];
  double    zernike_measured[LOWFS_N_ZERNIKE];
  double    zernike_calibrate[LOWFS_N_ZERNIKE];
  alp_t     alp;
  hex_t     hex;
} shkevent_t;

typedef struct pktcell_struct{
  uint16    spot_found;
  uint16    spot_captured;
  uint16    maxval;
  uint16    boxsize;
  uint32    intensity;
  float     background;
  float     xorigin;
  float     yorigin;
  float     xtarget;
  float     ytarget;
  float     xtarget_deviation[SHK_NSAMPLES];
  float     ytarget_deviation[SHK_NSAMPLES];
  float     xcommand[SHK_NSAMPLES];
  float     ycommand[SHK_NSAMPLES];
} pktcell_t;

typedef struct shkpkt_struct{
  pkthed_t  hed;
  pktcell_t cells[SHK_BEAM_NCELLS];
  float     ccd_temp;
  float     gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID];
  float     gain_alp_cell[LOWFS_N_PID];
  float     gain_hex_zern[LOWFS_N_PID];
  float     zernike_target[LOWFS_N_ZERNIKE];
  float     zernike_measured[LOWFS_N_ZERNIKE][SHK_NSAMPLES];
  float     alp_acmd[ALP_NACT];
  float     alp_zcmd[LOWFS_N_ZERNIKE][SHK_NSAMPLES];
  float     hex_acmd[HEX_NAXES];
  float     hex_zcmd[LOWFS_N_ZERNIKE];
  uint8     zernike_control[LOWFS_N_ZERNIKE];
  uint8     padding1;
  uint32    nsamples;
} shkpkt_t;

typedef struct lytevent_struct{
  pkthed_t  hed;
  float     ccd_temp;
  uint16    xorigin;
  uint16    yorigin;
  double    xcentroid;
  double    ycentroid;
  uint32    locked;
  uint32    background;
  double    gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID];
  double    zernike_measured[LOWFS_N_ZERNIKE];
  double    zernike_target[LOWFS_N_ZERNIKE];
  double    zernike_calibrate[LOWFS_N_ZERNIKE];
  double    alp_measured[ALP_NACT];
  alp_t     alp;
  lyt_t     image;
} lytevent_t;

typedef struct lytpkt_struct{
  pkthed_t  hed;
  float     ccd_temp;
  uint16    xorigin;
  uint16    yorigin;
  float     xcentroid[LYT_NSAMPLES];
  float     ycentroid[LYT_NSAMPLES];
  uint32    locked;
  uint32    background;
  float     gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID];
  float     zernike_target[LOWFS_N_ZERNIKE];
  float     zernike_measured[LOWFS_N_ZERNIKE][LYT_NSAMPLES];
  float     alp_acmd[ALP_NACT];
  float     alp_zcmd[LOWFS_N_ZERNIKE][LYT_NSAMPLES];
  uint32    nsamples;
  uint8     zernike_control[LOWFS_N_ZERNIKE];
  uint8     padding1;
  lyt_t     image;
} lytpkt_t;

typedef struct scievent_struct{
  pkthed_t     hed;
  float        ccd_temp;
  float        backplane_temp;
  float        tec_power;
  int16        tec_setpoint;
  uint8        tec_enable;
  uint8        ihowfs;
  uint32       xorigin[SCI_NBANDS];
  uint32       yorigin[SCI_NBANDS];
  double       refmax[SCI_NBANDS];       
  sci_bands_t  bands;
  bmc_t        bmc;
  bmc_status_t bmc_status;
} scievent_t;

typedef struct acqevent_struct{
  pkthed_t  hed;
  uint16    xcen;
  uint16    ycen;
  uint32    padding;
  hex_t     hex;
  acq_t     image;
} acqevent_t;

typedef struct wfsevent_struct{
  pkthed_t      hed;
  sci_field_t   field[SCI_NBANDS];
} wfsevent_t;

typedef struct thmevent_struct{
  pkthed_t  hed;
  float     cpu1_temp;
  float     cpu2_temp;
  float     adc1_temp[ADC1_NCHAN];
  float     adc2_temp[ADC2_NCHAN];
  float     adc3_temp[ADC3_NCHAN];
  htr_t     htr[SSR_NCHAN];
  hum_t     hum[HUM_NSENSORS];
} thmevent_t;

typedef struct mtrevent_struct{
  pkthed_t  hed;
  uint16_t  door_status[MTR_NDOORS];
} mtrevent_t;

typedef struct msgevent_struct{
  pkthed_t  hed;
  char      message[MAX_LINE];
} msgevent_t;

/*************************************************
 * Full Frame Structures
 *************************************************/
typedef struct shkfull_struct{
  pkthed_t   hed;
  shk_t      image;
  shkevent_t shkevent;
} shkfull_t;

typedef struct acqfull_struct{
  pkthed_t  hed;
  acqread_t image;
} acqfull_t;


/*************************************************
 * Circular Buffer Structure
 *************************************************/
typedef struct circbuf_struct{
  volatile void *buffer;
  uint32 read_offsets[NCLIENTS]; //last entry read
  uint32 write_offset; //last entry written
  uint32 nbytes;    //number of bytes in structure
  uint32 bufsize;   //number of structures in circular buffer
  int    write;     //switch to enable writing to buffer
  int    read;      //switch to enable TLM reading buffer
  int    send;      //switch to enable TLM sending buffer
  int    save;      //switch to enable TLM saving buffer
  char   name[128]; //name of buffer
} circbuf_t;

/*************************************************
 * Shared Memory Layout
 *************************************************/
typedef volatile struct {

  //Runtime switches
  int die;            //Kill all processes

  //Process information
  procinfo_t w[NCLIENTS];

  //Device ready flags
  int alp_ready;
  int bmc_ready;
  int hex_ready;
  int tlm_ready;
  
  //RTD board descriptor
  DM7820_Board_Descriptor* p_rtd_alp_board;
  DM7820_Board_Descriptor* p_rtd_tlm_board;

  //BMC file descriptor
  libbmc_device_t libbmc_device;

  //Hexapod file descriptor
  int hexfd;

  //State
  int state;                    //Current operational state
  state_t state_array[NSTATES]; //Array of states

  //Camera exposure & frame times
  float sci_exptime;
  float sci_frmtime;
  float shk_exptime;
  float shk_frmtime;
  float lyt_exptime;
  float lyt_frmtime;
  float acq_exptime;
  float acq_frmtime;
   
  //ALP Command
  int   alp_command_lock;
  alp_t alp_command;
  int   alp_proc_id;
  int   alp_n_dither;
  alp_t alp_shk2lyt;
  int   alp_shk2lyt_lock;
  int   alp_shk2lyt_set;
  

  //BMC Command
  int   bmc_command_lock;
  bmc_t bmc_command;
  bmc_t bmc_flat[BMC_NFLAT];
  uint32_t bmc_iflat;

  //HEX Command
  int   hex_command_lock;
  hex_t hex_command;

  //Calibration Modes
  int alp_calmode;
  int hex_calmode;
  int bmc_calmode;
  int tgt_calmode;

  //ALP Calibration Structure
  alpcal_t alpcal;

  //BMC Calibration Structure
  bmccal_t bmccal;

  //DM Calibration Command Scale
  double alp_cal_scale;
  double bmc_cal_scale;

  //Calibration file name
  char calfile[MAX_FILENAME];

  //Shack-Hartmann Settings
  int shk_boxsize;                                         //SHK centroid boxsize
  double shk_gain_alp_cell[LOWFS_N_PID];                   //SHK ALP cell gains
  double shk_gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID];  //SHK ALP zern gains
  double shk_gain_hex_zern[LOWFS_N_PID];                   //SHK HEX zern gains
  int    shk_alp_pid_type;                                 //Double or single integrator

  //Lyot LOWFS Settings
  double lyt_gain_alp_zern[LOWFS_N_ZERNIKE][LOWFS_N_PID];  //LYT ALP zernike PID gains
  int    lyt_xorigin;                                      //LYT ROI bottom-left X
  int    lyt_yorigin;                                      //LYT ROI bottom-left Y
  int    lyt_mag_enable;                                   //LYT Magnification switch
  double lyt_mag;                                          //LYT Magnification
  double lyt_mag_xoff;                                     //LYT Magnification X offset
  double lyt_mag_yoff;                                     //LYT Magnification Y offset
  int    lyt_roi[4];                                       //LYT ROI
  int    lyt_cen_enable;                                   //LYT Enable centroid control
  int    lyt_alp_pid_type;                                 //Double or single integrator

  //SCI Settings
  uint32 sci_xorigin[SCI_NBANDS];                          //SCI ROI center X
  uint32 sci_yorigin[SCI_NBANDS];                          //SCI ROI center Y
  
  //Camera Process Reset Commands
  int shk_reset;
  int acq_reset;
  int sci_reset;
  int lyt_reset;

  //Camera Exposure Reset Commands
  int shk_reset_camera;
  int acq_reset_camera;
  int sci_reset_camera;
  int lyt_reset_camera;
  
  //SCI Commands
  int sci_setorigin;
  int sci_findorigin;
  int sci_trackorigin;
  int sci_revertorigin;
  int sci_saveorigin;
  int sci_loadorigin;
  int sci_xshiftorigin;
  int sci_yshiftorigin;
  int sci_setref;

  //SHK Commands
  int shk_setorigin;
  int shk_revertorigin;
  int shk_saveorigin;
  int shk_loadorigin;
  int shk_xshiftorigin;
  int shk_yshiftorigin;
  
  //LYT Commands
  int lyt_setref;
  int lyt_defref;
  int lyt_modref;
  int lyt_saveref;
  int lyt_loadref;
  int lyt_setdark;
  int lyt_zerodark;
  int lyt_savedark;
  int lyt_loaddark;
  int lyt_subdark;
  
  //Camera Telemetry
  float shk_ccd_temp;
  float lyt_ccd_temp;
  float sci_ccd_temp;
  float sci_backplane_temp;
  float sci_tec_power;
  int   sci_tec_setpoint;
  int   sci_tec_enable;
  
  //Other Commands
  int acq_thresh;
  int thm_enable_vref;
  int hex_spiral_autostop;
  int bmc_hv_enable;
  int bmc_hv_on;

  //EFC Parameters
  double efc_sci_thresh;
  double efc_bmc_max;
  double efc_gain;
  int    efc_probe_amp;

  //Door Commands
  int open_door[MTR_NDOORS];
  int close_door[MTR_NDOORS];
  int stop_door[MTR_NDOORS];
  
  //Heater Settings
  htr_t htr[SSR_NCHAN];
        
  //Zernike Targets
  double shk_zernike_target[LOWFS_N_ZERNIKE];
  double lyt_zernike_target[LOWFS_N_ZERNIKE];

  //Zernike control switches
  int shk_zernike_control[LOWFS_N_ZERNIKE];
  int lyt_zernike_control[LOWFS_N_ZERNIKE];
  int alp_zernike_control[LOWFS_N_ZERNIKE]; //for calibration

  //Events circular buffers
  scievent_t scievent[SCIEVENTSIZE];
  wfsevent_t wfsevent[WFSEVENTSIZE];
  shkevent_t shkevent[SHKEVENTSIZE];
  lytevent_t lytevent[LYTEVENTSIZE];
  acqevent_t acqevent[ACQEVENTSIZE];
  thmevent_t thmevent[THMEVENTSIZE];
  mtrevent_t mtrevent[MTREVENTSIZE];
  msgevent_t msgevent[MSGEVENTSIZE];
  shkpkt_t   shkpkt[SHKPKTSIZE];
  lytpkt_t   lytpkt[LYTPKTSIZE];

  //Full frame circular buffers
  shkfull_t shkfull[SHKFULLSIZE];
  acqfull_t acqfull[ACQFULLSIZE];
    
  //Circular buffer package
  circbuf_t circbuf[NCIRCBUF];

} sm_t;


#endif
