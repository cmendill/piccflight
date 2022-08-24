#ifndef _WATCHDOG
#define _WATCHDOG

/*************************************************
 * Client ID Runtime Info
 *************************************************/

/*!!!!!!!!!! ALL NUMBERS MUST BE < 255 !!!!!!!!!*/
//      procids {WATID, SCIID, SHKID, LYTID, TLMID, ACQID, MTRID, THMID, MSGID, DIAID};
#define PROCRUN {    1,     1,     1,     1,     1,     1,     1,     1,     1,     0}
#define PROCASK {    0,     0,     0,     0,     0,     0,     0,     0,     0,     1}
#define PROCTMO {   10,    10,    10,    10,    10,    10,    10,    10,    10,    10}
#define PROCNAM {"WAT", "SCI", "SHK", "LYT", "TLM", "ACQ", "MTR", "THM", "MSG", "DIA"}
#define PROCPER {    1,     1,     1,     1,     1,     1,     1,     1,     1,     1}


/*************************************************
 * Shared Memory DEFAULTS
 *************************************************/

//Circular Buffer Switches
#define WRITE_SCIEVENT_DEFAULT     1
#define WRITE_WFSEVENT_DEFAULT     1
#define WRITE_SHKEVENT_DEFAULT     0
#define WRITE_LYTEVENT_DEFAULT     0
#define WRITE_ACQEVENT_DEFAULT     1
#define WRITE_THMEVENT_DEFAULT     1
#define WRITE_MTREVENT_DEFAULT     1
#define WRITE_MSGEVENT_DEFAULT     1
#define WRITE_SHKPKT_DEFAULT       1
#define WRITE_LYTPKT_DEFAULT       1
#define WRITE_SHKFULL_DEFAULT      0
#define WRITE_ACQFULL_DEFAULT      0

#define READ_SCIEVENT_DEFAULT      1
#define READ_WFSEVENT_DEFAULT      1
#define READ_SHKEVENT_DEFAULT      0
#define READ_LYTEVENT_DEFAULT      0
#define READ_ACQEVENT_DEFAULT      2 //2 means always read newest data
#define READ_THMEVENT_DEFAULT      1
#define READ_MTREVENT_DEFAULT      1
#define READ_MSGEVENT_DEFAULT      1
#define READ_SHKPKT_DEFAULT        1
#define READ_LYTPKT_DEFAULT        1
#define READ_SHKFULL_DEFAULT       0
#define READ_ACQFULL_DEFAULT       0

#define SEND_SCIEVENT_DEFAULT      1
#define SEND_WFSEVENT_DEFAULT      1
#define SEND_SHKEVENT_DEFAULT      0
#define SEND_LYTEVENT_DEFAULT      0
#define SEND_ACQEVENT_DEFAULT      1
#define SEND_THMEVENT_DEFAULT      1
#define SEND_MTREVENT_DEFAULT      1
#define SEND_MSGEVENT_DEFAULT      1
#define SEND_SHKPKT_DEFAULT        1
#define SEND_LYTPKT_DEFAULT        1
#define SEND_SHKFULL_DEFAULT       0
#define SEND_ACQFULL_DEFAULT       0

#define SAVE_SCIEVENT_DEFAULT      1
#define SAVE_WFSEVENT_DEFAULT      1
#define SAVE_SHKEVENT_DEFAULT      0
#define SAVE_LYTEVENT_DEFAULT      0
#define SAVE_ACQEVENT_DEFAULT      0
#define SAVE_THMEVENT_DEFAULT      1
#define SAVE_MTREVENT_DEFAULT      1
#define SAVE_MSGEVENT_DEFAULT      1
#define SAVE_SHKPKT_DEFAULT        1
#define SAVE_LYTPKT_DEFAULT        1
#define SAVE_SHKFULL_DEFAULT       0
#define SAVE_ACQFULL_DEFAULT       0

//Exposure times
#define SCI_EXPTIME_DEFAULT        0.010
#define SCI_FRMTIME_DEFAULT        0.010
#define SHK_EXPTIME_DEFAULT        0.025  //40hz
#define SHK_FRMTIME_DEFAULT        0.025  //40hz
#define LYT_EXPTIME_DEFAULT        0.0025 //400hz
#define LYT_FRMTIME_DEFAULT        0.0025 //400hz
#define ACQ_EXPTIME_DEFAULT        0.200
#define ACQ_FRMTIME_DEFAULT        0.200

//DM Calibration Command Scale
#define ALP_CAL_SCALE_DEFAULT 1
#define BMC_CAL_SCALE_DEFAULT 1

//EFC Settings
#define EFC_BMC_MAX_DEFAULT        50   //nm
#define EFC_SCI_THRESH_DEFAULT     0    //ADU
#define EFC_GAIN_DEFAULT          -0.5
#define EFC_PROBE_AMP_DEFAULT      30   //nm
#define EFC_MATRIX_DEFAULT         0

//Speckle Nulling Settings
#define SPECKLE_SCALE_DEFAULT      1

//LYT Settings
#define LYT_XORIGIN_DEFAULT        (LYTREADXS - LYTXS)/2
#define LYT_YORIGIN_DEFAULT        (LYTREADYS - LYTYS)/2
#define LYT_ROI_X_DEFAULT          0
#define LYT_ROI_Y_DEFAULT          30
#define LYT_CEN_ENABLE_DEFAULT     1

//SCI Setting
#define SCI_XORIGIN_DEFAULT        {1293} //band cutout x centers, must match SCI_NBANDS
#define SCI_YORIGIN_DEFAULT        {1111} //band cutout y centers, must match SCI_NBANDS
#define SCI_TEC_ENABLE_DEFAULT     1
#define SCI_TEC_SETPOINT_DEFAULT   20
#define SCI_PHASE_N_ZERNIKE_DEFAULT 10
#define SCI_PHASE_EXPSCALE_DEFAULT  50

//Timers
#define ALP_CAL_TIMER_LENGTH_DEFAULT 30
#define BMC_CAL_TIMER_LENGTH_DEFAULT 30

//Shack-Hartmann Settings
#define SHK_BOXSIZE_DEFAULT        7

//SHK LOWFS Gains                       P           I            D
#define SHK_GAIN_HEX_ZERN_DEFAULT {     -0.04,       0.0,       0.0}
#define SHK_GAIN_ALP_CELL_DEFAULT {      -0.5,      -0.05,      0.0}
#define SHK_GAIN_ALP_ZERN_DEFAULT {					\
                                     {-0.500000,  -0.100000,       0.0},					\
                                     {-0.500000,  -0.100000,       0.0},					\
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}, \
				     {-0.500000,  -0.100000,       0.0}}  
  
				    
//Lyot LOWFS Gains                      P           I            D
#define LYT_GAIN_ALP_ZERN_DEFAULT {					\
                                     {-0.600000,  -0.100000,       0.0},					\
				     {-0.600000,  -0.100000,       0.0}, \
				     {-0.600000,  -0.100000,       0.0}, \
				     {-0.050000,  -0.010000,       0.0}, \
				     {-0.050000,  -0.010000,       0.0}, \
				     {-0.050000,  -0.010000,       0.0}, \
				     {-0.050000,  -0.010000,       0.0}, \
				     {-0.050000,  -0.010000,       0.0}, \
				     {-0.050000,  -0.010000,       0.0}, \
				     {-0.050000,  -0.010000,       0.0}, \
				     {-0.050000,  -0.010000,       0.0}, \
				     {-0.050000,  -0.010000,       0.0}, \
				     {-0.005000,  -0.001000,       0.0}, \
				     {-0.005000,  -0.001000,       0.0}, \
				     {-0.005000,  -0.001000,       0.0}, \
				     {-0.005000,  -0.001000,       0.0}, \
				     {-0.005000,  -0.001000,       0.0}, \
				     {-0.005000,  -0.001000,       0.0}, \
				     {-0.005000,  -0.001000,       0.0}, \
				     {-0.005000,  -0.001000,       0.0}, \
				     {-0.005000,  -0.001000,       0.0}, \
				     {-0.005000,  -0.001000,       0.0}, \
				     {-0.005000,  -0.001000,       0.0}}  

//ACQ Settings
#define ACQ_THRESH_DEFAULT        5

//THM Settings
#define THM_ENABLE_VREF_DEFAULT   1

//HEX Settings
#define HEX_SPIRAL_AUTOSTOP_DEFAULT 1  //0 --> Don't stop when star is found
                                       //1 --> Stop when star is found

#endif
