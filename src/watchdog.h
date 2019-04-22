#ifndef _WATCHDOG
#define _WATCHDOG

/*************************************************
 * Client ID Runtime Info
 *************************************************/

/*!!!!!!!!!! ALL NUMBERS MUST BE < 255 !!!!!!!!!*/
//      procids {WATID, SCIID, SHKID, LYTID, TLMID, ACQID, MTRID, THMID, SRVID, DIAID};
#define PROCRUN {    1,     1,     1,     1,     1,     1,     1,     1,     1,     0}
#define PROCASK {    0,     0,     0,     0,     0,     0,     0,     0,     0,     1}
#define PROCTMO {   10,    10,    10,    10,    10,    10,    10,    10,    10,    10}
#define PROCNAM {"WAT", "SCI", "SHK", "LYT", "TLM", "ACQ", "MTR", "THM", "SRV", "DIA"}
#define PROCPER {    1,     1,     1,     1,     1,     1,     1,     1,     1,     1}


/*************************************************
 * Shared Memory DEFAULTS
 *************************************************/

//Circular Buffer Switches
#define WRITE_SCIEVENT_DEFAULT     1
#define WRITE_SHKEVENT_DEFAULT     0
#define WRITE_LYTEVENT_DEFAULT     0
#define WRITE_ACQEVENT_DEFAULT     1
#define WRITE_THMEVENT_DEFAULT     1
#define WRITE_MTREVENT_DEFAULT     1
#define WRITE_SHKPKT_DEFAULT       1
#define WRITE_LYTPKT_DEFAULT       1
#define WRITE_SHKFULL_DEFAULT      0
#define WRITE_ACQFULL_DEFAULT      0

#define SEND_SCIEVENT_DEFAULT      1
#define SEND_SHKEVENT_DEFAULT      0
#define SEND_LYTEVENT_DEFAULT      0
#define SEND_ACQEVENT_DEFAULT      2 //2 means always send newest data
#define SEND_THMEVENT_DEFAULT      1
#define SEND_MTREVENT_DEFAULT      1
#define SEND_SHKPKT_DEFAULT        1
#define SEND_LYTPKT_DEFAULT        1
#define SEND_SHKFULL_DEFAULT       0
#define SEND_ACQFULL_DEFAULT       0

#define SAVE_SCIEVENT_DEFAULT      1
#define SAVE_SHKEVENT_DEFAULT      0
#define SAVE_LYTEVENT_DEFAULT      0
#define SAVE_ACQEVENT_DEFAULT      0
#define SAVE_THMEVENT_DEFAULT      1
#define SAVE_MTREVENT_DEFAULT      1
#define SAVE_SHKPKT_DEFAULT        1
#define SAVE_LYTPKT_DEFAULT        1
#define SAVE_SHKFULL_DEFAULT       0
#define SAVE_ACQFULL_DEFAULT       0

//Exposure times
#define SCI_EXPTIME_DEFAULT 0.010
#define SCI_FRMTIME_DEFAULT 0.010
#define SHK_EXPTIME_DEFAULT 0.025  //40hz
#define SHK_FRMTIME_DEFAULT 0.025  //40hz
#define LYT_EXPTIME_DEFAULT 0.0025 //400hz
#define LYT_FRMTIME_DEFAULT 0.0025 //400hz
#define ACQ_EXPTIME_DEFAULT 0.200
#define ACQ_FRMTIME_DEFAULT 0.200

//LYT ROI Settings
#define LYT_XORIGIN_DEFAULT 16
#define LYT_YORIGIN_DEFAULT 16

//Shack-Hartmann Settings
#define SHK_BOXSIZE_DEFAULT        7

//SHK LOWFS Gains                       P           I            D
#define SHK_GAIN_ALP_CELL_DEFAULT {      -0.5,      -0.05,       0.0}
#define SHK_GAIN_HEX_ZERN_DEFAULT {     -0.04,        0.0,       0.0}
#define SHK_GAIN_ALP_ZERN_DEFAULT {                                    \
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
				   {-0.500000,  -0.100000,       0.0}, \
				   {-0.500000,  -0.100000,       0.0}, \
				   {-0.500000,  -0.100000,       0.0}}  
  
				    

//Lyot LOWFS Gains                      P           I            D
#define LYT_GAIN_ALP_ZERN_DEFAULT {                                    \
                                   {-0.600000,  -0.100000,       0.0}, \
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
