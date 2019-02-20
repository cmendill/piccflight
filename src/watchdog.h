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
#define PROCMOD {   "",    "",    "",    "",    "",    "",    "",    "",    "",    ""}
#define PROCPRI {    1,     1,     1,     1,     1,     1,     1,     1,     1,     1}
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
#define WRITE_SCIFULL_DEFAULT      0
#define WRITE_SHKFULL_DEFAULT      0
#define WRITE_LYTFULL_DEFAULT      0
#define WRITE_ACQFULL_DEFAULT      0
#define WRITE_SHKPKT_DEFAULT       1
#define WRITE_LYTPKT_DEFAULT       1

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
#define LYT_XORIGIN_DEFAULT 8
#define LYT_YORIGIN_DEFAULT 8

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
  
#define LYT_GAIN_ALP_ACT_DEFAULT   {     -0.5,       -0.1,       0.0}


//Hexapod Settings
#define HEX_TILT_CORRECT_DEFAULT  1

/*************************************************
 * System Settings & Messages
 *************************************************/
#define WARNING   "WARNING...WARNING...WARNING...WARNING\nWARNING...WARNING...WARNING...WARNING\nWARNING...WARNING...WARNING...WARNING\nWARNING...WARNING...WARNING...WARNING\n"
#define REBOOT   "REBOOT...REBOOT...REBOOT...REBOOT\nREBOOT...REBOOT...REBOOT...REBOOT\nREBOOT...REBOOT...REBOOT...REBOOT\nREBOOT...REBOOT...REBOOT...REBOOT\n"
#define EXIT_TIMEOUT    25  //procwait exit timeout
#define PROC_TIMEOUT    5   //procwait process timeout
#define ERASE_TIMEOUT   25  //Time to wait for TLM to exit on command: erase flight data

#endif
