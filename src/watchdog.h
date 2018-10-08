#ifndef _WATCHDOG
#define _WATCHDOG

/*************************************************
 * Client ID Runtime Info
 *************************************************/

/*!!!!!!!!!! ALL NUMBERS MUST BE < 255 !!!!!!!!!*/
//      procids {WATID, SCIID, SHKID, LYTID, TLMID, ACQID, MOTID, THMID, SRVID, DIAID};
#define PROCRUN {    1,     0,     1,     1,     1,     1,     0,     0,     1,     0}
#define PROCASK {    0,     0,     0,     0,     0,     0,     0,     0,     0,     0}
#define PROCTMO {    5,     5,     5,     5,     5,     5,     5,     5,     5,     5}
#define PROCNAM {"WAT", "SCI", "SHK", "LYT", "TLM", "ACQ", "MOT", "THM", "SRV", "DIA"}
#define PROCMOD {   "",    "",    "",    "",    "",    "",    "",    "",    "",    ""}
#define PROCPRI {    1,     1,     1,     1,     1,     1,     1,     1,     1,     1}
#define PROCPER {    1,     1,     1,     1,     1,     1,     1,     1,     1,     1}


/*************************************************
 * Shared Memory DEFAULTS
 *************************************************/
//Camera modes
#define SCI_MODE_DEFAULT 0
#define SHK_MODE_DEFAULT 0
#define LYT_MODE_DEFAULT 0
#define ACQ_MODE_DEFAULT 0

//Shack-Hartmann Settings
#define SHK_BOXSIZE_DEFAULT       7
#define SHK_KP_ALP_CELL_DEFAULT  -0.5
#define SHK_KI_ALP_CELL_DEFAULT  -0.2
#define SHK_KD_ALP_CELL_DEFAULT   0.0
#define SHK_KP_ALP_ZERN_DEFAULT  -0.5
#define SHK_KI_ALP_ZERN_DEFAULT  -0.2
#define SHK_KD_ALP_ZERN_DEFAULT   0.0
#define SHK_KP_HEX_ZERN_DEFAULT  -0.04
#define SHK_KI_HEX_ZERN_DEFAULT   0.0
#define SHK_KD_HEX_ZERN_DEFAULT   0.0

//Lyot LOWFS Gains                      P           I            D
#define LYT_GAIN_ALP_ZERN_DEFAULT {                                   \
				  {-0.500000,  -0.100000,       0.0}, \
				  {-0.500000,  -0.100000,       0.0}, \
				  {-0.500000,  -0.100000,       0.0}, \
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
				  {-0.000025,  -0.000005,       0.0}, \
				  {-0.000025,  -0.000005,       0.0}, \
				  {-0.000025,  -0.000005,       0.0}, \
				  {-0.000025,  -0.000005,       0.0}, \
				  {-0.000025,  -0.000005,       0.0}, \
				  {-0.000025,  -0.000005,       0.0}, \
				  {-0.000025,  -0.000005,       0.0}}

#define LYT_GAIN_ALP_ACT_DEFAULT {-0.5,-0.1,0.0}


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
