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

//Exposure times
#define SCI_EXPTIME_DEFAULT 0.010
#define SHK_EXPTIME_DEFAULT 0.050
#define LYT_EXPTIME_DEFAULT 0.002
#define ACQ_EXPTIME_DEFAULT 0.200


//Shack-Hartmann Settings
#define SHK_BOXSIZE_DEFAULT        7

//SHK LOWFS Gains                       P           I            D
#define SHK_GAIN_ALP_CELL_DEFAULT {    -0.5,      -0.05,       0.0}
#define SHK_GAIN_HEX_ZERN_DEFAULT {   -0.04,        0.0,       0.0}
#define SHK_GAIN_ALP_ZERN_DEFAULT {
                                  {-0.500000,  -0.100000,       0.0}, \ //0
                                  {-0.500000,  -0.100000,       0.0}, \ //1
                                  {-0.500000,  -0.100000,       0.0}, \ //2
                                  {-0.500000,  -0.100000,       0.0}, \ //3
                                  {-0.500000,  -0.100000,       0.0}, \ //4
                                  {-0.500000,  -0.100000,       0.0}, \ //5
                                  {-0.500000,  -0.100000,       0.0}, \ //6
                                  {-0.500000,  -0.100000,       0.0}, \ //7
                                  {-0.500000,  -0.100000,       0.0}, \ //8
                                  {-0.500000,  -0.100000,       0.0}, \ //9
                                  {-0.500000,  -0.100000,       0.0}, \ //10
                                  {-0.500000,  -0.100000,       0.0}, \ //11
                                  {-0.500000,  -0.100000,       0.0}, \ //12
                                  {-0.500000,  -0.100000,       0.0}, \ //13
                                  {-0.500000,  -0.100000,       0.0}, \ //14
                                  {-0.500000,  -0.100000,       0.0}, \ //15
                                  {-0.500000,  -0.100000,       0.0}, \ //16
                                  {-0.500000,  -0.100000,       0.0}, \ //17
                                  {-0.500000,  -0.100000,       0.0}, \ //18
                                  {-0.500000,  -0.100000,       0.0}, \ //19
                                  {-0.500000,  -0.100000,       0.0}, \ //20
                                  {-0.500000,  -0.100000,       0.0}, \ //21
                                  {-0.500000,  -0.100000,       0.0}, \ //22
                                  {-0.500000,  -0.100000,       0.0}}   //23
				    

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
