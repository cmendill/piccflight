#ifndef _WATCHDOG
#define _WATCHDOG

/*************************************************
 * Client ID Runtime Info
 *************************************************/

/*!!!!!!!!!! ALL NUMBERS MUST BE < 255 !!!!!!!!!*/
//      procids {WATID, SCIID, SHKID, LYTID, TLMID, ACQID, MOTID, THMID, SRVID, HEXID, DIAID};
#define PROCRUN {    1,     1,     1,     0,     0,     1,     0,     0,     1,     1,     0}
#define PROCASK {    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0}
#define PROCTMO {    5,     5,     5,     5,     5,     5,     5,     5,     5,     5,     5}
#define PROCNAM {"WAT", "SCI", "SHK", "LYT", "TLM", "ACQ", "MOT", "THM", "SRV", "HEX", "DIA"}
#define PROCMOD {   "",    "",    "",    "",    "",    "",    "",    "",    "",    "",    ""}
#define PROCPRI {    1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1}
#define PROCPER {    1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1}


/*************************************************
 * Shared Memory DEFAULTS
 *************************************************/
//Camera modes
#define SCI_MODE_DEFAULT 0
#define SHK_MODE_DEFAULT 0
#define LYT_MODE_DEFAULT 0
#define ACQ_MODE_DEFAULT 0

//Shack-Hartmann Settings
#define SHK_BOXSIZE_DEFAULT     7
#define SHK_KP_CELL_DEFAULT  -0.5
#define SHK_KI_CELL_DEFAULT  -0.2
#define SHK_KD_CELL_DEFAULT   0.0
#define SHK_KP_ZERN_DEFAULT  -0.5
#define SHK_KI_ZERN_DEFAULT  -0.2
#define SHK_KD_ZERN_DEFAULT   0.0

/*************************************************
 * System Settings & Messages
 *************************************************/
#define WARNING   "WARNING...WARNING...WARNING...WARNING\nWARNING...WARNING...WARNING...WARNING\nWARNING...WARNING...WARNING...WARNING\nWARNING...WARNING...WARNING...WARNING\n"
#define REBOOT   "REBOOT...REBOOT...REBOOT...REBOOT\nREBOOT...REBOOT...REBOOT...REBOOT\nREBOOT...REBOOT...REBOOT...REBOOT\nREBOOT...REBOOT...REBOOT...REBOOT\n"
#define EXIT_TIMEOUT    25  //procwait exit timeout
#define PROC_TIMEOUT    5   //procwait process timeout
#define ERASE_TIMEOUT   25  //Time to wait for TLM to exit on command: erase flight data

#endif
