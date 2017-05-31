/*************************************************
 * Client ID Runtime Info
 *************************************************/

/*!!!!!!!!!! ALL NUMBERS MUST BE < 255 !!!!!!!!!*/
//      NOTE: Keep DIAID at the end. And do not set to run. This ID is only used by diagnostic programs.
//      procids {WATID, SCIID, SHKID, LYTID, TLMID, ACQID, MOTID, THMID, SRVID, TMPID, HSKID, DIAID}; 
#define PROCRUN {    1,     0,     1,     1,     0,     0,     0,     0,     1,     0,     0,     0}
#define PROCASK {    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0}
#define PROCTMO {    5,     5,     5,     5,     5,     5,     5,     5,     5,     5,     5,     0}
#define PROCNAM {"WAT", "SCI", "SHK", "LYT", "TLM", "ACQ", "MOT", "THM", "SRV", "TMP", "HSK", "DIA"} 
#define PROCMOD {   "",    "",    "",    "",    "",    "",    "",    "",    "",    "",    "",    ""} //do we want to load modules this way?
#define PROCPRI {    1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1} //need to update
#define PROCPER {    1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1} //need to update


/*************************************************
 * Shared Memory DEFAULTS
 *************************************************/
//switches

//camera modes
#define SCI_MODE_DEFAULT 0 
#define SHK_MODE_DEFAULT 0 
#define LYT_MODE_DEFAULT 0 
#define ACQ_MODE_DEFAULT 0

//Shack-Hartmann Settings
#define SHK_BOXSIZE_DEFAULT 15
