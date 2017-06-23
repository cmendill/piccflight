/*************************************************
 * Client ID Runtime Info
 *************************************************/

/*!!!!!!!!!! ALL NUMBERS MUST BE < 255 !!!!!!!!!*/
//      procids {WATID, SCIID, SHKID, LYTID, TLMID, ACQID, MOTID, THMID, SRVID, TMPID, HSKID, HEXID, DIAID}; 
#define PROCRUN {    1,     0,     1,     0,     0,     0,     0,     0,     1,     0,     0,     1,     0}
#define PROCASK {    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0}
#define PROCTMO {    5,     5,     5,     5,     5,     5,     5,     5,     5,     5,     5,     5,     5}
#define PROCNAM {"WAT", "SCI", "SHK", "LYT", "TLM", "ACQ", "MOT", "THM", "SRV", "TMP", "HSK", "HEX", "DIA"} 
#define PROCMOD {   "",    "",    "",    "",    "",    "",    "",    "",    "",    "",    "",    "",    ""} 
#define PROCPRI {    1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1}
#define PROCPER {    1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1}


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
#define SHK_BOXSIZE_DEFAULT     7
#define SHK_FIT_ZERNIKE_DEFAULT 0
#define SHK_KP_DEFAULT -0.5
#define SHK_KI_DEFAULT  0.0
#define SHK_KD_DEFAULT  0.0
