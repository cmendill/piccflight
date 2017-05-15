/*************************************************
 * Client ID Runtime Info
 *************************************************/

/*!!!!!!!!!! ALL NUMBERS MUST BE < 255 !!!!!!!!!*/
//      procids {WATID, SCIID, SHKID, LYTID, TLMID, ACQID, MOTID, THMID, SRVID, TMPID, HSKID};
#define PROCRUN {    1,     0,     1,     1,     1,     1,     0,     1,     1,     1,     1}
#define PROCASK {    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0}
#define PROCTMO {    5,     5,     5,     5,     5,     5,     5,     5,     5,     5,     5}
#define PROCNAM {"WAT", "SCI", "SHK", "LYT", "TLM", "ACQ", "MOT", "THM", "SRV", "TMP", "HSK"} 
#define PROCMOD {   "",    "",    "",    "",    "",    "",    "",    "",    "",    "",    ""} //do we want to load modules this way?
#define PROCPRI {    1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1} //need to update
#define PROCPER {    1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1} //need to update


/*************************************************
 * Shared Memory DEFAULTS
 *************************************************/
//switches
#define FAKE_MODE_DEFAULT  0 // Fake data in various ways

//camera modes
#define SCI_MODE_DEFAULT 1
