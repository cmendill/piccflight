#ifndef _TLM_PROC
#define _TLM_PROC

/*************************************************
 * TLM Parameters
 *************************************************/
//SYNCWORDS
#define TLM_PRESYNC   0x12345678
#define TLM_POSTSYNC  0xDEADBEEF

//SEND SWITCHES
#define SEND_SCIEVENT  1
#define SEND_SHKPKT    1
#define SEND_LYTPKT    1
#define SEND_ACQEVENT  0

//SAVE SWITCHES
#define SAVE_SCIEVENT  1
#define SAVE_SHKPKT    1
#define SAVE_LYTPKT    1
#define SAVE_ACQEVENT  0

#endif
