/*************************************************
 * TLM Parameters
 *************************************************/
//RTD Board
#define RTD_DMA 1 //Use DMA for RTD data transfers

//DATA ID
#define TLM_SCIENCE 0xAAAA0000
   
//SYNCWORDS
#define TLM_PRESYNC   0x12345678
#define TLM_POSTSYNC  0xDEADBEEF

//SEND SWITCHES
#define SCI_SEND    1
#define SHK_SEND    1
#define LYT_SEND    1
#define ACQ_SEND    1
#define  DM_SEND    1
#define IWC_SEND    1

//SAVE SWITCHES
#define SCI_SAVE    1
#define SHK_SAVE    1
#define LYT_SAVE    1
#define ACQ_SAVE    1
#define  DM_SAVE    1
#define IWC_SAVE    1
