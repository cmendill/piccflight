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

//SWITCHES
#define DATA_SEND   1
#define SCI_SEND    1
#define DM_SEND     1
#define IWC_SEND    1
#define LYOT_SEND   1
#define SH_SEND     1
