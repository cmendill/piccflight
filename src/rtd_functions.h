//RTD Board
#define RTD_DMA          1                                 //Use DMA to write TM out parallel board
#define RTD_BUF_SIZE     0x8000                            //Individual segment size
#define RTD_BUF_NUM      16                                //Number of segments for each DMA channel
#define RTD_FRAME_NUM    16                                //Number of segments per frame
#define RTD_SAMPLES      ((RTD_BUF_SIZE*RTD_FRAME_NUM)/2)  //Number of samples per frame
#define RTD_DMA_BUF_SIZE (RTD_BUF_SIZE*RTD_FRAME_NUM)      //Size of DMA Buffer
#define RTD_BUF_SAMPLES  (RTD_BUF_SIZE/2)                  //Number of samples per segment
#define RTD_EMPTY_CODE   0xFADE                            //Code to send when there is no data
#define RTD_REPLACE_CODE 0xFFFF                            //Code to replace empty code with in valid data
