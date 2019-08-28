#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <error.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <dm7820_library.h>

/* piccflight headers */
#include "controller.h"
#include "rtd_functions.h"
#include "alpao_map.h"

//DMA Buffers
uint16_t *rtd_alp_dma_buffer=NULL;     // ALP DMA buffer pointer
uint16_t *rtd_tlm_dma_buffer=NULL;     // TLM DMA buffer pointer
uint32_t  rtd_alp_dma_buffer_size=0;   // ALP DMA buffer size
uint32_t  rtd_tlm_dma_buffer_size=0;   // TLM DMA buffer size
int       rtd_alp_dithers_per_frame=1; // Number of dither steps per frame

/**************************************************************/
/* RTD_OPEN                                                   */
/*  - Open the RTD board                                      */
/**************************************************************/
int rtd_open(unsigned long minor_number, DM7820_Board_Descriptor** p_p_rtd_board) {
  if(DM7820_General_Open_Board(minor_number, p_p_rtd_board)){
    perror("RTD: DM7820_General_Open_Board");
    return 1;
  }
  return 0;
}

/**************************************************************/
/* RTD_RESET                                                  */
/*  - Reset the RTD board                                     */
/**************************************************************/
int rtd_reset(DM7820_Board_Descriptor* p_rtd_board) {
  if(DM7820_General_Reset(p_rtd_board)){
    perror("RTD: DM7820_General_Reset");
    return 1;
  }
  return 0;
}

/**************************************************************/
/* RTD_CLOSE                                                  */
/*  - Close the RTD board                                     */
/**************************************************************/
int rtd_close(DM7820_Board_Descriptor* p_rtd_board) {
  if(DM7820_General_Close_Board(p_rtd_board)){
    perror("RTD: DM7820_General_Close_Board");
    return 1;
  }
  return 0;
}

/**************************************************************/
/* RTD_ALP_CLEANUP                                            */
/*  - Clean up ALPAO resources before closing                 */
/**************************************************************/
int rtd_alp_cleanup(DM7820_Board_Descriptor* p_rtd_board) {
  int retval=0;
  
  //Disable DMA
  if(DM7820_FIFO_DMA_Enable(p_rtd_board, DM7820_FIFO_QUEUE_0, 0x00, 0x00)){
    perror("RTD: DM7820_FIFO_DMA_Enable");
    retval=1;
  }
  
  //Free DMA buffer
  if(rtd_alp_dma_buffer_size > 0){
    if(DM7820_FIFO_DMA_Free_Buffer(&rtd_alp_dma_buffer, rtd_alp_dma_buffer_size)){
      perror("RTD: DM7820_FIFO_DMA_Free_Buffer");
      retval=1;
    }
  }
  
  //Stop output clock
  if(DM7820_PrgClk_Set_Mode(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_DISABLED)){
    perror("RTD: DM7820_PrgClk_Set_Mode");
    retval=1;
  }
  
  //Disable FIFO
  if(DM7820_FIFO_Enable(p_rtd_board, DM7820_FIFO_QUEUE_0, 0x00)){
    perror("RTD: DM7820_FIFO_Enable");
    retval=1;
  }
    
  return retval;
}

/**************************************************************/
/* RTD_TLM_CLEANUP                                            */
/*  - Clean up TLM resources before closing                   */
/**************************************************************/
int rtd_tlm_cleanup(DM7820_Board_Descriptor* p_rtd_board) {
  int retval=0;
  
  //Disable DMA
  if(DM7820_FIFO_DMA_Enable(p_rtd_board, DM7820_FIFO_QUEUE_1, 0x00, 0x00)){
    perror("RTD: DM7820_FIFO_DMA_Enable");
    retval=1;
  }
  
  //Free DMA buffer
  if(rtd_tlm_dma_buffer_size > 0){
    if(DM7820_FIFO_DMA_Free_Buffer(&rtd_tlm_dma_buffer, rtd_tlm_dma_buffer_size)){
      perror("RTD: DM7820_FIFO_DMA_Free_Buffer");
      retval=1;
    }
  }
  
  //Disable FIFO
  if(DM7820_FIFO_Enable(p_rtd_board, DM7820_FIFO_QUEUE_1, 0x00)){
    perror("RTD: DM7820_FIFO_Enable");
    retval=1;
  }
  
  return retval;
}

/**************************************************************/
/* RTD_START_ALP_CLOCK                                        */
/*  - Start the ALPAO data transmission clock signal          */
/**************************************************************/
static DM7820_Error rtd_start_alp_clock(DM7820_Board_Descriptor* p_rtd_board) {
  return DM7820_PrgClk_Set_Mode(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_CONTINUOUS);    
}

/**************************************************************/
/* RTD_STOP_ALP_CLOCK                                         */
/*  - Stop the ALPAO data transmission clock signal           */
/**************************************************************/
static DM7820_Error rtd_stop_alp_clock(DM7820_Board_Descriptor* p_rtd_board) {
  return DM7820_PrgClk_Set_Mode(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_DISABLED);
}

/**************************************************************/
/* RTD_ALP_LIMIT_COMMAND                                      */
/* - Limit analog values of ALPAO commands                    */
/* - Set average of all commands to zero                      */
/**************************************************************/
static void rtd_alp_limit_command(double *cmd){
  int i;
  
  //limit commands
  for(i=0;i<ALP_NACT;i++){
    cmd[i] = (cmd[i] > ALP_AMAX)?ALP_AMAX:cmd[i];
    cmd[i] = (cmd[i] < ALP_AMIN)?ALP_AMIN:cmd[i];
  }
}

/**************************************************************/
/* RTD_ALP_LIMIT_POWER                                        */
/* - Limit total power of ALPAO commands                      */
/**************************************************************/
static void rtd_alp_limit_power(double *cmd) {
  double power = 0.0;
  int i;
  double gain=1;
  
  //Calculate total power
  for(i=0;i<ALP_NACT;i++)
    power += cmd[i]*cmd[i];
  
  //Check and limit power
  if (power > ALP_MAX_POWER) {
    gain = sqrt(ALP_MAX_POWER / power);
    for(i=0;i<ALP_NACT;i++)
      cmd[i] *= gain;
  }
}

/**************************************************************/
/* RTD_ALP_CHECKSUM                                           */
/* - Calculate the checksum of an ALPAO command frame         */
/**************************************************************/
static uint8_t rtd_alp_checksum(uint16_t *frame) {
  uint32_t sum = 0;
  int i;
  uint8_t *p_sum = (uint8_t*)&sum;
  
  for (i=1; i<ALP_FRAME_LENGTH; ++i)
    sum += frame[i];
  while (sum > 0xFF)
    sum = p_sum[0] + p_sum[1] + p_sum[2] + p_sum[3];
  return ~p_sum[0];
}


/**************************************************************/
/* rtd_alp_dither_bit                                         */
/* - Set the dither bit up or down based on the analog        */
/*   fraction and frame index                                 */
/**************************************************************/
static uint16_t rtd_alp_dither_bit(int frame_index, double fraction) {
  //fix until we use dither
  return 0;

  
  uint16_t on, off;
  if (fraction == 1.0) {
    return 0;
  } else if ((fraction < 1.0) && (fraction >= 0.5)) {
    on = 0;
    fraction = 1.0 - fraction;
    off = 1;
  } else if ((fraction < 0.5) && (fraction > 0.0)) {
    on = 1;
    off = 0;
  } else if (fraction == 0.0) {
    return 0;
  } else {
    on = 1;
    off = 0;
  }
  
  //potential overflow or divide by zero
  return ( (((frame_index+1)%((int16_t)(1.0/fraction))) == 0) ? on:off );
}

/**************************************************************/
/* RTD_ALP_BUILD_DITHER_BLOCK                                 */
/* - Build a block of dither command frames                   */
/**************************************************************/
static void rtd_alp_build_dither_block(double *cmd) {
  long      iframe, iact, imain, isub;
  //Regular commands: 97 actuators
  double   fraction[ALP_NACT];
  uint16_t frame[ALP_NACT];
  double   multiplier[ALP_NACT] = ALP_MULTIPLIER;
  double   devcmd[ALP_NACT]     = {0};
  int      mapping[ALP_NACT]    = ALP_MAPPING;
  //Hidden commands: 12 actuators
  double   hidden_fraction[ALP_HIDDEN_NACT];
  uint16_t hidden_frame[ALP_HIDDEN_NACT];
  double   hidden_multiplier[ALP_HIDDEN_NACT] = ALP_HIDDEN_MULTIPLIER;
  double   hidden_devcmd[ALP_HIDDEN_NACT]     = {0};
  int      hidden_mapping[ALP_HIDDEN_NACT]    = ALP_HIDDEN_MAPPING;
  double   hidden_cmd[ALP_HIDDEN_NACT]        = {0};
  
  //Slave hidden actuators to neighbors
  hidden_cmd[0]  = 0.3*cmd[0]  + 0.3*cmd[5];
  hidden_cmd[1]  = 0.3*cmd[5]  + 0.3*cmd[12];
  hidden_cmd[2]  = 0.3*cmd[12] + 0.3*cmd[21];
  hidden_cmd[3]  = 0.3*cmd[65] + 0.3*cmd[76];
  hidden_cmd[4]  = 0.3*cmd[76] + 0.3*cmd[85];
  hidden_cmd[5]  = 0.3*cmd[85] + 0.3*cmd[92];
  hidden_cmd[6]  = 0.3*cmd[96] + 0.3*cmd[91];
  hidden_cmd[7]  = 0.3*cmd[91] + 0.3*cmd[84];
  hidden_cmd[8]  = 0.3*cmd[84] + 0.3*cmd[75];
  hidden_cmd[9]  = 0.3*cmd[31] + 0.3*cmd[20];
  hidden_cmd[10] = 0.3*cmd[20] + 0.3*cmd[11];
  hidden_cmd[11] = 0.3*cmd[11] + 0.3*cmd[4];
  
  //Do driver multiplictaion, initial A2D conversion and set dither fraction
  for(iact=0;iact<ALP_NACT;iact++){
    devcmd[iact]   = multiplier[iact]*cmd[iact];
    frame[iact]    = (uint16_t) ((devcmd[iact]+1.0) * ALP_DMID);
    fraction[iact] = fmod(devcmd[iact],ALP_MIN_ANALOG_STEP)/ALP_MIN_ANALOG_STEP + ((devcmd[iact]<=0.0)?1.0:0.0);
  }
  for(iact=0;iact<ALP_HIDDEN_NACT;iact++){
    hidden_devcmd[iact]   = hidden_multiplier[iact]*hidden_cmd[iact];
    hidden_frame[iact]    = (uint16_t) ((hidden_devcmd[iact]+1.0) * ALP_DMID);
    hidden_fraction[iact] = fmod(hidden_devcmd[iact],ALP_MIN_ANALOG_STEP)/ALP_MIN_ANALOG_STEP + ((hidden_devcmd[iact]<=0.0)?1.0:0.0);
  }
  
  //Loop through dither frames -- add one by one to the output DMA buffer
  for(iframe = 0; iframe < rtd_alp_dithers_per_frame; iframe++) {
    //Set buffer index of the start of this frame
    imain = iframe*ALP_DATA_LENGTH;
    
    //Insert frame header
    rtd_alp_dma_buffer[imain+0] = ALP_START_WORD;
    rtd_alp_dma_buffer[imain+1] = ALP_INIT_COUNTER;

    //Choose the dither bits for this frame
    for(iact=0;iact<ALP_NACT;iact++){
      isub = imain+mapping[iact]+ALP_HEADER_LENGTH;
      rtd_alp_dma_buffer[isub] = frame[iact] + rtd_alp_dither_bit(iframe,fraction[iact]);
      if(rtd_alp_dma_buffer[isub] > ALP_DMAX) rtd_alp_dma_buffer[isub] = ALP_DMAX;
      if(rtd_alp_dma_buffer[isub] < ALP_DMIN) rtd_alp_dma_buffer[isub] = ALP_DMIN;
    }
    for(iact=0;iact<ALP_HIDDEN_NACT;iact++){
      isub = imain+hidden_mapping[iact]+ALP_HEADER_LENGTH;
      rtd_alp_dma_buffer[isub] = hidden_frame[iact] + rtd_alp_dither_bit(iframe,hidden_fraction[iact]);
      if(rtd_alp_dma_buffer[isub] > ALP_DMAX) rtd_alp_dma_buffer[isub] = ALP_DMAX;
      if(rtd_alp_dma_buffer[isub] < ALP_DMIN) rtd_alp_dma_buffer[isub] = ALP_DMIN;
    }
    
    //Closeout frame
    isub = imain+ALP_N_CHANNEL+ALP_HEADER_LENGTH;
    rtd_alp_dma_buffer[isub]  = ALP_END_WORD;
    rtd_alp_dma_buffer[isub] += rtd_alp_checksum(&rtd_alp_dma_buffer[imain]);
    rtd_alp_dma_buffer[isub+ALP_PAD_LENGTH] = ALP_FRAME_END;
  }
}


/**************************************************************/
/* RTD_ALP_WRITE_DMA_FIFO                                     */
/* - Write data to the ALPAO FIFO via DMA                     */
/**************************************************************/
static int rtd_alp_write_dma_fifo(DM7820_Board_Descriptor* p_rtd_board) {
  uint8_t fifo_status;
  static int fifo_warn = 1;
  uint32_t count=0;
  
  //NOTE: This function DOES NOT block waiting for the next DMA transfer
  
  //DMA should ALWAYS be done
  if(DM7820_General_Check_DMA_0_Transfer(p_rtd_board) == 0){
    printf("RTD: ALP DMA NOT DONE!\n");
    return 1;
  }
  
  //Check FIFO status
  if(DM7820_FIFO_Get_Status(p_rtd_board,DM7820_FIFO_QUEUE_0,DM7820_FIFO_STATUS_EMPTY,&fifo_status)){
    perror("RTD: DM7820_FIFO_Get_Status");
    return 1;
  }
  
  //FIFO should be empty, but timing glitches may cause command pile up 
  if(!fifo_status){
    if(fifo_warn){
      printf("RTD: ALP FIFO NOT EMPTY! (Suppressing additional warnings)\n");
      fifo_warn=0;
    }
    return 1;
  }
  
  //Write data to driver's DMA buffer
  if(DM7820_FIFO_DMA_Write(p_rtd_board, DM7820_FIFO_QUEUE_0, rtd_alp_dma_buffer, 1)){
    perror("RTD: DM7820_FIFO_DMA_Write");
    return 1;
  }
  
  //Enable & Start DMA transfer
  if(DM7820_FIFO_DMA_Enable(p_rtd_board, DM7820_FIFO_QUEUE_0, 0xFF, 0xFF)){
    perror("RTD: DM7820_FIFO_DMA_Enable");
    return 1;
  }
  
  //Return 0 on good write
  return 0;
}

/**************************************************************/
/* RTD_TLM_WRITE_DMA_FIFO                                     */
/* - Write data to the TLM FIFO via DMA                       */
/**************************************************************/
static int rtd_tlm_write_dma_fifo(DM7820_Board_Descriptor* p_rtd_board) {
  uint8_t fifo_status;
  uint32_t count=0;
  
  //NOTE: This function BLOCKS wating for the next DMA transfer
  
  //Sleep until current DMA transfer is done
  count=0;
  while(DM7820_General_Check_DMA_1_Transfer(p_rtd_board) == 0){
    usleep(100);
    if(count++ > 10000){
      //One second timeout
      printf("RTD: rtd_tlm_write_dma_fifo DMA TIMEOUT\n");
      return 1;
    }
  }

  //Extra sleep to make sure kernel ISR has handled the interrupt
  usleep(500);

  //Write data to driver's DMA buffer
  if(DM7820_FIFO_DMA_Write(p_rtd_board, DM7820_FIFO_QUEUE_1, rtd_tlm_dma_buffer, 1)){
    perror("RTD: DM7820_FIFO_DMA_Write");
    return 1;
  }
  
  //Check FIFO status
  if(DM7820_FIFO_Get_Status(p_rtd_board,DM7820_FIFO_QUEUE_1,DM7820_FIFO_STATUS_READ_REQUEST,&fifo_status)){
    perror("RTD: DM7820_FIFO_Get_Status");
    return 1;
  }
  
  //Sleep until fifo is ready for data
  count=0;
  while(fifo_status){
    //READ_REQUEST returns 1 when there is at least 256 words in the fifo
    //READ_REQUEST returns 0 when the data in the fifo drops below 128 words --> wait for this 
    usleep(100);
    if(DM7820_FIFO_Get_Status(p_rtd_board,DM7820_FIFO_QUEUE_1,DM7820_FIFO_STATUS_READ_REQUEST,&fifo_status)){
      perror("RTD: DM7820_FIFO_Get_Status");
      return 1;
    }
    if(count++ > 10000){
      //One second timeout
      printf("RTD: rtd_tlm_write_dma_fifo FIFO TIMEOUT\n");
      return 1;
    }
  }
  
  //Enable & Start DMA transfer
  if(DM7820_FIFO_DMA_Enable(p_rtd_board, DM7820_FIFO_QUEUE_1, 0xFF, 0xFF)){
    perror("RTD: DM7820_FIFO_DMA_Enable");
    return 1;
  }
  
  //Return 0 on good write
  return 0;
}


/**************************************************************/
/* RTD_SEND_ALP                                               */
/* - Send a command to the ALPAO controller                   */
/**************************************************************/
int rtd_send_alp(DM7820_Board_Descriptor* p_rtd_board, double *cmd) {
  rtd_alp_limit_command(cmd);
  rtd_alp_limit_power(cmd);
  rtd_alp_build_dither_block(cmd);
  return rtd_alp_write_dma_fifo(p_rtd_board);
}


/**************************************************************/
/* RTD_SEND_TLM                                               */
/*  - Write telemetry data out the RTD interface              */
/*  - Buffer data until the DMA buffer is full. Then send.    */
/**************************************************************/
int rtd_send_tlm(DM7820_Board_Descriptor* p_rtd_board, char *buf, uint32_t num, int flush){
  static volatile uint32_t m=0;
  uint32_t nwords = 0;
  uint16_t *buf16;
  uint32_t i=0,l=0,n=0;
  uint32_t buffer_length = rtd_tlm_dma_buffer_size/2;
  
  //Everything written must be an integer number of 16bit words
  if(num % 2){
    printf("rtd_write_dma: BAD DATA SIZE\n");
    return 1;
  }
  
  //Setup pointers
  buf16  = (uint16_t *)buf;
  nwords = num/2;
  
  //Filter out real empty codes from data
  for(i=0;i<nwords;i++)
    if(buf16[i] == TLM_EMPTY_CODE)
      buf16[i] = TLM_REPLACE_CODE;
  
  //Write data into output buffer
  while(l<nwords){
    //l --> number of words out of nwords written to the buffer
    //m --> total number of words already written to buffer
    //n --> number of words to write this time through while loop
    n = (nwords-l)<(buffer_length-m-1) ? (nwords-l):(buffer_length-m-1);
    
    //Copy local data into output buffer
    memcpy(&rtd_tlm_dma_buffer[m],&buf16[l],n*sizeof(uint16_t));
      
    //Add n to m
    m+=n;
    
    //Add n to l
    l+=n;

    //Flush buffer with empty code if requested
    if(flush)
      while(m < (buffer_length-1))
	rtd_tlm_dma_buffer[m++]=TLM_EMPTY_CODE;
    
    //Check if the buffer is full and we need to do a transfer
    if(m==buffer_length-1){
      
      //Set last word of buffer to empty code
      rtd_tlm_dma_buffer[m]=TLM_EMPTY_CODE;

      //Zero out m
      m=0;

      //Write data
      if(rtd_tlm_write_dma_fifo(p_rtd_board))
	return 1;
    }
  }

  //Return 0 on good write
  return 0;
}

/**************************************************************/
/* RTD_INIT_ALP                                               */
/*  - Initialize the RTD board to control the ALPAO DM        */
/**************************************************************/
int rtd_init_alp(DM7820_Board_Descriptor* p_rtd_board, int dithers_per_frame) {
  DM7820_Error dm7820_status;
  uint32_t dma_buffer_size,dma_buffer_length,i;
  
  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Setup Definition
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    IO Headers:
    CN10: Parallel interface
          Output clock signal on port 2
    	  Port 0 data clocked out of FIFO 0
	  DMA to FIFO 0
    Objects to configure:
    FIFO 0:   Input data from user over PCI via DMA
    8254TC:   Setup internal timer/counter to produce ALPAO clock signal
    Port 0:   Write data from FIFO 0 on ALPAO clock
    Port 2:   Output ALPAO clock signal
  */

  /* ========================== Cleanup ALP Settings ========================== */
  if(rtd_alp_cleanup(p_rtd_board))
    return 1;
  
  /* ================================ Setup DMA buffer size ================================ */
  dma_buffer_length = ((dithers_per_frame<3)?0x200:(dithers_per_frame*ALP_DATA_LENGTH)); //in 16bit words
  dma_buffer_size   = dma_buffer_length*2; // in bytes
  rtd_alp_dma_buffer_size = dma_buffer_size;
  rtd_alp_dithers_per_frame = dithers_per_frame;

  /* ================================ Standard output initialization ================================ */

  /* Set Port 0 to peripheral output */
  dm7820_status = DM7820_StdIO_Set_IO_Mode(p_rtd_board, DM7820_STDIO_PORT_0, 0xFFFF, DM7820_STDIO_MODE_PER_OUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

  /* Set Port 0 peripheral to the fifo 0 peripheral */
  dm7820_status = DM7820_StdIO_Set_Periph_Mode(p_rtd_board, DM7820_STDIO_PORT_0, 0xFFFF, DM7820_STDIO_PERIPH_FIFO_0);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

  /* Set Port 2 to peripheral output */
  dm7820_status = DM7820_StdIO_Set_IO_Mode(p_rtd_board, DM7820_STDIO_PORT_2, 0xFFFF, DM7820_STDIO_MODE_PER_OUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

  /* Set Port 2 peripheral to the clock and timer peripherals */
  dm7820_status = DM7820_StdIO_Set_Periph_Mode(p_rtd_board, DM7820_STDIO_PORT_2, 0xFFFF, DM7820_STDIO_PERIPH_CLK_OTHER);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");


  /* ================================ Programmable clock 0 initialization ================================ */
  
  /* Set master clock to 25 MHz clock */
  dm7820_status = DM7820_PrgClk_Set_Master(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Master()");

  /* Set clock stop trigger so that clock is never stopped */
  dm7820_status = DM7820_PrgClk_Set_Stop_Trigger(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_STOP_NONE);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Stop_Trigger()");

  /* Set clock period to obtain 25/RTD_PRGCLK_0_DIVISOR [MHz] */
  dm7820_status = DM7820_PrgClk_Set_Period(p_rtd_board, DM7820_PRGCLK_CLOCK_0, RTD_PRGCLK_0_DIVISOR);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");

  /* Set clock start trigger to start immediately */
  dm7820_status = DM7820_PrgClk_Set_Start_Trigger(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_START_IMMEDIATE);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Start_Trigger()");


  /* ================================ 8254 timer/counter A0 initialization ================================ */
  dm7820_status = DM7820_TmrCtr_Select_Clock(p_rtd_board, DM7820_TMRCTR_TIMER_A_0, DM7820_TMRCTR_CLOCK_PROG_CLOCK_0);
  DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Clock()");

  /* Set up the timer by
   * 1) setting waveform mode to square wave generator,
   * 2) setting count mode to binary, and
   * 3) loading divisor value to obtain the frequency */
  dm7820_status = DM7820_TmrCtr_Program(p_rtd_board, DM7820_TMRCTR_TIMER_A_0, DM7820_TMRCTR_WAVEFORM_SQUARE_WAVE, DM7820_TMRCTR_COUNT_MODE_BINARY, RTD_TIMER_A0_DIVISOR);
  DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Program()");

  /* Set timer gate to high to enable counting */
  dm7820_status = DM7820_TmrCtr_Select_Gate(p_rtd_board, DM7820_TMRCTR_TIMER_A_0, DM7820_TMRCTR_GATE_LOGIC_1);
  DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");


  /* ========================== FIFO 0 initialization ========================== */
  
  /* Set input clock to PCI write to FIFO 0 Read/Write Port Register */
  dm7820_status = DM7820_FIFO_Set_Input_Clock(p_rtd_board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_INPUT_CLOCK_PCI_WRITE);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Input_Clock()");

  /* Set FIFO 0 output clock to timer A0 */
  dm7820_status = DM7820_FIFO_Set_Output_Clock(p_rtd_board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_OUTPUT_CLOCK_8254_A_0);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

  /* Set data input to PCI data */
  dm7820_status = DM7820_FIFO_Set_Data_Input(p_rtd_board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_0_DATA_INPUT_PCI_DATA);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Data_Input()");

  /* ========================== DMA initialization ========================== */

  /* Set FIFO 0 DREQ to REQUEST WRITE */
  dm7820_status = DM7820_FIFO_Set_DMA_Request(p_rtd_board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_DMA_REQUEST_WRITE);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_DMA_Request()");
  
  /* Create the DMA buffer */
  dm7820_status = DM7820_FIFO_DMA_Create_Buffer(&rtd_alp_dma_buffer, rtd_alp_dma_buffer_size);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Create_Buffer()");
  
  /* Initialize the DMA buffer */
  dm7820_status = DM7820_FIFO_DMA_Initialize(p_rtd_board, DM7820_FIFO_QUEUE_0, 1, rtd_alp_dma_buffer_size);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Initialize()");

  /* Configure DMA direction*/
  dm7820_status = DM7820_FIFO_DMA_Configure(p_rtd_board, DM7820_FIFO_QUEUE_0, DM7820_DMA_DEMAND_ON_PCI_TO_DM7820, rtd_alp_dma_buffer_size);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Configure()");

  /* Initialize DMA buffer */
  memset(rtd_alp_dma_buffer,0,rtd_alp_dma_buffer_size);

  /* ========================== Secondary FIFO 0 configuration ========================== */

  /* Enable FIFO 0 */
  dm7820_status = DM7820_FIFO_Enable(p_rtd_board, DM7820_FIFO_QUEUE_0, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");
 
  //Start output clock 
  dm7820_status = DM7820_PrgClk_Set_Mode(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_CONTINUOUS);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");
  
  return 0;
}



/**************************************************************/
/* RTD_INIT_TLM                                               */
/*  - Initialize the RTD board output telemetry               */
/**************************************************************/
int rtd_init_tlm(DM7820_Board_Descriptor* p_rtd_board, uint32_t dma_size){
  DM7820_Error dm7820_status;
 
  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Setup Definition
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    IO Headers:
    CN11: Parallel interface
          Strobe 1 receives the WFF93 read strobe
    	  Port 1 data clocked out of FIFO 1
	  DMA to FIFO 1
    Objects to configure:
    Strobe 1: Input (WFF93)
    FIFO 1:   Input data from user over PCI via DMA
    Port 1:   Write data from FIFO 1 on Strobe 1
  */

  //Set global DMA buffer size
  rtd_tlm_dma_buffer_size = dma_size;

  
  /*============================== Strobe Initialization ================================*/

  /* Set strobe signal 1 to input */
  dm7820_status = DM7820_StdIO_Strobe_Mode(p_rtd_board, DM7820_STDIO_STROBE_1, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Mode()");

  /*============================== FIFO 1 Initialization ================================*/

  /* Disable FIFO 1*/
  dm7820_status = DM7820_FIFO_Enable(p_rtd_board, DM7820_FIFO_QUEUE_1, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

  /* Set input clock to PCI write for FIFO 1 Read/Write Port Register */
  dm7820_status = DM7820_FIFO_Set_Input_Clock(p_rtd_board,DM7820_FIFO_QUEUE_1,DM7820_FIFO_INPUT_CLOCK_PCI_WRITE);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Input_Clock()");
  
  /* Set output clock to Strobe 1 */
  dm7820_status = DM7820_FIFO_Set_Output_Clock(p_rtd_board,DM7820_FIFO_QUEUE_1,DM7820_FIFO_OUTPUT_CLOCK_STROBE_1);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

  /* Set data input to PCI for FIFO 1 */
  dm7820_status = DM7820_FIFO_Set_Data_Input(p_rtd_board,DM7820_FIFO_QUEUE_1,DM7820_FIFO_1_DATA_INPUT_PCI_DATA);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Data_Input()");
  
  /* Set FIFO 1 DREQ to REQUEST WRITE */
  dm7820_status = DM7820_FIFO_Set_DMA_Request(p_rtd_board,DM7820_FIFO_QUEUE_1,DM7820_FIFO_DMA_REQUEST_WRITE);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_DMA_Request()");

  /*============================== Port 1 Initialization ================================*/

  /* Set Port 1 as output */
  dm7820_status = DM7820_StdIO_Set_IO_Mode(p_rtd_board,DM7820_STDIO_PORT_1,0xFFFF,DM7820_STDIO_MODE_PER_OUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

  /* Set Port 1 data source as FIFO 1 */
  dm7820_status = DM7820_StdIO_Set_Periph_Mode(p_rtd_board,DM7820_STDIO_PORT_1,0xFFFF,DM7820_STDIO_PERIPH_FIFO_1);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

  /*============================== DMA Setup  ================================*/

  /* Allocate DMA buffer */
  dm7820_status = DM7820_FIFO_DMA_Create_Buffer(&rtd_tlm_dma_buffer, rtd_tlm_dma_buffer_size);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Create_Buffer()");

  /* Initializing DMA 1 */
  dm7820_status = DM7820_FIFO_DMA_Initialize(p_rtd_board,DM7820_FIFO_QUEUE_1,1,rtd_tlm_dma_buffer_size);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Initialize()");
  
  /* Configuring DMA 1 */
  dm7820_status = DM7820_FIFO_DMA_Configure(p_rtd_board,DM7820_FIFO_QUEUE_1,DM7820_DMA_DEMAND_ON_PCI_TO_DM7820,rtd_tlm_dma_buffer_size);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Configure()");

  /* Initialize DMA buffer */
  memset(rtd_tlm_dma_buffer,0,rtd_tlm_dma_buffer_size);

  /*============================== Secondary FIFO 1 Setup  ================================*/
  
  /* Enable FIFO 1 */
  dm7820_status = DM7820_FIFO_Enable(p_rtd_board, DM7820_FIFO_QUEUE_1, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

  return 0;
}


