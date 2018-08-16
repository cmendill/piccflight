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
DM7820_Error rtd_open(unsigned long minor_number, DM7820_Board_Descriptor** p_p_rtd_board) {
  return DM7820_General_Open_Board(minor_number, p_p_rtd_board);
}

/**************************************************************/
/* RTD_RESET                                                  */
/*  - Reset the RTD board                                     */
/**************************************************************/
DM7820_Error rtd_reset(DM7820_Board_Descriptor* p_rtd_board) {
  DM7820_Error dm7820_status=0,dm7820_return=0;
  int i;

  //Reset board
  if((dm7820_status=DM7820_General_Reset(p_rtd_board)))
    perror("DM7820_General_Reset");
  dm7820_return |= dm7820_status;

  //Disable all interrupts
  for(i=0;i<DM7820_INTERRUPT_NONE;i++){
    //Users cannot disable DMA_DONE interrupts
    if(i==DM7820_INTERRUPT_FIFO_0_DMA_DONE)
      continue;
    if(i==DM7820_INTERRUPT_FIFO_1_DMA_DONE)
      continue;
    if((dm7820_status = DM7820_General_Enable_Interrupt(p_rtd_board, i, 0x00))){
      perror("DM7820_General_Enable_Interrupt");
    }
    dm7820_return |= dm7820_status;
  }
  return dm7820_return;
 }

/**************************************************************/
/* RTD_CLOSE                                                  */
/*  - Close the RTD board                                     */
/**************************************************************/
DM7820_Error rtd_close(DM7820_Board_Descriptor* p_rtd_board) {
  return DM7820_General_Close_Board(p_rtd_board);
}

/**************************************************************/
/* RTD_ALP_CLEANUP                                            */
/*  - Clean up ALPAO resources before closing                 */
/**************************************************************/
DM7820_Error rtd_alp_cleanup(DM7820_Board_Descriptor* p_rtd_board) {
  DM7820_Error dm7820_status=0,dm7820_return=0;
  int i;
  
  //Disable DMA
  if((dm7820_status = DM7820_FIFO_DMA_Enable(p_rtd_board, DM7820_FIFO_QUEUE_0, 0x00, 0x00)))
    perror("DM7820_FIFO_DMA_Enable");
  dm7820_return |= dm7820_status;
  
  //Free DMA buffer
  if(rtd_alp_dma_buffer_size > 0){
    if((dm7820_status = DM7820_FIFO_DMA_Free_Buffer(&rtd_alp_dma_buffer, rtd_alp_dma_buffer_size)))
      perror("DM7820_FIFO_DMA_Free_Buffer");
    dm7820_return |= dm7820_status;
  }
  
  //Stop output clock
  if((dm7820_status = DM7820_PrgClk_Set_Mode(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_DISABLED)))
    perror("DM7820_PrgClk_Set_Mode");
  dm7820_return |= dm7820_status;

  //Disable FIFO
  if((dm7820_status = DM7820_FIFO_Enable(p_rtd_board, DM7820_FIFO_QUEUE_0, 0x00)))
    perror("DM7820_FIFO_Enable");
  dm7820_return |= dm7820_status;
  
  return dm7820_return;
}

/**************************************************************/
/* RTD_TLM_CLEANUP                                            */
/*  - Clean up TLM resources before closing                   */
/**************************************************************/
DM7820_Error rtd_tlm_cleanup(DM7820_Board_Descriptor* p_rtd_board) {
  DM7820_Error dm7820_status=0,dm7820_return=0;
  //Disable DMA
  if((dm7820_status = DM7820_FIFO_DMA_Enable(p_rtd_board, DM7820_FIFO_QUEUE_1, 0x00, 0x00)))
    perror("DM7820_FIFO_DMA_Enable");
  dm7820_return |= dm7820_status;
  
  //Free DMA buffer
  if(rtd_tlm_dma_buffer_size > 0){
    if((dm7820_status = DM7820_FIFO_DMA_Free_Buffer(&rtd_tlm_dma_buffer, rtd_tlm_dma_buffer_size)))
      perror("DM7820_FIFO_DMA_Free_Buffer");
    dm7820_return |= dm7820_status;
  }
  
  //Disable FIFO
  if((dm7820_status = DM7820_FIFO_Enable(p_rtd_board, DM7820_FIFO_QUEUE_1, 0x00)))
    perror("DM7820_FIFO_Enable");
  dm7820_return |= dm7820_status;
  
  return dm7820_return;
}

/**************************************************************/
/* RTD_START_ALP_CLOCK                                        */
/*  - Start the ALPAO data transmission clock signal          */
/**************************************************************/
DM7820_Error rtd_start_alp_clock(DM7820_Board_Descriptor* p_rtd_board) {
  return DM7820_PrgClk_Set_Mode(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_CONTINUOUS);    
}

/**************************************************************/
/* RTD_STOP_ALP_CLOCK                                         */
/*  - Stop the ALPAO data transmission clock signal           */
/**************************************************************/
DM7820_Error rtd_stop_alp_clock(DM7820_Board_Descriptor* p_rtd_board) {
  return DM7820_PrgClk_Set_Mode(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_DISABLED);
}

/**************************************************************/
/* RTD_ALP_LIMIT_COMMAND                                      */
/* - Limit analog values of ALPAO commands                    */
/**************************************************************/
static void rtd_alp_limit_command(double *cmd){
  int i;
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
  return ( (((frame_index+1)%((int16_t)(1.0/fraction))) == 0)?on:off );
}

/**************************************************************/
/* RTD_ALP_BUILD_DITHER_BLOCK                                 */
/* - Build a block of dither command frames                   */
/**************************************************************/
static void rtd_alp_build_dither_block(double *cmd) {
  int      iframe, iact, imain, isub;
  double   fraction[ALP_NACT];
  uint16_t frame[ALP_NACT];
  double   multiplier[ALP_NACT] = ALP_MULTIPLIER;
  double   devcmd[ALP_NACT]     = {0};
  int      mapping[ALP_NACT]    = ALP_MAPPING;
  int      i;
  int      dma_buffer_length    = rtd_alp_dma_buffer_size/2;
  
  //Initialize the DMA buffer
  for(i=0;i<dma_buffer_length;i++)
    rtd_alp_dma_buffer[i]=ALP_DMID;

  //Do driver multiplictaion, initial A2D conversion and set dither fraction
  for(iact=0;iact<ALP_NACT;iact++){
    devcmd[iact]   = multiplier[iact]*cmd[iact];
    frame[iact]    = (uint16_t) ((devcmd[iact]+1.0) * ALP_DMID);
    fraction[iact] = fmod(devcmd[iact],ALP_MIN_ANALOG_STEP)/ALP_MIN_ANALOG_STEP + ((devcmd[iact]<=0.0)?1.0:0.0);
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
static DM7820_Error rtd_alp_write_dma_fifo(DM7820_Board_Descriptor* p_rtd_board) {
  DM7820_Error dm7820_status=0,dm7820_return=0;
  uint8_t fifo_status;
  
  //Sleep until current DMA transfer is done (PREVIOUS DMA MUST BE DONE, WARN IF IT ISN'T)
  if(DM7820_General_Check_DMA_0_Transfer(p_rtd_board) == 0){
    printf("RTD: ALP DMA NOT DONE!\n");
    usleep(10);
    while(DM7820_General_Check_DMA_0_Transfer(p_rtd_board) == 0)
      usleep(10);
  }
  
  //Sleep until fifo is empty (FIFO MUST BE EMPTY, WARN IF IT ISN'T)
  if((dm7820_status = DM7820_FIFO_Get_Status(p_rtd_board,DM7820_FIFO_QUEUE_0,DM7820_FIFO_STATUS_EMPTY,&fifo_status)))
    perror("DM7820_FIFO_Get_Status");
  dm7820_return |= dm7820_status;
  if(!fifo_status){
    printf("RTD: ALP FIFO NOT EMPTY!\n");
    usleep(10);
    while(!fifo_status){
      if((dm7820_status = DM7820_FIFO_Get_Status(p_rtd_board,DM7820_FIFO_QUEUE_0,DM7820_FIFO_STATUS_EMPTY,&fifo_status)))
	perror("DM7820_FIFO_Get_Status");
      dm7820_return |= dm7820_status;
    }
  }
  
  //Write data to driver's DMA buffer
  if((dm7820_status = DM7820_FIFO_DMA_Write(p_rtd_board, DM7820_FIFO_QUEUE_0, rtd_alp_dma_buffer, 1)))
    perror("DM7820_FIFO_DMA_Write");
  dm7820_return |= dm7820_status;
  
  /* Enable & Start DMA transfer */
  if((dm7820_status = DM7820_FIFO_DMA_Enable(p_rtd_board, DM7820_FIFO_QUEUE_0, 0xFF, 0xFF)))
    perror("DM7820_FIFO_DMA_Enable");
  dm7820_return |= dm7820_status;


  return dm7820_return;
}

/**************************************************************/
/* RTD_TLM_WRITE_DMA_FIFO                                     */
/* - Write data to the TLM FIFO via DMA                     */
/**************************************************************/
static DM7820_Error rtd_tlm_write_dma_fifo(DM7820_Board_Descriptor* p_rtd_board) {
  DM7820_Error dm7820_status=0,dm7820_return=0;
  uint8_t fifo_status;
  
  //Sleep until current DMA transfer is done
  while(DM7820_General_Check_DMA_1_Transfer(p_rtd_board) == 0)
    usleep(10);
  
  //Sleep until fifo is ready
  if((dm7820_status = DM7820_FIFO_Get_Status(p_rtd_board,DM7820_FIFO_QUEUE_1,DM7820_FIFO_STATUS_READ_REQUEST,&fifo_status)))
    perror("DM7820_FIFO_Get_Status");
  dm7820_return |= dm7820_status;
  while(fifo_status){
    //READ_REQUEST returns 1 when there is at least 256 words in the fifo
    //READ_REQUEST returns 0 when the data in the fifo drops below 128 words --> wait for this 
    usleep(100);
    if((dm7820_status = DM7820_FIFO_Get_Status(p_rtd_board,DM7820_FIFO_QUEUE_1,DM7820_FIFO_STATUS_READ_REQUEST,&fifo_status)))
      perror("DM7820_FIFO_Get_Status");
    dm7820_return |= dm7820_status;
  }

  //Write data to driver's DMA buffer
  if((dm7820_status = DM7820_FIFO_DMA_Write(p_rtd_board, DM7820_FIFO_QUEUE_1, rtd_tlm_dma_buffer, 1)))
    perror("DM7820_FIFO_DMA_Write");
  dm7820_return |= dm7820_status;
  
  /* Enable & Start DMA transfer */
  if((dm7820_status = DM7820_FIFO_DMA_Enable(p_rtd_board, DM7820_FIFO_QUEUE_1, 0xFF, 0xFF)))
    perror("DM7820_FIFO_DMA_Enable");
  dm7820_return |= dm7820_status;

  return dm7820_return;
}


/**************************************************************/
/* RTD_SEND_ALP                                               */
/* - Send a command to the ALPAO controller                   */
/**************************************************************/
DM7820_Error rtd_send_alp(DM7820_Board_Descriptor* p_rtd_board, double *cmd) {
  //dma_done_count links back through the user code to the ISR through shared memory
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
DM7820_Error rtd_send_tlm(DM7820_Board_Descriptor* p_rtd_board, char *buf, uint32_t num){
  DM7820_Error dm7820_status=0;
  static volatile uint32_t m=0;
  uint32_t nwords = 0;
  uint16_t *buf16;
  uint32_t i=0,l=0,n=0;
  uint32_t buffer_length = rtd_tlm_dma_buffer_size/2;
  
  //Everything written must be an integer number of 16bit words
  if(num % 2)
    printf("rtd_write_dma: BAD DATA SIZE\n");
  
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
    
    //Check if the buffer is full and we need to do a transfer
    if(m==buffer_length-1){
      
      //Set last word of buffer to empty code
      rtd_tlm_dma_buffer[m]=TLM_EMPTY_CODE;

      //Write data
      dm7820_status = rtd_tlm_write_dma_fifo(p_rtd_board);
      
      //Zero out m
      m=0;
    }
  }
  return dm7820_status;
}

/**************************************************************/
/* RTD_INIT_ALP                                               */
/*  - Initialize the RTD board to control the ALPAO DM        */
/**************************************************************/
DM7820_Error rtd_init_alp(DM7820_Board_Descriptor* p_rtd_board, int dithers_per_frame) {
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
  dm7820_status = rtd_alp_cleanup(p_rtd_board);
  DM7820_Return_Status(dm7820_status, "rtd_cleanup_alp()");
  
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

  /* ========================== Secondary FIFO 0 configuration ========================== */

  /* Enable FIFO 0 */
  dm7820_status = DM7820_FIFO_Enable(p_rtd_board, DM7820_FIFO_QUEUE_0, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");
 
  //Start output clock 
  dm7820_status = DM7820_PrgClk_Set_Mode(p_rtd_board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_CONTINUOUS);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");
  
  return dm7820_status;
}



/**************************************************************/
/* RTD_INIT_TLM                                               */
/*  - Initialize the RTD board output telemetry               */
/**************************************************************/
DM7820_Error rtd_init_tlm(DM7820_Board_Descriptor* p_rtd_board, uint32_t dma_size){
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

  /*============================== Secondary FIFO 1 Setup  ================================*/
  
  /* Enable FIFO 1 */
  dm7820_status = DM7820_FIFO_Enable(p_rtd_board, DM7820_FIFO_QUEUE_1, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

  return dm7820_status;
}


