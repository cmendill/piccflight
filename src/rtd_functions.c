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

//EXTERNAL GLOBALS
//--RTD DM7820 device descriptor
extern DM7820_Board_Descriptor *board;  
//--number of interrupts counted
extern volatile unsigned long fifo_empty_interrupts;
extern volatile unsigned long dma_done_frames;
extern volatile unsigned long dma_done_interrupts;

//DMA Buffer
uint16_t *dma_out_buf;


#define PRINT_RTD_SETUP   0
#define PRINT_RTD_CLEANUP 0

//rtd_ISR
void rtd_ISR(dm7820_interrupt_info interrupt_info)
{
  
  DM7820_Return_Status(interrupt_info.error, "ISR Failed\n");
  
  switch (interrupt_info.source) {
  
  case DM7820_INTERRUPT_FIFO_1_EMPTY:
    fifo_empty_interrupts++;
    break;
    
  case DM7820_INTERRUPT_FIFO_1_DMA_DONE:
    dma_done_interrupts++;
    dma_done_interrupts %= RTD_FRAME_NUM;
    if(dma_done_interrupts == 0){
      dma_done_frames++;
    }
    break;
  default:
    
    break;
  }
  
}

//rtd_clean_up
void rtd_cleanup(void)
{
  DM7820_Error dm7820_status;

  if(PRINT_RTD_CLEANUP) printf("TLM: Cleaning up RTD board:\n");
  if(RTD_DMA){
    
    if(PRINT_RTD_CLEANUP) printf(" -- Removing user ISR\n");
    if(PRINT_RTD_CLEANUP) printf("     -- DMA Frames: %lu\n", dma_done_frames);
    if(PRINT_RTD_CLEANUP) printf("     -- FIFO Empty Interrupts: %lu\n", fifo_empty_interrupts);
    dm7820_status = DM7820_General_RemoveISR(board);
    DM7820_Return_Status(dm7820_status, "DM7820_General_RemoveISR()");
   
    if(PRINT_RTD_CLEANUP) printf(" -- Freeing DMA buffer\n");
    dm7820_status =
      DM7820_FIFO_DMA_Free_Buffer(&dma_out_buf, RTD_DMA_BUF_SIZE);
    DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Create_Buffer()");
  }

  if(PRINT_RTD_CLEANUP) printf(" -- Disabling programmable clock 0\n");
  dm7820_status = DM7820_PrgClk_Set_Mode(board,
					 DM7820_PRGCLK_CLOCK_0,
					 DM7820_PRGCLK_MODE_DISABLED);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

  if(PRINT_RTD_CLEANUP) printf(" -- Disabling DMA 0\n");

  dm7820_status = DM7820_FIFO_DMA_Enable(board,
					 DM7820_FIFO_QUEUE_0, 0x00, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Enable()");

  if(PRINT_RTD_CLEANUP) printf(" -- Disabling DMA 1\n");

  dm7820_status = DM7820_FIFO_DMA_Enable(board,
					 DM7820_FIFO_QUEUE_1, 0x00, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Enable()");

  if(PRINT_RTD_CLEANUP) printf(" -- Disabling FIFO 0\n");

  dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

  if(PRINT_RTD_CLEANUP) printf(" -- Disabling FIFO 1\n");

  dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_1, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

  if(PRINT_RTD_CLEANUP) printf(" -- Resetting frame counters\n");
  dma_done_frames=0;
  dma_done_interrupts=0;    
  
  if(PRINT_RTD_CLEANUP) printf(" -- Closing device\n");

  dm7820_status = DM7820_General_Close_Board(board);
  board=NULL;//just to be sure
  DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");
}

void rtd_init(void){
  unsigned long minor_number = 0;
  unsigned long i;
  uint8_t fifo_status = 0;
  DM7820_Error dm7820_status;
 
  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Setup Definition
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    IO Headers:
    CN10: No Connection
    CN11: Parallel interface
          Strobe 1 receives the WFF93 read strobe
    	  Port 1 data clocked out of FIFO 1
    Objects to configure:
    Strobe 1: Input (WFF93)
    Strobe 2: Disabled
    FIFO 0:   Disabled
    FIFO 1:   Input data from user over PCI (DMA or direct write to FIFO)
    Port 0:   Disabled
    Port 1:   Write data from FIFO 1 on Strobe 1
    Port 2:   Disabled
  */



  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Device initialization
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

  if(PRINT_RTD_SETUP) printf("Opening device with minor number %lu\n",minor_number);

  dm7820_status = DM7820_General_Open_Board(minor_number, &board);
  DM7820_Return_Status(dm7820_status, "DM7820_General_Open_Board()");

  /*
   * Reset device
   */

  if(PRINT_RTD_SETUP) printf("Resetting device\n");

  dm7820_status = DM7820_General_Reset(board);
  DM7820_Return_Status(dm7820_status, "DM7820_General_Reset()");

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Interrupt generic initialization
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

  /*
   * Make sure FIFO 0 empty interrupt is disabled to prevent stray interrupts
   */

  if(PRINT_RTD_SETUP) printf("Disabling FIFO 0 empty interrupt\n");

  dm7820_status = DM7820_General_Enable_Interrupt(board,
						  DM7820_INTERRUPT_FIFO_0_EMPTY,
						  0x00);
  DM7820_Return_Status(dm7820_status,
		       "DM7820_General_Enable_Interrupt()");

  if(PRINT_RTD_SETUP) printf("Disabling FIFO 1 empty interrupt\n");

  dm7820_status = DM7820_General_Enable_Interrupt(board,
						  DM7820_INTERRUPT_FIFO_1_EMPTY,
						  0x00);
  DM7820_Return_Status(dm7820_status,
		       "DM7820_General_Enable_Interrupt()");
  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    FIFO generic initialization
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

  /*
   * Disable FIFO 0 to put it into a known state
   */

  if(PRINT_RTD_SETUP) printf("Disabling FIFO 0\n");

  dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

  /*
   * Disable FIFO 1 to put it into a known state
   */

  if(PRINT_RTD_SETUP) printf("Disabling FIFO 1\n");

  dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_1, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Programmable clock generic initialization
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

  /*
   * Disable clock 0 to put it into a known state; any clock should
   * be disabled before programming it.
   */

  if(PRINT_RTD_SETUP) printf("Disabling programmable clocks:\n");

  if(PRINT_RTD_SETUP) printf(" -- Clock 0\n");

  dm7820_status = DM7820_PrgClk_Set_Mode(board,
					 DM7820_PRGCLK_CLOCK_0,
					 DM7820_PRGCLK_MODE_DISABLED);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    STROBE initialization
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

  /*
   * Set strobe signal 1 to input
   */

  if(PRINT_RTD_SETUP) printf("Setting strobe signal 1 to input\n");

  dm7820_status =
    DM7820_StdIO_Strobe_Mode(board, DM7820_STDIO_STROBE_1, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Mode()");


  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    FIFO 1 initialization 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

  /*
   * Set input clock to PCI write for FIFO 1 Read/Write Port Register
   */
   
  if(PRINT_RTD_SETUP) printf("Initializing FIFO 1:\n");

  if(PRINT_RTD_SETUP) printf(" -- Setting FIFO 1 input clock: PCI WRITE\n");

  dm7820_status = DM7820_FIFO_Set_Input_Clock(board,
					      DM7820_FIFO_QUEUE_1,
					      DM7820_FIFO_INPUT_CLOCK_PCI_WRITE);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Input_Clock()");
  
  /*
   * Set output clock to Strobe 1
   */

  if(PRINT_RTD_SETUP) printf(" -- Setting FIFO 1 output clock: STROBE 1\n");

  dm7820_status = DM7820_FIFO_Set_Output_Clock(board,
  					       DM7820_FIFO_QUEUE_1,
  					       DM7820_FIFO_OUTPUT_CLOCK_STROBE_1);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

  /*
   * Set data input to PCI for FIFO 1
   */
  
  if(PRINT_RTD_SETUP) printf("    Setting FIFO 1 data input: PCI\n");
  
  dm7820_status = DM7820_FIFO_Set_Data_Input(board,
					     DM7820_FIFO_QUEUE_1,
					     DM7820_FIFO_1_DATA_INPUT_PCI_DATA);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Data_Input()");


  if(RTD_DMA){
    /*
     *Set FIFO 1 DREQ to REQUEST WRITE
     */
    
    if(PRINT_RTD_SETUP) printf(" -- Setting FIFO 1 DREQ source ...\n");
    
    dm7820_status = DM7820_FIFO_Set_DMA_Request(board,
						DM7820_FIFO_QUEUE_1,
						DM7820_FIFO_DMA_REQUEST_WRITE);
    DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_DMA_Request()");
  }


  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Port 1 initialization (OUTPUT)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  
  if(PRINT_RTD_SETUP) printf("Configuring Port 1: OUTPUT\n");
  
  dm7820_status = DM7820_StdIO_Set_IO_Mode(board,
					   DM7820_STDIO_PORT_1,
					   0xFFFF,
					   DM7820_STDIO_MODE_PER_OUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

  dm7820_status = DM7820_StdIO_Set_Periph_Mode(board,
					       DM7820_STDIO_PORT_1,
					       0xFFFF,
					       DM7820_STDIO_PERIPH_FIFO_1);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");


  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Secondary FIFO initialization
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

  /*
   * Enable FIFO 1
   */

  if(PRINT_RTD_SETUP) printf("Enabling FIFO 1\n");

  dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_1, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

  
  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Allocate DMA Buffer
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  if(RTD_DMA){
    if(PRINT_RTD_SETUP) printf("Allocating DMA buffer\n");
    dm7820_status =
      DM7820_FIFO_DMA_Create_Buffer(&dma_out_buf, RTD_DMA_BUF_SIZE);
    DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Create_Buffer()");


  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    DMA 1 Configuration
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

    /*
     *  Initializing DMA 1
     */
    
    if(PRINT_RTD_SETUP) printf("Initializing DMA 1: \n");
    
    dm7820_status = DM7820_FIFO_DMA_Initialize(board,
					       DM7820_FIFO_QUEUE_1,
					       RTD_BUF_NUM, RTD_BUF_SIZE);
    DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Initialize()");
    
    /*
     *  Configuring DMA 1
     */
    
    if(PRINT_RTD_SETUP) printf(" -- Configuring DMA 1: PCI_TO_DM7820\n");
    
    dm7820_status = DM7820_FIFO_DMA_Configure(board,
					      DM7820_FIFO_QUEUE_1,
					      DM7820_DMA_DEMAND_ON_PCI_TO_DM7820,
					      RTD_BUF_SIZE);
    DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Configure()");

    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      Install ISR
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
    if(PRINT_RTD_SETUP) printf("Resetting frame counters\n");
    dma_done_frames=0;
    dma_done_interrupts=0;
    
    if(PRINT_RTD_SETUP) printf("Installing user ISR\n");
    dm7820_status = DM7820_General_InstallISR(board, rtd_ISR);
    DM7820_Return_Status(dm7820_status, "DM7820_General_InstallISR()");
    
    if(PRINT_RTD_SETUP) printf("Setting ISR priority: 99\n");
    dm7820_status = DM7820_General_SetISRPriority(board, 99);
    DM7820_Return_Status(dm7820_status, "DM7820_General_SetISRPriority()");
  }

}

static void
get_fifo_status(DM7820_Board_Descriptor * board,
		dm7820_fifo_queue fifo,
		dm7820_fifo_status_condition condition, uint8_t * status)
{
	if (DM7820_FIFO_Get_Status(board, fifo, condition, status) == -1) {
		error(EXIT_FAILURE, errno,
		      "ERROR: DM7820_FIFO_Get_Status() FAILED");
	}
}


/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  RTD_WRITE_FIFO
                - Writes parallel data out port 1.
                - Write data one word at a time directly to fifo.
                - Inserts empty code after each write.
                - DOES NOT WORK. TOO SLOW, DATA IS DOUBLE CLOCKED.
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
void rtd_write_fifo(char *buf, uint32_t num, int replace_empty){
  uint8_t  fifo_status     = 0;
  uint8_t  fifo_empty      = 0;
  uint8_t  fifo_full       = 0;
  uint8_t  fifo_underflow  = 0;
  uint8_t  fifo_overflow   = 0;
  uint32_t nwords          = 0;
  uint16_t *buf16;        
  uint32_t n=0,m=0;
  uint16_t empty           = 0xFADE;
  uint32_t fullcount       = 0;
  uint16_t output          = 0;

  //Everything written must be an integer number of 16bit words
  if(num % 2)
    printf("rtd_write_fifo: BAD DATA SIZE\n");
  
  //Setup pointers
  buf16  = (uint16_t *)buf;
  nwords = num/2;
  
  

 
  while(m<nwords){
    /*
     * Determine whether or not FIFO is full
     */
    
    get_fifo_status(board, DM7820_FIFO_QUEUE_1, DM7820_FIFO_STATUS_FULL,&fifo_full);
    
    /*#########################################################################
      Write data to fifo
      ######################################################################### */
    if(!fifo_full){
      output=*(buf16+m);
      if(replace_empty){
	if(output == RTD_EMPTY_CODE)
	  output = RTD_REPLACE_CODE;
      }
      if(DM7820_FIFO_Write(board, DM7820_FIFO_QUEUE_1,output)==0)
	m++;
    }
    else{
      if(fullcount++ == 100){
	printf("rtd_write_fifo: FIFO Full!\n");
	break;
      }
      usleep(100);
    }
  }
  
  //write trailing 0xFADE
    while(DM7820_FIFO_Write(board, DM7820_FIFO_QUEUE_1,empty))
      usleep(10);
}




/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  RTD_WRITE_DMA
                - Writes parallel data out port 1.
                - Inserts empty code after each write.
		- Uses DMA for transfering large blocks of data.
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
void rtd_write_dma(char *buf, uint32_t num,int replace_empty){
  static volatile uint32_t dma_started_frames=0;
  static volatile uint32_t m=0;
  uint32_t nwords = 0;
  uint16_t *buf16;
  uint16_t nwait;
  uint32_t maxdata = 65536;
  uint32_t i=0,l=0,n=0;
  uint8_t fifo_status = 0;
  uint8_t  fifo_full       = 0;
  uint32_t fullcount       = 0;
  DM7820_Error dm7820_status;
  
  
  //Everything written must be an integer number of 16bit words
  if(num % 2)
    printf("rtd_write_dma: BAD DATA SIZE\n");
  
  //Setup pointers
  buf16  = (uint16_t *)buf;
  nwords = num/2;
  
  //Filter out real empty codes from data
  if(replace_empty){
    for(i=0;i<nwords;i++)
      if(buf16[i] == RTD_EMPTY_CODE)
	buf16[i] = RTD_REPLACE_CODE;
  }
  
  
  l=0;
  n=0;
  while(l<nwords){
    //l --> number of words out of nwords written to the buffer
    //m --> total number of words already written to buffer
    //n --> number of words to write this time
    n = (nwords-l)<(RTD_SAMPLES-m-1) ? (nwords-l):(RTD_SAMPLES-m-1);
    
    //Copy local data into output buffer
    memcpy(&dma_out_buf[m],&buf16[l],n*sizeof(uint16_t));
      
    //Add n to m
    m+=n;
    
    //Add n to l
    l+=n;
    
    //Check if the buffer is full and we need to do a transfer
    if(m==RTD_SAMPLES-1){
      
      //Set last word of buffer to empty code
      dma_out_buf[m]=RTD_EMPTY_CODE;
      
      //Check if fifo is full
      get_fifo_status(board, DM7820_FIFO_QUEUE_1, DM7820_FIFO_STATUS_FULL,&fifo_full);
      
      //Wait for current transfer to finish
      nwait=0;
      while(dma_done_frames < dma_started_frames){
	if(nwait == 0)
	  printf("RTD: DMA timeout!\n");
	nwait++;
	usleep(10);
      }
      
      
      //Copy data to DMA buffer
      dm7820_status = DM7820_FIFO_DMA_Write(board,
					    DM7820_FIFO_QUEUE_1,
					    dma_out_buf, RTD_FRAME_NUM);
    
      //Reconfigure DMA
      dm7820_status = DM7820_FIFO_DMA_Configure(board,
						DM7820_FIFO_QUEUE_1,
						DM7820_DMA_DEMAND_ON_PCI_TO_DM7820,
						RTD_BUF_SIZE);
      DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Configure()");
      
      //Enable & Start DMA transfer
      dm7820_status = DM7820_FIFO_DMA_Enable(board,
					     DM7820_FIFO_QUEUE_1, 0xFF, 0xFF);
      DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Enable()");
      
      //First time through
      if(dma_started_frames == 0){
	//Wait for FIFO 1 to start filling
	fifo_status=0;
	do {
	  dm7820_status = DM7820_FIFO_Get_Status(board,
						 DM7820_FIFO_QUEUE_1,
						 DM7820_FIFO_STATUS_EMPTY,
						 &fifo_status);
	  DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Get_Status()");
	
	} while (fifo_status);
	
	
	//Enable FIFO 1 empty interrupt
	dm7820_status = DM7820_General_Enable_Interrupt(board,
							DM7820_INTERRUPT_FIFO_1_EMPTY,
							0xFF);
	DM7820_Return_Status(dm7820_status,"DM7820_General_Enable_Interrupt()");
      }
      
      //Increment started frames
      dma_started_frames++;
      
      //Zero out m
      m=0;
    }
  }
}
