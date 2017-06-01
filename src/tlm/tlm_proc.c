#define _XOPEN_SOURCE 500
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/stat.h>

/* piccflight headers */
#include "../common/controller.h"
#include "../common/common_functions.h"
#include "../rtd/dm7820_library.h"
#include "../rtd/rtd.h"
#include "tlm_proc.h"

#define SLEEP_TIME 10000
#define NFAKE 100000
#define FAKEMAX 65536
#define TLM_RTD 0
#define TLM_DEVICE TLM_RTD

/* Globals */
//--shared memory
int tlm_shmfd;
//--ethernet server
volatile int ethfd=-1;
//--RTD DM7820 device descriptor
DM7820_Board_Descriptor *board;

//--number of interrupts counted
volatile unsigned long fifo_empty_interrupts = 0;  
volatile unsigned long dma_done_interrupts = 0;  
volatile unsigned long dma_done_frames = 0;


/*RTD Prototypes*/
void rtd_cleanup(void);
void rtd_init(void);
void rtd_write_fifo(char *buf, uint32_t num, int replace_empty);
void rtd_write_dma(char *buf, uint32_t num, int replace_empty);


/* Listener stuff */
void *tlm_listener(void *t);
pthread_t listener_thread;


/******************************************************************************
        Data Checking Functions
******************************************************************************/
int checkdata(sm_t *sm_p){
  static int s,k,l,a;
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  s = check_buffer(sm_p, SCIEVENT, TLMID);
  k = check_buffer(sm_p, SHKEVENT, TLMID);
  l = check_buffer(sm_p, LYTEVENT, TLMID);
  a = check_buffer(sm_p, ACQEVENT, TLMID);
  
  return(s || k || l || a);
}

int checksci(sm_t *sm_p){
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  return(check_buffer(sm_p, SCIEVENT, TLMID));
}
int checkshk(sm_t *sm_p){
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  return(check_buffer(sm_p, SHKEVENT, TLMID));
}
int checklyt(sm_t *sm_p){
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  return(check_buffer(sm_p, LYTEVENT, TLMID));
}
int checkacq(sm_t *sm_p){
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  return(check_buffer(sm_p, ACQEVENT, TLMID));
}



void tlmctrlC(int sig){
#if MSG_CTRLC
  printf("TLM: ctrlC! exiting.\n");
#endif
  close(tlm_shmfd);

#if TLM_DEVICE == TLM_RTD
  rtd_cleanup();
#endif

  close(ethfd);
  exit(sig);
}

void write_block(char *hed,char *buf, uint32 num, int invert_buf){
  static uint32 presync  = TLM_PRESYNC;
  static uint32 postsync = TLM_POSTSYNC;
  static unsigned long message;
  
  /*Send TM over ethernet*/
  if(ethfd >= 0){
    /*Write presync to socket */
    write_to_socket(ethfd,&presync,sizeof(presync));
    /*Write header to socket */
    write_to_socket(ethfd,hed,sizeof(tlmheader_t));
    /*Write image to socket */
    write_to_socket(ethfd,buf,num);
    /*Write postsync to socket */
    write_to_socket(ethfd,&postsync,sizeof(postsync));
    
#if TLM_DEBUG
    printf("TLM: write_block sent over ethernet\n");
#endif
  }
  else{
    /*Send TM over RTD*/
    if(board != NULL){
      if(RTD_DMA){
	/*Write presync to dma */
	rtd_write_dma((char *)&presync,sizeof(presync),1);
	/*Write header to dma */
	rtd_write_dma(hed,sizeof(tlmheader_t),1);
	/*Write image to dma */
	rtd_write_dma(buf,num,1);
	/*Write postsync to dma */
	rtd_write_dma((char *)&postsync,sizeof(postsync),1);
      }
      else{
	/*Write presync to fifo */
	rtd_write_fifo((char *)&presync,sizeof(presync),1);
	/*Write header to fifo */
	rtd_write_fifo(hed,sizeof(tlmheader_t),1);
	/*Write image to fifo */
	rtd_write_fifo(buf,num,1);
	/*Write postsync to fifo */
	rtd_write_fifo((char *)&postsync,sizeof(postsync),1);
      }
#if TLM_DEBUG
      printf("TLM: write_block sent over RTD\n");
#endif
    }
  }
}


/* Save data to disk*/
void save_data(void *buf, uint32 num, char *tag, uint32 framenumber, uint32 folderindex){
  int fd;
  char filename[100];
  
  /*Create filename*/
  sprintf(filename,DATANAME,folderindex,tag,framenumber);
  /*Open file*/
  fd = open(filename,O_RDWR | O_CREAT, 0666);
  if(fd<0){
    printf("TLM: error opening %s\n",filename);
    perror("TLM: open");
    close(fd);
    return;
  }
  /*Write buffer to file */
  if(write(fd,buf,num))
    printf("TLM: save_data failed to write data!\n");
  
  /*Close file*/
  close(fd);
#if MSG_SAVEDATA
  printf("TLM: wrote: %s\n",filename);
#endif 
}

void tlm_proc(void){
  uint32 i,j;
  unsigned long count=0;
  unsigned long message;
  char tag[3];
  uint32 folderindex=0;
  char datpath[200];
  char pathcmd[200];
  struct stat st;
  static scievent_t sci;
  static shkevent_t shk;
  static lytevent_t lyt;
  static acqevent_t acq;
  static tlmheader_t tlmHED;
  
  
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&tlm_shmfd)) == NULL){
    printf("openshm fail: tlm_proc\n");
    tlmctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, tlmctrlC);	/* usually ^C */
  
  /* Start listener */
  printf("TLM: Starting listener\n");
  pthread_create(&listener_thread,NULL,tlm_listener,(void *)0);

  /* Init RTD */
  rtd_init();
 
  /*Create fake data */
  static uint32 ilast=0;
  uint16 *fakeword;
  if((fakeword = (uint16 *)malloc(sizeof(uint16)*NFAKE))==NULL){
    perror("malloc");
    free(fakeword);
    exit(0);
  }
  
  /*Create empty data*/
  uint16 *emptybuf;
  if((emptybuf = (uint16 *)malloc(sizeof(uint16)*RTD_BUF_SAMPLES))==NULL){
    perror("malloc");
    free(emptybuf);
    exit(0);
  }
  for(i=0;i<RTD_BUF_SAMPLES;i++)
    emptybuf[i]=RTD_EMPTY_CODE;

  
  /* Create folder for saved data */
  if(SAVE_SCI || SAVE_SHK || SAVE_LYT || SAVE_ACQ){
    while(1){
      sprintf(datpath,DATAPATH,folderindex);
      if(stat(datpath,&st))
	break;
      folderindex++;
    }
    sprintf(pathcmd,"mkdir %s",datpath);
    if(system(pathcmd)){
      printf("TLM: Failed to create save data folder (%d)\n",errno);
      tlmctrlC(0);
    }
    if(MSG_SAVEDATA)
      printf("TLM: Saving data in: %s\n",datpath);
  }
  while(1){
    
    //check if we want to fake the TM data
    if(sm_p->tlm_fake_mode & FAKE_TM_TEST_PATTERN){
      for(i=0;i<NFAKE;i++){
	fakeword[i] = ilast++ % FAKEMAX;
	//skip empty code
	if(fakeword[i]==RTD_EMPTY_CODE){
	  fakeword[i]++;
	  ilast++;
	}
      }
      ilast%=FAKEMAX;
      
      checkin(sm_p,TLMID);
      
      /*Write Data*/
      if(ethfd >= 0){
	write_to_socket(ethfd,fakeword,sizeof(uint16)*NFAKE);
	//sleep (time @ 200000 Wps)
	usleep((long)(ONE_MILLION * ((double)NFAKE / (double)200000)));
	
      }else{
	//RTD write fake data
	if(board != NULL){
	  if(RTD_DMA)
	    rtd_write_dma((char *)fakeword,sizeof(uint16)*NFAKE,1);
	  else
	    rtd_write_fifo((char *)fakeword,sizeof(uint16)*NFAKE,1);
	}
	//sleep (time @ 200000 Wps)
	usleep((long)(ONE_MILLION * ((double)NFAKE / (double)200000)));
      }
     
    }
    else{
      //Check if we've been asked to exit
      if(sm_p->w[TLMID].die)
	tlmctrlC(0);
      
      //Check for new data
      if(!checkdata(sm_p)){
	if(ethfd >= 0){
	  //Do nothing
	}
	else{
	  //Flush DMA buffer
	  if(board != NULL){
	    if(RTD_DMA){
	      rtd_write_dma((char *)emptybuf,sizeof(uint16)*RTD_BUF_SAMPLES,0);
	    }
	  }
	}
	usleep(100000);
      }
      else{

	/*Get SCI data*/
	if(read_from_buffer(sm_p, &sci, SCIEVENT, TLMID)){
	  /*Check in with watchdog*/
	  checkin(sm_p,TLMID);
	  //save science data 
	  if(SAVE_SCI){
	    sprintf(tag,"sci");
	      save_data(&sci, sizeof(sci),tag,sci.frame_number,folderindex);
	  }
	  /*Fill out packet header*/
	  tlmHED.packet_type  = TLM_SCI;
	  
	  if(SEND_SCI){
	    //write data
	    write_block((char *)&tlmHED, (char *)&sci.image, sizeof(sci.image),0);
	    if(TLM_DEBUG)
	      printf("TLM: Frame %d - SCI\n",tlmHED.frame_number);
	  }
	}


	
      }
    }
  }
      
  tlmctrlC(0);
  return;
}
