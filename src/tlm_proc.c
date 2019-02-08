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
#include <dm7820_library.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "rtd_functions.h"
#include "tlm_proc.h"
#include "fakemodes.h"

#define SLEEP_TIME 10000
#define NFAKE 100000
#define FAKEMAX 65536

/* Globals */
//--shared memory
int tlm_shmfd;
//--ethernet server
volatile int ethfd=-1;

/* Listener Setup */
void *tlm_listen(void *t);
pthread_t listener_thread;


/******************************************************************************
        Data Checking Functions
******************************************************************************/
int checkdata(sm_t *sm_p){
  static int s,k,l,a;
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  s = check_buffer(sm_p, BUFFER_SCIEVENT, TLMID);
  k = check_buffer(sm_p, BUFFER_SHKEVENT, TLMID);
  l = check_buffer(sm_p, BUFFER_LYTEVENT, TLMID);
  a = check_buffer(sm_p, BUFFER_ACQEVENT, TLMID);
  
  return(s || k || l || a);
}

int checksci(sm_t *sm_p){
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  return(check_buffer(sm_p, BUFFER_SCIEVENT, TLMID));
}
int checkshk(sm_t *sm_p){
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  return(check_buffer(sm_p, BUFFER_SHKEVENT, TLMID));
}
int checklyt(sm_t *sm_p){
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  return(check_buffer(sm_p, BUFFER_LYTEVENT, TLMID));
}
int checkacq(sm_t *sm_p){
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  return(check_buffer(sm_p, BUFFER_ACQEVENT, TLMID));
}



void tlmctrlC(int sig){
  close(tlm_shmfd);
  close(ethfd);
#if MSG_CTRLC
  printf("TLM: exiting\n");
#endif
  exit(sig);
}

void write_block(DM7820_Board_Descriptor* p_rtd_board, char *buf, uint32 num){
  static uint32 presync  = TLM_PRESYNC;
  static uint32 postsync = TLM_POSTSYNC;
  
  /*Send TM over ethernet*/
  if(ethfd >= 0){
    /*Write presync to socket */
    write_to_socket(ethfd,&presync,sizeof(presync));
    /*Write buffer to socket */
    write_to_socket(ethfd,buf,num);
    /*Write postsync to socket */
    write_to_socket(ethfd,&postsync,sizeof(postsync));
    
#if TLM_DEBUG
    printf("TLM: write_block sent over ethernet\n");
#endif
  }
  else{
    /*Send TM over RTD*/
    if(p_rtd_board != NULL){
      /*Write presync to dma */
      rtd_send_tlm(p_rtd_board, (char *)&presync,sizeof(presync));
      /*Write buffer to dma */
      rtd_send_tlm(p_rtd_board, buf,num);
      /*Write postsync to dma */
      rtd_send_tlm(p_rtd_board, (char *)&postsync,sizeof(postsync));
      
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
  char tag[3];
  uint32 folderindex=0;
  char datpath[200];
  char pathcmd[200];
  uint16_t fakeword[NFAKE];
  uint16_t emptybuf[TLM_BUFFER_LENGTH];
  static uint32 ilast=0;
  struct stat st;
  static scievent_t sci;
  static shkevent_t shk;
  static lytevent_t lyt;
  static acqevent_t acq;
  
  
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
  pthread_create(&listener_thread,NULL,tlm_listen,(void *)0);

  /* Init RTD */
  if(sm_p->tlm_ready)
    rtd_init_tlm(sm_p->p_rtd_board,TLM_BUFFER_SIZE);
   
  /* Fill out empty buffer*/
  for(i=0;i<TLM_BUFFER_LENGTH;i++)
    emptybuf[i]=TLM_EMPTY_CODE;
  
  
  /* Create folder for saved data */
  if(SAVE_SCI || SAVE_SHK || SAVE_LYT || SAVE_ACQ){
    while(1){
      sprintf(datpath,DATAPATH,folderindex);
      if(stat(datpath,&st))
	break;
      folderindex++;
    }
    recursive_mkdir(datpath, 0777);
    if(MSG_SAVEDATA)
      printf("TLM: Saving data in: %s\n",datpath);
  }
  while(1){
    
    //check if we want to fake the TM data
    if(sm_p->w[TLMID].fakemode != FAKEMODE_NONE){
      if(sm_p->w[TLMID].fakemode == FAKEMODE_TEST_PATTERN){
	for(i=0;i<NFAKE;i++){
	  fakeword[i] = ilast++ % FAKEMAX;
	  //skip empty code
	  if(fakeword[i]==TLM_EMPTY_CODE){
	    fakeword[i]++;
	    ilast++;
	  }
	}
	ilast%=FAKEMAX;
	
	checkin(sm_p,TLMID);
	
	/*Write Data*/
	if(ethfd >= 0){
	  write_to_socket(ethfd,fakeword,sizeof(uint16)*NFAKE);
	  //sleep (time @ 250000 Wps)
	  usleep((long)(ONE_MILLION * ((double)NFAKE / (double)TLM_DATA_RATE)));
	  
	}else{
	  //RTD write fake data
	  if(sm_p->tlm_ready){
	    if(rtd_send_tlm(sm_p->p_rtd_board,(char *)fakeword,sizeof(uint16)*NFAKE))
	      printf("TLM: rtd_send_tlm failed!\n");
	    usleep(50000);
	  }
	}
      }
    }
    else{
      //Check if we've been asked to exit
      if(sm_p->w[TLMID].die)
	tlmctrlC(0);
      
      //Check for new data
      if(!checkdata(sm_p)){
	usleep(100000);
      }
      else{
	
	/*Get SCI data*/
	if(read_from_buffer(sm_p, &sci, BUFFER_SCIEVENT, TLMID)){
	  /*Check in with watchdog*/
	  checkin(sm_p,TLMID);
	  //save science data 
	  if(SAVE_SCI){
	    sprintf(tag,"sci");
	      save_data(&sci, sizeof(sci),tag,sci.hed.frame_number,folderindex);
	  }
	  if(SEND_SCI){
	    //write data
	    if(sm_p->tlm_ready){
	      write_block(sm_p->p_rtd_board,(char *)&sci, sizeof(sci));
	      if(TLM_DEBUG)
		printf("TLM: Frame %d - SCI\n",sci.hed.frame_number);
	    }
	  }
	}

	/*Get SHK data*/
	if(read_from_buffer(sm_p, &shk, BUFFER_SHKEVENT, TLMID)){
	  //check in with watchdog
	  checkin(sm_p,TLMID);
	  //save shk data 
	  if(SAVE_SHK){
	    sprintf(tag,"shk");
	      save_data(&shk, sizeof(shk),tag,shk.hed.frame_number,folderindex);
	  }
	  //send shk data
	  if(SEND_SHK){
	    if(sm_p->tlm_ready){
	      write_block(sm_p->p_rtd_board,(char *)&shk, sizeof(shk));
	      if(TLM_DEBUG)
		printf("TLM: Frame %d - SHK\n",shk.hed.frame_number);
	    }
	  }
	}
	
	/*Get LYT data*/
	if(read_from_buffer(sm_p, &lyt, BUFFER_LYTEVENT, TLMID)){
	  //check in with watchdog
	  checkin(sm_p,TLMID);
	  //save lyt data 
	  if(SAVE_LYT){
	    sprintf(tag,"lyt");
	      save_data(&lyt, sizeof(lyt),tag,lyt.hed.frame_number,folderindex);
	  }
	  //send lyt data
	  if(SEND_LYT){
	    if(sm_p->tlm_ready){
	      write_block(sm_p->p_rtd_board,(char *)&lyt, sizeof(lyt));
	      if(TLM_DEBUG)
		printf("TLM: Frame %d - LYT\n",lyt.hed.frame_number);
	    }
	  }
	}
	
	/*Get ACQ data*/
	if(read_from_buffer(sm_p, &acq, BUFFER_ACQEVENT, TLMID)){
	  //check in with watchdog
	  checkin(sm_p,TLMID);
	  //save acq data 
	  if(SAVE_ACQ){
	    sprintf(tag,"acq");
	      save_data(&acq, sizeof(acq),tag,acq.hed.frame_number,folderindex);
	  }
	  //send acq data
	  if(SEND_ACQ){
	    if(sm_p->tlm_ready){
	      write_block(sm_p->p_rtd_board,(char *)&acq, sizeof(acq));
	      if(TLM_DEBUG)
		printf("TLM: Frame %d - ACQ\n",acq.hed.frame_number);
	    }
	  }
	}
      }
    }
  }
      
  tlmctrlC(0);
  return;
}
