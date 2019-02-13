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
  k = check_buffer(sm_p, BUFFER_SHKPKT, TLMID);
  l = check_buffer(sm_p, BUFFER_LYTPKT, TLMID);
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
  return(check_buffer(sm_p, BUFFER_SHKPKT, TLMID));
}
int checklyt(sm_t *sm_p){
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  return(check_buffer(sm_p, BUFFER_LYTPKT, TLMID));
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
  FILE *fd=NULL;
  char filename[MAX_FILENAME];
  
  /*Create filename*/
  sprintf(filename,DATANAME,folderindex,tag,framenumber);

  /*Open file*/
  if((fd = fopen(filename, "w")) == NULL){
    perror("TLM: save_data fopen()\n");
    return;
  }
  
  /*Write buffer to file */
  if(fwrite(buf,num,1,fd) != 1)
    printf("TLM: save_data failed to write %s\n",filename);
  
  /*Close file*/
  fclose(fd);

#if MSG_SAVEDATA
  printf("TLM: wrote: %s\n",filename);
#endif 
}

void tlm_proc(void){
  uint32 i,j;
  unsigned long count=0;
  char tag[30];
  uint32 folderindex=0;
  char datpath[200];
  char pathcmd[200];
  uint16_t fakeword[NFAKE];
  uint16_t emptybuf[TLM_BUFFER_LENGTH];
  static uint32 ilast=0;
  struct stat st;
  static scievent_t scievent;
  static shkpkt_t   shkpkt;
  static lytpkt_t   lytpkt;
  static acqevent_t acqevent;
  
  
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
  if(SAVE_SCIEVENT || SAVE_SHKPKT || SAVE_LYTPKT || SAVE_ACQEVENT){
    while(1){
      sprintf(datpath,DATAPATH,folderindex);
      if(stat(datpath,&st))
	break;
      folderindex++;
    }
    recursive_mkdir(datpath, 0777);
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

	/*Get SCIEVENT data*/
	if(read_from_buffer(sm_p, &scievent, BUFFER_SCIEVENT, TLMID)){
	  //check in with watchdog
	  checkin(sm_p,TLMID);
	  //save scievent data 
	  if(SAVE_SCIEVENT){
	    sprintf(tag,"scievent");
	    save_data(&scievent, sizeof(scievent),tag,scievent.hed.frame_number,folderindex);
	  }
	  //send scievent data
	  if(SEND_SCIEVENT){
	    if(sm_p->tlm_ready){
	      write_block(sm_p->p_rtd_board,(char *)&scievent, sizeof(scievent));
	      if(TLM_DEBUG)
		printf("TLM: Frame %d - SCIEVENT\n",scievent.hed.frame_number);
	    }
	  }
	}
	
	/*Get SHKPKT data*/
	if(read_from_buffer(sm_p, &shkpkt, BUFFER_SHKPKT, TLMID)){
	  //check in with watchdog
	  checkin(sm_p,TLMID);
	  //save shkpkt data 
	  if(SAVE_SHKPKT){
	    sprintf(tag,"shkpkt");
	    save_data(&shkpkt, sizeof(shkpkt),tag,shkpkt.hed.frame_number,folderindex);
	  }
	  //send shkpkt data
	  if(SEND_SHKPKT){
	    if(sm_p->tlm_ready){
	      write_block(sm_p->p_rtd_board,(char *)&shkpkt, sizeof(shkpkt));
	      if(TLM_DEBUG)
		printf("TLM: Frame %d - SHKPKT\n",shkpkt.hed.frame_number);
	    }
	  }
	}
	
	/*Get LYTPKT data*/
	if(read_from_buffer(sm_p, &lytpkt, BUFFER_LYTPKT, TLMID)){
	  //check in with watchdog
	  checkin(sm_p,TLMID);
	  //save lytpkt data 
	  if(SAVE_LYTPKT){
	    sprintf(tag,"lytpkt");
	    save_data(&lytpkt, sizeof(lytpkt),tag,lytpkt.hed.frame_number,folderindex);
	  }
	  //send lytpkt data
	  if(SEND_LYTPKT){
	    if(sm_p->tlm_ready){
	      write_block(sm_p->p_rtd_board,(char *)&lytpkt, sizeof(lytpkt));
	      if(TLM_DEBUG)
		printf("TLM: Frame %d - LYTPKT\n",lytpkt.hed.frame_number);
	    }
	  }
	}
	
	/*Get ACQEVENT data*/
	if(read_from_buffer(sm_p, &acqevent, BUFFER_ACQEVENT, TLMID)){
	  //check in with watchdog
	  checkin(sm_p,TLMID);
	  //save acqevent data 
	  if(SAVE_ACQEVENT){
	    sprintf(tag,"acqevent");
	    save_data(&acqevent, sizeof(acqevent),tag,acqevent.hed.frame_number,folderindex);
	  }
	  //send acqevent data
	  if(SEND_ACQEVENT){
	    if(sm_p->tlm_ready){
	      write_block(sm_p->p_rtd_board,(char *)&acqevent, sizeof(acqevent));
	      if(TLM_DEBUG)
		printf("TLM: Frame %d - ACQEVENT\n",acqevent.hed.frame_number);
	    }
	  }
	}
      }
    }
  }
  
  tlmctrlC(0);
  return;
}
