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
#include <time.h>
#include <sys/stat.h>
#include <dm7820_library.h>

/* Networking */
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "rtd_functions.h"
#include "fakemodes.h"

#define NFAKE   102000
#define FAKEMAX 65536

/* Globals */
//--shared memory
int tlm_shmfd;
//--tcp server
volatile int tcpfd=-1;
//--udp server
volatile int udpfd=-1;

/* Listener Setup */
void *tlm_listen(void *t);
pthread_t listener_thread;


/* CTRL-C Function */
void tlmctrlC(int sig){
  close(tlm_shmfd);
  close(tcpfd);
  close(udpfd);
#if MSG_CTRLC
  printf("TLM: exiting\n");
#endif
  exit(sig);
}

/* Send data over TM */
void write_block(sm_t *sm_p, struct addrinfo *udp_ai, char *buf, uint32 num, int flush){
  static uint32 presync  = TLM_PRESYNC;
  static uint32 postsync = TLM_POSTSYNC;
  int tcpsend=0;
  
  
  /*Send TM over UDP ethernet*/
  if(udpfd >= 0){
    /*Write presync to socket */
    write_to_socket_udp(udpfd,udp_ai,&presync,sizeof(presync));
    /*Write buffer to socket */
    write_to_socket_udp(udpfd,udp_ai,buf,num);
    /*Write postsync to socket */
    write_to_socket_udp(udpfd,udp_ai,&postsync,sizeof(postsync));
#if TLM_DEBUG
    printf("TLM: write_block sent over UDP\n");
#endif
  }
  
  /*Send TM over TCP ethernet*/
  if(tcpfd >= 0){
    /*Write presync to socket */
    write_to_socket(tcpfd,&presync,sizeof(presync));
    /*Write buffer to socket */
    write_to_socket(tcpfd,buf,num);
    /*Write postsync to socket */
    write_to_socket(tcpfd,&postsync,sizeof(postsync));
    tcpsend=1;
#if TLM_DEBUG
    printf("TLM: write_block sent over TCP\n");
#endif
  }

  
  /*Send TM over RTD*/
  if(sm_p->tlm_ready && !tcpsend){
    /*Write presync to RTD FPGA*/
    if(rtd_send_tlm(sm_p->p_rtd_tlm_board, (char *)&presync,sizeof(presync),0)){
      printf("TLM: rtd_send_tlm failed!\n");
      return;
    }
    /*Write buffer to RTD FPGA*/
    if(rtd_send_tlm(sm_p->p_rtd_tlm_board, buf, num, 0)){
      printf("TLM: rtd_send_tlm failed!\n");
      return;
    }
    /*Write postsync to RTD FPGA*/
    if(rtd_send_tlm(sm_p->p_rtd_tlm_board, (char *)&postsync,sizeof(postsync),flush)){
      printf("TLM: rtd_send_tlm failed!\n");
      return;
    }
#if TLM_DEBUG
    printf("TLM: write_block sent over RTD\n");
#endif
  }
}


/* Save data to disk*/
void save_data(void *buf, uint32 num, char *tag, uint32 framenumber, uint32 folderindex){
  int fd;
  char filename[100];
  time_t now = time(NULL);
  
  /*Create filename*/
  sprintf(filename,DATANAME,folderindex,now,tag,framenumber);
  /*Open file*/
  fd = open(filename,O_RDWR | O_CREAT, 0666);
  if(fd<0){
    printf("TLM: error opening %s\n",filename);
    perror("TLM: open");
    close(fd);
    return;
  }
  /*Write buffer to file */
  if(write(fd,buf,num)<0){
    printf("TLM: save_data failed to write data to %s\n",filename);
    perror("TLM: write");
  }
  /*Close file*/
  close(fd);
#if MSG_SAVEDATA
  printf("TLM: wrote: %s\n",filename);
#endif 
}

/* Main tlm_proc */
void tlm_proc(void){
  uint32 i,j;
  unsigned long count=0;
  uint32 folderindex=0;
  char datpath[200];
  char pathcmd[200];
  uint16_t fakeword[NFAKE];
  uint16_t emptybuf[TLM_BUFFER_LENGTH];
  static uint32 ilast=0;
  struct stat st;
  char *buffer;
  int maxsize=0;
  int savedata=0;
  int sentdata=0;
  int readdata=0;
  uint32 savecount[NCIRCBUF]={0};
  struct timespec now,delta,last[NCIRCBUF],last_send;
  double dt,dt_send;
  int fakeflush=0;
  struct addrinfo hints, *ai, *p,*udp_ai;
  int rv;
  
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&tlm_shmfd)) == NULL){
    printf("openshm fail: tlm_proc\n");
    tlmctrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, tlmctrlC);	/* usually ^C */

  /* Start UDP connection */
  memset(&hints, 0, sizeof hints);
  hints.ai_family   = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags    = AI_PASSIVE;
  if ((rv = getaddrinfo(TLM_UDP_ADDR, TLM_UDP_PORT, &hints, &ai)) != 0) {
    printf("TLM: getaddrinfo error\n");
    fprintf(stderr, "TLM: %s\n", gai_strerror(rv));
    tlmctrlC(0);
  }
  // -- loop through all the results and make a socket
  for(p = ai; p != NULL; p = p->ai_next) {
    if ((udpfd = socket(p->ai_family, p->ai_socktype,p->ai_protocol)) == -1) {
      perror("TLM: socket");
      continue;
    }
    break;
  }
  if (p == NULL) {
    fprintf(stderr, "TLM: failed to create UDP socket\n");
    tlmctrlC(0);
  }
  udp_ai = p; //set final UDP AI struct
  printf("TLM: Opened UDP socket\n");
 
  /* Start listener */
  printf("TLM: Starting listener\n");
  pthread_create(&listener_thread,NULL,tlm_listen,(void *)0);

  /* Init RTD */
  if(sm_p->tlm_ready){
    if(rtd_init_tlm(sm_p->p_rtd_tlm_board,TLM_BUFFER_SIZE)){
      printf("TLM: rtd_init_tlm failed!\n");
      tlmctrlC(0);
    }
  }
  
  /* Fill out empty buffer*/
  for(i=0;i<TLM_BUFFER_LENGTH;i++)
    emptybuf[i]=TLM_EMPTY_CODE;

  /* Allocate packet buffer */
  for(i=0;i<NCIRCBUF;i++)
    if(sm_p->circbuf[i].nbytes > maxsize)
      maxsize = sm_p->circbuf[i].nbytes;
  if((buffer = malloc(maxsize)) == NULL){
    printf("TLM: buffer malloc failed!\n");
    tlmctrlC(0);
  }
  
  /* Check if we are saving any data */
  for(i=0;i<NCIRCBUF;i++)
    if(sm_p->circbuf[i].save)
      savedata=1;

  /* Create folder for saved data */
  if(savedata){
    while(1){
      sprintf(datpath,DATAPATH,folderindex);
      if(stat(datpath,&st))
	break;
      folderindex++;
    }
    recursive_mkdir(datpath, 0777);

    printf("TLM: Saving data to: %s\n",datpath);
  }

  /* Init last data times */
  clock_gettime(CLOCK_REALTIME,&last_send);
  for(i=0;i<NCIRCBUF;i++)
    clock_gettime(CLOCK_REALTIME,&last[i]);
  
  /*****************************************************/
  /* MAIN LOOP *****************************************/
  /*****************************************************/
  while(1){
    
    /* Check if we want to fake the TM data */
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
      }
      if(sm_p->w[TLMID].fakemode == FAKEMODE_TEST_PATTERN2){
	for(i=0;i<NFAKE;i++){
	  if(i % 2) fakeword[i] = 0x0000; else fakeword[i] = 0xFFFF;
	}
      }
      
      //Check if we've been asked to exit
      if(sm_p->w[TLMID].die)
	tlmctrlC(0);
      
      //Check in with watchdog
      checkin(sm_p,TLMID);
      
      /*Write Data*/
      if(tcpfd >= 0){
	write_to_socket(tcpfd,fakeword,sizeof(uint16)*NFAKE);
	//sleep (time @ 250000 Wps)
	usleep((long)(ONE_MILLION * ((double)NFAKE / (double)TLM_DATA_RATE)));
	
      }else{
	//RTD write fake data
	if(sm_p->tlm_ready){
	  fakeflush=0;
	  if(sm_p->w[TLMID].fakemode == FAKEMODE_TEST_PATTERN3)
	    fakeflush=2; 
	  if(rtd_send_tlm(sm_p->p_rtd_tlm_board,(char *)fakeword,sizeof(uint16)*NFAKE,fakeflush)){
	    printf("TLM: rtd_send_tlm failed!\n");
	    tlmctrlC(0);
	  }
	  if(sm_p->w[TLMID].fakemode != FAKEMODE_TEST_PATTERN3) usleep(50000);
	}
      }
      continue;
    }

    /* Send real TM data */
    sentdata=0;
    
    //Check if we've been asked to exit
    if(sm_p->w[TLMID].die)
      tlmctrlC(0);

    //Loop over circular buffers
    for(i=0;i<NCIRCBUF;i++){
      if(sm_p->circbuf[i].send || sm_p->circbuf[i].save){
	//Get time
	clock_gettime(CLOCK_REALTIME,&now);
	if(timespec_subtract(&delta,&now,&last[i]))
	  printf("SRV: timespec_subtract error!\n");
	ts2double(&delta,&dt);
	if(timespec_subtract(&delta,&now,&last_send))
	  printf("SRV: timespec_subtract error!\n");
	ts2double(&delta,&dt_send);
	
	//Read data
	readdata=0;
	if(sm_p->circbuf[i].send == 1)
	  if(read_from_buffer(sm_p, buffer, i, TLMID))
	    readdata=1;
	if(sm_p->circbuf[i].send == 2)
	  if(read_newest_buffer(sm_p, buffer, i, TLMID))
	    readdata=1;
	//Send & save data
	if(readdata){
	  //Send data
	  if(sm_p->circbuf[i].send){
	    write_block(sm_p,udp_ai,buffer,sm_p->circbuf[i].nbytes,(dt_send > 0.5));
	    //save last send time
	    memcpy(&last_send,&now,sizeof(struct timespec));
	  }
	  //Save data
	  if(sm_p->circbuf[i].save){
	    save_data(buffer, sm_p->circbuf[i].nbytes,(char *)sm_p->circbuf[i].name,savecount[i]++,folderindex);
	  }
	  sentdata=1;
	  //save time
	  memcpy(&last[i],&now,sizeof(struct timespec));
	}
      }
    }

    //Checkin with watchdog
    checkin(sm_p,TLMID);

    //Sleep if no data
    if(!sentdata){
      usleep(10000);
    }
  }
      
  tlmctrlC(0);
  return;
}
