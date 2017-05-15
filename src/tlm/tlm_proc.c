#include <user_header.h>
#include <controller.h>
#include <common_functions.h>
#include <dm7820_library.h>

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
  static int s,p,f,h;
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  s = check_science_buffer(sm_p, TLMID);
  f = check_frame_buffer(sm_p, TLMID);
  p = check_phase_buffer(sm_p, TLMID);
  h = check_hsk_buffer(sm_p, TLMID);
  
  return(s || f || p || h);
}

int checkscience(sm_t *sm_p){
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  return(check_science_buffer(sm_p, TLMID));
}

int checkframe(sm_t *sm_p){
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  return(check_frame_buffer(sm_p, TLMID));
}

int checkphase(sm_t *sm_p){
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  return(check_phase_buffer(sm_p, TLMID));
}

int checkhsk(sm_t *sm_p){
  /*Check in with watchdog*/
  checkin(sm_p,TLMID);
  return(check_hsk_buffer(sm_p, TLMID));
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
  /*Write buffer to fifo */
  write(fd,buf,num);
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
  float dm_image[DMXS][DMYS];
  char tag[1];
  static science_t science;
  static frame_t   frame;
  static phase_t   phase;
  static hsk_t     hsk;
  static tlmheader_t tlmHED;
  const  char *phase_str[NUM_PHASE]  = PHASE_STR;
  const  char *frame_str[NUM_FRAME]  = FRAME_STR;
  const  int  frame_send[NUM_FRAME] = FRAME_SEND; 
  const  int  phase_send[NUM_PHASE] = PHASE_SEND;
  int val=0;
  
  /*Init structures*/
  memset(&frame,0,sizeof(frame));
  memset(&phase,0,sizeof(phase));
  memset(&tlmHED,0,sizeof(tlmHED));
  
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
#if (SAVE_PHASE || SAVE_FRAME || SAVE_SCIENCE)
  uint32 folderindex=0;
  char datpath[200];
  char pathcmd[200];
  struct stat st;
  while(1){
    sprintf(datpath,DATAPATH,folderindex);
    if(stat(datpath,&st))
      break;
    folderindex++;
  }
  sprintf(pathcmd,"mkdir %s",datpath);
#if MSG_SAVEDATA 
  printf("TLM: Saving data in: %s\n",datpath);
#endif
  system(pathcmd);
#endif
  
  while(1){
    
    //check if we want to fake the TM data
    if(sm_p->fake_mode & FAKE_TM_TEST_PATTERN){
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
	/*Get science data*/
	if(read_from_science_buffer(sm_p, &science, TLMID)){
	  /*Check in with watchdog*/
	  checkin(sm_p,TLMID);
	  //save science data 
	  if(SAVE_SCIENCE){
	    sprintf(tag,"S");
	      save_data(&science, sizeof(science),tag,science.frame_number,folderindex);
	  }
	  /*Fill out packet header*/
	  tlmHED.packet_type  = TLM_SCIENCE;
	  tlmHED.frame_number = science.frame_number;
	  tlmHED.exptime      = science.exptime;
	  tlmHED.ontime       = science.ontime;
	  tlmHED.temp         = science.temp;
	  tlmHED.state        = science.state;
	  tlmHED.mode         = science.mode;
	  tlmHED.timestamp    = science.timestamp;
	  tlmHED.imxsize      = science.imxsize;
	  tlmHED.imysize      = science.imysize;
	  if(SCI_SEND){
	    //write data
	    write_block((char *)&tlmHED, (char *)&science.S, sizeof(science.S),0);
#if TLM_DEBUG
	    printf("TLM: Frame %d - SCI\n",tlmHED.frame_number);
#endif
	  }
	}
	
	/*Get frame data*/
	if(read_from_frame_buffer(sm_p, &frame, TLMID)){
	  /*Check in with watchdog*/
	  checkin(sm_p,TLMID);
	  //save frame data 
	  if(SAVE_FRAME){
	    sprintf(tag,"F");
	    save_data(&frame, sizeof(frame),tag,frame.frame_number,folderindex);
	  }
	  tlmHED.frame_number = frame.frame_number;
	  tlmHED.exptime      = frame.exptime;
	  tlmHED.imxsize      = frame.imxsize;
	  tlmHED.imysize      = frame.imysize;
	  tlmHED.temp         = frame.temp;
	  tlmHED.state        = frame.state;
	  tlmHED.mode         = frame.mode;
	  for(i=0;i<NUM_FRAME;i++){
	    tlmHED.packet_type  = TLM_FRAME | i;
	    tlmHED.ontime       = frame.ontime[i];
	    tlmHED.timestamp    = frame.timestamp[i];
	    if(frame_send[i]){
	      //write data
	      write_block((char *)&tlmHED, (char *)&frame.im[i], sizeof(frame.im[i]),0);
#if TLM_DEBUG
	      printf("TLM: Frame %d - WFS %s\n",tlmHED.frame_number,frame_str[i]);
#endif
	    }
	  }
	}
	
	
	/*Get phase data*/
	if(read_from_phase_buffer(sm_p, &phase, TLMID)){
	  /*Check in with watchdog*/
	  checkin(sm_p,TLMID);
	  //save phase data 
	  if(SAVE_PHASE){
	    sprintf(tag,"P");
	    save_data(&phase, sizeof(phase),tag,phase.frame_number,folderindex);
	  }
	  /*Fill out packet header*/
	  tlmHED.frame_number = phase.frame_number;
	  tlmHED.exptime      = phase.exptime;
	  tlmHED.ontime       = phase.ontime;
	  tlmHED.timestamp    = phase.timestamp;
	  tlmHED.imxsize      = phase.imxsize;
	  tlmHED.imysize      = phase.imysize;
	  tlmHED.temp         = phase.temp;
	  tlmHED.state        = phase.state;
	  tlmHED.mode         = phase.mode;
	  
	  /*SEND numerical data*/
	  tlmHED.packet_type  = TLM_DATA;
	  if(DATA_SEND){
	    //write data
	    write_block((char *)&tlmHED, (char *)&phase.data, sizeof(data_t),1);
#if TLM_DEBUG
	    printf("TLM: Frame %d - DATA\n",tlmHED.frame_number);
#endif
	  }
	  /*SEND data products*/
	  for(i=0;i<NUM_PHASE;i++){
	    tlmHED.packet_type  = TLM_PHASE | i;
	    if(phase_send[i]){
	      //write data
	      write_block((char *)&tlmHED, (char *)&phase.im[i], sizeof(phase.im[i]),1);
#if TLM_DEBUG
	      printf("TLM: Phase %d %s\n",tlmHED.frame_number,phase_str[i]);
#endif
	    }
	  }
	  
	  /*SEND DM Map*/
	  if(DM_SEND){
	    for(i=0;i<DMXS;i++)
	      for(j=0;j<DMYS;j++)
		dm_image[i][j] = (float)phase.dm_image[i][j];
	    tlmHED.packet_type  = TLM_DM;
	    tlmHED.imxsize  = DMXS;
	    tlmHED.imysize  = DMYS;
	    write_block((char *)&tlmHED, (char *)&dm_image, sizeof(dm_image),1);
#if TLM_DEBUG
	    printf("TLM: DM %d\n",tlmHED.frame_number);
#endif
	  }
	}
	
	
#if PARALLEL_HSK
	/*Get hsk*/
	while(read_from_hsk_buffer(sm_p, &hsk, TLMID)){
	  /*Check in with watchdog*/
	  checkin(sm_p,TLMID);
	  /*SEND housekeeping*/
	  tlmHED.packet_type  = TLM_HSK;
	  tlmHED.frame_number = 1;
	  tlmHED.exptime      = 1;
	  tlmHED.ontime       = 1;
	  tlmHED.temp         = sm_p->wfs_temp;
	  tlmHED.state        = sm_p->state;
	  tlmHED.mode         = sm_p->wfs_mode;
	  tlmHED.timestamp    = 1;
	  tlmHED.imxsize      = 1;
	  tlmHED.imysize      = 1;
	  //write data
	  write_block((char *)&tlmHED, (char *)&hsk, sizeof(hsk_t),1);
#if TLM_DEBUG
	  printf("TLM: HSK\n");
#endif
	}
	
#endif //PARALLEL_HSK
      }
    }
  }
  tlmctrlC(0);
  return;
}
