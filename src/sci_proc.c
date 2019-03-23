#define _XOPEN_SOURCE 500
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <libfli.h>
#include <libgen.h>
#include <sys/stat.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "fakemodes.h"

#define SCI_ROI_XSIZE 2840
#define SCI_ROI_YSIZE 2224
#define SCI_HBIN 1 //do not change, will break code below
#define SCI_VBIN 1 //do not change, will break code below
#define SCI_UL_X 0
#define SCI_UL_Y 0
#define SCI_LR_X (SCI_UL_X+(SCI_ROI_XSIZE/SCI_HBIN))
#define SCI_LR_Y (SCI_UL_Y+(SCI_ROI_YSIZE/SCI_VBIN))
#define SCI_NFLUSHES 4
#define SCI_XORIGIN {334,852,1363,1849,2327}; //band cutout x centers (relative to the ROI)
#define SCI_YORIGIN {450,502,755,879,610};    //band cutout y centers (relative to the ROI)
#define SCI_SEARCH   400                      //px search diameter to find star in each band

/* Process File Descriptor */
int sci_shmfd;

/* FLI File Descriptor */
flidev_t dev;

/**************************************************************/
/* SCICTRLC                                                   */
/*  - Main process interrupt routine                          */
/**************************************************************/
void scictrlC(int sig){
  uint32 err = 0;
  /* Cancel Exposure */
  if((err = FLICancelExposure(dev))){
    fprintf(stderr, "SCI: Error FLICancelExposure: %s\n", strerror((int)-err));
  }else{
    if(SCI_DEBUG) printf("SCI: Exposure stopped\n");
  }
  /* Disable TEC */
  if((err = FLISetTemperature(dev, SCI_TEC_SETPOINT_MAX))){
    fprintf(stderr, "SCI: Error FLISetTemperature: %s\n", strerror((int)-err));
  }else{
    if(SCI_DEBUG) printf("SCI: FLI TEC Disabled\n");
  }
  /* Close FLI camera */
  if((err = FLIClose(dev))){
    fprintf(stderr, "SCI: Error FLIClose: %s\n", strerror((int)-err));
  }else{
    if(SCI_DEBUG) printf("SCI: FLI closed\n");
  }
  /* Close shared memory */
  close(sci_shmfd);

  if (MSG_CTRLC) printf("SCI: exiting\n");

  /* Exit */
  exit(sig);
}

/**************************************************************/
/* SCI_XY2INDEX                                               */
/*  - Convert image (x,y) to image buffer index               */
/**************************************************************/
uint64_t sci_xy2index(int x,int y){
  return y*SCI_ROI_XSIZE + x;
}

/***************************************************************/
/* SCI_SETORIGIN                                               */
/*  - Set band origins from current image                      */
/***************************************************************/
void sci_setorigin(scievent_t *sci,uint16_t *img_buffer){
  int i,j,k,x,y;
  int imax=0,jmax=0;
  uint16_t pmax=0,p=0;
  int64_t  index;
  int      xorigin[SCI_NBANDS]={0};
  int      yorigin[SCI_NBANDS]={0};
  /* Loop through band images, find max pixel */
  for(k=0;k<SCI_NBANDS;k++){
    xorigin[k] = sci->xorigin[k];
    yorigin[k] = sci->yorigin[k];
    pmax=0;
    for(i=0;i<SCI_SEARCH;i++){
      for(j=0;j<SCI_SEARCH;j++){
	x = sci->xorigin[k]-(SCI_SEARCH/2)+i;
	y = sci->yorigin[k]-(SCI_SEARCH/2)+j;
	x = x < 0 ? 0 : x;
	x = x >= SCI_ROI_XSIZE ? SCI_ROI_XSIZE-1 : x;
	y = y < 0 ? 0 : y;
	y = y >= SCI_ROI_YSIZE ? SCI_ROI_YSIZE-1 : y;
	index = sci_xy2index(x,y);
	index = index < 0 ? 0 : index;
	index = index > (SCI_ROI_XSIZE*SCI_ROI_YSIZE - 1) ? (SCI_ROI_XSIZE*SCI_ROI_YSIZE - 1) : index;
	p = img_buffer[index];
	if(p > pmax){
	  pmax = p;
	  imax = i;
	  jmax = j;
	}
      }
    }
    //Set new origin
    xorigin[k] += imax - (SCI_SEARCH/2);
    yorigin[k] += jmax - (SCI_SEARCH/2);
  }
  int check=1;
  for(k=0;k<SCI_NBANDS;k++){
    if(xorigin[k] < SCIXS || xorigin[k] >= SCI_ROI_XSIZE-SCIXS) check = 0; 
    if(yorigin[k] < SCIYS || yorigin[k] >= SCI_ROI_YSIZE-SCIYS) check = 0; 
  }
  if(check){
    for(k=0;k<SCI_NBANDS;k++){
      sci->xorigin[k] = xorigin[k];
      sci->yorigin[k] = yorigin[k];
    }
  }
}

/***************************************************************/
/* SCI_FINDORIGIN                                              */
/*  - Set band origins from current image                      */
/*  - Run a search over the full image                         */
/***************************************************************/
void sci_findorigin(scievent_t *sci,uint16_t *img_buffer){
  int i,j,x,y;
  uint8_t  mask[SCI_ROI_XSIZE][SCI_ROI_YSIZE]={{0}};
  uint16_t maxval;
  uint16_t thresh=10000;
  int      boxsize = 100;
  int      xorigin[SCI_NBANDS]={0};
  int      yorigin[SCI_NBANDS]={0};
  int      k=0,imin=0,xmin=0;
  
  //Init mask
  memset(&mask[0][0],0,sizeof(mask));
  
  //Loop over entire image
  for(i=0;i<SCI_ROI_XSIZE;i++){
    for(j=0;j<SCI_ROI_YSIZE;j++){
      if(!mask[i][j] && (img_buffer[sci_xy2index(i,j)] > thresh)){
	//Spot found, find maximum pixel over boxsize search region
	maxval = 0;
	for(x=i-boxsize/2;x<i+boxsize/2;x++){
	  for(y=j-boxsize/2;y<j+boxsize/2;y++){
	    if(x >= 0 && y >= 0 && x < SCI_ROI_XSIZE && y < SCI_ROI_YSIZE){
	      if(!mask[x][y] && (img_buffer[sci_xy2index(x,y)] > maxval)){
		maxval = img_buffer[sci_xy2index(x,y)];
		xorigin[k] = x;
		yorigin[k] = y;
	      }
	      //Mark pixel used
	      mask[x][y] = 1;
	    }
	  }
	}
	//Increment spot counter
	k++;
	if(k==SCI_NBANDS) goto sort_spots;
      }
    }
  }
  
 sort_spots:
  //Check that we found the correct number of spots
  if(k != SCI_NBANDS){
    printf("SCI: Error: Found %d spots\n",k);
    return;
  }

  //Sort spots by x coordinate
  for(k=0;k<SCI_NBANDS;k++){
    xmin = SCI_ROI_XSIZE;
    for(i=0;i<SCI_NBANDS;i++){
      if(xorigin[i] < xmin){
	imin = i;
	xmin = xorigin[imin];
      }
    }
    //Save Kth origin
    sci->xorigin[k] = xorigin[imin];
    sci->yorigin[k] = yorigin[imin];
    //Max out xorigin so we skip it next time
    xorigin[imin] = SCI_ROI_XSIZE;
  }
}

/**************************************************************/
/* SCI_LOADORIGIN                                             */
/*  - Load band origins from file                             */
/**************************************************************/
void sci_loadorigin(scievent_t *sci){
  FILE *fd=NULL;
  char filename[MAX_FILENAME];
  uint64 fsize,rsize;
  uint32 xorigin[SCI_NBANDS] = SCI_XORIGIN;
  uint32 yorigin[SCI_NBANDS] = SCI_YORIGIN;
  int k;
  
  /* Initialize with default origins */
  memcpy(sci->xorigin,xorigin,sizeof(xorigin));
  memcpy(sci->yorigin,yorigin,sizeof(yorigin));
  
  /* Open origin file */
  //--setup filename
  sprintf(filename,SCI_ORIGIN_FILE);
  //--open file
  if((fd = fopen(filename,"r")) == NULL){
    perror("SCI: loadorigin fopen");
    return;
  }
  //--check file size
  fseek(fd, 0L, SEEK_END);
  fsize = ftell(fd);
  rewind(fd);
  rsize = sizeof(xorigin) + sizeof(yorigin);
  if(fsize != rsize){
    printf("SCI: incorrect SCI_ORIGIN_FILE size %lu != %lu\n",fsize,rsize);
    fclose(fd);
    return;
  }
  
  //Read file
  if(fread(xorigin,sizeof(xorigin),1,fd) != 1){
    perror("SCI: loadorigin fread");
    fclose(fd);
    return;
  }
  if(fread(yorigin,sizeof(yorigin),1,fd) != 1){
    perror("SCI: loadorigin fread");
    fclose(fd);
    return;
  }
  
  //Close file
  fclose(fd);
  printf("SCI: Read: %s\n",filename);

  //Copy origins
  memcpy(sci->xorigin,xorigin,sizeof(xorigin));
  memcpy(sci->yorigin,yorigin,sizeof(yorigin));
}

/***************************************************************/
/* SCI_SAVEORIGIN                                              */
/*  - Saves cell origins to file                               */
/***************************************************************/
void sci_saveorigin(scievent_t *sci){
  struct stat st = {0};
  FILE *fd=NULL;
  static char outfile[MAX_FILENAME];
  char temp[MAX_FILENAME];
  char path[MAX_FILENAME];
  int k;

  /* Open output file */
  //--setup filename
  sprintf(outfile,"%s",SCI_ORIGIN_FILE);
  //--create output folder if it does not exist
  strcpy(temp,outfile);
  strcpy(path,dirname(temp));
  if (stat(path, &st) == -1){
    printf("SCI: creating folder %s\n",path);
    recursive_mkdir(path, 0777);
  }
  //--open file
  if((fd = fopen(outfile, "w")) == NULL){
    perror("SCI: saveorigin fopen()\n");
    return;
  }
  
  //Save origins
  if(fwrite(sci->xorigin,sizeof(sci->xorigin),1,fd) != 1){
    printf("SCI: saveorigin fwrite error!\n");
    fclose(fd);
    return;
  }
  if(fwrite(sci->yorigin,sizeof(sci->yorigin),1,fd) != 1){
    printf("SCI: saveorigin fwrite error!\n");
    fclose(fd);
    return;
  }
  printf("SCI: Wrote: %s\n",outfile);

  //Close file
  fclose(fd);
}

/**************************************************************/
/* SCI_REVERTORIGIN                                           */
/*  - Set band origins to default                             */
/**************************************************************/
void sci_revertorigin(scievent_t *sci){
  const uint32 xorigin[SCI_NBANDS] = SCI_XORIGIN;
  const uint32 yorigin[SCI_NBANDS] = SCI_YORIGIN;
  int k;
  
  /* Copy default origins */
  memcpy(sci->xorigin,xorigin,sizeof(xorigin));
  memcpy(sci->yorigin,yorigin,sizeof(yorigin));
}

/**************************************************************/
/* SCI_GET_TEMP                                               */
/*  - Get CCD Temperature                                     */
/**************************************************************/
double sci_get_temp(flidev_t dev){
  double temp;
  uint32 err;
  
  /* Get temperature */
  if((err = FLIGetTemperature(dev, &temp))){
    fprintf(stderr, "SCI: Error FLIGetTemperature: %s\n", strerror((int)-err));
    return 99;
  }else{
    if(SCI_DEBUG) printf("SCI: FLI temperature: %f\n",temp);
  }
  return temp;
}

/**************************************************************/
/* SCI_EXPOSE                                                 */
/*  - Run image exposure                                      */
/**************************************************************/
int sci_expose(sm_t *sm_p, flidev_t dev, uint16 *img_buffer){
  int row,i;
  uint32 err;
  long int timeleft;
  
  
  /* Start exposure */
  if((err = FLIExposeFrame(dev))){
    fprintf(stderr, "SCI: Error FLIExposeFrame: %s\n", strerror((int)-err));
    return 1;
  }else{
    if(SCI_DEBUG) printf("SCI: Exposure started\n");
  }

  /* Wait for exposure to finish */
  while(1){
    /* Check if we've been asked to exit */
    if(sm_p->w[SCIID].die)
      scictrlC(0);
    
    /* Check in with the watchdog */
    checkin(sm_p,SCIID);
    
    //Sleep 1 decisecond
    usleep(100000);
    
    //Get exposure status
    if((err = FLIGetExposureStatus(dev,&timeleft))){
      fprintf(stderr, "SCI: Error FLIGetExposureStatus: %s\n", strerror((int)-err));
      return 1;
    }
    if(timeleft == 0){
      if(SCI_DEBUG) printf("SCI: Exposure done after %d checks\n",i+1);
      break;
    }
  }
  
  /* Grab data one row at a time */
  for(row=0;row<SCI_ROI_YSIZE;row++)
    err = err | FLIGrabRow(dev, img_buffer+(row*SCI_ROI_XSIZE), SCI_ROI_XSIZE);

  /* Error checking */
  if(err){
    fprintf(stderr, "SCI: Error FLIGrabRow: %s\n", strerror((int)-err));
    return 1;
  }else{
    if(SCI_DEBUG) printf("SCI: FLI rows grabbed\n");
  }
  return 0;
}

/**************************************************************/
/* SCI_PROCESS_IMAGE                                          */
/*  - Process SCI camera image                                */
/**************************************************************/
void sci_process_image(uint16 *img_buffer, sm_t *sm_p){
  static scievent_t scievent;
  scievent_t* scievent_p;
  static struct timespec start,end,delta,last;
  static int init = 0;
  double dt;
  uint16 fakepx=0;
  uint32 i,j,k;
  static unsigned long frame_number=0;
  int print_origin=0;
  int state;
  
  //Get time immidiately
  clock_gettime(CLOCK_REALTIME,&start);
  
  //Get state
  state = sm_p->state;

  //Debugging
  if(SCI_DEBUG) printf("SCI: Got frame\n"); 
  
  //Check in with the watchdog 
  checkin(sm_p,SCIID);
  
  //Check reset 
  if(sm_p->sci_reset){
   init=0;
   sm_p->sci_reset=0;
  }

  //Initialize 
  if(!init){
    memset(&scievent,0,sizeof(scievent));
    memcpy(&last,&start,sizeof(struct timespec));
    sci_loadorigin(&scievent);
    frame_number=0;
    init=1;
    if(SCI_DEBUG) printf("SCI: Initialized\n");
  }

  //Measure exposure time 
  if(timespec_subtract(&delta,&start,&last))
    printf("SCI: call back --> timespec_subtract error!\n");
  ts2double(&delta,&dt);

  //Save time 
  memcpy(&last,&start,sizeof(struct timespec));
 
  //Command: sci_setorigin 
  if(sm_p->sci_setorigin){
    sci_setorigin(&scievent,img_buffer);
    sm_p->sci_setorigin=0;
    print_origin=1;
  }
  //Command: sci_findorigin 
  if(sm_p->sci_findorigin){
    sci_findorigin(&scievent,img_buffer);
    sm_p->sci_findorigin=0;
    print_origin=1;
  }
  //Command: sci_trackorigin 
  if(sm_p->sci_trackorigin){
    //--set origin every time, user must disable
    sci_setorigin(&scievent,img_buffer);
  }
  //Command: sci_saveorigin 
  if(sm_p->sci_saveorigin){
    sci_saveorigin(&scievent);
    sm_p->sci_saveorigin=0;
    print_origin=1;
  }
  //Command: sci_loadorigin 
  if(sm_p->sci_loadorigin){
    sci_loadorigin(&scievent);
    sm_p->sci_loadorigin=0;
    print_origin=1;
  }
  //Command: sci_revertorigin 
  if(sm_p->sci_revertorigin){
    sci_revertorigin(&scievent);
    sm_p->sci_revertorigin=0;
    print_origin=1;
  }
  //Print origin
  if(print_origin){
    printf("SCI: Origin:");
    for(k=0;k<SCI_NBANDS;k++){
      printf(" (%d,%d)",scievent.xorigin[k],scievent.yorigin[k]);
    }
    printf("\n");
  }

  //Fill out event header 
  scievent.hed.version       = PICC_PKT_VERSION;
  scievent.hed.type          = BUFFER_SCIEVENT;
  scievent.hed.frame_number  = frame_number++;
  scievent.hed.exptime       = sm_p->sci_exptime;
  scievent.hed.frmtime       = sm_p->sci_frmtime;
  scievent.hed.ontime        = dt;
  scievent.hed.state         = state;
  scievent.hed.alp_commander = sm_p->state_array[state].alp_commander;
  scievent.hed.hex_commander = sm_p->state_array[state].hex_commander;
  scievent.hed.bmc_commander = sm_p->state_array[state].bmc_commander;
  scievent.hed.start_sec     = start.tv_sec;
  scievent.hed.start_nsec    = start.tv_nsec;

  //Save Camera telemetry
  scievent.ccd_temp         = sm_p->sci_ccd_temp;
  scievent.backplane_temp   = sm_p->sci_backplane_temp;
  scievent.tec_power        = sm_p->sci_tec_power;
  scievent.tec_setpoint     = sm_p->sci_tec_setpoint;
  scievent.tec_enable       = sm_p->sci_tec_enable;
  
  //Fake data
  if(sm_p->w[SCIID].fakemode != FAKEMODE_NONE){
    if(sm_p->w[SCIID].fakemode == FAKEMODE_TEST_PATTERN)
      for(k=0;k<SCI_NBANDS;k++)
	for(i=0;i<SCIXS;i++)
	  for(j=0;j<SCIYS;j++)
	    scievent.image[k].data[i][j]=fakepx++;
  }
  else{
    //Cut out bands 
    for(k=0;k<SCI_NBANDS;k++)
      for(i=0;i<SCIXS;i++)
	for(j=0;j<SCIYS;j++)
    	  scievent.image[k].data[i][j] = img_buffer[sci_xy2index(scievent.xorigin[k]-(SCIXS/2)+i,scievent.yorigin[k]-(SCIYS/2)+j)];
  }

  //Write SCIEVENT to circular buffer 
  if(sm_p->write_circbuf[BUFFER_SCIEVENT]){
    //Open circular buffer 
    scievent_p=(scievent_t *)open_buffer(sm_p,BUFFER_SCIEVENT);
    
    //Copy scievent 
    memcpy(scievent_p,&scievent,sizeof(scievent_t));;
    
    //Get final timestamp 
    clock_gettime(CLOCK_REALTIME,&end);
    scievent_p->hed.end_sec = end.tv_sec;
    scievent_p->hed.end_nsec = end.tv_nsec;
    
    //Close buffer 
    close_buffer(sm_p,BUFFER_SCIEVENT);
  }
}

/**************************************************************/
/* SCI_PROC                                                   */
/*  - Main SCI camera process                                 */
/**************************************************************/
void sci_proc(void){
  char file[MAX_FILENAME], name[MAX_FILENAME];
  uint32_t err = 0;
  uint32_t count = 0;
  long domain = (FLIDOMAIN_USB | FLIDEVICE_CAMERA);
  uint16_t *img_buffer;
  int camera_running=0;
  long exptime;
  
  /* Open Shared Memory */
  sm_t *sm_p;
  if((sm_p = openshm(&sci_shmfd)) == NULL){
    printf("openshm fail: sci_proc\n");
    scictrlC(0);
  }

  /* Set soft interrupt handler */
  sigset(SIGINT, scictrlC);	/* usually ^C */

  /* Malloc image buffer */
  img_buffer = (uint16_t *)malloc(SCI_ROI_XSIZE*SCI_ROI_YSIZE*sizeof(uint16_t));
  
  /* Create Device List */
  if((err = FLICreateList(domain))){
    fprintf(stderr, "SCI: Error FLICreateList: %s\n", strerror((int)-err));
    scictrlC(0);
  }else{
    if(SCI_DEBUG) printf("SCI: FLI list created\n");
    
    
    /* Create Device List */
    if((err = FLIListFirst(&domain, file, MAX_FILENAME, name, MAX_FILENAME))){
      fprintf(stderr, "SCI: Error FLIListFirst: %s\n", strerror((int)-err));
      scictrlC(0);
    }else{
      if(SCI_DEBUG) printf("SCI: FLI first device found\n");
      FLIDeleteList();
    }
  }
  
  /* Open Device */
  if((err = FLIOpen(&dev, file, domain))){
    fprintf(stderr, "SCI: Error FLIOpen: %s\n", strerror((int)-err));
    scictrlC(0);
  }
    
  /* ----------------------- Enter Main Loop ----------------------- */
  while(1){

    /* Check in with the watchdog */
    checkin(sm_p,SCIID);
       
    /* Cancel Exposure, put camera in known state */
    if((err = FLICancelExposure(dev))){
      fprintf(stderr, "SCI: Error FLICancelExposure: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: Exposure stopped\n");
    }
    camera_running = 0;
      
    /* Set temperature */
    if(sm_p->sci_tec_enable){
      if((err = FLISetTemperature(dev, sm_p->sci_tec_setpoint))){
	fprintf(stderr, "SCI: Error FLISetTemperature: %s\n", strerror((int)-err));
      }else{
	if(SCI_DEBUG) printf("SCI: FLI temperature set to %d\n",sm_p->sci_tec_setpoint);
      }
    }
    
    /* Set exposure time */
    exptime = lround(sm_p->sci_exptime * 1000);
    if((err = FLISetExposureTime(dev, exptime))){
      fprintf(stderr, "SCI: Error FLISetExposureTime: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI exposure time set\n");
    }

    /* Set frame type */
    if((err = FLISetFrameType(dev, FLI_FRAME_TYPE_NORMAL))){
      fprintf(stderr, "SCI: Error FLISetFrameType: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI frame type set\n");
    }

    /* Set ROI */
    if((err = FLISetImageArea(dev, SCI_UL_X, SCI_UL_Y, SCI_LR_X, SCI_LR_Y))){
      fprintf(stderr, "SCI: Error FLISetImageArea: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI image area set\n");
    }
      
    /* Set horizontal binning */
    if((err = FLISetHBin(dev, SCI_HBIN))){
      fprintf(stderr, "SCI: Error FLISetHBin: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI HBin set\n");
    }

    /* Set vertical binning */
    if((err = FLISetVBin(dev, SCI_VBIN))){
      fprintf(stderr, "SCI: Error FLISetVBin: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI VBin set\n");
    }

    /* Set number of flushes */
    if((err = FLISetNFlushes(dev, SCI_NFLUSHES))){
      fprintf(stderr, "SCI: Error FLISetNFlushes: %s\n", strerror((int)-err));
    }else{
      if(SCI_DEBUG) printf("SCI: FLI NFlushes set\n");
    }
	
    /* ----------------------- Enter Exposure Loop ----------------------- */
    while(1){
      /* Check if we've been asked to exit */
      if(sm_p->w[SCIID].die)
	scictrlC(0);

      /* Check if we've been asked to reset the exposure */
      if(sm_p->sci_reset_camera){
	sm_p->sci_reset_camera = 0;
	break;
      }
      
      /* Check if camera should start */
      if(!camera_running){
	camera_running = 1;
	printf("SCI: Camera started\n");
      }
      
      /* Run Camera */
      if(camera_running){
	/* Run Exposure */
	if(sci_expose(sm_p,dev,img_buffer))
	  printf("SCI: Exposure failed\n");
	else{
	  /*Process Image*/
	  sci_process_image(img_buffer,sm_p);
	}
      }
      
      /* Get CCD Temperature */
      sm_p->sci_ccd_temp = sci_get_temp(dev);
	
      /* Sleep */
      sleep(sm_p->w[SCIID].per);
    }
  }

  /* Exit */
  scictrlC(0);
  return;
}
