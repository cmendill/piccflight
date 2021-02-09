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
#include <libbmc.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "fakemodes.h"
#include "bmc_functions.h"
#include "numeric.h"
#include "sci_functions.h"

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
  uint32 xorigin[SCI_NBANDS];
  uint32 yorigin[SCI_NBANDS];
  
  //Read files
  if(read_file(SCI_XORIGIN_FILE,xorigin,sizeof(xorigin))){
    printf("SCI: ERROR: X Origin read_file\n");
    return;
  }
  if(read_file(SCI_YORIGIN_FILE,yorigin,sizeof(yorigin))){
    printf("SCI: ERROR: Y Origin read_file\n");
    return;
  }
  
  //Copy origins
  memcpy(sci->xorigin,xorigin,sizeof(xorigin));
  memcpy(sci->yorigin,yorigin,sizeof(yorigin));
  
  return;
}

/***************************************************************/
/* SCI_SAVEORIGIN                                              */
/*  - Saves cell origins to file                               */
/***************************************************************/
void sci_saveorigin(scievent_t *sci){

  //Write files
  if(write_file(SCI_XORIGIN_FILE,sci->xorigin,sizeof(sci->xorigin)))
    printf("SCI: ERROR: X Origin write_file\n");
  else
    printf("SCI: Wrote file: %s\n",xfile);
  
  if(write_file(SCI_YORIGIN_FILE,sci->yorigin,sizeof(sci->yorigin)))
    printf("SCI: ERROR: Y Origin write_file\n");
  else
    printf("SCI: Wrote file: %s\n",yfile);

  return;
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
/* SCI_HOWFS_CONSTRUCT_FIELD                                  */
/*  - Construct field measurement from a series of probes     */
/**************************************************************/
void sci_howfs_construct_field(sci_howfs_t *frames,sci_field_t *field){
  //NOTE: *field is a pointer to a SCI_NBANDS array of fields
  static int init=0;
  static int xind[SCI_NPIX], yind[SCI_NPIX];
  static double rmatrix0[SCI_NPIX][SCI_NBANDS];
  static double imatrix0[SCI_NPIX][SCI_NBANDS];
  static double rmatrix1[SCI_NPIX][SCI_NBANDS];
  static double imatrix1[SCI_NPIX][SCI_NBANDS];
  int i,k,c;
  char scimask[SCIXS][SCIYS];

  //Initialize
  if(!init){
    //Read HOWFS matrix from file
    if(read_file(HOWFS_RMATRIX0_FILE,rmatrix0,sizeof(rmatrix0)))
      memset(rmatrix0,0,sizeof(rmatrix0));
    if(read_file(HOWFS_RMATRIX1_FILE,rmatrix1,sizeof(rmatrix1)))
      memset(rmatrix1,0,sizeof(rmatrix1));
    if(read_file(HOWFS_IMATRIX0_FILE,imatrix0,sizeof(imatrix0)))
      memset(imatrix0,0,sizeof(imatrix0));
    if(read_file(HOWFS_IMATRIX1_FILE,imatrix1,sizeof(imatrix1)))
      memset(imatrix1,0,sizeof(imatrix1));
    //Read SCI pixel selection
    if(read_file(SCI_MASK_FILE,&scimask[0][0],sizeof(scimask)))
      memset(rmatrix0,0,sizeof(rmatrix0));
    //Build index arrays
    c=0;
    for(i=0;i<SCIXS;i++){
      for(j=0;j<SCIYS;j++){
	if(scimask[i][j]){
	  xind[c]=i;
	  yind[c]=j;
	  c++
	}
      }
    }
    init=1;
  }
  
  //Construct field
  for(k=0;k<SCI_NBANDS;k++){
    for(i=0;i<SCI_NPIX;i++){
      field[k].r[i] = 0.25*(rmatrix0[i][k] * (frames->step[0].band[k].data[xind[i]][yind[i]] - frames->step[2].band[k].data[xind[i]][yind[i]]) +
			    rmatrix1[i][k] * (frames->step[1].band[k].data[xind[i]][yind[i]] - frames->step[3].band[k].data[xind[i]][yind[i]]));
      field[k].i[i] = 0.25*(imatrix0[i][k] * (frames->step[0].band[k].data[xind[i]][yind[i]] - frames->step[2].band[k].data[xind[i]][yind[i]]) +
			    imatrix1[i][k] * (frames->step[1].band[k].data[xind[i]][yind[i]] - frames->step[3].band[k].data[xind[i]][yind[i]]));
    }
  }
  
  return;
}

/**************************************************************/
/* SCI_HOWFS_EFC                                              */
/*  - Perform EFC matrix multiply                             */
/**************************************************************/
void sci_howfs_efc(sci_field_t *field, double *delta_length){
  //NOTE: *field is a pointer to a SCI_NBANDS array of fields
  static int init=0;
  static double matrix[2*SCI_NPIX*SCI_NBANDS*BMC_NACT]={0};
  int i;
  
  //Initialize
  if(!init){
    //Read EFC matrix from file
    if(read_file(EFC_MATRIX_FILE,matrix,sizeof(matrix)))
      memset(matrix,0,sizeof(matrix));
    init=1;
  }
  
  //Perform matrix multiply
  num_dgemv(matrix, (double *)field, delta_length, BMC_NACT, 2*SCI_NPIX*SCI_NBANDS);

  //Apply gain
  for(i=0;i<BMC_NACT;i++)
    delta_length[i] *= SCI_EFC_GAIN;
  return;
}

/**************************************************************/
/* SCI_PROCESS_IMAGE                                          */
/*  - Process SCI camera image                                */
/**************************************************************/
void sci_process_image(uint16 *img_buffer, sm_t *sm_p){
  static scievent_t scievent;
  static wfsevent_t wfsevent;
  scievent_t* scievent_p;
  static struct timespec start,end,delta,last;
  static int init = 0;
  static int howfs_init = 0;
  static int howfs_got_frame[SCI_HOWFS_NSTEP]={0};
  static sci_howfs_t howfs_frames;
  double dt;
  double delta_length[BMC_NACT]={0};
  uint16 fakepx=0;
  uint32 i,j,k;
  static unsigned long frame_number=0,howfs_istart=0;
  int ihowfs;
  int print_origin=0;
  int state;
  static bmc_t bmc, bmc_try, bmc_flat;
  int rc;
  time_t t;
  
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
    //Reset calibration routines
    bmc_calibrate(sm_p,0,NULL,NULL,SCIID,FUNCTION_RESET);
    howfs_init=0;
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
	    scievent.bands.band[k].data[i][j]=fakepx++;
  }
  else{
    //Cut out bands 
    for(k=0;k<SCI_NBANDS;k++)
      for(i=0;i<SCIXS;i++)
	for(j=0;j<SCIYS;j++)
    	  scievent.bands.band[k].data[i][j] = img_buffer[sci_xy2index(scievent.xorigin[k]-(SCIXS/2)+i,scievent.yorigin[k]-(SCIYS/2)+j)];
  }

  /*************************************************************/
  /********************  BMC DM Control Code  ******************/
  /*************************************************************/
  
  //Check if we will send a command
  if((sm_p->state_array[state].bmc_commander == SCIID) && sm_p->bmc_ready && sm_p->bmc_hv_on){
    
    //Get last BMC command
    if(bmc_get_command(sm_p,&bmc)){
      //Skip this image
      return;
    }
    memcpy(&bmc_try,&bmc,sizeof(bmc_t));

    //Run HOWFS
    if(sm_p->state_array[state].sci.run_howfs){
      if(!howfs_init){
	//Set starting frame number
	howfs_istart = frame_number;
	//Save current BMC command as starting flat
	memcpy(&bmc_flat,&bmc,sizeof(bmc_t));
	//Set init
	howfs_init = 1;
      }
      //Define HOWFS index
      ihowfs = (frame_number - howfs_istart) % SCI_HOWFS_NSTEP;

      //********** HOWFC Indexing ************
      //ihowfs:  0      1   2   3   4
      //dmcmd:   p0    p1  p2  p3  new_flat
      //image:   flat  p0  p1  p2  p3

      //Save frames
      memcpy(&howfs_frames.step[ihowfs],&scievent.bands,sizeof(sci_bands_t));
            
      //HOWFS Operations
      if(ihowfs == SCI_HOWFS_NSTEP-1){
	//Calculate field
	sci_howfs_construct_field(&howfs_frames,wfsevent.field);

	//Run EFC
	if(sm_p->state_array[state].sci.run_efc){
	  //Get DM acuator deltas from field
	  sci_howfs_efc(wfsevent.field,delta_length);
	  //Add deltas to current flat
	  bmc_add_length(bmc_flat.acmd,bmc_flat.acmd,delta_length);
	}
	
	//Write WFSEVENT to circular buffer 
	memcpy(&wfsevent.hed,&scievent.hed,sizeof(pkthed_t));
	wfsevent.hed.type = BUFFER_WFSEVENT;
	if(sm_p->write_circbuf[BUFFER_WFSEVENT])
	  write_to_buffer(sm_p,(void *)&wfsevent,BUFFER_WFSEVENT);
      }

      //Set BMC Probe: try = flat + probe
      bmc_add_probe(bmc_flat.acmd,bmc_try.acmd,ihowfs);
    }
    
    
    //Calibrate BMC
    if(scievent.hed.bmc_calmode != BMC_CALMODE_NONE)
      sm_p->bmc_calmode = bmc_calibrate(sm_p,scievent.hed.bmc_calmode,&bmc_try,&scievent.hed.bmc_calstep,SCIID,FUNCTION_NO_RESET);
    
    //Send command to BMC
    if(bmc_send_command(sm_p,&bmc_try,SCIID)){
      // - command failed
      // - do nothing for now
    }else{
      // - copy command to current position
      memcpy(&bmc,&bmc_try,sizeof(bmc_t));
    }
  }
  
  //Get BMC Status
  if((sm_p->state_array[state].bmc_commander == SCIID) && sm_p->bmc_ready){
    if(libbmc_get_status((libbmc_device_t *)&sm_p->libbmc_device))
      printf("SCI: Failed to get BMC status\n");
    else
      memcpy(&scievent.bmc_status,&sm_p->libbmc_device.status,sizeof(bmc_status_t));
  }
  
  //Write SCIEVENT to circular buffer 
  if(sm_p->write_circbuf[BUFFER_SCIEVENT])
    write_to_buffer(sm_p,(void *)&scievent,BUFFER_SCIEVENT);
}

