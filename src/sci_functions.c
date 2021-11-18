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
/* SCI_FUNCTION_RESET                                         */
/*  - Resets all SCI functions                                */
/**************************************************************/
void sci_function_reset(sm_t *sm_p){
  sci_howfs_construct_field(sm_p,NULL,NULL,NULL,FUNCTION_RESET);
  sci_howfs_efc(sm_p,NULL, NULL, FUNCTION_RESET);
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
    printf("SCI: Wrote file: %s\n",SCI_XORIGIN_FILE);
  
  if(write_file(SCI_YORIGIN_FILE,sci->yorigin,sizeof(sci->yorigin)))
    printf("SCI: ERROR: Y Origin write_file\n");
  else
    printf("SCI: Wrote file: %s\n",SCI_YORIGIN_FILE);

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
      return 2;
    
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
void sci_howfs_construct_field(sm_t *sm_p,sci_howfs_t *frames,scievent_t *scievent,sci_field_t *field,int reset){
  //NOTE: *field is a pointer to a SCI_NBANDS array of fields
  static int init=0;
  static int xind[SCI_NPIX], yind[SCI_NPIX];
  static double rmatrix0[SCI_NPIX][SCI_NBANDS];
  static double imatrix0[SCI_NPIX][SCI_NBANDS];
  static double rmatrix1[SCI_NPIX][SCI_NBANDS];
  static double imatrix1[SCI_NPIX][SCI_NBANDS];
  uint16_t px1,px2,px3,px4;
  double dpx1,dpx2,dpx3,dpx4;
  int i,j,b,c;
  uint8_t scimask[SCIXS][SCIYS];
  const double sci_sim_max[SCI_NBANDS] = SCI_SIM_MAX;
  const double sci_scale_default[SCI_NBANDS] = SCI_SCALE_DEFAULT;
  double scale;
  
  //Initialize
  if(!init || reset){
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
      memset(&scimask[0][0],0,sizeof(scimask));
    printf("SCI: Read HOWFS matrix\n");
    //Build index arrays
    c=0;
    for(j=0;j<SCIYS;j++){
      for(i=0;i<SCIXS;i++){
	if(scimask[i][j]){
	  xind[c]=i;
	  yind[c]=j;
	  c++;
	}
      }
    }
    init=1;
    if(reset) return;
  }
  
  //Construct field
  //NOTE: frame index: 0=flat, 1=probe1, 2=probe2, 3=probe3, 4=probe4
  for(b=0;b<SCI_NBANDS;b++){
    //Calculate image normalization
    scale = sci_scale_default[b];
    if(scievent->refmax[b] > 0) scale = sci_sim_max[b] / (scievent->hed.exptime * scievent->refmax[b]);
    for(c=0;c<SCI_NPIX;c++){
      field[b].r[c] = 0;
      field[b].i[c] = 0;
      if(frames->step[0].band[b].data[xind[c]][yind[c]] > sm_p->efc_sci_thresh){
	px1 = frames->step[1].band[b].data[xind[c]][yind[c]];
	px2 = frames->step[2].band[b].data[xind[c]][yind[c]];
	px3 = frames->step[3].band[b].data[xind[c]][yind[c]];
	px4 = frames->step[4].band[b].data[xind[c]][yind[c]];

	dpx1 = scale * (double)px1;
	dpx2 = scale * (double)px2;
	dpx3 = scale * (double)px3;
	dpx4 = scale * (double)px4;
	
	//Debugging
	//if(c < 5) printf("SCI: %d | %8.2E %8.2E %8.2E %8.2E | %8.2E %8.2E %8.2E %8.2E\n",c,dpx1,dpx2,dpx3,dpx4,rmatrix0[c][b],rmatrix1[c][b],imatrix0[c][b],imatrix1[c][b]);

	//Calculate field
	field[b].r[c] = 0.25*((rmatrix0[c][b] * (dpx1 - dpx3)) + (rmatrix1[c][b] * (dpx2 - dpx4)));
	field[b].i[c] = 0.25*((imatrix0[c][b] * (dpx1 - dpx3)) + (imatrix1[c][b] * (dpx2 - dpx4)));

	//Saturation check
	if((px1 == SCI_SATURATION) || (px2 == SCI_SATURATION) || (px3 == SCI_SATURATION) || (px4 == SCI_SATURATION)){
	  field[b].r[c] = 0;
	  field[b].i[c] = 0;
	}
      }
    }
  }
  
  return;
}

/**************************************************************/
/* SCI_HOWFS_EFC                                              */
/*  - Perform EFC matrix multiply                             */
/**************************************************************/
void sci_howfs_efc(sm_t *sm_p, sci_field_t *field, double *delta_length, int reset){
  //NOTE: *field is a pointer to a SCI_NBANDS array of fields
  static int init=0;
  static double matrix[2*SCI_NPIX*SCI_NBANDS*BMC_NACTIVE]={0};
  static uint16 active2full[BMC_NACTIVE]={0};
  double field_pixels[2*SCI_NPIX];
  double dl[BMC_NACTIVE]={0};
  double maxdl,mindl,max,scale;
  int i;
  
  //Initialize
  if(!init || reset){
    //Read EFC matrix from file
    if(read_file(EFC_MATRIX_FILE,matrix,sizeof(matrix)))
      memset(matrix,0,sizeof(matrix));
    //Read BMC active2full mapping
    if(read_file(BMC_ACTIVE2FULL_FILE,active2full,sizeof(active2full)))
      memset(active2full,0,sizeof(active2full));
    printf("SCI: Read EFC matrix\n");
    init=1;
    if(reset) return;
  }

  //NOTE: We don't have a reference field here (fine if its zero)
  memcpy(&field_pixels[0],field->r,sizeof(field->r));
  memcpy(&field_pixels[SCI_NPIX],field->i,sizeof(field->i));
  
  //Perform matrix multiply
  num_dgemv(matrix, field_pixels, dl, BMC_NACTIVE, 2*SCI_NPIX*SCI_NBANDS);

  //Find max & min deltas
  maxdl = dl[0];
  mindl = dl[0];
  for(i=0;i<BMC_NACTIVE;i++){
    if(dl[i] > maxdl)
      maxdl = dl[i];
    if(dl[i] < mindl)
      mindl = dl[i];
  }

  //Rescale command if efc_bmc_max is set
  if(sm_p->efc_bmc_max > 0){
    max = fabs(maxdl);
    if(fabs(mindl) > max) max = fabs(mindl);
    if(max > sm_p->efc_bmc_max){
      scale = sm_p->efc_bmc_max / max;
      maxdl = dl[0]*scale;
      mindl = dl[0]*scale;
      for(i=0;i<BMC_NACTIVE;i++){
	dl[i] *= scale;
	if(dl[i] > maxdl)
	  maxdl = dl[i];
	if(dl[i] < mindl)
	  mindl = dl[i];
      }
    }
  }
  
  printf("SCI: DM MIN | MAX = %f | %f\n",mindl,maxdl);
  write_file("output/data/calibration/dm_delta.dat",dl,sizeof(dl));
  printf("Wrote: output/data/calibration/dm_delta.dat\n");
  
  //Apply gain and active2full mapping
  for(i=0;i<BMC_NACTIVE;i++)
    delta_length[active2full[i]] = dl[i] * SCI_EFC_GAIN;
  return;
}

/**************************************************************/
/* SCI_PROCESS_IMAGE                                          */
/*  - Process SCI camera image                                */
/**************************************************************/
void sci_process_image(uint16 *img_buffer, sm_t *sm_p){
  static scievent_t scievent;
  static wfsevent_t wfsevent;
  static struct timespec start,end,delta,last;
  static int init = 0;
  static int howfs_init = 0;
  static int howfs_got_frame[SCI_HOWFS_NSTEP]={0};
  static sci_howfs_t howfs_frames;
  const double sci_sim_max[SCI_NBANDS] = SCI_SIM_MAX;
  double dt;
  double delta_length[BMC_NACT]={0};
  uint16 fakepx=0;
  uint32 i,j,k,b;
  static unsigned long frame_number=0,howfs_istart=0,howfs_ilast=0,iefc=0;
  int ihowfs=0;
  int print_origin=0;
  int state;
  int bmc_calibrate_advance=1;
  int bmc_calibrate_delta=0;
  static bmc_t bmc, bmc_flat;
  int bmc_set_flat=BMC_NOSET_FLAT;
  bmc_t bmc_try;
  int rc;
  time_t t;
  uint8_t scimask[SCIXS][SCIYS];
  char filename[MAX_FILENAME];
  int bmc_calmode_temp;
  int bmc_calmode_change;
  
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
    //Reset BMC & SCI functions
    bmc_function_reset(sm_p);
    sci_function_reset(sm_p);
    howfs_init=0;
    //Read SCI pixel selection
    if(read_file(SCI_MASK_FILE,&scimask[0][0],sizeof(scimask)))
      memset(&scimask[0][0],0,sizeof(scimask));
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
  //Command: Shift x origin
  if(sm_p->sci_xshiftorigin){
    for(k=0;k<SCI_NBANDS;k++){
      scievent.xorigin[k] += sm_p->sci_xshiftorigin;
    }
    sm_p->sci_xshiftorigin = 0;
    print_origin=1;
  }
  //Command: Shift y origin
  if(sm_p->sci_yshiftorigin){
    for(k=0;k<SCI_NBANDS;k++){
      scievent.yorigin[k] += sm_p->sci_yshiftorigin;
    }
    sm_p->sci_yshiftorigin = 0;
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

  //Save calmodes
  bmc_calmode_temp          = sm_p->bmc_calmode;
  bmc_calmode_change        = 0;
  if(bmc_calmode_temp != scievent.hed.bmc_calmode) bmc_calmode_change=1;
  scievent.hed.bmc_calmode  = bmc_calmode_temp;
  
  //Fake data
  if(sm_p->w[SCIID].fakemode != FAKEMODE_NONE){
    if(sm_p->w[SCIID].fakemode == FAKEMODE_TEST_PATTERN){
      for(k=0;k<SCI_NBANDS;k++)
	for(i=0;i<SCIXS;i++)
	  for(j=0;j<SCIYS;j++)
	    scievent.bands.band[k].data[i][j]=fakepx++;
    }
    if(sm_p->w[SCIID].fakemode == FAKEMODE_SCI_MASK){
      //Cut out bands & apply mask
      for(k=0;k<SCI_NBANDS;k++)
	for(i=0;i<SCIXS;i++)
	  for(j=0;j<SCIYS;j++)
	    if(scimask[i][j])
	      scievent.bands.band[k].data[i][j] = img_buffer[sci_xy2index(scievent.xorigin[k]-(SCIXS/2)+i,scievent.yorigin[k]-(SCIYS/2)+j)];
	    else
	      scievent.bands.band[k].data[i][j] = 0;
    }
  }
  else{
    //Real data: cut out bands 
    for(b=0;b<SCI_NBANDS;b++)
      for(i=0;i<SCIXS;i++)
	for(j=0;j<SCIYS;j++)
    	  scievent.bands.band[b].data[i][j] = img_buffer[sci_xy2index(scievent.xorigin[b]-(SCIXS/2)+i,scievent.yorigin[b]-(SCIYS/2)+j)];
  }

  //Command: sci_setref 
  if(sm_p->sci_setref){
    for(b=0;b<SCI_NBANDS;b++){
      //Find max pixel for this band
      scievent.refmax[b] = -1;
      for(i=0;i<SCIXS;i++){
	for(j=0;j<SCIYS;j++){
	  if(scievent.bands.band[b].data[i][j] > scievent.refmax[b])
	    scievent.refmax[b] = scievent.bands.band[b].data[i][j];
	}
      }
      //Divide by exposure time
      scievent.refmax[b] /= scievent.hed.exptime; //ADU/second
    }
    printf("SCI: Reference image updated\n");
    sm_p->sci_setref=0;
  }

  /*************************************************************/
  /********************  BMC DM Control Code  ******************/
  /*************************************************************/
  
  //Get BMC command & flat for telemetry & commands 
  if((sm_p->state_array[state].bmc_commander == WATID) || (sm_p->state_array[state].bmc_commander == SCIID)){
    if(bmc_get_command(sm_p,&bmc)){
      //Skip this image
      return;
    }
    if(bmc_get_flat(sm_p,&bmc_flat,0)){
      //Skip this image
      return;
    }
  }

  //Check if we will send a command
  if((sm_p->state_array[state].bmc_commander == SCIID) && sm_p->bmc_ready && sm_p->bmc_hv_on){
    
    //Copy last BMC command
    memcpy(&bmc_try,&bmc,sizeof(bmc_t));

    //Init BMC flat flag
    bmc_set_flat = BMC_NOSET_FLAT;

    //Run HOWFS
    if(sm_p->state_array[state].sci.run_howfs){
      //Reinitialize if there has been a break in the data
      if(frame_number != howfs_ilast+1) howfs_init=0;

      //Reinitialize if bmc calmode has changed
      if(bmc_calmode_change) howfs_init=0;
      
      //Disable BMC calibrate advance by default when HOWFS is running
      bmc_calibrate_advance = 0;

      //Enable BMC calibrate delta -- during HOWFS we want to only add deltas, not reset to the flat
      bmc_calibrate_delta = 1;
      
      //Init HOWFS
      if(!howfs_init){
	//Set starting frame number
	howfs_istart = frame_number;
	//Initialize EFC counter
	iefc=0;
	//Set init
	howfs_init = 1;
      }
      
      //Define HOWFS index
      ihowfs = (frame_number - howfs_istart) % SCI_HOWFS_NSTEP;
      howfs_ilast = frame_number;
      scievent.ihowfs = ihowfs;
      
      //********** HOWFC Indexing ************
      //ihowfs:  0      1   2   3   4
      //image:   flat  p0  p1  p2  p3
      //pktcmd:  flat  p0  p1  p2  p3
      //dmcmd:   p0    p1  p2  p3  new_flat

      //Fake HOWFS Data
      if(sm_p->w[SCIID].fakemode == FAKEMODE_SCI_PROBE){
	sprintf(filename,SCI_FAKE_PROBE_FILE,ihowfs);
	for(k=0;k<SCI_NBANDS;k++)
	  read_file(filename,&scievent.bands.band[k].data[0][0],sizeof(sci_t));
      }
      
      //Save frames
      memcpy(&howfs_frames.step[ihowfs],&scievent.bands,sizeof(sci_bands_t));
      printf("SCI: iWFS = %d\n",ihowfs);
            
      //HOWFS Operations
      if(ihowfs == SCI_HOWFS_NSTEP-1){
	//Calculate field
	sci_howfs_construct_field(sm_p,&howfs_frames,&scievent,wfsevent.field,FUNCTION_NO_RESET);
	//Run EFC
	if(sm_p->state_array[state].sci.run_efc){
	  //Get DM acuator deltas from field
	  sci_howfs_efc(sm_p,wfsevent.field,delta_length,FUNCTION_NO_RESET);
	  //Add deltas to current flat (NOTE: Local copy will become global when command is sent)
	  bmc_add_length(bmc_flat.acmd,bmc_flat.acmd,delta_length,FUNCTION_NO_RESET);
	  //Set flat flag
	  bmc_set_flat=BMC_SET_FLAT;
	  printf("SCI: iEFC = %lu\n",iefc);
	  iefc++;
	}

	//Write WFSEVENT to circular buffer 
	memcpy(&wfsevent.hed,&scievent.hed,sizeof(pkthed_t));
	wfsevent.hed.type = BUFFER_WFSEVENT;
	if(sm_p->circbuf[BUFFER_WFSEVENT].write)
	  write_to_buffer(sm_p,&wfsevent,BUFFER_WFSEVENT);
	
	//Allow BMC calibration to advance
	bmc_calibrate_advance = 1;
      }

      //Set BMC Probe: try = flat + probe (NOTE: when ihowfs == 4, nothing is added to the flat)
      bmc_add_probe(bmc_flat.acmd,bmc_try.acmd,ihowfs,FUNCTION_NO_RESET);
    }

    
    //Calibrate BMC
    if(scievent.hed.bmc_calmode != BMC_CALMODE_NONE)
      sm_p->bmc_calmode = bmc_calibrate(sm_p,scievent.hed.bmc_calmode,&bmc_try,&scievent.hed.bmc_calstep,bmc_calibrate_advance,bmc_calibrate_delta,SCIID,FUNCTION_NO_RESET);
    
    //Send command to BMC
    if(bmc_send_command(sm_p,&bmc_try,SCIID,bmc_set_flat))
      printf("SCI: BMC_SEND_COMMAND failed\n");
    
  }
  
  //Copy BMC command to scievent
  //NOTE: We save the command from the last iteration so that the image will match the command
  memcpy(&scievent.bmc,&bmc,sizeof(bmc_t));

  //Get BMC Status
  if(sm_p->bmc_ready){
    if(libbmc_get_status((libbmc_device_t *)&sm_p->libbmc_device))
      printf("SCI: Failed to get BMC status\n");
    else
      memcpy((void *)&scievent.bmc_status,(void *)&sm_p->libbmc_device.status,sizeof(bmc_status_t));
  }
  
  //Write SCIEVENT to circular buffer 
  if(sm_p->circbuf[BUFFER_SCIEVENT].write)
    write_to_buffer(sm_p,&scievent,BUFFER_SCIEVENT);
}

