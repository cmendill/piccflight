#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <math.h>
#include <sys/io.h>
#include <phx_api.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "numeric.h"
#include "phx_config.h"
#include "fakemodes.h"
#include "alp_functions.h"
#include "hex_functions.h"
#include "bmc_functions.h"
#include "tgt_functions.h"

/**************************************************************/
/* LYT_XY2INDEX                                               */
/*  - Transform x,y to buffer index                           */
/**************************************************************/
uint64 lyt_xy2index(int x, int y){
  //NOTE: This is inverted from the equivalent SHK function because the SHK has the lens tube that inverts the image optically.
  //      Here we need to invert the image in code.
  return (uint64)(LYTREADXS-x-1) + ((uint64)(LYTREADYS-y-1))*LYTREADXS;
}

/**************************************************************/
/* LYT_INITREF                                                */
/*  - Initialize reference image structure                    */
/**************************************************************/
void lyt_initref(lytref_t *lytref){
  
  /****** READ DEFAULT REFERENCE IMAGE FILE ******/
  if(read_file(LYTPIX2ALPZER_REFIMG_FILE,&lytref->refdef[0][0],sizeof(lytref->refdef)))
    memset(&lytref->refdef[0][0],0,sizeof(lytref->refdef));
  //--copy to current reference image
  memcpy(&lytref->refimg[0][0],&lytref->refdef[0][0],sizeof(lytref->refimg));
  
  /****** READ MODEL REFERENCE IMAGE FILE ******/
  if(read_file(LYTPIX2ALPZER_REFMOD_FILE,&lytref->refmod[0][0],sizeof(lytref->refmod)))
    memset(&lytref->refmod[0][0],0,sizeof(lytref->refmod));
  
  /****** READ PIXEL MASK FILE ******/
  if(read_file(LYTPIX2ALPZER_PXMASK_FILE,&lytref->pxmask[0][0],sizeof(lytref->pxmask)))
    memset(&lytref->pxmask[0][0],0,sizeof(lytref->pxmask));

  return;
}

/**************************************************************/
/* LYT_LOADREF                                                */
/*  - Load reference image from file                          */
/**************************************************************/
void lyt_loadref(lytref_t *lytref){
  double refimg[LYTXS][LYTYS];

  //Read file
  if(read_file(LYT_REFIMG_FILE,&refimg[0][0],sizeof(refimg)))
    return;
  printf("LYT: Read: %s\n",LYT_REFIMG_FILE);

  //Copy image into reference structure
  memcpy(&lytref->refimg[0][0],&refimg[0][0],sizeof(refimg));
}

/***************************************************************/
/* LYT_SAVEREF                                                 */
/*  - Save reference image to file                             */
/***************************************************************/
void lyt_saveref(lytref_t *lytref){

  //Write file
  if(write_file(LYT_REFIMG_FILE,&lytref->refimg[0][0],sizeof(lytref->refimg)))
    return;
  printf("LYT: Wrote: %s\n",LYT_REFIMG_FILE);

  return;
}

/**************************************************************/
/* LYT_LOADDARK                                               */
/*  - Load dark image from file                               */
/**************************************************************/
void lyt_loaddark(lytdark_t *lytdark){
  lytdark_t readimg;

  //Read file
  if(read_file(LYT_DARKIMG_FILE,&readimg,sizeof(readimg)))
    return;
  printf("LYT: Read: %s\n",LYT_DARKIMG_FILE);

  //Copy image
  memcpy(lytdark,&readimg,sizeof(lytdark_t));
}

/***************************************************************/
/* LYT_SAVEDARK                                                */
/*  - Save dark image to file                                  */
/***************************************************************/
void lyt_savedark(lytdark_t *lytdark){
      
  //Write file
  if(write_file(LYT_DARKIMG_FILE,lytdark,sizeof(lytdark_t)))
    return;
  
  printf("LYT: Wrote: %s\n",LYT_DARKIMG_FILE);

  return;
}

/**************************************************************/
/* LYT_ALP_ZERNPID                                            */
/*  - Run PID controller on measured Zernikes for ALP         */
/**************************************************************/
void lyt_alp_zernpid(lytevent_t *lytevent, double *zernike_delta, int *zernike_switch, int cen_enable, int *cen_used, int reset){
  static int init = 0;
  static double zint[LOWFS_N_ZERNIKE] = {0};
  double zerr;
  int i;
  const double zdelta_max =  0.020;
  const double zdelta_min = -0.020;

  //Centroid settings
  static int usezern=0;
  const double minthresh = 2.0;  //Switch to zernikes when centroid crosses minthresh going in (both centroids must be < minthresh)
  const double maxthresh = 5.0;  //Switch to centroid when centroid crosses maxthresh going out (either centroid > max thresh)
  const double dCXdZ1 =  40;   //pixels/micron  
  const double dCYdZ0 =  40;   //pixels/micron
  const double cgain  =  -0.5; //centroid control gain
  //NOTE: X&Y are transposed relative to Z0 and Z1 because of image distortion that happens near the VVC.
  //We should import an influence matrix from the calibration data to account for this automatically instead of hard coding it. 
  
  //Initialize
  if(!init || reset){
    memset(zint,0,sizeof(zint));
    init=1;
    if(reset == FUNCTION_RESET_RETURN) return;
  }

  //Clear cen_used
  *cen_used = 0;

  //Check centroid
  if(!usezern && ((fabs(lytevent->xcentroid) < minthresh) && (fabs(lytevent->ycentroid) < minthresh)))
    usezern=1;
  if(usezern && ((fabs(lytevent->xcentroid) > maxthresh) || (fabs(lytevent->ycentroid) > maxthresh)))
    usezern=0;
  
  //Check centroid enable
  if(!cen_enable)
    usezern=1;
  
  if(usezern){
    //Run Zernike PID
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      if(zernike_switch[i]){
	//Calculate error
	zerr = lytevent->zernike_measured[i] - lytevent->zernike_target[i];
	//Calculate integral
	zint[i] += zerr;
	//Limit integral
	if(zint[i] > LYT_ALP_ZERN_INT_MAX) zint[i]=LYT_ALP_ZERN_INT_MAX;
	if(zint[i] < LYT_ALP_ZERN_INT_MIN) zint[i]=LYT_ALP_ZERN_INT_MIN;
	//Calculate command delta
	zernike_delta[i] = lytevent->gain_alp_zern[i][0] * zerr + lytevent->gain_alp_zern[i][1] * zint[i];
	//Limit delta
	if(zernike_delta[i] > zdelta_max) zernike_delta[i] = zdelta_max;
	if(zernike_delta[i] < zdelta_min) zernike_delta[i] = zdelta_min;
	//Set locked flag
	lytevent->locked = 1;
      }
    }
  }
  else{
    //Run centroid controller
    if(zernike_switch[0])
      zernike_delta[1] = cgain * lytevent->xcentroid/dCXdZ1;
    if(zernike_switch[1])
      zernike_delta[0] = cgain * lytevent->ycentroid/dCYdZ0;
    *cen_used = 1;
    //Reset integrator
    memset(zint,0,sizeof(zint));
  }
}

/**************************************************************/
/* LYT_ZERNIKE_FIT                                            */
/*  - Fit Zernikes to LYT pixels                              */
/*  - Measure image centroid                                  */
/*  - Return 0 on success, 1 on error                         */
/**************************************************************/
int lyt_zernike_fit(lyt_t *image, lytref_t *lytref, double *zernikes, double *xcentroid, double *ycentroid, int reset){
  static lytevent_t lytevent;
  static int init=0;
  static double lyt2zern_matrix[LYTXS*LYTYS*LOWFS_N_ZERNIKE]={0}; //zernike fitting matrix (max size)
  static int lyt_npix=0;
  double lyt_delta[LYTXS*LYTYS]={0}; //measured image - reference (max size)
  double img_total,ref_total,value,xnum,ynum,xnum_ref,ynum_ref;
  double xhist[LYTXS]={0},xhist_ref[LYTXS]={0};
  double yhist[LYTYS]={0},yhist_ref[LYTYS]={0};
  int    i,j,count;
  uint16 maxpix;

  /* Initialize Fitting Matrix */
  if(!init || reset){

    /****** COUNT NUMBER OF CONTROLED PIXELS ******/
    lyt_npix=0;
    for(i=0;i<LYTXS;i++)
      for(j=0;j<LYTYS;j++)
	if(lytref->pxmask[i][j])
	  lyt_npix++;


    /****** READ ZERNIKE MATRIX FILE ******/
    if(read_file(LYTPIX2ALPZER_FILE,lyt2zern_matrix,lyt_npix*LOWFS_N_ZERNIKE*sizeof(double))){
      printf("LYT: ERROR reading lyt2zern file\n");
      memset(lyt2zern_matrix,0,sizeof(lyt2zern_matrix));
    }
    else
      printf("LYT: Read: %s\n",LYTPIX2ALPZER_FILE);
    
    //Set init flag
    init=1;
    //Return if reset
    if(reset == FUNCTION_RESET_RETURN) return 0;
  }

  //Get image totals for normalization & histograms for centroid
  img_total=0;
  ref_total=0;
  maxpix=0;
  for(i=0;i<LYTXS;i++){
    for(j=0;j<LYTYS;j++){
      value = (double)image->data[i][j];
      if(image->data[i][j] > maxpix) maxpix = image->data[i][j];
      //Sum values inside pixel mask
      if(lytref->pxmask[i][j]){
	img_total += value;
	ref_total += lytref->refimg[i][j];
	//Build x,y histograms 
	xhist[i]     += value;
	yhist[j]     += value;
	xhist_ref[i] += lytref->refimg[i][j];
	yhist_ref[j] += lytref->refimg[i][j];
      }
    }
  }
  
  //Weight histograms
  xnum  = 0;
  ynum  = 0;
  xnum_ref  = 0;
  ynum_ref  = 0;
  for(i=0;i<LYTXS;i++){
    xnum     += (i - LYTXS/2) * xhist[i];
    xnum_ref += (i - LYTXS/2) * xhist_ref[i];
  }
  for(j=0;j<LYTYS;j++){
    ynum     += (j - LYTYS/2) * yhist[j];
    ynum_ref += (j - LYTYS/2) * yhist_ref[j];
  }

  //Calculate centroid relative to reference image
  if(img_total > 0){
    *xcentroid = (xnum/img_total) - (xnum_ref/ref_total);
    *ycentroid = (ynum/img_total) - (ynum_ref/ref_total);
  }else{
    *xcentroid = 0;
    *ycentroid = 0;
  }
  
  //Fit zernikes if above pixel threshold
  if(maxpix > LYT_PIXEL_THRESH){
    
    //Fill out pixel delta array -- populate using row majority to match IDL matrix
    if(img_total > 0 && ref_total > 0){
      count=0;
      for(j=0;j<LYTYS;j++)
	for(i=0;i<LYTXS;i++)
	  if(lytref->pxmask[i][j])
	    lyt_delta[count++]  = ((double)image->data[i][j])/img_total - lytref->refimg[i][j]/ref_total;
    }
    
    //Do matrix multiply
    num_dgemv(lyt2zern_matrix, lyt_delta, zernikes, LOWFS_N_ZERNIKE, lyt_npix);

    //Limit zernikes
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      zernikes[i] = zernikes[i] < LYT_ZERNIKE_MIN ? LYT_ZERNIKE_MIN : zernikes[i];
      zernikes[i] = zernikes[i] > LYT_ZERNIKE_MAX ? LYT_ZERNIKE_MAX : zernikes[i];
    }

  }
  else{
    //Zero out zernikes
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      zernikes[i] = 0;
    //Zero out centroids
    *xcentroid = 0;
    *ycentroid = 0;
    //Return error
    return 1;
  }
  return 0;
}

/**************************************************************/
/* LYT_PROCESS_IMAGE                                          */
/*  - Main image processing function for LYT                  */
/**************************************************************/
int lyt_process_image(stImageBuff *buffer,sm_t *sm_p){
  static lytevent_t lytevent;
  static lytpkt_t lytpkt;
  static calmode_t alpcalmodes[ALP_NCALMODES];
  static calmode_t hexcalmodes[HEX_NCALMODES];
  static calmode_t bmccalmodes[BMC_NCALMODES];
  static calmode_t tgtcalmodes[TGT_NCALMODES];
  static struct timespec start,end,delta,last,pkt_last;
  static uint32 frame_number=0, sample=0;
  static int init=0;
  static lytref_t lytref;
  static int pid_reset=FUNCTION_RESET;
  static lytread_t readimage;
  static lytdark_t darkimage;
  static int darkcount=0;
  static int pid_single_init=0;
  double dt;
  int i,j;
  uint16_t fakepx=0;
  int state;
  alp_t alp,alp_try,alp_delta={},alp_shk2lyt={};
  static alp_t alp_base={};
  int zernike_control=0;
  int zernike_switch[LOWFS_N_ZERNIKE] = {0};
  int cen_used=0;
  uint32_t n_dither=1;
  int zfit_error=0;
  double background=0;
  const long nbkg = LYTREADXS*LYTREADYS - LYTXS*LYTYS;
  uint16 *image = (uint16 *)buffer->pvAddress;

  //Image magnification
  double x,y,f_x_y1,f_x_y2;
  int x1,x2,y1,y2;
  
  //Get time immidiately
  clock_gettime(CLOCK_REALTIME,&start);

  //Get state
  state = sm_p->state;

  //Check reset
  if(sm_p->lyt_reset){
    init=0;
    sm_p->lyt_reset=0;
  }

  //Initialize
  if(!init){
    //Zero out events & commands
    memset(&lytevent,0,sizeof(lytevent_t));
    memset(&lytpkt,0,sizeof(lytpkt_t));
    memset(&darkimage,0,sizeof(lytdark_t));
    //Init frame number & sample
    frame_number=0;
    sample=0;
    //Reset zern2alp mapping
    alp_zern2alp(NULL,NULL,FUNCTION_RESET_RETURN);
    //Reset calibration routines
    alp_calibrate(sm_p,0,NULL,NULL,NULL,LYTID,FUNCTION_RESET_RETURN);
    tgt_calibrate(sm_p,0,NULL,NULL,LYTID,FUNCTION_RESET_RETURN);
    //Reset PID controllers
    lyt_alp_zernpid(NULL,NULL,NULL,0,NULL,FUNCTION_RESET_RETURN);
    //Init reference image structure
    lyt_initref(&lytref);
    //Load dark image
    lyt_loaddark(&darkimage);
    //Reset zernike fitter
    lyt_zernike_fit(NULL,&lytref,NULL,NULL,NULL,FUNCTION_RESET_RETURN);
    //Init ALP calmodes
    for(i=0;i<ALP_NCALMODES;i++)
      alp_init_calmode(i,&alpcalmodes[i]);
    //Init HEX calmodes
    for(i=0;i<HEX_NCALMODES;i++)
      hex_init_calmode(i,&hexcalmodes[i]);
    //Init BMC calmodes
    for(i=0;i<BMC_NCALMODES;i++)
      bmc_init_calmode(i,&bmccalmodes[i]);
    //Init TGT calmodes
    for(i=0;i<TGT_NCALMODES;i++)
      tgt_init_calmode(i,&tgtcalmodes[i]);
    //Reset last times
    memcpy(&last,&start,sizeof(struct timespec));
    memcpy(&pkt_last,&start,sizeof(struct timespec));
    //Set init flag
    init=1;
    //Debugging
    if(LYT_DEBUG) printf("LYT: Initialized\n");
  }

  //Measure exposure time
  if(timespec_subtract(&delta,&start,&last))
    printf("LYT: lyt_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);

  //Save time
  memcpy(&last,&start,sizeof(struct timespec));

  //Fill out event header
  lytevent.hed.version       = PICC_PKT_VERSION;
  lytevent.hed.type          = BUFFER_LYTEVENT;
  lytevent.hed.frame_number  = frame_number;
  lytevent.hed.exptime       = sm_p->lyt_exptime;
  lytevent.hed.frmtime       = sm_p->lyt_frmtime;
  lytevent.hed.ontime        = dt;
  lytevent.hed.state         = state;
  lytevent.hed.alp_commander = sm_p->state_array[state].alp_commander;
  lytevent.hed.hex_commander = sm_p->state_array[state].hex_commander;
  lytevent.hed.bmc_commander = sm_p->state_array[state].bmc_commander;
  lytevent.hed.start_sec     = start.tv_sec;
  lytevent.hed.start_nsec    = start.tv_nsec;
  lytevent.hed.hex_calmode   = sm_p->hex_calmode;
  lytevent.hed.alp_calmode   = sm_p->alp_calmode;
  lytevent.hed.bmc_calmode   = sm_p->bmc_calmode;
  lytevent.hed.tgt_calmode   = sm_p->tgt_calmode;

  //Increment frame number
  frame_number++;

  //Save temperature
  lytevent.ccd_temp          = sm_p->lyt_ccd_temp;

  //Save origin
  lytevent.xorigin           = sm_p->lyt_xorigin;
  lytevent.yorigin           = sm_p->lyt_yorigin;

  //Init locked flag
  lytevent.locked            = 0;
  
  //Save gains
  for(i=0;i<LOWFS_N_ZERNIKE;i++)
    for(j=0;j<LOWFS_N_PID;j++)
      lytevent.gain_alp_zern[i][j] = sm_p->lyt_gain_alp_zern[i][j] * sm_p->lyt_zernike_control[i];
  
  //Save zernike targets
  memcpy(lytevent.zernike_target,(double *)sm_p->lyt_zernike_target,sizeof(lytevent.zernike_target));

  //Run target calibration
  if((sm_p->state_array[state].alp_commander == LYTID))
    if(lytevent.hed.tgt_calmode != TGT_CALMODE_NONE)
      sm_p->tgt_calmode = tgt_calibrate(sm_p,lytevent.hed.tgt_calmode,lytevent.zernike_target,&lytevent.hed.tgt_calstep,LYTID,FUNCTION_NO_RESET);

  //Copy full readout image to event
  //memcpy(&readimage.data[0][0],buffer->pvAddress,sizeof(lytread_t));
  for(i=0;i<LYTREADXS;i++)
    for(j=0;j<LYTREADYS;j++)
      readimage.data[i][j]=image[lyt_xy2index(i,j)];
 
  //Init background
  lytevent.background = 0;
  
  //Fake data
  if(sm_p->w[LYTID].fakemode != FAKEMODE_NONE){
    if(sm_p->w[LYTID].fakemode == FAKEMODE_TEST_PATTERN)
      for(i=0;i<LYTXS;i++)
	for(j=0;j<LYTYS;j++)
	  lytevent.image.data[i][j]=fakepx++;
    if(sm_p->w[LYTID].fakemode == FAKEMODE_LYT_REFIMG)
      for(i=0;i<LYTXS;i++)
	for(j=0;j<LYTYS;j++)
	  lytevent.image.data[i][j]=lytref.refimg[i][j];
    if(sm_p->w[LYTID].fakemode == FAKEMODE_IMREG){
      memset(&lytevent.image,0,sizeof(lytevent.image));
      lytevent.image.data[CAM_IMREG_X][CAM_IMREG_Y] = 1;
    }
  }
  else{
    //Real data
    if(sm_p->lyt_mag_enable){
      //Run image magnification
      for(i=0;i<LYTXS;i++){
	for(j=0;j<LYTYS;j++){
	  //Define location of interpolated pixel
	  x = (i - LYTXS/2)/sm_p->lyt_mag + (LYTREADXS/2) + lytevent.xorigin + sm_p->lyt_mag_xoff - (LYTREADXS-LYTXS)/2;
	  y = (j - LYTYS/2)/sm_p->lyt_mag + (LYTREADYS/2) + lytevent.yorigin + sm_p->lyt_mag_yoff - (LYTREADYS-LYTYS)/2;
	  
	  //Pick 4 pixels for interpolation
	  x1 = (int)x;
	  x2 = x1 + 1;
	  y1 = (int)y;
	  y2 = y1 + 1;

	  //Calculate interpolated value
	  if(x1 >= 0 && x1 < LYTREADXS && x2 >= 0 && x2 < LYTREADXS && y1 >= 0 && y1 < LYTREADYS && y2 >= 0 && y2 < LYTREADYS){
	    f_x_y1  = (x2 - x) * readimage.data[x1][y1] / (x2 - x1) + (x - x1) * readimage.data[x2][y1] / (x2 - x1);
	    f_x_y2  = (x2 - x) * readimage.data[x1][y2] / (x2 - x1) + (x - x1) * readimage.data[x2][y2] / (x2 - x1);
	    lytevent.image.data[i][j] = (y2 - y) * f_x_y1 / (y2 - y1) + (y - y1) * f_x_y2 / (y2-y1);
	  }else{
	    //1 of 4 pixels is out of bounds --> use closest value
	    x = x < 0 ? 0 : x;
	    y = y < 0 ? 0 : y;
	    x = x >= LYTREADXS ? LYTREADXS-1 : x;
	    y = y >= LYTREADYS ? LYTREADYS-1 : y;
	    lytevent.image.data[i][j] = readimage.data[(int)x][(int)y];
	  }
	}
      }
    }
    else{
      //Cut out ROI & measure background
      for(i=0;i<LYTREADXS;i++){
	for(j=0;j<LYTREADYS;j++){
	  if((i >= lytevent.xorigin) && (i < lytevent.xorigin+LYTXS) && (j >= lytevent.yorigin) && (j < lytevent.yorigin+LYTYS)){
	    lytevent.image.data[i-lytevent.xorigin][j-lytevent.yorigin]=(double)readimage.data[i][j];
	    if(sm_p->lyt_subdark) lytevent.image.data[i-lytevent.xorigin][j-lytevent.yorigin] -= darkimage.data[i][j]; //dark subtraction
	  }
	  else{
	    background += readimage.data[i][j];
	  }
	}
      }
      //Take average
      lytevent.background = background / nbkg;
    }
  }
  
  //Command: lyt_setref 
  if(sm_p->lyt_setref){
    //Copy current image to reference image
    for(i=0;i<LYTXS;i++)
      for(j=0;j<LYTYS;j++)
	lytref.refimg[i][j] = lytevent.image.data[i][j];
    sm_p->lyt_setref=0;
  }
  //Command: lyt_defref 
  if(sm_p->lyt_defref){
    //Switch to default reference image
    memcpy(&lytref.refimg[0][0],&lytref.refdef[0][0],sizeof(lytref.refimg));
    sm_p->lyt_defref=0;
  }
  //Command: lyt_modref 
  if(sm_p->lyt_modref){
    //Switch to model reference image
    memcpy(&lytref.refimg[0][0],&lytref.refmod[0][0],sizeof(lytref.refimg));
    sm_p->lyt_modref=0;
  }
  //Command: lyt_saveref 
  if(sm_p->lyt_saveref){
    //Save current reference image to disk
    lyt_saveref(&lytref);
    sm_p->lyt_saveref=0;
  }
  //Command: lyt_loadref 
  if(sm_p->lyt_loadref){
    //Load saved reference image from disk
    lyt_loadref(&lytref);
    sm_p->lyt_loadref=0;
  }

  
  //Command: lyt_setdark 
  if(sm_p->lyt_setdark){
    //Clear current dark image
    if(darkcount==0) memset(&darkimage,0,sizeof(darkimage));
    //Add current full image to dark image average
    for(i=0;i<LYTREADXS;i++)
      for(j=0;j<LYTREADYS;j++)
	darkimage.data[i][j] += (double)readimage.data[i][j] / LYT_NDARK;
    if(++darkcount == LYT_NDARK){
      sm_p->lyt_setdark=0;
      darkcount=0;
      printf("LYT: %d frames averaged into dark image\n",LYT_NDARK);
    }
  }
  //Command: lyt_zerodark 
  if(sm_p->lyt_zerodark){
    //Clear current dark image
    memset(&darkimage,0,sizeof(darkimage));
    sm_p->lyt_zerodark=0;
    printf("LYT: dark image set to zero\n");
  }
  //Command: lyt_savedark 
  if(sm_p->lyt_savedark){
    //Save current dark image to disk
    lyt_savedark(&darkimage);
    sm_p->lyt_savedark=0;
  }
  //Command: lyt_loaddark 
  if(sm_p->lyt_loaddark){
    //Load saved dark image from disk
    lyt_loaddark(&darkimage);
    sm_p->lyt_loaddark=0;
  }
  

  //Fit Zernikes
  if(sm_p->state_array[state].lyt.fit_zernikes)
    zfit_error = lyt_zernike_fit(&lytevent.image,&lytref,lytevent.zernike_measured, &lytevent.xcentroid, &lytevent.ycentroid, FUNCTION_NO_RESET);
  

  /*************************************************************/
  /*******************  ALPAO DM Control Code  *****************/
  /*************************************************************/

  //Check if we will send a command
  if((sm_p->state_array[state].alp_commander == LYTID) && sm_p->alp_ready){

    //Get last ALP command
    if(alp_get_command(sm_p,&alp)){
      //Skip this image
      return 0;
    }
    memcpy(&alp_try,&alp,sizeof(alp_t));
    
    //Check if ALP is controlling any Zernikes
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      if(sm_p->lyt_zernike_control[i] && (sm_p->state_array[state].lyt.zernike_control[i] == ACTUATOR_ALP)){
	zernike_switch[i] = 1;
	zernike_control = 1;
      }
    }

    //Check if we have a valid measurement
    if(zfit_error){
      zernike_control=0;
      pid_reset=FUNCTION_RESET_RETURN;
    }
    
    //Run Zernike control
    if(zernike_control){
      //Run Zernike PID
      lyt_alp_zernpid(&lytevent, alp_delta.zcmd, zernike_switch, sm_p->lyt_cen_enable,&cen_used, pid_reset);
      pid_reset = FUNCTION_NO_RESET;
      
      //Zero out uncontrolled Zernikes
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(!zernike_switch[i])
	  alp_delta.zcmd[i] = 0;
      
      //Convert zernike deltas to actuator deltas
      alp_zern2alp(alp_delta.zcmd,alp_delta.acmd,FUNCTION_NO_RESET);
      
      //Set base command
      memcpy(&alp_base,&alp,sizeof(alp_t));

      //Update shk2lyt base with SHK command everytime a new one is ready
      if(sm_p->state_array[state].shk.shk2lyt)
	if(alp_get_shk2lyt(sm_p,&alp_shk2lyt) == 0)
	  memcpy(&alp_base,&alp_shk2lyt,sizeof(alp_t));
      
      //Apply Zernike commands
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	alp_try.zcmd[i] = alp_base.zcmd[i] + alp_delta.zcmd[i];
      
      //Apply actuator commands
      for(i=0;i<ALP_NACT;i++)
	alp_try.acmd[i] = alp_base.acmd[i] + alp_delta.acmd[i];
    }
    
    //Calibrate ALP
    if(lytevent.hed.alp_calmode != ALP_CALMODE_NONE)
      sm_p->alp_calmode = alp_calibrate(sm_p,lytevent.hed.alp_calmode,&alp_try,&lytevent.hed.alp_calstep,lytevent.zernike_calibrate,LYTID,FUNCTION_NO_RESET);
    
    //Send command to ALP
    if(alp_send_command(sm_p,&alp_try,LYTID,n_dither)){
      // - command failed
      // - do nothing for now
    }else{
      // - copy command to current position
      memcpy(&alp,&alp_try,sizeof(alp_t));
    }
  }
  
  //Copy ALP command to lytevent
  memcpy(&lytevent.alp,&alp,sizeof(alp_t));
  
  //Write LYTEVENT circular buffer
  if(sm_p->circbuf[BUFFER_LYTEVENT].write)
    write_to_buffer(sm_p,&lytevent,BUFFER_LYTEVENT);
    
  /*************************************************************/
  /**********************  LYT Packet Code  ********************/
  /*************************************************************/
  if(sm_p->circbuf[BUFFER_LYTPKT].write){
    //Samples, collected each time through
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      lytpkt.zernike_measured[i][sample] = lytevent.zernike_measured[i];
      lytpkt.alp_zcmd[i][sample] = lytevent.alp.zcmd[i];
    }
    lytpkt.xcentroid[sample] = lytevent.xcentroid;
    lytpkt.ycentroid[sample] = lytevent.ycentroid;

    //Init & set locked flag
    if(sample == 0) lytpkt.locked=1;
    lytpkt.locked &= lytevent.locked;
      
    //Increment sample counter
    sample++;
    
    //Get time since last packet write
    if(timespec_subtract(&delta,&start,&pkt_last))
      printf("LYT: lyt_process_image --> timespec_subtract error!\n");
    ts2double(&delta,&dt);
    
    //Last sample, fill out rest of packet and write to circular buffer
    if((sample == LYT_NSAMPLES) || (dt > LYT_LYTPKT_TIME)){
      //Header
      memcpy(&lytpkt.hed,&lytevent.hed,sizeof(pkthed_t));
      lytpkt.hed.type = BUFFER_LYTPKT;

      //Image
      memcpy(&lytpkt.image,&lytevent.image,sizeof(lyt_t));

      //Zernike gains and targets
      for(i=0;i<LOWFS_N_ZERNIKE;i++){
	lytpkt.zernike_target[i]         = lytevent.zernike_target[i]; 
	for(j=0;j<LOWFS_N_PID;j++){
	  lytpkt.gain_alp_zern[i][j]     = lytevent.gain_alp_zern[i][j];
	}
      }
      
      //Actuator commands
      for(i=0;i<ALP_NACT;i++){
	lytpkt.alp_acmd[i] = lytevent.alp.acmd[i];
      }
      
      //Set LYTPKT zernike control flags
      for(i=0;i<LOWFS_N_ZERNIKE;i++){
	lytpkt.zernike_control[i] = 0;
	if(sm_p->lyt_zernike_control[i] && sm_p->state_array[state].lyt.zernike_control[i] == ACTUATOR_ALP)
	  lytpkt.zernike_control[i] = ACTUATOR_ALP;
      }

      //CCD Temp
      lytpkt.ccd_temp = lytevent.ccd_temp;

      //Origins
      lytpkt.xorigin  = lytevent.xorigin;
      lytpkt.yorigin  = lytevent.yorigin;

      //Background
      lytpkt.background = lytevent.background;
      
      //Number of samples
      lytpkt.nsamples = sample;
      sample = 0;

      //Write LYTPKT to circular buffer
      write_to_buffer(sm_p,&lytpkt,BUFFER_LYTPKT);
          
      //Reset time
      memcpy(&pkt_last,&start,sizeof(struct timespec));
      
      //Reset sample counter
      sample = 0;
    }
  }

  return 0;
}
