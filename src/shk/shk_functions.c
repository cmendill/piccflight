#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>


/* piccflight headers */
#include "../common/controller.h"
#include "../common/common_functions.h"
#include "../common/numeric.h"
#include "../xin/xin_functions.h"
#include "../phx/include/phx_api.h"
#include "../phx/config.h"

/**************************************************************/
/*                      SHK_INIT_CELLS                        */
/**************************************************************/
void shk_init_cells(shkcell_t *cells){
  float cell_size_px = SHK_LENSLET_PITCH_UM/SHK_PX_PITCH_UM;
  int i,j,c;
  
  //Zero out cells
  memset(cells,0,sizeof(shkcell_t));
  
  //Initialize cells
  c=0;
  for(i=0;i<SHK_XCELLS;i++){
    for(j=0;j<SHK_YCELLS;j++){
      cells[c].index = c;
      cells[c].origin[0] = i*cell_size_px + cell_size_px/2 + SHK_CELL_XOFF;
      cells[c].origin[1] = j*cell_size_px + cell_size_px/2 + SHK_CELL_YOFF;
      c++;
    }
  }
}

/**************************************************************/
/*                      SHK_CENTROID_CELL                     */
/**************************************************************/
void shk_centroid_cell(uint16 *image, shkcell_t *cell, sm_t *sm_p){
  double xnum=0, ynum=0, total=0;
  uint32 maxpix=0,maxval=0;
  int blx,bly,trx,try;
  int boxsize,boxsize_new;
  int x,y,px;
  double xhist[SHKXS]={0};
  double yhist[SHKYS]={0};
  double val;
  

  //Set boxsize & spot_captured flag
  boxsize     = SHK_MAX_BOXSIZE;
  boxsize_new = sm_p->shk_boxsize;
  if(cell->spot_found){
    if(cell->spot_captured){
      //If spot was captured, but is now outside the box, unset captured
      if((abs(cell->deviation[0]) > boxsize_new) && (abs(cell->deviation[1]) > boxsize_new)){
	cell->spot_captured=0;
      }
    }
    else{
      //If spot was not captured, check if it is within the capture region
      if((abs(cell->deviation[0]) < (boxsize_new-SHK_BOX_DEADBAND)) && (abs(cell->deviation[1]) < (boxsize_new-SHK_BOX_DEADBAND))){
	cell->spot_captured=1;
	boxsize=sm_p->shk_boxsize;
      }
    }
  }
  
  //Calculate corners of centroid box
  blx = floor(cell->origin[0] - boxsize);
  bly = floor(cell->origin[1] - boxsize);
  trx = floor(cell->origin[0] + boxsize);
  try = floor(cell->origin[1] + boxsize);

  //Impose limits
  blx = blx > SHKXS ? SHKXS : blx;
  bly = bly > SHKYS ? SHKYS : bly;
  blx = blx < 0 ? 0 : blx;
  bly = bly < 0 ? 0 : bly;
  trx = trx > SHKXS ? SHKXS : trx;
  try = try > SHKYS ? SHKYS : try;
  trx = trx < 0 ? 0 : trx;
  try = try < 0 ? 0 : try;

  //Save limits
  cell->blx = blx;
  cell->bly = bly;
  cell->trx = trx;
  cell->try = try;
  
  //Build x,y histograms
  for(x=blx;x<trx;x++){
    for(y=bly;y<try;y++){
      px = x + y*SHKYS;
      val = (double)image[px];
      xhist[x] += val;
      yhist[y] += val;
      if(image[px] > maxval){
	maxval=image[px];
	maxpix=px;
      }
    }
  }
  
  //Weight histograms
  for(x=blx;x<trx;x++){
    xnum  += ((double)x+0.5) * xhist[x];
  }
  for(y=bly;y<try;y++){
    ynum  += ((double)y+0.5) * yhist[y];
    total += yhist[y];
  }
  
  //Calculate centroid
  cell->centroid[0] = xnum/total;
  cell->centroid[1] = ynum/total;
  
  //Calculate deviation
  cell->deviation[0] = cell->origin[0] - cell->centroid[0]; 
  cell->deviation[1] = cell->origin[1] - cell->centroid[1];
 
  //set max pixel
  cell->maxpix = maxpix;
  cell->maxval = maxval;
  
  //check if spot is above threshold
  if(cell->spot_found && (maxval < SHK_SPOT_LOWER_THRESH))
    cell->spot_found=0;
  
  if(maxval > SHK_SPOT_UPPER_THRESH)
    cell->spot_found=1;
  
  //check spot captured flag
  if(!cell->spot_found)
    cell->spot_captured=0;
  
  //decide if spot is in the beam
  cell->beam_select=0;
  if(cell->spot_found) //could add more tests here
    cell->beam_select=1;
}

/**************************************************************/
/*                      SHK_CENTROID                          */
/**************************************************************/
void shk_centroid(uint16 *image, shkcell_t *cells, sm_t *sm_p){
  int i;
  for(i=0;i<SHK_NCELLS;i++)
    shk_centroid_cell(image,&cells[i],sm_p);
}

/**************************************************************/
/*                   SHK_ZERNIKE_MATRIX                       */
/**************************************************************/
void shk_zernike_matrix(shkcell_t *cells, double *matrix_inv){
  int i, beam_ncells=0;
  int beam_cell_index[SHK_NCELLS] = {0};
  double max_x = 0, max_y = 0, min_x = 0, min_y = 0;
  double temp_beam_center[2] = {0}, beam_center[2] = {0}, beam_radius[2]={0}, beam_radius_m[2]={0};
  double beam_radius_multiplier[2];
  // collect beam cell indices on to beam_cell_index[SHK_NCELLS]
  // beam_center = [mean(x), mean(y)] of beam cell origins
  // beam_radius = [(max(x)-min(x))/2, (max(y)-min(y))/2] of beam cell origins
  for(i=0; i<SHK_NCELLS; i++) {
    if (cells[i].beam_select) {
      beam_cell_index[beam_ncells] = i;
      temp_beam_center[0] += cells[i].origin[0]; // center
      temp_beam_center[1] += cells[i].origin[1]; // center
      max_x = (cells[i].origin[0] > max_x) ? cells[i].origin[0] : max_x; // hold on to max x
      max_y = (cells[i].origin[1] > max_y) ? cells[i].origin[1] : max_y; // hold on to max y
      min_x = (cells[i].origin[0] < min_x) ? cells[i].origin[0] : min_x; // hold on to min x
      min_y = (cells[i].origin[1] < min_y) ? cells[i].origin[1] : min_y; // hold on to min y
      beam_ncells++;
    }
  }

  beam_center[0] = temp_beam_center[0]/beam_ncells; // beam center x
  beam_center[1] = temp_beam_center[1]/beam_ncells; // beam center y
  beam_radius[0] = (max_x-min_x)/2.0; // beam radius x
  beam_radius[1] = (max_y-min_y)/2.0; // beam radius y
  printf("SHK: Rebuilding Zernike Matrix\n");
  printf("SHK: Beam center %f %f\n",beam_center[0],beam_center[1]);
  printf("SHK: Beam radius %f %f\n",beam_radius[0],beam_radius[1]);
  printf("SHK: number of beam cells %d\n", beam_ncells);

  //multiplying vector is in slopes
  beam_radius_multiplier[0] = 1/(beam_radius[0]*SHK_PX_PITCH_UM);
  beam_radius_multiplier[1] = 1/(beam_radius[1]*SHK_PX_PITCH_UM);

  printf("SHK: building the z matrix\n");
  double dz_dxdy[2*SHK_NCELLS*LOWFS_N_ZERNIKE] = {0};
  double x_1 = 0, x_2 = 0, x_3 = 0, x_4 = 0, x_5=0;
  double y_1 = 0, y_2 = 0, y_3 = 0, y_4 = 0, y_5=0;
  for(i=0; i<beam_ncells; i++) {
    x_1 = (cells[beam_cell_index[i]].origin[0] - beam_center[0])/beam_radius[0];
    x_2 = pow(x_1,2);
    x_3 = pow(x_1,3);
    x_4 = pow(x_1,4);
    x_5 = pow(x_1,5);
    y_1 = (cells[beam_cell_index[i]].origin[1] - beam_center[1])/beam_radius[1];
    y_2 = pow(y_1,2);
    y_3 = pow(y_1,3);
    y_4 = pow(y_1,4);
    y_5 = pow(y_1,5);
    //NOTE: This is hard coded for LOWFS_N_ZERNIKE == 24
    dz_dxdy[i                 ] =      1.0 * (0)                                                                    *beam_radius_multiplier[0];
    dz_dxdy[i+( 2*beam_ncells)] =      2.0 * (1)                                                                    *beam_radius_multiplier[0];
    dz_dxdy[i+( 4*beam_ncells)] =      2.0 * (0)                                                                    *beam_radius_multiplier[0];
    dz_dxdy[i+( 6*beam_ncells)] =  sqrt(3) * (4*x_1)                                                                *beam_radius_multiplier[0];
    dz_dxdy[i+( 8*beam_ncells)] =  sqrt(6) * (2*y_1)                                                                *beam_radius_multiplier[0];
    dz_dxdy[i+(10*beam_ncells)] =  sqrt(6) * (2*x_1)                                                                *beam_radius_multiplier[0];
    dz_dxdy[i+(12*beam_ncells)] =  sqrt(8) * (9*x_2 + 3*y_2 - 2)                                                    *beam_radius_multiplier[0];
    dz_dxdy[i+(14*beam_ncells)] =  sqrt(8) * (6*x_1*y_1)                                                            *beam_radius_multiplier[0];
    dz_dxdy[i+(16*beam_ncells)] =  sqrt(8) * (3*x_2 - 3*y_2)                                                        *beam_radius_multiplier[0];
    dz_dxdy[i+(18*beam_ncells)] =  sqrt(8) * (6*x_1*y_1)                                                            *beam_radius_multiplier[0];
    dz_dxdy[i+(20*beam_ncells)] = sqrt( 5) * (24*x_3 - 12*x_1 + 24*x_1*y_2)                                         *beam_radius_multiplier[0];
    dz_dxdy[i+(22*beam_ncells)] = sqrt(10) * (16*x_3 - 6*x_1)                                                       *beam_radius_multiplier[0];
    dz_dxdy[i+(24*beam_ncells)] = sqrt(10) * (24*x_2*y_1 - 6*y_1 + 8*y_3)                                           *beam_radius_multiplier[0];
    dz_dxdy[i+(26*beam_ncells)] = sqrt(10) * (4*x_3 - 12*x_1*y_2)                                                   *beam_radius_multiplier[0];
    dz_dxdy[i+(28*beam_ncells)] = sqrt(10) * (12*x_2*y_1 - 4*y_3)                                                   *beam_radius_multiplier[0];
    dz_dxdy[i+(30*beam_ncells)] = sqrt(12) * (50*x_4 - 36*x_2 + 60*x_2*y_2 - 12*y_2 + 10*y_4 + 3)                   *beam_radius_multiplier[0];
    dz_dxdy[i+(32*beam_ncells)] = sqrt(12) * (40*x_3*y_1 - 24*x_1*y_1 + 40*x_1*y_3)                                 *beam_radius_multiplier[0];
    dz_dxdy[i+(34*beam_ncells)] = sqrt(12) * (25*x_4 - 12*x_2 - 30*x_2*y_2 + 12*y_2 - 15*y_4)                       *beam_radius_multiplier[0];
    dz_dxdy[i+(36*beam_ncells)] = sqrt(12) * (60*x_3*y_1 - 24*x_1*y_1 + 20*x_1*y_3)                                 *beam_radius_multiplier[0];
    dz_dxdy[i+(38*beam_ncells)] = sqrt(12) * (5*x_4 - 30*x_2*y_2 + 5*y_4)                                           *beam_radius_multiplier[0];
    dz_dxdy[i+(40*beam_ncells)] = sqrt(12) * (20*x_3*y_1 - 20*x_1*y_3)                                              *beam_radius_multiplier[0];
    dz_dxdy[i+(42*beam_ncells)] = sqrt( 7) * (120*x_5 - 120*x_3 + 240*x_3*y_2 + 24*x_1 - 120*x_1*y_2 + 120*x_1*y_4) *beam_radius_multiplier[0];
    dz_dxdy[i+(44*beam_ncells)] = sqrt(14) * (90*x_5 + 60*x_3*y_2 - 80*x_3 + 12*x_1 - 30*x_1*y_4)                   *beam_radius_multiplier[0];
    dz_dxdy[i+(46*beam_ncells)] = sqrt(14) * (150*x_4*y_1 + 180*x_2*y_3 - 120*x_2*y_1 + 12*y_1 - 40*y_3 + 30*y_5)   *beam_radius_multiplier[0];

    dz_dxdy[i+(   beam_ncells)] =      1.0 * (0)                                                                    *beam_radius_multiplier[1];
    dz_dxdy[i+( 3*beam_ncells)] =      2.0 * (0)                                                                    *beam_radius_multiplier[1];
    dz_dxdy[i+( 5*beam_ncells)] =      2.0 * (1)                                                                    *beam_radius_multiplier[1];
    dz_dxdy[i+( 7*beam_ncells)] =  sqrt(3) * (4*y_1)                                                                *beam_radius_multiplier[1];
    dz_dxdy[i+( 9*beam_ncells)] =  sqrt(6) * (2*x_1)                                                                *beam_radius_multiplier[1];
    dz_dxdy[i+(11*beam_ncells)] =  sqrt(6) * (-2*y_1)                                                               *beam_radius_multiplier[1];
    dz_dxdy[i+(13*beam_ncells)] =  sqrt(8) * (6*x_1*y_1)                                                            *beam_radius_multiplier[1];
    dz_dxdy[i+(15*beam_ncells)] =  sqrt(8) * (3*x_2 + 9*y_2 - 2)                                                    *beam_radius_multiplier[1];
    dz_dxdy[i+(17*beam_ncells)] =  sqrt(8) * (-6*x_1*y_1)                                                           *beam_radius_multiplier[1];
    dz_dxdy[i+(19*beam_ncells)] =  sqrt(8) * (3*x_2 - 3*y_2)                                                        *beam_radius_multiplier[1];
    dz_dxdy[i+(21*beam_ncells)] = sqrt( 5) * (24*x_2*y_1 - 12*y_1 + 24*y_3)                                         *beam_radius_multiplier[1];
    dz_dxdy[i+(23*beam_ncells)] = sqrt(10) * (6*y_1 - 16*y_3)                                                       *beam_radius_multiplier[1];
    dz_dxdy[i+(25*beam_ncells)] = sqrt(10) * (8*x_3 - 6*x_1 + 24*x_1*y_2)                                           *beam_radius_multiplier[1];
    dz_dxdy[i+(27*beam_ncells)] = sqrt(10) * (-12*x_2*y_1 + 4*y_3)                                                  *beam_radius_multiplier[1];
    dz_dxdy[i+(29*beam_ncells)] = sqrt(10) * (4*x_3 - 12*x_1*y_2)                                                   *beam_radius_multiplier[1];
    dz_dxdy[i+(31*beam_ncells)] = sqrt(12) * (40*x_3*y_1 -24*x_1*y_1 + 40*x_1*y_3)                                  *beam_radius_multiplier[1];
    dz_dxdy[i+(33*beam_ncells)] = sqrt(12) * (10*x_4 - 12*x_2 + 60*x_2*y_2 - 36*y_2 + 50*y_4 + 3)                   *beam_radius_multiplier[1];
    dz_dxdy[i+(35*beam_ncells)] = sqrt(12) * (-20*x_3*y_1 + 24*x_1*y_1 - 60*x_1*y_3)                                *beam_radius_multiplier[1];
    dz_dxdy[i+(37*beam_ncells)] = sqrt(12) * (15*x_4 - 12*x_2 + 30*x_2*y_2 + 12*y_2 - 25*y_4)                       *beam_radius_multiplier[1];
    dz_dxdy[i+(39*beam_ncells)] = sqrt(12) * (-20*x_3*y_1 + 20*x_1*y_3)                                             *beam_radius_multiplier[1];
    dz_dxdy[i+(41*beam_ncells)] = sqrt(12) * (5*x_4 - 30*x_2*y_2 + 5*y_4)                                           *beam_radius_multiplier[1];
    dz_dxdy[i+(43*beam_ncells)] = sqrt( 7) * (120*x_4*y_1 - 120*x_2*y_1 + 240*x_2*y_3 + 24*y_1 - 120*y_3 + 120*y_5) *beam_radius_multiplier[1];
    dz_dxdy[i+(45*beam_ncells)] = sqrt(14) * (30*x_4*y_1 - 60*x_2*y_3 - 12*y_1 + 80*y_3 - 90*y_5)                   *beam_radius_multiplier[1];
    dz_dxdy[i+(47*beam_ncells)] = sqrt(14) * (30*x_5 - 40*x_3 + 180*x_3*y_2 + 12*x_1 - 120*x_1*y_2 + 150*x_1*y_4)   *beam_radius_multiplier[1];
  }
  printf("SHK: inverting the z matrix\n");
  num_dgesvdi(dz_dxdy, matrix_inv, 2*beam_ncells, LOWFS_N_ZERNIKE);
}

/**************************************************************/
/*                      SHK_ZERNIKE_FIT                       */
/**************************************************************/
void shk_zernike_fit(shkevent_t *shkevent){
  int i, beam_ncells=0, beam_ncells_2=0, regen_matrix=0;
  int beam_cell_index[SHK_NCELLS] = {0};
  static int beam_cell_used[SHK_NCELLS] = {0};
  const double px2slope = SHK_PX_PITCH_UM/SHK_FOCAL_LENGTH_UM;
  static double matrix_inv[SHK_NCELLS*LOWFS_N_ZERNIKE*2] = {0};
  static double dphi_dxdy[2*SHK_NCELLS] = {0};

  for(i=0; i<SHK_NCELLS; i++) {
    if (shkevent->cells[i].beam_select) {
      //save the spot index
      beam_cell_index[beam_ncells] = i;
      //check if we should regenerate the matrix
      if(!beam_cell_used[i]) regen_matrix=1;
      beam_cell_used[i]=1;
      beam_ncells++;
    }
  }
  beam_ncells_2 = 2*beam_ncells;

  //generate the zernike matrix
  if(regen_matrix) {
    shk_zernike_matrix(shkevent->cells, matrix_inv);
    printf("SHK: matrix inverted\n");
  }

  for(i=0; i<beam_ncells; i++) {
    // multiplying vector is in slopes
    dphi_dxdy[i]             = shkevent->cells[beam_cell_index[i]].deviation[0]*px2slope;
    dphi_dxdy[i+beam_ncells] = shkevent->cells[beam_cell_index[i]].deviation[1]*px2slope;
  }
  num_dgemv(matrix_inv, dphi_dxdy, shkevent->zernikes, beam_ncells_2, LOWFS_N_ZERNIKE);
}

/**************************************************************/
/*                      SHK_PROCESS_IMAGE                     */
/**************************************************************/
void shk_process_image(stImageBuff *buffer,sm_t *sm_p, uint32 frame_number){
  static shkfull_t shkfull;
  static shkevent_t shkevent;
  shkfull_t *shkfull_p;
  shkevent_t *shkevent_p;
  static struct timespec first,start,end,delta,last;
  static int init=0;
  double dt;
  int32 i,j;
  uint16 fakepx=0;

  //Get time immidiately
  clock_gettime(CLOCK_REALTIME,&start);
  
  //Initialize
  if(!init){
    memset(&shkfull,0,sizeof(shkfull));
    memset(&shkevent,0,sizeof(shkevent));
    shk_init_cells(shkevent.cells);
    iwc_init(&shkevent.iwc);
    memcpy(&first,&start,sizeof(struct timespec));
    init=1;
  }

  //Measure exposure time
  if(timespec_subtract(&delta,&start,&last))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
  
  //Fill out event header
  shkevent.frame_number = frame_number;
  shkevent.exptime = 0;
  shkevent.ontime = dt;
  shkevent.temp = 0;
  shkevent.imxsize = SHKXS;
  shkevent.imysize = SHKYS;
  shkevent.mode = 0;
  shkevent.start_sec = start.tv_sec;
  shkevent.start_nsec = start.tv_nsec;

  //Calculate centroids
  shk_centroid(buffer->pvAddress,shkevent.cells,sm_p);

  //Fit Zernikes
  if(sm_p->shk_fit_zernike) shk_zernike_fit(&shkevent);

  //Calibrate IWC
  iwc_calibrate(sm_p->iwc_calmode,&shkevent.iwc);

  //Get new IWC position
  if(sm_p->iwc_calmode == 0){
    //Do something
  }

  //Save update to shared memory
  memcpy((void *)&sm_p->iwc,(void *)&shkevent.iwc,sizeof(iwc_t));
  
  //Apply update
  if(xin_write(sm_p->xin_dev,(iwc_t *)&sm_p->iwc,(dm_t *)&sm_p->dm,(pez_t *)&sm_p->pez)){
    printf("SHK: xin_write failed!\n");
    //Do something??
  }
  
  //Open circular buffer
  shkevent_p=(shkevent_t *)open_buffer(sm_p,SHKEVENT);
  
  //Copy data
  memcpy(shkevent_p,&shkevent,sizeof(shkevent_t));;

  //Get final timestamp
  clock_gettime(CLOCK_REALTIME,&end);
  shkevent_p->end_sec = end.tv_sec;
  shkevent_p->end_nsec = end.tv_nsec;

  //Close buffer
  close_buffer(sm_p,SHKEVENT);

  //Save time
  memcpy(&last,&start,sizeof(struct timespec));


  
  /*******************  Full Image Code  *****************/
  if(timespec_subtract(&delta,&start,&first))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
  if(dt > SHK_FULL_IMAGE_TIME){
    //Fill out full image header
    shkfull.packet_type  = SHKFULL;
    shkfull.frame_number = shkevent.frame_number;
    shkfull.exptime = shkevent.exptime;
    shkfull.ontime = shkevent.ontime;
    shkfull.temp = shkevent.temp;
    shkfull.imxsize = SHKXS;
    shkfull.imysize = SHKYS;
    shkfull.mode = shkevent.mode;
    shkfull.start_sec = shkevent.start_sec;
    shkfull.start_nsec = shkevent.start_nsec;

    //Fake data
    if(sm_p->shk_fake_mode > 0){
      if(sm_p->shk_fake_mode == 1)
	for(i=0;i<SHKXS;i++)
	  for(j=0;j<SHKYS;j++)
	    shkfull.image.data[i][j]=fakepx++;
      
      if(sm_p->shk_fake_mode == 2)
	for(i=0;i<SHKXS;i++)
	  for(j=0;j<SHKYS;j++)
	    shkfull.image.data[i][j]=2*fakepx++;

      if(sm_p->shk_fake_mode == 3)
	for(i=0;i<SHKXS;i++)
	  for(j=0;j<SHKYS;j++)
	    shkfull.image.data[i][j]=3*fakepx++;
    }
    else{
      //Copy full image -- takes about 700 us
      memcpy(&(shkfull.image.data[0][0]),buffer->pvAddress,sizeof(shkfull.image.data));
    }  
    
    //Copy event
    shkevent.end_sec = end.tv_sec;
    shkevent.end_nsec = end.tv_nsec;
    memcpy(&shkfull.shkevent,&shkevent,sizeof(shkevent));
 
    //Open circular buffer
    shkfull_p=(shkfull_t *)open_buffer(sm_p,SHKFULL);

    //Copy data
    memcpy(shkfull_p,&shkfull,sizeof(shkfull_t));;
    
    //Get final timestamp
    clock_gettime(CLOCK_REALTIME,&end);
    shkfull_p->end_sec = end.tv_sec;
    shkfull_p->end_nsec = end.tv_nsec;

    //Close buffer
    close_buffer(sm_p,SHKFULL);
    
    //Reset time
    memcpy(&first,&start,sizeof(struct timespec));
  }
}
