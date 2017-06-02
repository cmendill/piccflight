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
  
  //if(maxval > SHK_SPOT_UPPER_THRESH)
  cell->spot_found=1;
  
  //check spot captured flag
  if(!cell->spot_found)
    cell->spot_captured=0;
  
  //decide if spot is in the beam
  cell->beam_select=0;
  //if(cell->spot_found) //could add more tests here
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
/*                      SHK_PROCESS_IMAGE                     */
/**************************************************************/
void shk_process_image(stImageBuff *buffer,sm_t *sm_p, uint32 frame_number){
  static shkfull_t shkfull;
  static shkevent_t shkevent;
  shkfull_t *shkfull_p;
  shkevent_t *shkevent_p;
  static struct timespec first,start,end,delta;
  static int init=0;
  double dt;
  int32 i,j;
  uint16 fakepx=0;

  //Get time immidiately
  clock_gettime(CLOCK_REALTIME,&start);
  if(frame_number == 0)
    memcpy(&first,&start,sizeof(struct timespec));

  //Initialize
  if(!init){
    memset(&shkfull,0,sizeof(shkfull));
    memset(&shkevent,0,sizeof(shkevent));
    shk_init_cells(shkevent.cells);
    init=1;
  }
  
  //Fill out event header
  shkevent.frame_number = frame_number;
  shkevent.exptime = 0;
  shkevent.ontime = 0;
  shkevent.temp = 0;
  shkevent.imxsize = SHKXS;
  shkevent.imysize = SHKYS;
  shkevent.mode = 0;
  shkevent.start_sec = start.tv_sec;
  shkevent.start_nsec = start.tv_nsec;

  //Calculate centroids
  shk_centroid(buffer->pvAddress,shkevent.cells,sm_p);
  
  //Calculate update

  //Apply update
  
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

  
  /*******************************************************/
  /*                   Full image code                   */
  /*******************************************************/
  
  if(timespec_subtract(&delta,&start,&first))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
  if(dt > SHK_FULL_IMAGE_TIME){
    //Fill out full image header
    shkfull.packet_type  = SHKFULL;
    shkfull.frame_number = frame_number;
    shkfull.exptime = 0;
    shkfull.ontime = 0;
    shkfull.temp = 0;
    shkfull.imxsize = SHKXS;
    shkfull.imysize = SHKYS;
    shkfull.mode = 0;
    shkfull.start_sec = start.tv_sec;
    shkfull.start_nsec = start.tv_nsec;

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
