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
#include "../hex/hex_functions.h"
#include "../alp/alp_functions.h"
#include "../alp/acedev5.h"
#include "../phx/include/phx_api.h"
#include "../phx/config.h"

/**************************************************************/
/*                      SHK_INIT_CELLS                        */
/**************************************************************/
void shk_init_cells(shkevent_t *shkevent){
  #include "shk_beam_select.h"
  float cell_size_px = SHK_LENSLET_PITCH_UM/SHK_PX_PITCH_UM;
  int i,j,c,x,y;
  int beam_ncells=0;

  //Zero out cells
  memset(shkevent->cells,0,sizeof(shkcell_t));
  shkevent->beam_ncells = 0;

  //Initialize cells
  c=0;
  for(j=0;j<SHK_YCELLS;j++){
    for(i=0;i<SHK_XCELLS;i++){
      x = c % SHK_XCELLS;
      y = SHK_YCELLS - 1 - c / SHK_YCELLS;
      shkevent->cells[c].index = c;
      shkevent->cells[c].origin[0] = i*cell_size_px + cell_size_px/2 + SHK_CELL_XOFF;
      shkevent->cells[c].origin[1] = j*cell_size_px + cell_size_px/2 + SHK_CELL_YOFF;
      shkevent->cells[c].cell_origin[0] = i*cell_size_px + cell_size_px/2 + SHK_CELL_XOFF;
      shkevent->cells[c].cell_origin[1] = j*cell_size_px + cell_size_px/2 + SHK_CELL_YOFF;
      shkevent->cells[c].beam_select = shk_beam_select[y*SHK_XCELLS + x];
      shkevent->beam_ncells += shkevent->cells[c].beam_select;
      c++;
    }
  }
}

/**************************************************************/
/*                      SHK_SETORIGIN                         */
/**************************************************************/
int shk_setorigin(shkevent_t *shkevent){
  int i;
  static double cx[SHK_NCELLS]={0}, cy[SHK_NCELLS]={0};
  static int count=0;
  const int navg = SHK_ORIGIN_NAVG;

  //Average the centroids
  if(count < navg){
    for(i=0;i<SHK_NCELLS;i++){
      if(shkevent->cells[i].beam_select){
	cx[i] += shkevent->cells[i].centroid[0] / navg;
	cy[i] += shkevent->cells[i].centroid[1] / navg;
      }
    }
    count++;
    return 1;
  }
  else{
    //Set origins = averaged centroids
    for(i=0;i<SHK_NCELLS;i++){
      if(shkevent->cells[i].beam_select){
	shkevent->cells[i].cell_origin[0] = cx[i];
	shkevent->cells[i].cell_origin[1] = cy[i];
      }
    }

    //Reset
    count = 0;
    memset(cx,0,sizeof(cx));
    memset(cy,0,sizeof(cy));
    printf("SHK: New origin set\n");
    return 0;
  }
}
/**************************************************************/
/*                      SHK_CENTROID_CELL                     */
/**************************************************************/
void shk_centroid_cell(uint16 *image, shkcell_t *cell, int shk_boxsize, int iwc_calmode){
  double xnum=0, ynum=0, total=0;
  static double background = 0;
  uint32 maxpix=0,maxval=0;
  int blx,bly,trx,try;
  int boxsize,boxsize_new;
  int x,y,px,npix;
  double xhist[SHKXS]={0};
  double yhist[SHKYS]={0};
  double val;


  //Set boxsize & spot_captured flag
  boxsize = SHK_MAX_BOXSIZE;
  boxsize_new = shk_boxsize;
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
      }
    }
  }
  if(cell->spot_captured)
    boxsize = boxsize_new;



  //Calculate corners of centroid box
  blx = floor(cell->cell_origin[0] - boxsize);
  bly = floor(cell->cell_origin[1] - boxsize);
  trx = floor(cell->cell_origin[0] + boxsize);
  try = floor(cell->cell_origin[1] + boxsize);

  //Impose limits
  blx = blx > SHK_XMAX ? SHK_XMAX : blx;
  bly = bly > SHK_YMAX ? SHK_YMAX : bly;
  blx = blx < SHK_XMIN ? SHK_XMIN : blx;
  bly = bly < SHK_YMIN ? SHK_YMIN : bly;
  trx = trx > SHK_XMAX ? SHK_XMAX : trx;
  try = try > SHK_YMAX ? SHK_YMAX : try;
  trx = trx < SHK_XMIN ? SHK_XMIN : trx;
  try = try < SHK_YMIN ? SHK_YMIN : try;

  //Save limits
  cell->blx = blx;
  cell->bly = bly;
  cell->trx = trx;
  cell->try = try;

  //Unset background
  if(cell->index == 0)
    background = 0;

  //Build x,y histograms
  npix=0;
  for(x=blx;x<=trx;x++){
    for(y=bly;y<=try;y++){
      px = x + y*SHKYS;
      val = (double)image[px] - background;
      xhist[x] += val;
      yhist[y] += val;
      npix++;
      if(image[px] > maxval){
	maxval=image[px];
	maxpix=px;
      }
    }
  }

  //Weight histograms
  for(x=blx;x<=trx;x++){
    xnum  += ((double)x+0.5) * xhist[x];
  }
  for(y=bly;y<=try;y++){
    ynum  += ((double)y+0.5) * yhist[y];
    total += yhist[y];
  }


  //Calculate centroid
  cell->centroid[0] = xnum/total;
  cell->centroid[1] = ynum/total;

  //Calculate deviation
  cell->deviation[0] = cell->centroid[0] - cell->origin[0];
  cell->deviation[1] = cell->centroid[1] - cell->origin[1];

  //Set max pixel
  cell->maxpix = maxpix;
  cell->maxval = maxval;

  //Save total intensity and background
  cell->intensity  = total;
  cell->background = background;

  //Set background
  if(cell->index == 0)
    background = total/npix;

  //Check if spot is above threshold
  if(cell->spot_found && (maxval < SHK_SPOT_LOWER_THRESH))
    cell->spot_found=0;

  if(maxval > SHK_SPOT_UPPER_THRESH)
    cell->spot_found=1;

  //Check spot captured flag
  if(!cell->spot_found){
    cell->spot_captured=0;
    cell->deviation[0] = 0;
    cell->deviation[1] = 0;
  }
}

/**************************************************************/
/*                      SHK_CENTROID_IWC                      */
/**************************************************************/
// void shk_centroid(uint16 *image, shkevent_t *shkevent){
//   int i;
//
//   //Zero out tilts
//   shkevent->xtilt=0;
//   shkevent->ytilt=0;
//
//   //Set boxsize
//   if(shkevent->iwc.calmode == 2) //NEED TO MAKE THIS MORE CLEAR
//     shkevent->boxsize = SHK_MAX_BOXSIZE;
//
//   //Get background
//   shk_centroid_cell(image,&shkevent->cells[0],shkevent->boxsize,shkevent->iwc.calmode);
//
//   //Centroid cells
//   for(i=0;i<SHK_NCELLS;i++){
//     if(shkevent->cells[i].beam_select){
//       shk_centroid_cell(image,&shkevent->cells[i],shkevent->boxsize,shkevent->iwc.calmode);
//       shkevent->xtilt += shkevent->cells[i].deviation[0];
//       shkevent->ytilt += shkevent->cells[i].deviation[1];
//     }
//   }
//
//   //Average tilts
//   shkevent->xtilt /= shkevent->beam_ncells;
//   shkevent->ytilt /= shkevent->beam_ncells;
//
//   //Subtract tilts
//   // for(i=0;i<SHK_NCELLS;i++){
//   //   if(shkevent->cells[i].beam_select){
//   //     shkevent->cells[i].deviation[0] -= shkevent->xtilt;
//   //     shkevent->cells[i].deviation[1] -= shkevent->ytilt;
//   //   }
//   // }
//
// }

/**************************************************************/
/*                      SHK_CENTROID_ALP                      */
/**************************************************************/
void shk_centroid(uint16 *image, shkevent_t *shkevent){
  int i;

  //Zero out tilts
  shkevent->xtilt=0;
  shkevent->ytilt=0;

  //Set boxsize
  // if(shkevent->alp.calmode > 0) //NEED TO MAKE THIS MORE CLEAR
  //   shkevent->boxsize = SHK_MAX_BOXSIZE;

  //Get background
  shk_centroid_cell(image,&shkevent->cells[0],shkevent->boxsize,shkevent->alp.calmode);

  //Centroid cells
  for(i=0;i<SHK_NCELLS;i++){
    if(shkevent->cells[i].beam_select){
      shk_centroid_cell(image,&shkevent->cells[i],shkevent->boxsize,shkevent->alp.calmode);
      shkevent->xtilt += shkevent->cells[i].deviation[0];
      shkevent->ytilt += shkevent->cells[i].deviation[1];
    }
  }

  //Average tilts
  shkevent->xtilt /= shkevent->beam_ncells;
  shkevent->ytilt /= shkevent->beam_ncells;

  //Subtract tilts
  // for(i=0;i<SHK_NCELLS;i++){
  //   if(shkevent->cells[i].beam_select){
  //     shkevent->cells[i].deviation[0] -= shkevent->xtilt;
  //     shkevent->cells[i].deviation[1] -= shkevent->ytilt;
  //   }
  // }

}

/**************************************************************/
/*                   SHK_ZERNIKE_MATRIX                       */
/**************************************************************/
void shk_zernike_matrix(shkevent_t *shkevent, double *matrix_inv){
  int i, beam_ncells=0;
  int beam_cell_index[SHK_NCELLS] = {0};
  double max_x = 0, max_y = 0, min_x = 0, min_y = 0;
  double temp_beam_center[2] = {0}, beam_center[2] = {0}, beam_radius[2]={0}, beam_radius_m[2]={0};
  double beam_radius_multiplier[2];
  // collect beam cell indices on to beam_cell_index[SHK_NCELLS]
  // beam_center = [mean(x), mean(y)] of beam cell origins
  // beam_radius = [(max(x)-min(x))/2, (max(y)-min(y))/2] of beam cell origins
  for(i=0; i<SHK_NCELLS; i++) {
    if (shkevent->cells[i].beam_select) {
      beam_cell_index[beam_ncells] = i;
      temp_beam_center[0] += shkevent->cells[i].origin[0]; // center
      temp_beam_center[1] += shkevent->cells[i].origin[1]; // center
      max_x = (shkevent->cells[i].origin[0] > max_x) ? shkevent->cells[i].origin[0] : max_x; // hold on to max x
      max_y = (shkevent->cells[i].origin[1] > max_y) ? shkevent->cells[i].origin[1] : max_y; // hold on to max y
      min_x = (shkevent->cells[i].origin[0] < min_x) ? shkevent->cells[i].origin[0] : min_x; // hold on to min x
      min_y = (shkevent->cells[i].origin[1] < min_y) ? shkevent->cells[i].origin[1] : min_y; // hold on to min y
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
    x_1 = (shkevent->cells[beam_cell_index[i]].origin[0] - beam_center[0])/beam_radius[0];
    x_2 = pow(x_1,2);
    x_3 = pow(x_1,3);
    x_4 = pow(x_1,4);
    x_5 = pow(x_1,5);
    y_1 = (shkevent->cells[beam_cell_index[i]].origin[1] - beam_center[1])/beam_radius[1];
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
  FILE *matrix=NULL;
  char matrix_file[MAX_FILENAME];
  static double shk2zern[2*SHK_NCELLS*LOWFS_N_ZERNIKE]={0};
  double shk_xydev[2*SHK_NCELLS];
  uint64 fsize,rsize;
  int i, c, beam_ncells=0, beam_ncells_2=0, regen_matrix=0;
  int beam_cell_index[SHK_NCELLS] = {0};
  static int beam_cell_used[SHK_NCELLS] = {0};
  const double px2slope = SHK_PX_PITCH_UM/SHK_FOCAL_LENGTH_UM;
  static double matrix_inv[SHK_NCELLS*LOWFS_N_ZERNIKE*2] = {0};
  static double dphi_dxdy[2*SHK_NCELLS] = {0};

  //set up file names
  sprintf(matrix_file, SHK2ZERNIKE_FILE);
  //open file
  if((matrix = fopen(matrix_file, "r")) == NULL){
    perror("fopen");
    printf("shk2zernike_file\n");
  }
  //check file size
  fseek(matrix, 0L, SEEK_END);
  fsize = ftell(matrix);
  rewind(matrix);
  rsize = 2*shkevent->beam_ncells*LOWFS_N_ZERNIKE*sizeof(double);
  if(fsize != rsize){
    printf("SHK: incorrect shk2zern matrix file size %lu != %lu\n", fsize, rsize);
  }
  //read matrix
  if(fread(shk2zern,2*shkevent->beam_ncells*LOWFS_N_ZERNIKE*sizeof(double),1,matrix) !=1){
    perror("fread");
    printf("shk2zern file\r");
  }
  //close file
  fclose(matrix);

  //format displacement array
  c=0;
  for(i=0;i<SHK_NCELLS;i++){
    if(shkevent->cells[i].beam_select){
      shk_xydev[2*c + 0] = shkevent->cells[i].deviation[0];
      shk_xydev[2*c + 1] = shkevent->cells[i].deviation[1];
      c++;
    }
  }
  beam_ncells_2 = 2*shkevent->beam_ncells;

  //Do matrix multiply
  num_dgemv(shk2zern, shk_xydev, shkevent->zernikes, LOWFS_N_ZERNIKE, beam_ncells_2);

  //Record measured zernikes
  for(i=0; i<LOWFS_N_ZERNIKE;i++){
    shkevent->alp.zern_now[i] = shkevent->zernikes[i];
  }

  // double zern_offset[24]={      -0.00000,   0.0135805,    -0.0271113,  0.00863601,   0.0277714,   0.0382247,  -0.000627516,   -0.00352233,    -0.0194335,  0.00962264,    -0.0138212,  -9.02948e-05,    -0.0343523,  0.00119619,
  // 0.00885264,   -0.00791998,    -0.0176545,   0.0158954,   0.0110489,   0.0252110,  0.00504202,    -0.0121719,  0.00607450,   -0.00243058};
  //
  // for(i=0;i<LOWFS_N_ZERNIKE;i++){
  //   shkevent->zernikes[i] += zern_offset[i];
  // }

  for(i=0; i<SHK_NCELLS; i++) {
    if (shkevent->cells[i].beam_select) {
      //save the spot index
      beam_cell_index[beam_ncells] = i;
      //check if we should regenerate the matrix
      if(!beam_cell_used[i]) regen_matrix=1;
      beam_cell_used[i]=1;
      beam_ncells++;
    }else{
      if(beam_cell_used[i]){
	//if the spot was used last time, regenerate
	regen_matrix=1;
	beam_cell_used[i]=0;
      }
    }
  }
  beam_ncells_2 = 2*beam_ncells;

  //generate the zernike matrix
  if(regen_matrix) {
    shk_zernike_matrix(shkevent, matrix_inv);
    printf("SHK: matrix inverted\n");
  }

  for(i=0; i<beam_ncells; i++) {
    // multiplying vector is in slopes
    dphi_dxdy[i]             = shkevent->cells[beam_cell_index[i]].deviation[0]*px2slope;
    dphi_dxdy[i+beam_ncells] = shkevent->cells[beam_cell_index[i]].deviation[1]*px2slope;
  }

  /* Previous matrix multiplication */
  // num_dgemv(matrix_inv, dphi_dxdy,shkevent->zernikes, LOWFS_N_ZERNIKE, beam_ncells_2);
}

/**************************************************************/
/*                        SHK_SHK2IWC                         */
/**************************************************************/
// int shk_shk2iwc(shkevent_t *shkevent, sm_t *sm_p, int reset){
//   FILE *matrix=NULL;
//   FILE *zern_matrix=NULL;
//   char matrix_file[MAX_FILENAME];
//   char zern_matrix_file[MAX_FILENAME];
//   static int init=0;
//   static double shk2iwc[2*SHK_NCELLS*IWC_NSPA]={0};
//   static double zern2iwc[LOWFS_N_ZERNIKE*IWC_NSPA]={0};
//   double shk_xydev[2*SHK_NCELLS];
//   double shk_zern[LOWFS_N_ZERNIKE];
//   uint64 fsize,rsize;
//   uint64 zfsize, zrsize;
//   int c,i,beam_ncells_2;
//   static int test=0;
//
//   if(!init || reset){
//     /* Open matrix files */
//     //--setup filenames
//     sprintf(matrix_file,SHKMATRIX_FILE);
//     //--open files
//     if((matrix = fopen(matrix_file,"r")) == NULL){
//       perror("fopen");
//       printf("shkmatrix_file\n")
// ;      return 1;
//     }
//     sprintf(zern_matrix_file, ZERNIKE2SPA_FILE);
//     if((zern_matrix = fopen(zern_matrix_file,"r")) == NULL){
//       perror("fopen");
//       printf("zernike2spa_file\n");
//       return 1;
//     }
//
//     //--check file size
//     fseek(matrix, 0L, SEEK_END);
//     fsize = ftell(matrix);
//     rewind(matrix);
//     rsize = 2*shkevent->beam_ncells*IWC_NSPA*sizeof(double);
//     if(fsize != rsize){
//       printf("SHK: incorrect spot2spa matrix file size %lu != %lu\n",fsize,rsize);
//       return 1;
//     }
//     fseek(zern_matrix, 0L, SEEK_END);
//     zfsize = ftell(zern_matrix);
//     rewind(zern_matrix);
//     zrsize = LOWFS_N_ZERNIKE*IWC_NSPA*sizeof(double);
//     if(zfsize != zrsize){
//       printf("SHK: incorrect zern2spa matrix file size %lu != %lu\n",zfsize,zrsize);
//       return 1;
//     }
//
//       //--read matrix
//       if(fread(zern2iwc,LOWFS_N_ZERNIKE*IWC_NSPA*sizeof(double),1,zern_matrix) != 1){
//         perror("fread");
//         return 1;
//       }
//       //--close file
//       fclose(zern_matrix);
//
//       //--set init flag
//       init=1;
//
//     if(fread(shk2iwc,2*shkevent->beam_ncells*IWC_NSPA*sizeof(double),1,matrix) != 1){
//       perror("fread");
//       return 1;
//     }
//     //--close file
//     fclose(matrix);
//
//     //--set init flag
//     init=1;
//   }
//
//   //Format displacement array
//   c=0;
//   for(i=0;i<SHK_NCELLS;i++){
//     if(shkevent->cells[i].beam_select){
//       shk_xydev[2*c + 0] = shkevent->cells[i].command[0];
//       shk_xydev[2*c + 1] = shkevent->cells[i].command[1];
//       c++;
//     }
//   }
//   beam_ncells_2 = 2*shkevent->beam_ncells;
//
//   for(i=0;i<LOWFS_N_ZERNIKE;i++){
//     shk_zern[i] = shkevent->zernikes[i]*shkevent->kP;
//   }
//
//   //Do Matrix Multiply and recast to ints
//   if(sm_p->shk_fit_zernike){
//     num_dgemv(zern2iwc,shk_zern,shkevent->iwc_spa_matrix, IWC_NSPA, LOWFS_N_ZERNIKE);
//     for(i=0;i<IWC_NSPA;i++) shkevent->iwc.spa[i] += (uint16)(shkevent->iwc_spa_matrix[i]);
//   }else{
//     num_dgemv(shk2iwc,shk_xydev,shkevent->iwc_spa_matrix, IWC_NSPA, beam_ncells_2);
//     for(i=0;i<IWC_NSPA;i++) shkevent->iwc.spa[i] += (uint16)(shkevent->iwc_spa_matrix[i]);
//   }
//
//   return 0;
// }

/**************************************************************/
/*                        SHK_SHK2ALP                         */
/**************************************************************/
int shk_shk2alp(shkevent_t *shkevent, sm_t *sm_p, int reset){
  FILE *matrix=NULL;
  FILE *zern_matrix=NULL;
  char matrix_file[MAX_FILENAME];
  char zern_matrix_file[MAX_FILENAME];
  static int init=0;
  static double shk2alp[2*SHK_NCELLS*ALP_NACT]={0};
  static double zern2alp[LOWFS_N_ZERNIKE*ALP_NACT]={0};
  double shk_xydev[2*SHK_NCELLS];
  double shk_zern[LOWFS_N_ZERNIKE];
  double zern_trg[LOWFS_N_ZERNIKE];
  double total = 0;
  double low_zern_tot  = 0;
  double high_zern_tot = 0;
  uint64 fsize,rsize;
  uint64 zfsize, zrsize;
  int c,i,beam_ncells_2;
  static int test = 0;

  if(!init || reset){
    /* Open matrix files */
    //--setup filenames
    sprintf(matrix_file,SHKMATRIX_FILE);
    //--open files
    if((matrix = fopen(matrix_file,"r")) == NULL){
      perror("fopen");
      printf("shk2alp file\r");
      return 1;
    }
    sprintf(zern_matrix_file, ZERNIKE2ALP_FILE);
    if((zern_matrix = fopen(zern_matrix_file,"r")) == NULL){
      perror("fopen");
      printf("zern2alp file\r");
      return 1;
    }

    //--check file size
    fseek(matrix, 0L, SEEK_END);
    fsize = ftell(matrix);
    rewind(matrix);
    rsize = 2*shkevent->beam_ncells*ALP_NACT*sizeof(double);
    if(fsize != rsize){
      printf("SHK: incorrect spot2alp matrix file size %lu != %lu\n",fsize,rsize);
      return 1;
    }
    fseek(zern_matrix, 0L, SEEK_END);
    zfsize = ftell(zern_matrix);
    rewind(zern_matrix);
    zrsize = LOWFS_N_ZERNIKE*ALP_NACT*sizeof(double);
    if(zfsize != zrsize){
      printf("SHK: incorrect zern2alp matrix file size %lu != %lu\n",zfsize,zrsize);
      return 1;
    }

      //--read matrix
      if(fread(zern2alp,LOWFS_N_ZERNIKE*ALP_NACT*sizeof(double),1,zern_matrix) != 1){
        perror("fread");
        printf("zern2alp file\r");
        return 1;
      }
      //--close file
      fclose(zern_matrix);

      //--set init flag
      init=1;

    if(fread(shk2alp,2*shkevent->beam_ncells*ALP_NACT*sizeof(double),1,matrix) != 1){
      perror("fread");
      printf("shk2alp file\r");
      return 1;
    }
    //--close file
    fclose(matrix);

    //--set init flag
    init=1;
  }

  //Format displacement array
  c=0;
  for(i=0;i<SHK_NCELLS;i++){
    if(shkevent->cells[i].beam_select){
      shk_xydev[2*c + 0] = shkevent->cells[i].command[0];
      shk_xydev[2*c + 1] = shkevent->cells[i].command[1];
      c++;
    }
  }
  beam_ncells_2 = 2*shkevent->beam_ncells;

  //Calculate total of low/high zernikes
  for(i=0;i<5;i++){
    low_zern_tot += shkevent->zernikes[i];
  }
  for(i=5;i<LOWFS_N_ZERNIKE;i++){
    high_zern_tot += shkevent->zernikes[i];
  }

  if(low_zern_tot > 1.0){
    init=0;
    sm_p->shk_reset=1;
  }
  //Apply target / Zernike thresholding
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    sm_p->zern_targ[i] = shkevent->alp.zern_trg[i];
    shk_zern[i] = shkevent->zernikes[i] - shkevent->alp.zern_trg[i];
    if(low_zern_tot > 0.05){
      if(i > 4){
        shk_zern[i] = 0.0;
      }
    }
  }

  //Do Matrix Multiply set actuator commands
  if(sm_p->shk_fit_zernike){
    num_dgemv(zern2alp,shk_zern,shkevent->alp_act_matrix, ALP_NACT, LOWFS_N_ZERNIKE);
    for(i=0;i<ALP_NACT;i++){
      shkevent->alp.act_cmd[i] += (double)(shkevent->alp_act_matrix[i]*shkevent->kP);
      total += shkevent->alp.act_cmd[i];
    }

  }else{
    num_dgemv(shk2alp,shk_xydev,shkevent->alp_act_matrix, ALP_NACT, beam_ncells_2);
    for(i=0;i<ALP_NACT;i++){
      shkevent->alp.act_cmd[i] += (double)(shkevent->alp_act_matrix[i]*shkevent->kP);
      total += shkevent->alp.act_cmd[i];
    }
  }

  return 0;
}

/**************************************************************/
/*                        SHK_SHK2HEX                         */
/**************************************************************/
int shk_shk2hex(shkevent_t *shkevent, sm_t *sm_p, int reset){
  FILE *matrix=NULL;
  char matrix_file[MAX_FILENAME];
  static int init=0;
  static double shk2hex[LOWFS_N_ZERNIKE*HEX_NAXES]={0};
  double shk_zern[LOWFS_N_ZERNIKE];
  uint64 fsize,rsize;
  int c,i,beam_ncells_2;
  static int test=0;

  if(!init || reset){
    /* Open matrix file */
    //--setup filename
    sprintf(matrix_file,ZERNIKE2HEX_FILE);
    //--open file
    if((matrix = fopen(matrix_file,"r")) == NULL){
      perror("fopen");
      printf("zernike2hex_file\n");
      return 1;
    }
    //--check file size
    fseek(matrix, 0L, SEEK_END);
    fsize = ftell(matrix);
    rewind(matrix);
    rsize = (LOWFS_N_ZERNIKE-8)*HEX_NAXES*sizeof(double);
    if(fsize != rsize){
      printf("SHK: incorrect HEX matrix file size %lu != %lu\n",fsize,rsize);
      return 1;
    }
    //--read matrix
    if(fread(shk2hex,(LOWFS_N_ZERNIKE-8)*HEX_NAXES*sizeof(double),1,matrix) != 1){
      perror("fread");
      printf("shk2hex file\r");
      return 1;
    }
    //--close file
    fclose(matrix);

    //--set init flag
    init=1;
  }

  //Do Matrix Multiply
  num_dgemv(shk2hex, shkevent->zernikes ,shkevent->hex_axs_matrix, HEX_NAXES, LOWFS_N_ZERNIKE);

  //Recast to doubles
  if(sm_p->shk_fit_zernike  && sm_p->hex_calmode)
    for(i=0;i<HEX_NAXES;i++) shkevent->hex.axs[i] += (double)(shkevent->hex_axs_matrix[i])*sm_p->hex_kP;

  return 0;
}

/**************************************************************/
/*                        SHK_CELLPID                         */
/**************************************************************/
void shk_cellpid(shkevent_t *shkevent, int reset){
  static int init = 0;
  static double xint[SHK_NCELLS] = {0};
  static double yint[SHK_NCELLS] = {0};
  int i;

  //Initialize
  if(!init || reset){
    memset(xint,0,sizeof(xint));
    memset(yint,0,sizeof(yint));
    for(i=0;i<SHK_NCELLS;i++){
      shkevent->cells[i].command[0] = 0;
      shkevent->cells[i].command[1] = 0;
    }
    init=1;
    if(reset) return;
  }

  //Run PID
  int alp_calmode = shkevent->alp.calmode;
  if(alp_calmode != 5){
    for(i=0;i<SHK_NCELLS;i++){
      if(shkevent->cells[i].beam_select){
        //Calculate integrals
        xint[i] += shkevent->cells[i].deviation[0];
        yint[i] += shkevent->cells[i].deviation[1];
        //Calculate command
        shkevent->cells[i].command[0] = shkevent->kP * shkevent->cells[i].deviation[0] + shkevent->kI * xint[i];
        shkevent->cells[i].command[1] = shkevent->kP * shkevent->cells[i].deviation[1] + shkevent->kI * yint[i];
      }
    }
  }
  for(i=0; i<LOWFS_N_ZERNIKE; i++){
    shkevent->alp.zern_cmd[i] = shkevent->zernikes[i] * shkevent->kP;
  }

}

/**************************************************************/
/*                      SHK_PROCESS_IMAGE                     */
/**************************************************************/
void shk_process_image(stImageBuff *buffer,sm_t *sm_p, uint32 frame_number){
  static shkfull_t shkfull;
  static shkevent_t shkevent;
  shkfull_t *shkfull_p;
  shkevent_t *shkevent_p;
  dm_t dm;
  pez_t pez;
  static struct timespec first,start,end,delta,last;
  static int init=0;
  double dt;
  int32 i,j;
  uint16 fakepx=0;
  int iwc_calmode=0;
  int alp_calmode=0;
  int hex_calmode=0;


  //Get time immidiately
  clock_gettime(CLOCK_REALTIME,&start);

  //Check reset
  if(sm_p->shk_reset){
    init=0;
    sm_p->shk_reset=0;
  }

  //Initialize
  if(!init){
    memset(&shkfull,0,sizeof(shkfull));
    memset(&shkevent,0,sizeof(shkevent));
    memset(&dm,0,sizeof(dm_t));
    memset(&pez,0,sizeof(pez_t));
    shk_init_cells(&shkevent);
    iwc_init(&shkevent.iwc);
    alp_init(&shkevent.alp);
    hex_init(&shkevent.hex);
    // iwc_calibrate(iwc_calmode,&shkevent.iwc,1);
    alp_calibrate(alp_calmode,&shkevent.alp,1);
    hex_calibrate(hex_calmode,&shkevent.hex,1);
    shk_cellpid(&shkevent, 1);
    // shk_shk2iwc(&shkevent,sm_p, 1);
    shk_shk2alp(&shkevent,sm_p, 1);
    shk_shk2hex(&shkevent,sm_p, 1);
    memcpy(&first,&start,sizeof(struct timespec));
    init=1;
  }

  //Write current act positions
  for(i=0; i<ALP_NACT; i++){
    shkevent.alp.act_now[i] = shkevent.alp.act_cmd[i];
  }

  //Measure exposure time
  if(timespec_subtract(&delta,&start,&last))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);

  //Set iwc_calmode
  iwc_calmode = sm_p->iwc_calmode;

  //Set alp_calmode
  alp_calmode = sm_p->alp_calmode;

  //Set hex_calmode
  hex_calmode = sm_p->hex_calmode;

  //Fill out event header
  shkevent.frame_number = frame_number;
  shkevent.exptime = 0;
  shkevent.ontime = dt;
  shkevent.imxsize = SHKXS;
  shkevent.imysize = SHKYS;
  shkevent.mode = 0;
  shkevent.boxsize = sm_p->shk_boxsize;
  shkevent.start_sec = start.tv_sec;
  shkevent.start_nsec = start.tv_nsec;
  shkevent.iwc.calmode = iwc_calmode;
  shkevent.alp.calmode = alp_calmode;
  shkevent.hex.calmode = hex_calmode;
  shkevent.kP = sm_p->shk_kP;
  shkevent.kI = sm_p->shk_kI;
  shkevent.kD = sm_p->shk_kD;
  shkevent.kH = sm_p->hex_kP;

  //Calculate centroids
  shk_centroid(buffer->pvAddress,&shkevent);

  //Set origin
  if(sm_p->shk_setorigin) sm_p->shk_setorigin = shk_setorigin(&shkevent);

  //Fit Zernikes
  if(sm_p->shk_fit_zernike) shk_zernike_fit(&shkevent);

  //Run PID
  // if(!alp_calmode){
    shk_cellpid(&shkevent, 0);
  // }

  // OLD IWC CODE///////////////////////////////////////////////////////////////
  //Convert cells to IWC
  // shk_shk2iwc(&shkevent,sm_p,0);

  //Calibrate IWC
  // if(iwc_calmode) sm_p->iwc_calmode = iwc_calibrate(iwc_calmode,&shkevent.iwc,0);

  //Apply update to IWC
  // if(xin_write(sm_p->xin_dev,&shkevent.iwc,&dm,&pez)){
  //   printf("SHK: xin_write failed!\n");
  //   //Do something??
  // }
  //////////////////////////////////////////////////////////////////////////////


  shk_shk2alp(&shkevent, sm_p, 0);

  //Convert cells to HEX
  shk_shk2hex(&shkevent,sm_p,0);

  //Calibrate ALP
  if(alp_calmode) sm_p->alp_calmode = alp_calibrate(alp_calmode,&shkevent.alp,0);

  //Calibrate HEX
  if(hex_calmode) sm_p->hex_calmode = hex_calibrate(hex_calmode,&shkevent.hex,0);

  //Apply update to ALP
  // for(i=0; i<LOWFS_N_ZERNIKE; i++){
  //   shkevent.alp.zern_cmd[i] = shkevent.zernikes[i] * shkevent.kP;
  // }
  // if(sm_p->hex_godef == 1){
    if(alp_calmode != 5){
      int alp_dev;
      alp_dev = sm_p->alp_dev;
      alp_write(&alp_dev, &shkevent.alp);
    }
  // }





  //Apply update to HEX
  if(hex_calmode){
    for(i=0;i<HEX_NAXES;i++){
      if(sm_p->hex[i] != shkevent.hex.axs[i]){
        sm_p->hex[i] = shkevent.hex.axs[i];
        if(hex_calmode == 2){
          usleep(ONE_MILLION / 2.0);
        }
      }
    }
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
