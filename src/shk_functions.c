#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <acedev5.h>


/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "numeric.h"
#include "hex_functions.h"
#include "alp_functions.h"
#include "phx_config.h"

/**************************************************************/
/* SHK_INIT_CELLS                                             */
/*  - Set cell origins and beam select flags                  */
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
      shkevent->cells[c].cenbox_origin[0] = i*cell_size_px + cell_size_px/2 + SHK_CELL_XOFF;
      shkevent->cells[c].cenbox_origin[1] = j*cell_size_px + cell_size_px/2 + SHK_CELL_YOFF;
      shkevent->cells[c].beam_select = shk_beam_select[y*SHK_XCELLS + x];
      shkevent->beam_ncells += shkevent->cells[c].beam_select;
      c++;
    }
  }
}

/***************************************************************/
/* SHK_SETORIGIN                                               */
/*  - Set centroid box center of each cell to current centroid */
/***************************************************************/
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
    //Set centroid box origins
    for(i=0;i<SHK_NCELLS;i++){
      if(shkevent->cells[i].beam_select){
	shkevent->cells[i].cenbox_origin[0] = cx[i];
	shkevent->cells[i].cenbox_origin[1] = cy[i];
      }
    }

    //Reset
    count = 0;
    memset(cx,0,sizeof(cx));
    memset(cy,0,sizeof(cy));
    printf("SHK: New centroid box origin set\n");
    return 0;
  }
}
/**************************************************************/
/* SHK_CENTROID_CELL                                          */
/*  - Measure the centroid of a single SHK cell               */
/**************************************************************/
void shk_centroid_cell(uint16 *image, shkcell_t *cell, int shk_boxsize){
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
    // boxsize = boxsize_new;
    boxsize=SHK_MAX_BOXSIZE;

  //Calculate corners of centroid box
  blx = floor(cell->cenbox_origin[0] - boxsize);
  bly = floor(cell->cenbox_origin[1] - boxsize);
  trx = floor(cell->cenbox_origin[0] + boxsize);
  try = floor(cell->cenbox_origin[1] + boxsize);

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
    cell->centroid[0] = cell->origin[0];
    cell->centroid[1] = cell->origin[1];
  }
}

/**************************************************************/
/* SHK_CENTROID                                               */
/*  - Measure centroids of all SHK cells                      */
/**************************************************************/
void shk_centroid(uint16 *image, shkevent_t *shkevent){
  int i;

  //Zero out tilts
  shkevent->xtilt=0;
  shkevent->ytilt=0;

  //Get background
  shk_centroid_cell(image,&shkevent->cells[0],shkevent->boxsize);

  //Centroid cells
  for(i=0;i<SHK_NCELLS;i++){
    if(shkevent->cells[i].beam_select){
      shk_centroid_cell(image,&shkevent->cells[i],shkevent->boxsize);
      shkevent->xtilt += shkevent->cells[i].deviation[0];
      shkevent->ytilt += shkevent->cells[i].deviation[1];
    }
  }

  //Average tilts
  shkevent->xtilt /= shkevent->beam_ncells;
  shkevent->ytilt /= shkevent->beam_ncells;
}

/**************************************************************/
/* SHK_ZERNIKE_MATRIX                                         */
/*  - Build SHK Zernike fitting matrix                        */
/**************************************************************/
void shk_zernike_matrix(shkcell_t *cells, double *matrix_inv){
  int i, beam_ncells=0;
  int beam_cell_index[SHK_NCELLS] = {0};
  double max_x = 0, max_y = 0, min_x = SHKXS, min_y = SHKYS;
  double beam_center[2] = {0}, beam_radius[2]={0}, beam_radius_m[2]={0};
  double unit_conversion[2];
  
  // collect beam cell indices on to beam_cell_index[SHK_NCELLS]
  // beam_center = [mean(x), mean(y)] of beam cell origins
  // beam_radius = [(max(x)-min(x))/2, (max(y)-min(y))/2] of beam cell origins
  for(i=0; i<SHK_NCELLS; i++) {
    if (cells[i].beam_select) {
      beam_cell_index[beam_ncells++] = i;
      max_x = (cells[i].origin[0] > max_x) ? cells[i].origin[0] : max_x; // hold on to max x
      max_y = (cells[i].origin[1] > max_y) ? cells[i].origin[1] : max_y; // hold on to max y
      min_x = (cells[i].origin[0] < min_x) ? cells[i].origin[0] : min_x; // hold on to min x
      min_y = (cells[i].origin[1] < min_y) ? cells[i].origin[1] : min_y; // hold on to min y
    }
  }

  beam_center[0] = (max_x+min_x)/2.0; // beam center x
  beam_center[1] = (max_y+min_y)/2.0; // beam center y
  beam_radius[0] = (max_x-min_x)/2.0; // beam radius x
  beam_radius[1] = (max_y-min_y)/2.0; // beam radius y
  printf("SHK: Rebuilding Zernike Matrix\n");
  printf("SHK: (MinX,MaxX): (%f, %f) [px]\n",min_x,max_x);
  printf("SHK: (MinY,MaxY): (%f, %f) [px]\n",min_y,max_y);
  printf("SHK: Beam center: (%f, %f) [px]\n",beam_center[0],beam_center[1]);
  printf("SHK: Beam radius: (%f, %f) [px]\n",beam_radius[0],beam_radius[1]);
  printf("SHK: Beam ncells: %d\n", beam_ncells);

  //bake in the conversion from pixels to wavefront slope
  unit_conversion[0] = (SHK_FOCAL_LENGTH_UM/SHK_PX_PITCH_UM) * (1./(beam_radius[0]*SHK_PX_PITCH_UM));
  unit_conversion[1] = (SHK_FOCAL_LENGTH_UM/SHK_PX_PITCH_UM) * (1./(beam_radius[1]*SHK_PX_PITCH_UM));

  printf("SHK: building the zernike fitting matrix\n");
  double dz_dxdy[2*SHK_NCELLS*LOWFS_N_ZERNIKE] = {0};
  double x_1 = 0, x_2 = 0, x_3 = 0, x_4 = 0, x_5=0;
  double y_1 = 0, y_2 = 0, y_3 = 0, y_4 = 0, y_5=0;
  for(i=0; i<beam_ncells; i++) {
    x_1 = (cells[beam_cell_index[i]].origin[0] - beam_center[0])/beam_radius[0];    //unitless [px/px] (-1 to +1)
    x_2 = pow(x_1,2);
    x_3 = pow(x_1,3);
    x_4 = pow(x_1,4);
    x_5 = pow(x_1,5);
    y_1 = (cells[beam_cell_index[i]].origin[1] - beam_center[1])/beam_radius[1];    //unitless [px/px] (-1 to +1)
    y_2 = pow(y_1,2);
    y_3 = pow(y_1,3);
    y_4 = pow(y_1,4);
    y_5 = pow(y_1,5);
    //NOTE: This is hard coded for LOWFS_N_ZERNIKE == 24
    //Units before unit_conversion are wavefront slope [unitless = um/um] per 1 unit of RMS zernike coefficent= [um/um]
    //--> wavefront slope [um/um] of a wavefront with zernike coefficient = 1
    //unit_conversion contains (1/beam_radius [microns]) this converts the zernike coefficents to microns 
    //Inverse of this is zernike coefficent / wavefront slope [um/um]
    //Inverse * wavefront slope = Inverse * (displacment [um] / focal length [um]) = Zernike coefficent
    //Inverse * (dispacement [px] * pixel size [um] / focal length [um]) = Zernike Coeff
    //unit_conversion also cotains (focal_length / pixel_size) which, when inverted will convert pixel displacements to wavefront slopes
    dz_dxdy[i+( 0*beam_ncells)] =      1.0 * (0)                                                                    *unit_conversion[0];
    dz_dxdy[i+( 2*beam_ncells)] =      2.0 * (1)                                                                    *unit_conversion[0];
    dz_dxdy[i+( 4*beam_ncells)] =      2.0 * (0)                                                                    *unit_conversion[0];
    dz_dxdy[i+( 6*beam_ncells)] =  sqrt(3) * (4*x_1)                                                                *unit_conversion[0];
    dz_dxdy[i+( 8*beam_ncells)] =  sqrt(6) * (2*y_1)                                                                *unit_conversion[0];
    dz_dxdy[i+(10*beam_ncells)] =  sqrt(6) * (2*x_1)                                                                *unit_conversion[0];
    dz_dxdy[i+(12*beam_ncells)] =  sqrt(8) * (9*x_2 + 3*y_2 - 2)                                                    *unit_conversion[0];
    dz_dxdy[i+(14*beam_ncells)] =  sqrt(8) * (6*x_1*y_1)                                                            *unit_conversion[0];
    dz_dxdy[i+(16*beam_ncells)] =  sqrt(8) * (3*x_2 - 3*y_2)                                                        *unit_conversion[0];
    dz_dxdy[i+(18*beam_ncells)] =  sqrt(8) * (6*x_1*y_1)                                                            *unit_conversion[0];
    dz_dxdy[i+(20*beam_ncells)] = sqrt( 5) * (24*x_3 - 12*x_1 + 24*x_1*y_2)                                         *unit_conversion[0];
    dz_dxdy[i+(22*beam_ncells)] = sqrt(10) * (16*x_3 - 6*x_1)                                                       *unit_conversion[0];
    dz_dxdy[i+(24*beam_ncells)] = sqrt(10) * (24*x_2*y_1 - 6*y_1 + 8*y_3)                                           *unit_conversion[0];
    dz_dxdy[i+(26*beam_ncells)] = sqrt(10) * (4*x_3 - 12*x_1*y_2)                                                   *unit_conversion[0];
    dz_dxdy[i+(28*beam_ncells)] = sqrt(10) * (12*x_2*y_1 - 4*y_3)                                                   *unit_conversion[0];
    dz_dxdy[i+(30*beam_ncells)] = sqrt(12) * (50*x_4 - 36*x_2 + 60*x_2*y_2 - 12*y_2 + 10*y_4 + 3)                   *unit_conversion[0];
    dz_dxdy[i+(32*beam_ncells)] = sqrt(12) * (40*x_3*y_1 - 24*x_1*y_1 + 40*x_1*y_3)                                 *unit_conversion[0];
    dz_dxdy[i+(34*beam_ncells)] = sqrt(12) * (25*x_4 - 12*x_2 - 30*x_2*y_2 + 12*y_2 - 15*y_4)                       *unit_conversion[0];
    dz_dxdy[i+(36*beam_ncells)] = sqrt(12) * (60*x_3*y_1 - 24*x_1*y_1 + 20*x_1*y_3)                                 *unit_conversion[0];
    dz_dxdy[i+(38*beam_ncells)] = sqrt(12) * (5*x_4 - 30*x_2*y_2 + 5*y_4)                                           *unit_conversion[0];
    dz_dxdy[i+(40*beam_ncells)] = sqrt(12) * (20*x_3*y_1 - 20*x_1*y_3)                                              *unit_conversion[0];
    dz_dxdy[i+(42*beam_ncells)] = sqrt( 7) * (120*x_5 - 120*x_3 + 240*x_3*y_2 + 24*x_1 - 120*x_1*y_2 + 120*x_1*y_4) *unit_conversion[0];
    dz_dxdy[i+(44*beam_ncells)] = sqrt(14) * (90*x_5 + 60*x_3*y_2 - 80*x_3 + 12*x_1 - 30*x_1*y_4)                   *unit_conversion[0];
    dz_dxdy[i+(46*beam_ncells)] = sqrt(14) * (150*x_4*y_1 + 180*x_2*y_3 - 120*x_2*y_1 + 12*y_1 - 40*y_3 + 30*y_5)   *unit_conversion[0];

    dz_dxdy[i+( 1*beam_ncells)] =      1.0 * (0)                                                                    *unit_conversion[1];
    dz_dxdy[i+( 3*beam_ncells)] =      2.0 * (0)                                                                    *unit_conversion[1];
    dz_dxdy[i+( 5*beam_ncells)] =      2.0 * (1)                                                                    *unit_conversion[1];
    dz_dxdy[i+( 7*beam_ncells)] =  sqrt(3) * (4*y_1)                                                                *unit_conversion[1];
    dz_dxdy[i+( 9*beam_ncells)] =  sqrt(6) * (2*x_1)                                                                *unit_conversion[1];
    dz_dxdy[i+(11*beam_ncells)] =  sqrt(6) * (-2*y_1)                                                               *unit_conversion[1];
    dz_dxdy[i+(13*beam_ncells)] =  sqrt(8) * (6*x_1*y_1)                                                            *unit_conversion[1];
    dz_dxdy[i+(15*beam_ncells)] =  sqrt(8) * (3*x_2 + 9*y_2 - 2)                                                    *unit_conversion[1];
    dz_dxdy[i+(17*beam_ncells)] =  sqrt(8) * (-6*x_1*y_1)                                                           *unit_conversion[1];
    dz_dxdy[i+(19*beam_ncells)] =  sqrt(8) * (3*x_2 - 3*y_2)                                                        *unit_conversion[1];
    dz_dxdy[i+(21*beam_ncells)] = sqrt( 5) * (24*x_2*y_1 - 12*y_1 + 24*y_3)                                         *unit_conversion[1];
    dz_dxdy[i+(23*beam_ncells)] = sqrt(10) * (6*y_1 - 16*y_3)                                                       *unit_conversion[1];
    dz_dxdy[i+(25*beam_ncells)] = sqrt(10) * (8*x_3 - 6*x_1 + 24*x_1*y_2)                                           *unit_conversion[1];
    dz_dxdy[i+(27*beam_ncells)] = sqrt(10) * (-12*x_2*y_1 + 4*y_3)                                                  *unit_conversion[1];
    dz_dxdy[i+(29*beam_ncells)] = sqrt(10) * (4*x_3 - 12*x_1*y_2)                                                   *unit_conversion[1];
    dz_dxdy[i+(31*beam_ncells)] = sqrt(12) * (40*x_3*y_1 -24*x_1*y_1 + 40*x_1*y_3)                                  *unit_conversion[1];
    dz_dxdy[i+(33*beam_ncells)] = sqrt(12) * (10*x_4 - 12*x_2 + 60*x_2*y_2 - 36*y_2 + 50*y_4 + 3)                   *unit_conversion[1];
    dz_dxdy[i+(35*beam_ncells)] = sqrt(12) * (-20*x_3*y_1 + 24*x_1*y_1 - 60*x_1*y_3)                                *unit_conversion[1];
    dz_dxdy[i+(37*beam_ncells)] = sqrt(12) * (15*x_4 - 12*x_2 + 30*x_2*y_2 + 12*y_2 - 25*y_4)                       *unit_conversion[1];
    dz_dxdy[i+(39*beam_ncells)] = sqrt(12) * (-20*x_3*y_1 + 20*x_1*y_3)                                             *unit_conversion[1];
    dz_dxdy[i+(41*beam_ncells)] = sqrt(12) * (5*x_4 - 30*x_2*y_2 + 5*y_4)                                           *unit_conversion[1];
    dz_dxdy[i+(43*beam_ncells)] = sqrt( 7) * (120*x_4*y_1 - 120*x_2*y_1 + 240*x_2*y_3 + 24*y_1 - 120*y_3 + 120*y_5) *unit_conversion[1];
    dz_dxdy[i+(45*beam_ncells)] = sqrt(14) * (30*x_4*y_1 - 60*x_2*y_3 - 12*y_1 + 80*y_3 - 90*y_5)                   *unit_conversion[1];
    dz_dxdy[i+(47*beam_ncells)] = sqrt(14) * (30*x_5 - 40*x_3 + 180*x_3*y_2 + 12*x_1 - 120*x_1*y_2 + 150*x_1*y_4)   *unit_conversion[1];
  }
  printf("SHK: inverting the zernike matrix\n");
  num_dgesvdi(dz_dxdy, matrix_inv, 2*beam_ncells, LOWFS_N_ZERNIKE);
}

/**************************************************************/
/*                      SHK_ZERNIKE_FIT                       */
/*  - Fit Zernikes to SHK centroids                           */
/**************************************************************/
void shk_zernike_fit(shkevent_t *shkevent){
  FILE *matrix=NULL;
  char matrix_file[MAX_FILENAME];
  static double shk2zern[2*SHK_NCELLS*LOWFS_N_ZERNIKE]={0};
  static double dphi_dxdy[2*SHK_NCELLS] = {0};
  double shk_xydev[2*SHK_NCELLS];
  uint64 fsize,rsize;
  static int beam_cell_index[SHK_NCELLS] = {0};
  static int beam_cell_used[SHK_NCELLS] = {0};
  static int beam_ncells=0;
  static int init=0;
  int i;

  /* Initialize Fitting Matrix */
  if(!init){
    /* Get number of cells in the beam */
    for(i=0;i<SHK_NCELLS;i++)
      if(shkevent->cells[i].beam_select)
	beam_cell_index[beam_ncells++]=i;
    /* Read Fitting Matrix From File */
    if(SHK_READ_MATRIX){
      //Set up file names
      sprintf(matrix_file, SHK2ZERNIKE_FILE);
      //Open file
      if((matrix = fopen(matrix_file, "r")) == NULL){
	perror("fopen");
	printf("shk2zernike_file\n");
      }
      //Check file size
      fseek(matrix, 0L, SEEK_END);
      fsize = ftell(matrix);
      rewind(matrix);
      rsize = 2*shkevent->beam_ncells*LOWFS_N_ZERNIKE*sizeof(double);
      if(fsize != rsize){
	printf("SHK: incorrect shk2zern matrix file size %lu != %lu\n", fsize, rsize);
      }
      //Read matrix
      if(fread(shk2zern,2*shkevent->beam_ncells*LOWFS_N_ZERNIKE*sizeof(double),1,matrix) !=1){
	perror("fread");
	printf("shk2zern file\r");
      }
      //Close file
      fclose(matrix);
    }
    else{
      //Generate the zernike matrix
      shk_zernike_matrix(shkevent->cells, shk2zern);
    }
    //Set init flag
    init = 1;
  }

  //Format displacement array
  for(i=0;i<beam_ncells;i++){
    shk_xydev[2*i + 0] = shkevent->cells[i].deviation[0]; //pixels
    shk_xydev[2*i + 1] = shkevent->cells[i].deviation[1]; //pixels
  }
  
  //Do matrix multiply
  num_dgemv(shk2zern, shk_xydev, shkevent->zernike_measured, LOWFS_N_ZERNIKE, 2*shkevent->beam_ncells);

}

/**************************************************************/
/* SHK_CELLS2ALP                                              */
/*  - Convert SHK cell commands to ALPAO DM commands          */
/**************************************************************/
int shk_cells2alp(shkcell_t *cells, double *actuators){
  FILE *matrix=NULL;
  char matrix_file[MAX_FILENAME];
  uint64 fsize,rsize;
  static int init=0;
  static double cells2alp_matrix[2*SHK_NCELLS*ALP_NACT]={0};
  double shk_xydev[2*SHK_NCELLS];
  int i;
  static int beam_ncells = 0;
  static int beam_cell_index[SHK_NCELLS]={0};
  
  /* Initialize */
  if(!init){
    /* Get number of cells in the beam */
    for(i=0;i<SHK_NCELLS;i++)
      if(cells[i].beam_select)
	beam_cell_index[beam_ncells++]=i;
    
    /* Open matrix file */
    //--setup filename
    sprintf(matrix_file,CELLS2ALP_FILE);
    //--open file
    if((matrix = fopen(matrix_file,"r")) == NULL){
      printf("cells2alp file\r");
      perror("fopen");
      return 1;
    }
    //--check file size
    fseek(matrix, 0L, SEEK_END);
    fsize = ftell(matrix);
    rewind(matrix);
    rsize = 2*beam_ncells*ALP_NACT*sizeof(double);
    if(fsize != rsize){
      printf("SHK: incorrect cells2alp matrix file size %lu != %lu\n",fsize,rsize);
      return 1;
    }
    //--read matrix
    if(fread(cells2alp_matrix,2*beam_ncells*ALP_NACT*sizeof(double),1,matrix) != 1){
      perror("fread");
      printf("cells2alp file\r");
      return 1;
    }
    //--close file
    fclose(matrix);
    
    //--set init flag
    init=1;
  }

  //Format displacement array
  for(i=0;i<beam_ncells;i++){
    shk_xydev[2*i + 0] = cells[beam_cell_index[i]].command[0];
    shk_xydev[2*i + 1] = cells[beam_cell_index[i]].command[1];
  }
  
  //Do Matrix Multiply
  num_dgemv(cells2alp_matrix, shk_xydev, actuators, ALP_NACT, 2*beam_ncells);
  
  return 0;
}

/**************************************************************/
/* SHK_CELLS2HEX                                              */
/*  - Convert SHK cell commands to hexapod commands           */
/**************************************************************/
int shk_cells2hex(shkcell_t *cells, double *axes){
  FILE *matrix=NULL;
  char matrix_file[MAX_FILENAME];
  uint64 fsize,rsize;
  static int init=0;
  static double cells2hex_matrix[2*SHK_NCELLS*HEX_NAXES]={0};
  double shk_xydev[2*SHK_NCELLS];
  int i;
  static int beam_ncells = 0;
  static int beam_cell_index[SHK_NCELLS]={0};

  /* Initialize */
  if(!init){
    /* Get number of cells in the beam */
    for(i=0;i<SHK_NCELLS;i++)
      if(cells[i].beam_select)
	beam_cell_index[beam_ncells++]=i;
    /* Open matrix file */
    //--setup filename
    sprintf(matrix_file,CELLS2HEX_FILE);
    //--open file
    if((matrix = fopen(matrix_file,"r")) == NULL){
      printf("cells2hex file\r");
      perror("fopen");
      return 1;
    }
    //--check file size
    fseek(matrix, 0L, SEEK_END);
    fsize = ftell(matrix);
    rewind(matrix);
    rsize = 2*beam_ncells*HEX_NAXES*sizeof(double);
    if(fsize != rsize){
      printf("SHK: incorrect cells2hex matrix file size %lu != %lu\n",fsize,rsize);
      return 1;
    }
    //--read matrix
    if(fread(cells2hex_matrix,2*beam_ncells*HEX_NAXES*sizeof(double),1,matrix) != 1){
      perror("fread");
      printf("cells2hex file\r");
      return 1;
    }
    //--close file
    fclose(matrix);
    
    //--set init flag
    init=1;
  }

  //Format displacement array
  for(i=0;i<beam_ncells;i++){
    shk_xydev[2*i + 0] = cells[beam_cell_index[i]].command[0];
    shk_xydev[2*i + 1] = cells[beam_cell_index[i]].command[1];
  }
  
  //Do Matrix Multiply
  num_dgemv(cells2hex_matrix, shk_xydev, axes, HEX_NAXES, 2*beam_ncells);
  
  return 0;
}

/**************************************************************/
/* SHK_CELLPID                                                */
/*  - Run PID controller on centroid deviations               */
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
  for(i=0;i<SHK_NCELLS;i++){
    if(shkevent->cells[i].beam_select){
      //Calculate integrals
      xint[i] += shkevent->cells[i].deviation[0];
      yint[i] += shkevent->cells[i].deviation[1];
      //Calculate command
      shkevent->cells[i].command[0] = shkevent->kP_cell * shkevent->cells[i].deviation[0] + shkevent->kI_cell * xint[i];
      shkevent->cells[i].command[1] = shkevent->kP_cell * shkevent->cells[i].deviation[1] + shkevent->kI_cell * yint[i];
    }
  }
}

/**************************************************************/
/* SHK_ZERNPID                                                */
/*  - Run PID controller on measured Zernikes                 */
/**************************************************************/
void shk_zernpid(shkevent_t *shkevent, int reset){
  static int init = 0;
  static double zint[LOWFS_N_ZERNIKE] = {0};
  double error;
  int i;
  
  //Initialize
  if(!init || reset){
    memset(zint,0,sizeof(zint));
    memset(shkevent->zernike_command,0,sizeof(shkevent->zernike_command));
    init=1;
    if(reset) return;
  }
  
  //Run PID
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    if(shkevent->zernike_control[i]){
      //Calculate error
      error = shkevent->zernike_measured[i] - shkevent->zernike_target[i];
      //Calculate integral
      zint[i] += error;
      //Calculate command
      shkevent->zernike_command[i] = shkevent->kP_zern * error + shkevent->kI_zern * zint[i];
    }
  }
}

/**************************************************************/
/* SHK_PROCESS_IMAGE                                          */
/*  - Main image processing function for SHK                  */
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
  static int countA=0;
  int state;
  static alp_t alp;
  static hex_t hex;
  int move_hex=0,move_alp=0,found_zernike=0;
  double temp_alp[ALP_NACT]={0};
  double temp_hex[HEX_NAXES]={0};
  double temp_zernike[LOWFS_N_ZERNIKE]={0};
  static uint64 alp_counter=1,hex_counter=1;
  static uint64 hex_last_send=0,hex_last_recv=0;
  hexevent_t hexevent;
  
  //Get time immidiately
  clock_gettime(CLOCK_REALTIME,&start);

  //Get state
  state = sm_p->state;
  
  //Check reset
  if(sm_p->shk_reset){
    init=0;
    sm_p->shk_reset=0;
  }
  
  //Initialize
  if(!init){
    //Zero out events & commands
    memset(&shkfull,0,sizeof(shkfull_t));
    memset(&shkevent,0,sizeof(shkevent_t));
    memset(&alp,0,sizeof(alp_t));
    memset(&hex,0,sizeof(hex_t));
    //Init cells
    shk_init_cells(&shkevent);
    //Init actuator commands
    alp_init(&alp);
    hex_init(&hex);
    //Reset calibration routines
    alp_calibrate(0,&alp,1,0);
    hex_calibrate(0,&hex,NULL,1,0);
    //Reset PID controllers
    shk_cellpid(&shkevent,1);
    shk_zernpid(&shkevent,1);
    //Reset start time
    memcpy(&first,&start,sizeof(struct timespec));
    //Reset command counters
    alp_counter = 1;
    hex_counter = 1;
    hex_last_send = 0;
    hex_last_recv = 0;
    //Set init flag
    init=1;
    //Debugging
    if(SHK_DEBUG) printf("SHK: Initialized\n");
 }
  
  //Measure exposure time
  if(timespec_subtract(&delta,&start,&last))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
  
  //Read current hexapod position
  while(read_from_buffer(sm_p,&hexevent,HEXRECV,SHKID)){
    if(hexevent.status == HEX_CMD_ACCEPTED){
      //Copy accepted command to current position
      memcpy(&hex,&hexevent.hex,sizeof(hex_t));
      if(hexevent.clientid == SHKID){
	//If this was a SHK command:
	// - copy to shkevent
	memcpy(&shkevent.hex,&hexevent.hex,sizeof(hex_t));
	// - set recv index
	hex_last_recv = hexevent.command_number;
	// - increment command counter
	hex_counter++;
      }
    }
  }
  
  //Fill out event header
  shkevent.hed.packet_type  = SHKEVENT;
  shkevent.hed.frame_number = frame_number;
  shkevent.hed.exptime      = 0;
  shkevent.hed.ontime       = dt;
  shkevent.hed.temp         = 0;
  shkevent.hed.state        = state;
  shkevent.hed.imxsize      = SHKXS;
  shkevent.hed.imysize      = SHKYS;
  shkevent.hed.mode         = 0;
  shkevent.hed.start_sec    = start.tv_sec;
  shkevent.hed.start_nsec   = start.tv_nsec;

  //Save modes and gains
  shkevent.boxsize          = sm_p->shk_boxsize;
  shkevent.alp_calmode      = sm_p->alp_calmode;
  shkevent.hex_calmode      = sm_p->hex_calmode;
  shkevent.kP_cell          = sm_p->shk_kP_cell;
  shkevent.kI_cell          = sm_p->shk_kI_cell;
  shkevent.kD_cell          = sm_p->shk_kD_cell;
  shkevent.kP_zern          = sm_p->shk_kP_zern;
  shkevent.kI_zern          = sm_p->shk_kI_zern;
  shkevent.kD_zern          = sm_p->shk_kD_zern;

  //Calculate centroids
  shk_centroid(buffer->pvAddress,&shkevent);

  //Set cell origins
  if(sm_p->shk_setorigin)
    sm_p->shk_setorigin = shk_setorigin(&shkevent);
  
  //Fit Zernikes
  if(sm_p->state_array[state].shk.fit_zernikes)
    shk_zernike_fit(&shkevent);
  
  //Run PIDs
  if(sm_p->state_array[state].shk.pid_zernikes)
    shk_zernpid(&shkevent, 0);
  if(sm_p->state_array[state].shk.pid_cells)
    shk_cellpid(&shkevent, 0);
  
  //Calibrate ALP
  if(sm_p->alp_calmode != ALP_CALMODE_NONE){
    sm_p->alp_calmode = alp_calibrate(shkevent.alp_calmode,&alp,0,alp_counter);
    move_alp = 1;
  }
  
  //Calibrate HEX
  if(sm_p->hex_calmode != HEX_CALMODE_NONE){
    sm_p->hex_calmode = hex_calibrate(shkevent.hex_calmode,&hex,&shkevent.cal_step,0,hex_counter);
    move_hex = 1;
  }

  //Offload tilt to hexapod
  if(sm_p->state_array[state].shk.offload_tilt_to_hex){
    //How to deal with latency?
  }

  //Offload tilt to WASP
  if(sm_p->state_array[state].shk.offload_tilt_to_wasp){
    //How to deal with latency?
  }
  
  //Apply cell commands to ALPAO DM
  if(sm_p->state_array[state].shk.cell_control == ACTUATOR_ALP){
    // - zero out temporary variables
    memset(temp_alp,0,sizeof(temp_alp));
    // - convert cells to actuators
    if(shk_cells2alp(shkevent.cells,temp_alp))
      printf("SHK: shk_cells2alp failed!\n");
    // - add command deltas
    for(i=0;i<ALP_NACT;i++)
      alp.act_cmd[i] += temp_alp[i];
    // - trigger alp command
    move_alp = 1;
  }
  
  //Apply zernike commands to ALP
  // - zero out temporary variables
  memset(temp_alp,0,sizeof(temp_alp));
  memset(temp_zernike,0,sizeof(temp_zernike));
  found_zernike = 0;
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    if(sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_ALP){
      temp_zernike[i]     = shkevent.zernike_command[i];
      alp.zernike_cmd[i] += temp_zernike[i];
      found_zernike = 1;
    }
  }
  // - convert zernikes to actuators and add to current command
  if(found_zernike){
    alp_zern2alp(temp_zernike,temp_alp);
    for(i=0;i<ALP_NACT;i++)
      alp.act_cmd[i] += temp_alp[i];
    // - trigger alp command
    move_alp = 1;
  }

  //Apply zernike commands to HEX
  // - zero out temporary variables
  memset(temp_hex,0,sizeof(temp_hex));
  memset(temp_zernike,0,sizeof(temp_zernike));
  found_zernike = 0;
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    if(sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_HEX){
      temp_zernike[i]     = shkevent.zernike_command[i];
      hex.zernike_cmd[i] += temp_zernike[i];
      found_zernike = 1;
    }
  }
  // - convert zernikes to axes and add to current command
  if(found_zernike){
    hex_zern2hex(temp_zernike,temp_hex);
    for(i=0;i<HEX_NAXES;i++)
      hex.axis_cmd[i] += temp_hex[i];
    // - trigger hex command
    move_hex = 1;
  }
  
  //Send command to ALP
  if(ALP_ENABLE && sm_p->alp_dev >= 0 && move_alp){
    // - send command
    if(sm_p->state_array[state].alp_commander == SHKID){
      if(alp_write(sm_p->alp_dev,&alp))
	printf("SHK: alp_write failed!\n");
      // - copy command to shkevent
      memcpy(&shkevent.alp,&alp,sizeof(alp_t));
      // - increment command counter
      alp_counter++;
    }
  }
  
  //Send command to HEX
  if(HEX_ENABLE && move_hex){
    //Check if the last command was received 
    if(hex_last_send == hex_last_recv){
      // - send command
      if(sm_p->state_array[state].hex_commander == SHKID){
	hexevent.clientid = SHKID;
	hexevent.status   = 0;
	hexevent.command_number = ++hex_last_send;
	memcpy(&hexevent.hex,&hex,sizeof(hex_t));
	write_to_buffer(sm_p,&hexevent,SHK_HEXSEND);
      }
    }
  }
  
  //Open circular buffer
  shkevent_p=(shkevent_t *)open_buffer(sm_p,SHKEVENT);

  //Copy data
  memcpy(shkevent_p,&shkevent,sizeof(shkevent_t));;

  //Get final timestamp
  clock_gettime(CLOCK_REALTIME,&end);
  shkevent_p->hed.end_sec  = end.tv_sec;
  shkevent_p->hed.end_nsec = end.tv_nsec;

  //Close buffer
  close_buffer(sm_p,SHKEVENT);

  //Save time
  memcpy(&last,&start,sizeof(struct timespec));


  /*******************  Full Image Code  *****************/
  if(timespec_subtract(&delta,&start,&first))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
  if(dt > SHK_FULL_IMAGE_TIME){
    //Copy packet header
    memcpy(&shkfull.hed,&shkevent.hed,sizeof(pkthed_t));
    shkfull.hed.packet_type = SHKFULL;

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
    memcpy(&shkfull.shkevent,&shkevent,sizeof(shkevent));

    //Open circular buffer
    shkfull_p=(shkfull_t *)open_buffer(sm_p,SHKFULL);

    //Copy data
    memcpy(shkfull_p,&shkfull,sizeof(shkfull_t));;

    //Get final timestamp
    clock_gettime(CLOCK_REALTIME,&end);
    shkfull_p->hed.end_sec  = end.tv_sec;
    shkfull_p->hed.end_nsec = end.tv_nsec;

    //Close buffer
    close_buffer(sm_p,SHKFULL);

    //Reset time
    memcpy(&first,&start,sizeof(struct timespec));
  }
}
