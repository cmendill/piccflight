#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>


/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "numeric.h"
#include "hex_functions.h"
#include "alp_functions.h"
#include "phx_config.h"
#include "rtd_functions.h"
#include "fakemodes.h"


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
  double dz_dxdy[2*SHK_NCELLS*LOWFS_N_ZERNIKE] = {0};
  double x_1 = 0, x_2 = 0, x_3 = 0, x_4 = 0, x_5=0;
  double y_1 = 0, y_2 = 0, y_3 = 0, y_4 = 0, y_5=0;
  FILE *matrix=NULL;
  char matrix_file[MAX_FILENAME];
  float cell_size_px = SHK_LENSLET_PITCH_UM/SHK_PX_PITCH_UM;

 
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
  beam_radius[0] = (cell_size_px+max_x-min_x)/2.0; // beam radius x
  beam_radius[1] = (cell_size_px+max_y-min_y)/2.0; // beam radius y

  //bake in the conversion from pixels to wavefront slope
  unit_conversion[0] = (SHK_FOCAL_LENGTH_UM/SHK_PX_PITCH_UM) * (1./(beam_radius[0]*SHK_PX_PITCH_UM));
  unit_conversion[1] = (SHK_FOCAL_LENGTH_UM/SHK_PX_PITCH_UM) * (1./(beam_radius[1]*SHK_PX_PITCH_UM));
  printf("SHK: Rebuilding Zernike Matrix\n");
  printf("SHK: (MinX,MaxX): (%f, %f) [px]\n",min_x,max_x);
  printf("SHK: (MinY,MaxY): (%f, %f) [px]\n",min_y,max_y);
  printf("SHK: Beam center: (%f, %f) [px]\n",beam_center[0],beam_center[1]);
  printf("SHK: Beam radius: (%f, %f) [px]\n",beam_radius[0],beam_radius[1]);
  printf("SHK: Unit conver: (%f, %f)     \n",unit_conversion[0],unit_conversion[1]);
  printf("SHK: Beam ncells: %d\n", beam_ncells);

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
    //NOTE: This is hard coded for LOWFS_N_ZERNIKE == 23
    //Units before unit_conversion are wavefront slope [unitless = um/um] per 1 unit of RMS zernike coefficent= [um/um]
    //--> wavefront slope [um/um] of a wavefront with zernike coefficient = 1
    //unit_conversion contains (1/beam_radius [microns]) this converts the zernike coefficents to microns 
    //Inverse of this is zernike coefficent / wavefront slope [um/um]
    //Inverse * wavefront slope = Inverse * (displacment [um] / focal length [um]) = Zernike coefficent
    //Inverse * (dispacement [px] * pixel size [um] / focal length [um]) = Zernike Coeff
    //unit_conversion also cotains (focal_length / pixel_size) which, when inverted will convert pixel displacements to wavefront slopes
    dz_dxdy[2*i+0+( 0*beam_ncells)] =      2.0 * (1)                                                                    *unit_conversion[0];
    dz_dxdy[2*i+0+( 2*beam_ncells)] =      2.0 * (0)                                                                    *unit_conversion[0];
    dz_dxdy[2*i+0+( 4*beam_ncells)] =  sqrt(3) * (4*x_1)                                                                *unit_conversion[0];
    dz_dxdy[2*i+0+( 6*beam_ncells)] =  sqrt(6) * (2*x_1)                                                                *unit_conversion[0];
    dz_dxdy[2*i+0+( 8*beam_ncells)] =  sqrt(6) * (2*y_1)                                                                *unit_conversion[0];
    dz_dxdy[2*i+0+(10*beam_ncells)] =  sqrt(8) * (9*x_2 + 3*y_2 - 2)                                                    *unit_conversion[0];
    dz_dxdy[2*i+0+(12*beam_ncells)] =  sqrt(8) * (6*x_1*y_1)                                                            *unit_conversion[0];
    dz_dxdy[2*i+0+(14*beam_ncells)] =  sqrt(8) * (3*x_2 - 3*y_2)                                                        *unit_conversion[0];
    dz_dxdy[2*i+0+(16*beam_ncells)] =  sqrt(8) * (6*x_1*y_1)                                                            *unit_conversion[0];
    dz_dxdy[2*i+0+(18*beam_ncells)] = sqrt( 5) * (24*x_3 - 12*x_1 + 24*x_1*y_2)                                         *unit_conversion[0];
    dz_dxdy[2*i+0+(20*beam_ncells)] = sqrt(10) * (16*x_3 - 6*x_1)                                                       *unit_conversion[0];
    dz_dxdy[2*i+0+(22*beam_ncells)] = sqrt(10) * (24*x_2*y_1 - 6*y_1 + 8*y_3)                                           *unit_conversion[0];
    dz_dxdy[2*i+0+(24*beam_ncells)] = sqrt(10) * (4*x_3 - 12*x_1*y_2)                                                   *unit_conversion[0];
    dz_dxdy[2*i+0+(26*beam_ncells)] = sqrt(10) * (12*x_2*y_1 - 4*y_3)                                                   *unit_conversion[0];
    dz_dxdy[2*i+0+(28*beam_ncells)] = sqrt(12) * (50*x_4 - 36*x_2 + 60*x_2*y_2 - 12*y_2 + 10*y_4 + 3)                   *unit_conversion[0];
    dz_dxdy[2*i+0+(30*beam_ncells)] = sqrt(12) * (40*x_3*y_1 - 24*x_1*y_1 + 40*x_1*y_3)                                 *unit_conversion[0];
    dz_dxdy[2*i+0+(32*beam_ncells)] = sqrt(12) * (25*x_4 - 12*x_2 - 30*x_2*y_2 + 12*y_2 - 15*y_4)                       *unit_conversion[0];
    dz_dxdy[2*i+0+(34*beam_ncells)] = sqrt(12) * (60*x_3*y_1 - 24*x_1*y_1 + 20*x_1*y_3)                                 *unit_conversion[0];
    dz_dxdy[2*i+0+(36*beam_ncells)] = sqrt(12) * (5*x_4 - 30*x_2*y_2 + 5*y_4)                                           *unit_conversion[0];
    dz_dxdy[2*i+0+(38*beam_ncells)] = sqrt(12) * (20*x_3*y_1 - 20*x_1*y_3)                                              *unit_conversion[0];
    dz_dxdy[2*i+0+(40*beam_ncells)] = sqrt( 7) * (120*x_5 - 120*x_3 + 240*x_3*y_2 + 24*x_1 - 120*x_1*y_2 + 120*x_1*y_4) *unit_conversion[0];
    dz_dxdy[2*i+0+(42*beam_ncells)] = sqrt(14) * (90*x_5 + 60*x_3*y_2 - 80*x_3 + 12*x_1 - 30*x_1*y_4)                   *unit_conversion[0];
    dz_dxdy[2*i+0+(44*beam_ncells)] = sqrt(14) * (150*x_4*y_1 + 180*x_2*y_3 - 120*x_2*y_1 + 12*y_1 - 40*y_3 + 30*y_5)   *unit_conversion[0];

    dz_dxdy[2*i+1+( 0*beam_ncells)] =      2.0 * (0)                                                                    *unit_conversion[1];
    dz_dxdy[2*i+1+( 2*beam_ncells)] =      2.0 * (1)                                                                    *unit_conversion[1];
    dz_dxdy[2*i+1+( 4*beam_ncells)] =  sqrt(3) * (4*y_1)                                                                *unit_conversion[1];
    dz_dxdy[2*i+1+( 6*beam_ncells)] =  sqrt(6) * (-2*y_1)                                                               *unit_conversion[1];
    dz_dxdy[2*i+1+( 8*beam_ncells)] =  sqrt(6) * (2*x_1)                                                                *unit_conversion[1];
    dz_dxdy[2*i+1+(10*beam_ncells)] =  sqrt(8) * (6*x_1*y_1)                                                            *unit_conversion[1];
    dz_dxdy[2*i+1+(12*beam_ncells)] =  sqrt(8) * (3*x_2 + 9*y_2 - 2)                                                    *unit_conversion[1];
    dz_dxdy[2*i+1+(14*beam_ncells)] =  sqrt(8) * (-6*x_1*y_1)                                                           *unit_conversion[1];
    dz_dxdy[2*i+1+(16*beam_ncells)] =  sqrt(8) * (3*x_2 - 3*y_2)                                                        *unit_conversion[1];
    dz_dxdy[2*i+1+(18*beam_ncells)] = sqrt( 5) * (24*x_2*y_1 - 12*y_1 + 24*y_3)                                         *unit_conversion[1];
    dz_dxdy[2*i+1+(20*beam_ncells)] = sqrt(10) * (6*y_1 - 16*y_3)                                                       *unit_conversion[1];
    dz_dxdy[2*i+1+(22*beam_ncells)] = sqrt(10) * (8*x_3 - 6*x_1 + 24*x_1*y_2)                                           *unit_conversion[1];
    dz_dxdy[2*i+1+(24*beam_ncells)] = sqrt(10) * (-12*x_2*y_1 + 4*y_3)                                                  *unit_conversion[1];
    dz_dxdy[2*i+1+(26*beam_ncells)] = sqrt(10) * (4*x_3 - 12*x_1*y_2)                                                   *unit_conversion[1];
    dz_dxdy[2*i+1+(28*beam_ncells)] = sqrt(12) * (40*x_3*y_1 -24*x_1*y_1 + 40*x_1*y_3)                                  *unit_conversion[1];
    dz_dxdy[2*i+1+(30*beam_ncells)] = sqrt(12) * (10*x_4 - 12*x_2 + 60*x_2*y_2 - 36*y_2 + 50*y_4 + 3)                   *unit_conversion[1];
    dz_dxdy[2*i+1+(32*beam_ncells)] = sqrt(12) * (-20*x_3*y_1 + 24*x_1*y_1 - 60*x_1*y_3)                                *unit_conversion[1];
    dz_dxdy[2*i+1+(34*beam_ncells)] = sqrt(12) * (15*x_4 - 12*x_2 + 30*x_2*y_2 + 12*y_2 - 25*y_4)                       *unit_conversion[1];
    dz_dxdy[2*i+1+(36*beam_ncells)] = sqrt(12) * (-20*x_3*y_1 + 20*x_1*y_3)                                             *unit_conversion[1];
    dz_dxdy[2*i+1+(38*beam_ncells)] = sqrt(12) * (5*x_4 - 30*x_2*y_2 + 5*y_4)                                           *unit_conversion[1];
    dz_dxdy[2*i+1+(40*beam_ncells)] = sqrt( 7) * (120*x_4*y_1 - 120*x_2*y_1 + 240*x_2*y_3 + 24*y_1 - 120*y_3 + 120*y_5) *unit_conversion[1];
    dz_dxdy[2*i+1+(42*beam_ncells)] = sqrt(14) * (30*x_4*y_1 - 60*x_2*y_3 - 12*y_1 + 80*y_3 - 90*y_5)                   *unit_conversion[1];
    dz_dxdy[2*i+1+(44*beam_ncells)] = sqrt(14) * (30*x_5 - 40*x_3 + 180*x_3*y_2 + 12*x_1 - 120*x_1*y_2 + 150*x_1*y_4)   *unit_conversion[1];

  }

  /* Write forward matrix to file */
  //Set up file name
  sprintf(matrix_file, ZERN2SHK_OUTFILE);
  //Open file
  if((matrix = fopen(matrix_file, "w")) == NULL){
    perror("fopen");
    printf("zern2shk file\n");
  }
  //Write matrix
  if(fwrite(dz_dxdy,2*beam_ncells*LOWFS_N_ZERNIKE*sizeof(double),1,matrix) !=1){
    perror("fwrite");
    printf("zern2shk file\r");
  }
  //Close file
  fclose(matrix);
  printf("SHK: Wrote zernike matrix file: %s\n",matrix_file);


  //Invert Matrix NOTE: This changes the forward matrix
  printf("SHK: inverting the zernike matrix\n");
  num_dgesvdi(dz_dxdy, matrix_inv, 2*beam_ncells, LOWFS_N_ZERNIKE);



  /* Write inverse matrix to file */
  //Set up file name
  sprintf(matrix_file, SHK2ZERN_OUTFILE);
  //Open file
  if((matrix = fopen(matrix_file, "w")) == NULL){
    perror("fopen");
    printf("shk2zern file\n");
  }
  //Write matrix
  if(fwrite(matrix_inv,2*beam_ncells*LOWFS_N_ZERNIKE*sizeof(double),1,matrix) !=1){
    perror("fwrite");
    printf("shk2zern file\r");
  }
  //Close file
  fclose(matrix);
  printf("SHK: Wrote zernike matrix file: %s\n",matrix_file);
}

/**************************************************************/
/*                      SHK_ZERNIKE_FIT                       */
/*  - Fit Zernikes to SHK centroids                           */
/**************************************************************/
void shk_zernike_fit(shkcell_t *cells, double *zernikes){
  FILE *matrix=NULL;
  char matrix_file[MAX_FILENAME];
  static double shk2zern[2*SHK_NCELLS*LOWFS_N_ZERNIKE]={0};
  double shk_xydev[2*SHK_NCELLS];
  uint64 fsize,rsize;
  static int beam_cell_index[SHK_NCELLS] = {0};
  static int beam_ncells=0;
  static int init=0;
  int i;
  
  /* Initialize Fitting Matrix */
  if(!init){
    /* Get number of cells in the beam */
    beam_ncells=0;
    for(i=0;i<SHK_NCELLS;i++)
      if(cells[i].beam_select)
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
      rsize = 2*beam_ncells*LOWFS_N_ZERNIKE*sizeof(double);
      if(fsize != rsize){
	printf("SHK: incorrect shk2zern matrix file size %lu != %lu\n", fsize, rsize);
      }
      //Read matrix
      if(fread(shk2zern,2*beam_ncells*LOWFS_N_ZERNIKE*sizeof(double),1,matrix) !=1){
	perror("fread");
	printf("shk2zern file\r");
      }
      //Close file
      fclose(matrix);
      printf("SHK: Read: %s\n",matrix_file);
    }
    else{
      //Generate the zernike matrix
      shk_zernike_matrix(cells, shk2zern);
    }
    //Set init flag
    init = 1;
  }

  //Format displacement array
  for(i=0;i<beam_ncells;i++){
    shk_xydev[2*i + 0] = cells[beam_cell_index[i]].deviation[0]; //pixels
    shk_xydev[2*i + 1] = cells[beam_cell_index[i]].deviation[1]; //pixels
  }
  
  //Do matrix multiply
  num_dgemv(shk2zern, shk_xydev, zernikes, LOWFS_N_ZERNIKE, 2*beam_ncells);

}

/**************************************************************/
/* SHK_CELLS2ALP                                              */
/*  - Convert SHK cell commands to ALPAO DM commands          */
/**************************************************************/
void shk_cells2alp(shkcell_t *cells, double *actuators){
  FILE *matrix=NULL;
  char matrix_file[MAX_FILENAME];
  uint64 fsize,rsize;
  static int init=0;
  static double cells2alp_matrix[2*SHK_NCELLS*ALP_NACT]={0};
  double shk_xydev[2*SHK_NCELLS]={0};
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
      goto end_of_init;
    }
    //--check file size
    fseek(matrix, 0L, SEEK_END);
    fsize = ftell(matrix);
    rewind(matrix);
    rsize = 2*beam_ncells*ALP_NACT*sizeof(double);
    if(fsize != rsize){
      printf("SHK: incorrect cells2alp matrix file size %lu != %lu\n",fsize,rsize);
      fclose(matrix);
      goto end_of_init;
    }
    //--read matrix
    if(fread(cells2alp_matrix,2*beam_ncells*ALP_NACT*sizeof(double),1,matrix) != 1){
      perror("fread");
      printf("cells2alp file\r");
      fclose(matrix);
      goto end_of_init;
    }
    //--close file
    fclose(matrix);
    printf("SHK: Read: %s\n",matrix_file);

  end_of_init:
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
    printf("SHK: Read: %s\n",matrix_file);
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
/* SHK_ALP_CELLPID                                            */
/*  - Run PID controller on centroid deviations for ALP       */
/**************************************************************/
void shk_alp_cellpid(shkevent_t *shkevent, int reset){
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
      shkevent->cells[i].command[0] = shkevent->kP_alp_cell * shkevent->cells[i].deviation[0] + shkevent->kI_alp_cell * xint[i];
      shkevent->cells[i].command[1] = shkevent->kP_alp_cell * shkevent->cells[i].deviation[1] + shkevent->kI_alp_cell * yint[i];
    }
  }
}

/**************************************************************/
/* SHK_ALP_ZERNPID                                            */
/*  - Run PID controller on measured Zernikes for ALP         */
/**************************************************************/
void shk_alp_zernpid(shkevent_t *shkevent, double *zernike_delta, int reset){
  static int init = 0;
  static double zint[LOWFS_N_ZERNIKE] = {0};
  double error;
  int i;
  
  //Initialize
  if(!init || reset){
    memset(zint,0,sizeof(zint));
    init=1;
    if(reset) return;
  }
  
  //Run PID
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    //Calculate error
    error = shkevent->zernike_measured[i] - shkevent->zernike_target[i];
    //Calculate integral
    zint[i] += error;
    //Calculate command
    zernike_delta[i] = shkevent->kP_alp_zern * error + shkevent->kI_alp_zern * zint[i];
  }
}

/**************************************************************/
/* SHK_HEX_ZERNPID                                            */
/*  - Run PID controller on measured Zernikes for HEX         */
/**************************************************************/
void shk_hex_zernpid(shkevent_t *shkevent, double *zernike_delta, int reset){
  static int init = 0;
  static double zint[LOWFS_N_ZERNIKE] = {0};
  double error;
  int i;
  
  //Initialize
  if(!init || reset){
    memset(zint,0,sizeof(zint));
    init=1;
    if(reset) return;
  }
  
  //Run PID
  for(i=0;i<LOWFS_N_ZERNIKE;i++){
    //Calculate error
    error = shkevent->zernike_measured[i] - shkevent->zernike_target[i];
    //Calculate integral
    zint[i] += error;
    //Calculate command
    zernike_delta[i] = shkevent->kP_hex_zern * error + shkevent->kI_hex_zern * zint[i];
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
  static struct timespec start,end,delta,full_last,hex_last;
  static int init=0;
  double dt;
  int i,j;
  uint16_t fakepx=0;
  int state;
  hex_t hex,hex_delta;
  alp_t alp,alp_delta;
  int control_zernike=0;
  uint32_t n_dither=1;
  
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
    //Init cells
    shk_init_cells(&shkevent);
    //Reset calibration routines
    alp_calibrate(0,&alp,1);
    hex_calibrate(0,&hex,NULL,1);
    //Reset PID controllers
    shk_alp_cellpid(NULL,NULL,FUNCTION_RESET);
    shk_alp_zernpid(NULL,NULL,FUNCTION_RESET);
    shk_hex_zernpid(NULL,NULL,FUNCTION_RESET);
    //Reset last times
    memcpy(&full_last,&start,sizeof(struct timespec));
    memcpy(&hex_last,&start,sizeof(struct timespec));
    //Set init flag
    init=1;
    //Debugging
    if(SHK_DEBUG) printf("SHK: Initialized\n");
  }
  
  //Measure exposure time
  if(timespec_subtract(&delta,&start,&last))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
    
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
  shkevent.kP_alp_cell      = sm_p->shk_kP_alp_cell;
  shkevent.kI_alp_cell      = sm_p->shk_kI_alp_cell;
  shkevent.kD_alp_cell      = sm_p->shk_kD_alp_cell;
  shkevent.kP_alp_zern      = sm_p->shk_kP_alp_zern;
  shkevent.kI_alp_zern      = sm_p->shk_kI_alp_zern;
  shkevent.kD_alp_zern      = sm_p->shk_kD_alp_zern;
  shkevent.kP_hex_zern      = sm_p->shk_kP_hex_zern;
  shkevent.kI_hex_zern      = sm_p->shk_kI_hex_zern;
  shkevent.kD_hex_zern      = sm_p->shk_kD_hex_zern;

  //Calculate centroids
  shk_centroid(buffer->pvAddress,&shkevent);

  //Command: Set cell origins
  if(sm_p->shk_setorigin)
    sm_p->shk_setorigin = shk_setorigin(&shkevent);
  
  //Fit Zernikes
  if(sm_p->state_array[state].shk.fit_zernikes)
    shk_zernike_fit(shkevent.cells,shkevent.zernike_measured);

  /************************************************************/
  /*******************  Hexapod Control Code  *****************/
  /************************************************************/

  //Check time since last command
  if(timespec_subtract(&delta,&start,&hex_last))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);

  //Check if we will send a command
  if((sm_p->state_array[state].hex_commander == SHKID) && sm_p->hex_ready && (dt > HEX_PERIOD)){
    //Get last HEX command
    hex_get_command(sm_p,&hex);
    
    //Check if HEX is controlling any Zernikes
    zernike_control = 0;
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      if(sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_HEX)
	zernike_control = 1;

    //Run Zernike control
    if(zernike_control){
      // - run Zernike PID
      shk_hex_zernpid(&shkevent, hex_delta.zernike_cmd, FUNCTION_NO_RESET);

      // - convert Zernike deltas to axis deltas
      hex_zern2hex_alt(hex_delta.zernike_cmd, hex_delta.axis_cmd);
      
      // - add Zernike PID output deltas to HEX command
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_HEX)
	  hex.zernike_cmd[i] += hex_delta.zernike_cmd[i];

      // - add axis deltas to HEX command
      for(i=0;i<HEX_N_AXES;i++)
	hex.axis_cmd[i] += hex_delta.axis_cmd[i];
    }

    //Run HEX calibration
    if(sm_p->hex_calmode != HEX_CALMODE_NONE)
      sm_p->hex_calmode = hex_calibrate(shkevent.hex_calmode,&hex,&shkevent.hex_calstep,FUNCTION_NO_RESET);
    
    //Send command to HEX
    if(hex_command(sm_p,hex.axis_cmd,SHKID)){
      // - copy to shkevent
      memcpy(&shkevent.hex,&hex,sizeof(hex_t));
    }
    
    //Reset time
    memcpy(&hex_last,&start,sizeof(struct timespec));
  }
  
  /*************************************************************/
  /*******************  ALPAO DM Control Code  *****************/
  /*************************************************************/
  
  //Check if we will send a command
  if((sm_p->state_array[state].alp_commander == SHKID) && sm_p->alp_ready){
    //Get last ALP command
    alp_get_command(sm_p,&alp);
    
    //Check if ALP is controlling any Zernikes
    zernike_control = 0;
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      if(sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_ALP)
	zernike_control = 1;
    
    //Run Zernike control
    if(zernike_control){
      // - run Zernike PID
      shk_alp_zernpid(&shkevent, alp_delta.zernike_cmd, FUNCTION_NO_RESET);

      // - convert zernike deltas to actuator deltas
      alp_zern2alp(alp_delta.zernike_cmd,alp_delta.act_cmd);

      // - add Zernike PID output deltas to ALP command
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_ALP)
	  alp.zernike_cmd[i] += alp_delta.zernike_cmd[i];

      // - add actuator deltas to ALP command
      for(i=0;i<ALP_NACT;i++)
	alp.act_cmd[i] += alp_delta.act_cmd[i];
    }

    //Check if ALP is controlling SHK cells
    if(sm_p->state_array[state].shk.cell_control){
      // - run cell PID
      shk_alp_cellpid(&shkevent, FUNCTION_NO_RESET);

      // - convert cell commands to actuator deltas
      shk_cells2alp(shkevent.cells,alp_delta.act_cmd);

      // - add actuator deltas to ALP command
      for(i=0;i<ALP_NACT;i++)
	alp.act_cmd[i] += alp_delta.act_cmd[i];
    }  
    
    //Calibrate ALP
    if(sm_p->alp_calmode != ALP_CALMODE_NONE)
      sm_p->alp_calmode = alp_calibrate(shkevent.alp_calmode,&alp,&shkevent.alp_calstep,FUNCTION_NO_RESET);
    
    //Send command to ALP
    if(alp_command(sm_p,alp.act_cmd,SHKID,alp_cmdtype,n_dither)){
      // - copy command to shkevent
      memcpy(&shkevent.alp,&alp,sizeof(alp_t));
    }
  }
  
  
  //Open SHKEVENT circular buffer
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


  /*************************************************************/
  /**********************  Full Image Code  ********************/
  /*************************************************************/
  if(timespec_subtract(&delta,&start,&full_last))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
  if(dt > SHK_FULL_IMAGE_TIME){
    //Copy packet header
    memcpy(&shkfull.hed,&shkevent.hed,sizeof(pkthed_t));
    shkfull.hed.packet_type = SHKFULL;

    //Fake data
    if(sm_p->shk_fakemode > FAKEMODE_NONE){
      if(sm_p->shk_fakemode == FAKEMODE_GEN_IMAGE_CAMERA_SYNC)
	for(i=0;i<SHKXS;i++)
	  for(j=0;j<SHKYS;j++)
	    shkfull.image.data[i][j]=fakepx++;
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
    memcpy(&full_last,&start,sizeof(struct timespec));
  }
}
