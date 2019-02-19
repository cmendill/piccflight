#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <libgen.h>
#include <sys/stat.h>


/* piccflight headers */
#include "controller.h"
#include "common_functions.h"
#include "numeric.h"
#include "hex_functions.h"
#include "alp_functions.h"
#include "bmc_functions.h"
#include "tgt_functions.h"
#include "phx_config.h"
#include "rtd_functions.h"
#include "fakemodes.h"

/**************************************************************/
/* SHK_INIT_CELLS                                             */
/*  - Set cell origins and beam select flags                  */
/**************************************************************/
void shk_init_cells(shkevent_t *shkevent){
  double cell_size_px = SHK_LENSLET_PITCH_UM/SHK_PX_PITCH_UM;
  int i,j,c;
  int shk_beam_select[SHK_NCELLS] = SHK_BEAM_SELECT;
  
  //Zero out cells
  memset(shkevent->cells,0,sizeof(shkcell_t));
  
  //Initialize cells
  c=0;
  for(j=0;j<SHK_YCELLS;j++){
    for(i=0;i<SHK_XCELLS;i++){
      if(shk_beam_select[(SHK_YCELLS - j - 1)*SHK_XCELLS + i]){
	shkevent->cells[c].xorigin = i*cell_size_px + cell_size_px/2 + SHK_CELL_XOFF;
	shkevent->cells[c].yorigin = j*cell_size_px + cell_size_px/2 + SHK_CELL_YOFF;
	c++;
      }
    }
  }
}

/***************************************************************/
/* SHK_SETORIGIN                                               */
/*  - Set cell origins to current centroid                     */
/***************************************************************/
int shk_setorigin(shkevent_t *shkevent){
  int i;
  static double cx[SHK_BEAM_NCELLS]={0}, cy[SHK_BEAM_NCELLS]={0};
  static int count = 0;
  const double navg = SHK_ORIGIN_NAVG;
  
  //Average the centroids
  if(count++ < navg){
    for(i=0;i<SHK_BEAM_NCELLS;i++){
      cx[i] += shkevent->cells[i].xcentroid/navg;
      cy[i] += shkevent->cells[i].ycentroid/navg;
    }
    return 1;
  }
  else{
    //Set cell origins
    for(i=0;i<SHK_BEAM_NCELLS;i++){
      shkevent->cells[i].xorigin = cx[i];
      shkevent->cells[i].yorigin = cy[i];
    }
    
    //Reset
    count = 0;
    memset(cx,0,sizeof(cx));
    memset(cy,0,sizeof(cy));
    printf("SHK: New origin set\n");
    return 0;
  }
}

/***************************************************************/
/* SHK_REVERTORIGIN                                             */
/*  - Resets cell origins to default location                  */
/***************************************************************/
void shk_revertorigin(shkevent_t *shkevent){
  shk_init_cells(shkevent);
}

/***************************************************************/
/* SHK_SAVEORIGIN                                              */
/*  - Saves cell origins to file                               */
/***************************************************************/
void shk_saveorigin(shkevent_t *shkevent){
  struct stat st = {0};
  FILE *fd=NULL;
  static char outfile[MAX_FILENAME];
  int i;
  char temp[MAX_FILENAME];
  char path[MAX_FILENAME];
  
  /* Open output file */
  //--setup filename
  sprintf(outfile,"%s",SHK_ORIGIN_FILE);
  //--create output folder if it does not exist
  strcpy(temp,outfile);
  strcpy(path,dirname(temp));
  if (stat(path, &st) == -1){
    printf("SHK: creating folder %s\n",path);
    recursive_mkdir(path, 0777);
  }
  //--open file
  if((fd = fopen(outfile, "w")) == NULL){
    perror("SHK: saveorigin fopen()\n");
    return;
  }
  
  //Save origin
  for(i=0;i<SHK_BEAM_NCELLS;i++){
    if(fwrite(&shkevent->cells[i].xorigin,sizeof(shkevent->cells[i].xorigin),1,fd) != 1){
      printf("SHK: saveorigin fwrite error!\n");
      fclose(fd);
      return;
    }
    if(fwrite(&shkevent->cells[i].yorigin,sizeof(shkevent->cells[i].yorigin),1,fd) != 1){
      printf("SHK: saveorigin fwrite error!\n");
      fclose(fd);
      return;
    }
  }
  printf("SHK: Wrote: %s\n",outfile);

  //Close file
  fclose(fd);
  return;
}

/***************************************************************/
/* SHK_LOADORIGIN                                              */
/*  - Loads cell origins from file                             */
/***************************************************************/
void shk_loadorigin(shkevent_t*shkevent){
  FILE *fd=NULL;
  char filename[MAX_FILENAME];
  uint64 fsize,rsize;
  int i;
  shkcell_t cells[SHK_BEAM_NCELLS];
  
  /* Open origin file */
  //--setup filename
  sprintf(filename,SHK_ORIGIN_FILE);
  //--open file
  if((fd = fopen(filename,"r")) == NULL){
    perror("SHK: loadorigin fopen");
    return;
  }
  //--check file size
  fseek(fd, 0L, SEEK_END);
  fsize = ftell(fd);
  rewind(fd);
  rsize = SHK_BEAM_NCELLS * (sizeof(cells[0].xorigin) + sizeof(cells[0].yorigin)); 
  if(fsize != rsize){
    printf("SHK: incorrect SHK_ORIGIN_FILE size %lu != %lu\n",fsize,rsize);
    fclose(fd);
    return;
  }
  
  //Read file
  for(i=0;i<SHK_BEAM_NCELLS;i++){
    if(fread(&cells[i].xorigin,sizeof(cells[i].xorigin),1,fd) != 1){
      perror("SHK: loadorigin fread");
      fclose(fd);
      return;
    }
    if(fread(&cells[i].yorigin,sizeof(cells[i].yorigin),1,fd) != 1){
      perror("SHK: loadorigin fread");
      fclose(fd);
      return;
    }
  }
  
  //Close file
  fclose(fd);
  printf("SHK: Read: %s\n",filename);

  //Copy origins
  for(i=0;i<SHK_BEAM_NCELLS;i++){
    shkevent->cells[i].xorigin = cells[i].xorigin;
    shkevent->cells[i].yorigin = cells[i].yorigin;
  }
  return;
}



/**************************************************************/
/* SHK_CENTROID_CELL                                          */
/*  - Measure the centroid of a single SHK cell               */
/**************************************************************/
void shk_centroid_cell(uint8 *image, shkcell_t *cell, int cmd_boxsize){
  double xnum,ynum,total,intensity;
  uint16 maxval;
  double xhist[SHKXS]={0};
  double yhist[SHKYS]={0};
  int    x,y,px,blx,bly,trx,try,npix,boxsize;
  double wave2surf = 1;
  double xcentroid=0,ycentroid=0,xdeviation=0,ydeviation=0;

  /********************************************************************/
  //NOTES: The technique here is to first search the maximum boxsize 
  //for the brightest pixel, then calculate the true centroid either 
  //using the commanded boxsize or the max boxsize, depending on the
  //brightest pixel position. This minimize the CPU work when we have
  //a small commanded boxsize, but we still need to check for the spot
  //every time through. 
  /********************************************************************/

  /***********************************************************/
  /*********************** Find Spot *************************/
  /***********************************************************/

  //Set boxsize to maximum
  boxsize = SHK_MAX_BOXSIZE;
  
  //Calculate corners of centroid box
  blx = floor((cell->xtarget - boxsize)/SHKBIN);
  bly = floor((cell->ytarget - boxsize)/SHKBIN);
  trx = floor((cell->xtarget + boxsize)/SHKBIN);
  try = floor((cell->ytarget + boxsize)/SHKBIN);
  
  //Impose limits
  blx = blx > SHK_XMAX ? SHK_XMAX : blx;
  bly = bly > SHK_YMAX ? SHK_YMAX : bly;
  blx = blx < SHK_XMIN ? SHK_XMIN : blx;
  bly = bly < SHK_YMIN ? SHK_YMIN : bly;
  trx = trx > SHK_XMAX ? SHK_XMAX : trx;
  try = try > SHK_YMAX ? SHK_YMAX : try;
  trx = trx < SHK_XMIN ? SHK_XMIN : trx;
  try = try < SHK_YMIN ? SHK_YMIN : try;


  //Set centroid as brightest pixel
  maxval=0;
  intensity=0;
  for(x=blx;x<=trx;x++){
    for(y=bly;y<=try;y++){
      px = x + y*SHKYS;
      intensity += image[px];
      if(image[px] > maxval){
	maxval = image[px];
	xcentroid = (x + 0.5)*SHKBIN;
	ycentroid = (y + 0.5)*SHKBIN;
      }
    }
  }
  
  //Check if spot is above or below deadband threshold
  if(maxval > SHK_SPOT_UPPER_THRESH)
    cell->spot_found=1;
  if(maxval < SHK_SPOT_LOWER_THRESH){
    cell->spot_found=0;
    cell->spot_captured=0;
  }
  
  /***********************************************************/
  /********************** Spot Found *************************/
  /***********************************************************/
  if(cell->spot_found){
    //Set target deviations
    xdeviation = xcentroid - cell->xtarget;
    ydeviation = ycentroid - cell->ytarget;

    //Check if the spot is captured
    if(cell->spot_captured){
      //If spot was captured, but is now outside the box, unset captured
      if((fabs(xdeviation) > cmd_boxsize) || (fabs(ydeviation) > cmd_boxsize)){
	cell->spot_captured=0;
      }
    }
    else{
      //If spot was not captured, check if it is within the capture region
      if((fabs(xdeviation) < (cmd_boxsize-SHK_BOX_DEADBAND)) && (fabs(ydeviation) < (cmd_boxsize-SHK_BOX_DEADBAND))){
	cell->spot_captured=1;
      }
    }
    
    //Set boxsize to commanded value if spot is captured
    if(cell->spot_captured)
      boxsize = cmd_boxsize;
  
    //Calculate corners of centroid box
    blx = floor((cell->xtarget - boxsize)/SHKBIN);
    bly = floor((cell->ytarget - boxsize)/SHKBIN);
    trx = floor((cell->xtarget + boxsize)/SHKBIN);
    try = floor((cell->ytarget + boxsize)/SHKBIN);

    //Impose limits
    blx = blx > SHK_XMAX ? SHK_XMAX : blx;
    bly = bly > SHK_YMAX ? SHK_YMAX : bly;
    blx = blx < SHK_XMIN ? SHK_XMIN : blx;
    bly = bly < SHK_YMIN ? SHK_YMIN : bly;
    trx = trx > SHK_XMAX ? SHK_XMAX : trx;
    try = try > SHK_YMAX ? SHK_YMAX : try;
    trx = trx < SHK_XMIN ? SHK_XMIN : trx;
    try = try < SHK_YMIN ? SHK_YMIN : try;

    //Build x,y histograms
    intensity=0;
    for(x=blx;x<=trx;x++){
      for(y=bly;y<=try;y++){
	px = x + y*SHKYS;
	xhist[x]  += image[px];
	yhist[y]  += image[px];
	intensity += image[px];
      }
    }
    
    //Weight histograms
    total = 0;
    xnum  = 0;
    ynum  = 0;
    for(x=blx;x<=trx;x++){
      xnum  += ((double)x+0.5) * xhist[x];
    }
    for(y=bly;y<=try;y++){
      ynum  += ((double)y+0.5) * yhist[y];
      total += yhist[y];
    }
    
    //Calculate centroid
    xcentroid = (xnum/total) * SHKBIN;
    ycentroid = (ynum/total) * SHKBIN;
  }
  
  //Save boxsize
  cell->boxsize = boxsize;
  
  //Save max pixel
  cell->maxval = maxval;
  
  //Save total intensity (of final centroid box)
  cell->intensity  = intensity;
  
  //Save centroids
  cell->xcentroid = xcentroid;
  cell->ycentroid = ycentroid;
  
  //Save deviations
  cell->xtarget_deviation = xcentroid - cell->xtarget;
  cell->ytarget_deviation = ycentroid - cell->ytarget;
  cell->xorigin_deviation = xcentroid - cell->xorigin;
  cell->yorigin_deviation = ycentroid - cell->yorigin;
  
  /**********************************************************/
  /* APPLY UNIT CONVERSION                                  */
  /* -- This changes the units of deviation from [pixels]   */ 
  /*    to [pixels/surface deviation].                      */
  /* -- In single pass, a surface poke will create twice    */
  /*    as much pixel deviation because of reflection.      */
  /*    Therefore the multiplier is 0.5.                    */
  /* -- In double pass, a surface poke will create twice    */
  /*    as much pixel deviation as in single pass.          */
  /*    Therefore the multiplier is 0.25.                   */
  /* -- This will change Zernike fitting to surface units   */
  /* -- NOTE: This only changes the deviation. Centroid,    */
  /*    target and origin remain in pixel units.            */
  /**********************************************************/
  if(INSTRUMENT_INPUT_TYPE == INPUT_TYPE_SINGLE_PASS) wave2surf = 0.5;
  if(INSTRUMENT_INPUT_TYPE == INPUT_TYPE_DOUBLE_PASS) wave2surf = 0.25;
  cell->xtarget_deviation *= wave2surf;
  cell->ytarget_deviation *= wave2surf;
  cell->xorigin_deviation *= wave2surf;
  cell->yorigin_deviation *= wave2surf;
}

/**************************************************************/
/* SHK_CENTROID                                               */
/*  - Measure centroids of all SHK cells                      */
/**************************************************************/
void shk_centroid(uint8 *image, shkevent_t *shkevent){
  int i,j,px;
  int npix=0;
  double background=0;
  
  //Calculate detector background
  for(i=20/SHKBIN;i<50/SHKBIN;i++){
    for(j=20/SHKBIN;j<50/SHKBIN;j++){
      px = i + j*SHKYS;
      background += image[px];
      npix++;
    }
  }
  background /= npix;
  
  //Centroid cells
  for(i=0;i<SHK_BEAM_NCELLS;i++){
    shkevent->cells[i].background = background;
    shk_centroid_cell(image,&shkevent->cells[i],shkevent->boxsize);
  }
}

/**************************************************************/
/* SHK_ZERNIKE_MATRIX                                         */
/*  - Build SHK Zernike fitting matrix                        */
/**************************************************************/
void shk_zernike_matrix(shkcell_t *cells, double *matrix_fwd, double *matrix_inv){
  int i;
  double max_x = 0, max_y = 0, min_x = SHKXS*SHKBIN, min_y = SHKYS*SHKBIN;
  double beam_xcenter,beam_ycenter,beam_radius,beam_radius_m,unit_conversion;
  double dz_dxdy[2*SHK_BEAM_NCELLS*LOWFS_N_ZERNIKE] = {0};
  double dxdy_dz[2*SHK_BEAM_NCELLS*LOWFS_N_ZERNIKE] = {0};
  double x_1 = 0, x_2 = 0, x_3 = 0, x_4 = 0, x_5=0;
  double y_1 = 0, y_2 = 0, y_3 = 0, y_4 = 0, y_5=0;
  struct stat st = {0};
  FILE *fd=NULL;
  char outfile[MAX_FILENAME];
  char temp[MAX_FILENAME];
  char path[MAX_FILENAME];
  double cell_size_px = SHK_LENSLET_PITCH_UM/SHK_PX_PITCH_UM;
  
  
  // beam_center = [mean(x), mean(y)] of beam cell origins
  // beam_radius = [(max(x)-min(x))/2, (max(y)-min(y))/2] of beam cell origins
  for(i=0; i<SHK_BEAM_NCELLS; i++) {
    max_x = (cells[i].xorigin > max_x) ? cells[i].xorigin : max_x; // hold on to max x
    max_y = (cells[i].yorigin > max_y) ? cells[i].yorigin : max_y; // hold on to max y
    min_x = (cells[i].xorigin < min_x) ? cells[i].xorigin : min_x; // hold on to min x
    min_y = (cells[i].yorigin < min_y) ? cells[i].yorigin : min_y; // hold on to min y
  }
  
  beam_xcenter = (max_x+min_x)/2.0; // beam center x
  beam_ycenter = (max_y+min_y)/2.0; // beam center y
  beam_radius  = ((cell_size_px+max_x-min_x)/2.0 + (cell_size_px+max_y-min_y)/2.0) / 2.0;
  
  //bake in the conversion from pixels to wavefront slope
  unit_conversion = (SHK_FOCAL_LENGTH_UM/SHK_PX_PITCH_UM) * (1./(beam_radius*SHK_PX_PITCH_UM));
  
  printf("SHK: Building Zernike matrix for %d cells\n",SHK_BEAM_NCELLS);
  if(SHK_DEBUG) printf("SHK: (MinX,MaxX): (%f, %f) [px]\n",min_x,max_x);
  if(SHK_DEBUG) printf("SHK: (MinY,MaxY): (%f, %f) [px]\n",min_y,max_y);
  if(SHK_DEBUG) printf("SHK: Beam center: (%f, %f) [px]\n",beam_xcenter,beam_ycenter);
  if(SHK_DEBUG) printf("SHK: Beam radius: (%f) [px]\n",beam_radius);
  if(SHK_DEBUG) printf("SHK: Unit conver: (%f)     \n",unit_conversion);

  for(i=0; i<SHK_BEAM_NCELLS; i++) {
    x_1 = (cells[i].xorigin - beam_xcenter)/beam_radius;    //unitless [px/px] (-1 to +1)
    x_2 = pow(x_1,2);
    x_3 = pow(x_1,3);
    x_4 = pow(x_1,4);
    x_5 = pow(x_1,5);
    y_1 = (cells[i].yorigin - beam_ycenter)/beam_radius;    //unitless [px/px] (-1 to +1)
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
    dz_dxdy[2*i+0+( 0*SHK_BEAM_NCELLS)] =      2.0 * (1)                                                                    *unit_conversion;
    dz_dxdy[2*i+0+( 2*SHK_BEAM_NCELLS)] =      2.0 * (0)                                                                    *unit_conversion;
    dz_dxdy[2*i+0+( 4*SHK_BEAM_NCELLS)] =  sqrt(3) * (4*x_1)                                                                *unit_conversion;
    dz_dxdy[2*i+0+( 6*SHK_BEAM_NCELLS)] =  sqrt(6) * (2*x_1)                                                                *unit_conversion;
    dz_dxdy[2*i+0+( 8*SHK_BEAM_NCELLS)] =  sqrt(6) * (2*y_1)                                                                *unit_conversion;
    dz_dxdy[2*i+0+(10*SHK_BEAM_NCELLS)] =  sqrt(8) * (9*x_2 + 3*y_2 - 2)                                                    *unit_conversion;
    dz_dxdy[2*i+0+(12*SHK_BEAM_NCELLS)] =  sqrt(8) * (6*x_1*y_1)                                                            *unit_conversion;
    dz_dxdy[2*i+0+(14*SHK_BEAM_NCELLS)] =  sqrt(8) * (3*x_2 - 3*y_2)                                                        *unit_conversion;
    dz_dxdy[2*i+0+(16*SHK_BEAM_NCELLS)] =  sqrt(8) * (6*x_1*y_1)                                                            *unit_conversion;
    dz_dxdy[2*i+0+(18*SHK_BEAM_NCELLS)] = sqrt( 5) * (24*x_3 - 12*x_1 + 24*x_1*y_2)                                         *unit_conversion;
    dz_dxdy[2*i+0+(20*SHK_BEAM_NCELLS)] = sqrt(10) * (16*x_3 - 6*x_1)                                                       *unit_conversion;
    dz_dxdy[2*i+0+(22*SHK_BEAM_NCELLS)] = sqrt(10) * (24*x_2*y_1 - 6*y_1 + 8*y_3)                                           *unit_conversion;
    dz_dxdy[2*i+0+(24*SHK_BEAM_NCELLS)] = sqrt(10) * (4*x_3 - 12*x_1*y_2)                                                   *unit_conversion;
    dz_dxdy[2*i+0+(26*SHK_BEAM_NCELLS)] = sqrt(10) * (12*x_2*y_1 - 4*y_3)                                                   *unit_conversion;
    dz_dxdy[2*i+0+(28*SHK_BEAM_NCELLS)] = sqrt(12) * (50*x_4 - 36*x_2 + 60*x_2*y_2 - 12*y_2 + 10*y_4 + 3)                   *unit_conversion;
    dz_dxdy[2*i+0+(30*SHK_BEAM_NCELLS)] = sqrt(12) * (40*x_3*y_1 - 24*x_1*y_1 + 40*x_1*y_3)                                 *unit_conversion;
    dz_dxdy[2*i+0+(32*SHK_BEAM_NCELLS)] = sqrt(12) * (25*x_4 - 12*x_2 - 30*x_2*y_2 + 12*y_2 - 15*y_4)                       *unit_conversion;
    dz_dxdy[2*i+0+(34*SHK_BEAM_NCELLS)] = sqrt(12) * (60*x_3*y_1 - 24*x_1*y_1 + 20*x_1*y_3)                                 *unit_conversion;
    dz_dxdy[2*i+0+(36*SHK_BEAM_NCELLS)] = sqrt(12) * (5*x_4 - 30*x_2*y_2 + 5*y_4)                                           *unit_conversion;
    dz_dxdy[2*i+0+(38*SHK_BEAM_NCELLS)] = sqrt(12) * (20*x_3*y_1 - 20*x_1*y_3)                                              *unit_conversion;
    dz_dxdy[2*i+0+(40*SHK_BEAM_NCELLS)] = sqrt( 7) * (120*x_5 - 120*x_3 + 240*x_3*y_2 + 24*x_1 - 120*x_1*y_2 + 120*x_1*y_4) *unit_conversion;
    dz_dxdy[2*i+0+(42*SHK_BEAM_NCELLS)] = sqrt(14) * (90*x_5 + 60*x_3*y_2 - 80*x_3 + 12*x_1 - 30*x_1*y_4)                   *unit_conversion;
    dz_dxdy[2*i+0+(44*SHK_BEAM_NCELLS)] = sqrt(14) * (150*x_4*y_1 + 180*x_2*y_3 - 120*x_2*y_1 + 12*y_1 - 40*y_3 + 30*y_5)   *unit_conversion;

    dz_dxdy[2*i+1+( 0*SHK_BEAM_NCELLS)] =      2.0 * (0)                                                                    *unit_conversion;
    dz_dxdy[2*i+1+( 2*SHK_BEAM_NCELLS)] =      2.0 * (1)                                                                    *unit_conversion;
    dz_dxdy[2*i+1+( 4*SHK_BEAM_NCELLS)] =  sqrt(3) * (4*y_1)                                                                *unit_conversion;
    dz_dxdy[2*i+1+( 6*SHK_BEAM_NCELLS)] =  sqrt(6) * (-2*y_1)                                                               *unit_conversion;
    dz_dxdy[2*i+1+( 8*SHK_BEAM_NCELLS)] =  sqrt(6) * (2*x_1)                                                                *unit_conversion;
    dz_dxdy[2*i+1+(10*SHK_BEAM_NCELLS)] =  sqrt(8) * (6*x_1*y_1)                                                            *unit_conversion;
    dz_dxdy[2*i+1+(12*SHK_BEAM_NCELLS)] =  sqrt(8) * (3*x_2 + 9*y_2 - 2)                                                    *unit_conversion;
    dz_dxdy[2*i+1+(14*SHK_BEAM_NCELLS)] =  sqrt(8) * (-6*x_1*y_1)                                                           *unit_conversion;
    dz_dxdy[2*i+1+(16*SHK_BEAM_NCELLS)] =  sqrt(8) * (3*x_2 - 3*y_2)                                                        *unit_conversion;
    dz_dxdy[2*i+1+(18*SHK_BEAM_NCELLS)] = sqrt( 5) * (24*x_2*y_1 - 12*y_1 + 24*y_3)                                         *unit_conversion;
    dz_dxdy[2*i+1+(20*SHK_BEAM_NCELLS)] = sqrt(10) * (6*y_1 - 16*y_3)                                                       *unit_conversion;
    dz_dxdy[2*i+1+(22*SHK_BEAM_NCELLS)] = sqrt(10) * (8*x_3 - 6*x_1 + 24*x_1*y_2)                                           *unit_conversion;
    dz_dxdy[2*i+1+(24*SHK_BEAM_NCELLS)] = sqrt(10) * (-12*x_2*y_1 + 4*y_3)                                                  *unit_conversion;
    dz_dxdy[2*i+1+(26*SHK_BEAM_NCELLS)] = sqrt(10) * (4*x_3 - 12*x_1*y_2)                                                   *unit_conversion;
    dz_dxdy[2*i+1+(28*SHK_BEAM_NCELLS)] = sqrt(12) * (40*x_3*y_1 -24*x_1*y_1 + 40*x_1*y_3)                                  *unit_conversion;
    dz_dxdy[2*i+1+(30*SHK_BEAM_NCELLS)] = sqrt(12) * (10*x_4 - 12*x_2 + 60*x_2*y_2 - 36*y_2 + 50*y_4 + 3)                   *unit_conversion;
    dz_dxdy[2*i+1+(32*SHK_BEAM_NCELLS)] = sqrt(12) * (-20*x_3*y_1 + 24*x_1*y_1 - 60*x_1*y_3)                                *unit_conversion;
    dz_dxdy[2*i+1+(34*SHK_BEAM_NCELLS)] = sqrt(12) * (15*x_4 - 12*x_2 + 30*x_2*y_2 + 12*y_2 - 25*y_4)                       *unit_conversion;
    dz_dxdy[2*i+1+(36*SHK_BEAM_NCELLS)] = sqrt(12) * (-20*x_3*y_1 + 20*x_1*y_3)                                             *unit_conversion;
    dz_dxdy[2*i+1+(38*SHK_BEAM_NCELLS)] = sqrt(12) * (5*x_4 - 30*x_2*y_2 + 5*y_4)                                           *unit_conversion;
    dz_dxdy[2*i+1+(40*SHK_BEAM_NCELLS)] = sqrt( 7) * (120*x_4*y_1 - 120*x_2*y_1 + 240*x_2*y_3 + 24*y_1 - 120*y_3 + 120*y_5) *unit_conversion;
    dz_dxdy[2*i+1+(42*SHK_BEAM_NCELLS)] = sqrt(14) * (30*x_4*y_1 - 60*x_2*y_3 - 12*y_1 + 80*y_3 - 90*y_5)                   *unit_conversion;
    dz_dxdy[2*i+1+(44*SHK_BEAM_NCELLS)] = sqrt(14) * (30*x_5 - 40*x_3 + 180*x_3*y_2 + 12*x_1 - 120*x_1*y_2 + 150*x_1*y_4)   *unit_conversion;

  }

  /* Write forward matrix to file */
  if(SHK_SAVE_ZMATRIX){
    //Set up file name
    sprintf(outfile, SHKZER2SHKCEL_OUTFILE);
    //Create output folder if it does not exist
    strcpy(temp,outfile);
    strcpy(path,dirname(temp));
    if(stat(path, &st) == -1){
      printf("SHK: creating folder %s\n",path);
      recursive_mkdir(path, 0777);
    }
    //Open file
    if((fd = fopen(outfile, "w")) == NULL){
      perror("SHK: zern2shk fopen");
    }
    else{
      //Write matrix
      if(fwrite(dz_dxdy,sizeof(dz_dxdy),1,fd) !=1){
	perror("SHK: zern2shk fwrite");
	fclose(fd);
      }else{
	//Close file
	fclose(fd);
	if(SHK_DEBUG) printf("SHK: Wrote Zernike matrix file: %s\n",outfile);
      }
    }
  }
  
  //Copy forward matrix to calling routine
  memcpy(matrix_fwd,dz_dxdy,sizeof(dz_dxdy));
  
  //Invert Matrix NOTE: This changes the forward matrix
  if(SHK_DEBUG) printf("SHK: Inverting the Zernike matrix\n");
  num_dgesvdi(dz_dxdy, dxdy_dz, 2*SHK_BEAM_NCELLS, LOWFS_N_ZERNIKE);
    
  //Copy inverse matrix to calling routine
  memcpy(matrix_inv,dxdy_dz,sizeof(dxdy_dz));
  
  /* Write inverse matrix to file */
  if(SHK_SAVE_ZMATRIX){
    //Set up file name
    sprintf(outfile, SHKCEL2SHKZER_OUTFILE);
    //Create output folder if it does not exist
    strcpy(temp,outfile);
    strcpy(path,dirname(temp));
    if (stat(path, &st) == -1){
      printf("SHK: creating folder %s\n",path);
      recursive_mkdir(path, 0777);
    }
    //Open file
    if((fd = fopen(outfile, "w")) == NULL){
      perror("SHK: shk2zern fopen");
    }
    else{
      //Write matrix
      if(fwrite(dxdy_dz,sizeof(dxdy_dz),1,fd) !=1){
	perror("SHK: shk2zern fwrite");
	fclose(fd);
      }
      else{
	//Close file
	fclose(fd);
	if(SHK_DEBUG) printf("SHK: Wrote zernike matrix file: %s\n",outfile);
      }
    }
  }
}

/**************************************************************/
/* SHK_ZERNIKE_OPS                                            */
/*  - Fit Zernikes to SHK centroids                           */
/*  - Set cell targets based on zernike targets               */
/**************************************************************/
void shk_zernike_ops(shkevent_t *shkevent, int fit_zernikes, int set_targets, int reset){
  static double shk2zern[2*SHK_BEAM_NCELLS*LOWFS_N_ZERNIKE]={0};
  static double zern2shk[2*SHK_BEAM_NCELLS*LOWFS_N_ZERNIKE]={0};
  double shk_xydev[2*SHK_BEAM_NCELLS]={0};
  int i;
  double surf2wave=1;
  static int init = 0;
  
  /* Initialize Fitting Matrix */
  if(!init || reset){
    //Generate the zernike matrix
    shk_zernike_matrix(shkevent->cells, zern2shk, shk2zern);
    //Set init flag
    init = 1;
    //Return if reset
    if(reset) return;
  }

  /* Zernike Fitting */
  if(fit_zernikes){
    //Format displacement array (from origin for zernike fitting)
    for(i=0;i<SHK_BEAM_NCELLS;i++){
      shk_xydev[2*i + 0] = shkevent->cells[i].xorigin_deviation;
      shk_xydev[2*i + 1] = shkevent->cells[i].yorigin_deviation;
    }
    //Do Zernike fit matrix multiply
    num_dgemv(shk2zern, shk_xydev, shkevent->zernike_measured, LOWFS_N_ZERNIKE, 2*SHK_BEAM_NCELLS);
  }
  
  /* Set Targets */
  if(set_targets){
    //Do Zernike target to cell target matrix multiply 
    num_dgemv(zern2shk, shkevent->zernike_target, shk_xydev, 2*SHK_BEAM_NCELLS, LOWFS_N_ZERNIKE);
    //Convert xydev from pixels/surface back to pixels
    if(INSTRUMENT_INPUT_TYPE == INPUT_TYPE_SINGLE_PASS) surf2wave = 2.0;
    if(INSTRUMENT_INPUT_TYPE == INPUT_TYPE_DOUBLE_PASS) surf2wave = 4.0;
    //Set cell targets 
    for(i=0;i<SHK_BEAM_NCELLS;i++){
      shkevent->cells[i].xtarget = shkevent->cells[i].xorigin + shk_xydev[2*i + 0]*surf2wave;
      shkevent->cells[i].ytarget = shkevent->cells[i].yorigin + shk_xydev[2*i + 1]*surf2wave;
    }
  }
}

/**************************************************************/
/* SHK_CELLS2ALP                                              */
/*  - Convert SHK cell commands to ALPAO DM commands          */
/**************************************************************/
void shk_cells2alp(shkcell_t *cells, double *actuators, int reset){
  FILE *matrix=NULL;
  char matrix_file[MAX_FILENAME];
  uint64 fsize,rsize;
  static int init=0;
  static double cells2alp_matrix[2*SHK_BEAM_NCELLS*ALP_NACT]={0};
  double shk_xydev[2*SHK_BEAM_NCELLS]={0};
  int i;
    
  /* Initialize */
  if(!init || reset){
    /* Open matrix file */
    //--setup filename
    sprintf(matrix_file,SHKCEL2ALPACT_FILE);
    //--open file
    if((matrix = fopen(matrix_file,"r")) == NULL){
      perror("SHK: cells2alp fopen");
      goto end_of_init;
    }
    //--check file size
    fseek(matrix, 0L, SEEK_END);
    fsize = ftell(matrix);
    rewind(matrix);
    rsize = sizeof(cells2alp_matrix);
    if(fsize != rsize){
      printf("SHK: incorrect cells2alp matrix file size %lu != %lu\n",fsize,rsize);
      fclose(matrix);
      goto end_of_init;
    }
    //--read matrix
    if(fread(cells2alp_matrix,sizeof(cells2alp_matrix),1,matrix) != 1){
      perror("SHK: cells2alp fread");
      fclose(matrix);
      goto end_of_init;
    }
    //--close file
    fclose(matrix);
    printf("SHK: Read: %s\n",matrix_file);
    
  end_of_init:
    //--set init flag
    init=1;
    //--return if reset
    if(reset) return;
  }

  //Format displacement array
  for(i=0;i<SHK_BEAM_NCELLS;i++){
    shk_xydev[2*i + 0] = cells[i].xcommand;
    shk_xydev[2*i + 1] = cells[i].ycommand;
  }

  //Do Matrix Multiply
  num_dgemv(cells2alp_matrix, shk_xydev, actuators, ALP_NACT, 2*SHK_BEAM_NCELLS);
}

/**************************************************************/
/* SHK_ALP_CELLPID                                            */
/*  - Run PID controller on centroid deviations for ALP       */
/**************************************************************/
void shk_alp_cellpid(shkevent_t *shkevent, int reset){
  static int init = 0;
  static double xint[SHK_BEAM_NCELLS] = {0};
  static double yint[SHK_BEAM_NCELLS] = {0};
  int i;
    
  //Initialize
  if(!init || reset){
    memset(xint,0,sizeof(xint));
    memset(yint,0,sizeof(yint));
    init=1;
    if(reset) return;
  }
  
  //Run PID
  for(i=0;i<SHK_BEAM_NCELLS;i++){
    //Calculate integrals
    xint[i] += shkevent->cells[i].xtarget_deviation;
    yint[i] += shkevent->cells[i].ytarget_deviation;
    //Fix windup
    if(xint[i] > SHK_ALP_CELL_INT_MAX) xint[i]=SHK_ALP_CELL_INT_MAX;
    if(xint[i] < SHK_ALP_CELL_INT_MIN) xint[i]=SHK_ALP_CELL_INT_MIN;
    if(yint[i] > SHK_ALP_CELL_INT_MAX) yint[i]=SHK_ALP_CELL_INT_MAX;
    if(yint[i] < SHK_ALP_CELL_INT_MIN) yint[i]=SHK_ALP_CELL_INT_MIN;
    //Calculate command delta
    shkevent->cells[i].xcommand = shkevent->gain_alp_cell[0] * shkevent->cells[i].xtarget_deviation + shkevent->gain_alp_cell[1] * xint[i];
    shkevent->cells[i].ycommand = shkevent->gain_alp_cell[0] * shkevent->cells[i].ytarget_deviation + shkevent->gain_alp_cell[1] * yint[i];
  }
}

/**************************************************************/
/* SHK_ALP_ZERNPID                                            */
/*  - Run PID controller on measured Zernikes for ALP         */
/**************************************************************/
void shk_alp_zernpid(shkevent_t *shkevent, double *zernike_delta, int reset){
  static int init = 0;
  static double zint[LOWFS_N_ZERNIKE] = {0};
  double zerr;
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
    zerr = shkevent->zernike_measured[i] - shkevent->zernike_target[i];
    //Calculate integral
    zint[i] += zerr;
    //Fix windup
    if(zint[i] > SHK_ALP_ZERN_INT_MAX) zint[i]=SHK_ALP_ZERN_INT_MAX;
    if(zint[i] < SHK_ALP_ZERN_INT_MIN) zint[i]=SHK_ALP_ZERN_INT_MIN;
    //Calculate command delta
    zernike_delta[i] = shkevent->gain_alp_zern[i][0] * zerr + shkevent->gain_alp_zern[i][1] * zint[i];
  }
}

/**************************************************************/
/* SHK_HEX_ZERNPID                                            */
/*  - Run PID controller on measured Zernikes for HEX         */
/**************************************************************/
void shk_hex_zernpid(shkevent_t *shkevent, double *zernike_delta, int reset){
  static int init = 0;
  static double zint[LOWFS_N_ZERNIKE] = {0};
  double zerr;
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
    zerr = shkevent->zernike_measured[i] - shkevent->zernike_target[i];
    //Calculate integral
    zint[i] += zerr;
    //Calculate command
    zernike_delta[i] = shkevent->gain_hex_zern[0] * zerr + shkevent->gain_hex_zern[1] * zint[i];
  }
}

/**************************************************************/
/* SHK_PROCESS_IMAGE                                          */
/*  - Main image processing function for SHK                  */
/**************************************************************/
void shk_process_image(stImageBuff *buffer,sm_t *sm_p, uint32 frame_number){
  static shkfull_t shkfull;
  static shkevent_t shkevent;
  static shkpkt_t shkpkt;
  shkfull_t  *shkfull_p;
  shkevent_t *shkevent_p;
  shkpkt_t   *shkpkt_p;
  static calmode_t alpcalmodes[ALP_NCALMODES];
  static calmode_t hexcalmodes[HEX_NCALMODES];
  static calmode_t bmccalmodes[BMC_NCALMODES];
  static calmode_t tgtcalmodes[TGT_NCALMODES];
  static struct timespec start,end,delta,last,full_last,hex_last;
  static int init=0;
  double dt;
  int i,j;
  uint16_t fakepx=0;
  int state;
  hex_t hex,hex_try,hex_delta;
  alp_t alp,alp_try,alp_delta;
  int zernike_control=0;
  uint32_t n_dither=1;
  int reset_zernike=0;
  int sample;
  
  //Get time immidiately
  clock_gettime(CLOCK_REALTIME,&start);

  //Get state
  state = sm_p->state;

  //Get sample
  sample = frame_number % SHK_NSAMPLES;

  //Check reset
  if(sm_p->w[SHKID].reset){
    init=0;
    sm_p->w[SHKID].reset=0;
  }

  //Initialize
  if(!init){
    //Zero out events & commands
    memset(&shkfull,0,sizeof(shkfull_t));
    memset(&shkevent,0,sizeof(shkevent_t));
    memset(&shkpkt,0,sizeof(shkpkt_t));
    //Init cells
    shk_init_cells(&shkevent);
    //Load cell origins
    shk_loadorigin(&shkevent);
    //Reset zernike matrix
    shk_zernike_ops(&shkevent,0,0,FUNCTION_RESET);
    //Reset cells2alp mapping
    shk_cells2alp(shkevent.cells,NULL,FUNCTION_RESET);
    //Reset zern2alp mapping
    alp_zern2alp(NULL,NULL,FUNCTION_RESET);
    //Reset calibration routines
    alp_calibrate(0,NULL,NULL,SHKID,FUNCTION_RESET);
    hex_calibrate(0,NULL,NULL,SHKID,FUNCTION_RESET);
    tgt_calibrate(0,NULL,NULL,SHKID,FUNCTION_RESET);
    //Reset PID controllers
    shk_alp_cellpid(NULL,FUNCTION_RESET);
    shk_alp_zernpid(NULL,NULL,FUNCTION_RESET);
    shk_hex_zernpid(NULL,NULL,FUNCTION_RESET);
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
    memcpy(&full_last,&start,sizeof(struct timespec));
    memcpy(&hex_last,&start,sizeof(struct timespec));
    memcpy(&last,&start,sizeof(struct timespec));
    //Set init flag
    init=1;
    //Debugging
    if(SHK_DEBUG) printf("SHK: Initialized\n");
  }

  //Measure exposure time
  if(timespec_subtract(&delta,&start,&last))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);

  //Save time
  memcpy(&last,&start,sizeof(struct timespec));
  
  //Fill out event header
  shkevent.hed.version      = PICC_PKT_VERSION;
  shkevent.hed.type         = BUFFER_SHKEVENT;
  shkevent.hed.frame_number = frame_number;
  shkevent.hed.exptime      = sm_p->shk_exptime;
  shkevent.hed.ontime       = dt;
  shkevent.hed.state        = state;
  shkevent.hed.start_sec    = start.tv_sec;
  shkevent.hed.start_nsec   = start.tv_nsec;

  //Save gains
  memcpy(shkevent.gain_alp_cell,(void *)sm_p->shk_gain_alp_cell,sizeof(shkevent.gain_alp_cell));
  memcpy(shkevent.gain_hex_zern,(void *)sm_p->shk_gain_hex_zern,sizeof(shkevent.gain_hex_zern));
  for(i=0;i<LOWFS_N_ZERNIKE;i++)
    for(j=0;j<LOWFS_N_PID;j++)
      shkevent.gain_alp_zern[i][j] = sm_p->shk_gain_alp_zern[i][j] * sm_p->zernike_control[i];
  
  //Save calmodes
  shkevent.hed.hex_calmode = sm_p->hex_calmode;
  shkevent.hed.alp_calmode = sm_p->alp_calmode;
  shkevent.hed.bmc_calmode = sm_p->bmc_calmode;
  shkevent.hed.tgt_calmode = sm_p->tgt_calmode;
  
  //Save zernike targets
  memcpy(shkevent.zernike_target,(void *)sm_p->shk_zernike_target,sizeof(shkevent.zernike_target));
  
  //Run target calibration
  if(sm_p->state_array[state].alp_commander == SHKID)
    if(shkevent.hed.tgt_calmode != TGT_CALMODE_NONE)
      sm_p->tgt_calmode = tgt_calibrate(shkevent.hed.tgt_calmode,shkevent.zernike_target,&shkevent.hed.tgt_calstep,SHKID,FUNCTION_NO_RESET);
  
  //Set centroid targets based on zernike targets
  shk_zernike_ops(&shkevent,0,1,FUNCTION_NO_RESET);
  
  //Set centroid boxsize based on calmode
  shkevent.boxsize = sm_p->shk_boxsize;
  if((alpcalmodes[shkevent.hed.alp_calmode].shk_boxsize_cmd == SHK_BOXSIZE_CMD_MAX) ||
     (hexcalmodes[shkevent.hed.hex_calmode].shk_boxsize_cmd == SHK_BOXSIZE_CMD_MAX) ||
     (bmccalmodes[shkevent.hed.bmc_calmode].shk_boxsize_cmd == SHK_BOXSIZE_CMD_MAX) ||
     (tgtcalmodes[shkevent.hed.tgt_calmode].shk_boxsize_cmd == SHK_BOXSIZE_CMD_MAX)){
    shkevent.boxsize = SHK_MAX_BOXSIZE;
  }
  
  //Set centroid boxsize to max in STATE_STANDBY
  if(state == STATE_STANDBY) shkevent.boxsize = SHK_MAX_BOXSIZE;
 
  //Calculate centroids
  shk_centroid(buffer->pvAddress,&shkevent);
 
  //Command: Set cell origins
  if(sm_p->shk_setorigin){
    sm_p->shk_setorigin = shk_setorigin(&shkevent);
    //Trigger zernike reset
    if(!sm_p->shk_setorigin) reset_zernike=1;
  }
  
  //Command: Revert cell origins
  if(sm_p->shk_revertorigin){
    shk_revertorigin(&shkevent);
    sm_p->shk_revertorigin = 0;
    //Trigger zernike reset
    reset_zernike=1;
  }
  
  //Command: Save cell origins
  if(sm_p->shk_saveorigin){
    shk_saveorigin(&shkevent);
    sm_p->shk_saveorigin = 0;
  }
  
  //Command: Load cell origins
  if(sm_p->shk_loadorigin){
    shk_loadorigin(&shkevent);
    sm_p->shk_loadorigin = 0;
    //Trigger zernike reset
    reset_zernike=1;
  }
  
  //Command: Shift cell x origins
  if(sm_p->shk_xshiftorigin){
    for(i=0;i<SHK_BEAM_NCELLS;i++)
      shkevent.cells[i].xorigin += sm_p->shk_xshiftorigin;
    printf("SHK: Shifted origin %d pixels in X\n",sm_p->shk_xshiftorigin);
    sm_p->shk_xshiftorigin = 0;
    //Trigger zernike reset
    reset_zernike=1;
  }
  
  //Command: Shift cell y origins
  if(sm_p->shk_yshiftorigin){
    for(i=0;i<SHK_BEAM_NCELLS;i++)
      shkevent.cells[i].yorigin += sm_p->shk_yshiftorigin;
    printf("SHK: Shifted origin %d pixels in Y\n",sm_p->shk_yshiftorigin);
    sm_p->shk_yshiftorigin = 0;
    //Trigger zernike reset
    reset_zernike=1;
  }
  
  //Reset zernike matrix if cell origins have changed
  if(reset_zernike) shk_zernike_ops(&shkevent,0,0,FUNCTION_RESET);
  
  //Fit zernikes
  if(sm_p->state_array[state].shk.fit_zernikes)
    shk_zernike_ops(&shkevent,1,0,FUNCTION_NO_RESET);
  
  /************************************************************/
  /*******************  Hexapod Control Code  *****************/
  /************************************************************/

  //Check time since last command
  if(timespec_subtract(&delta,&start,&hex_last))
    printf("SHK: shk_process_image --> timespec_subtract error!\n");
  ts2double(&delta,&dt);
  
  //Get last HEX command
  hex_get_command(sm_p,&hex);
  memcpy(&hex_try,&hex,sizeof(hex_t));
  
  //Check if we will send a command
  if((sm_p->state_array[state].hex_commander == SHKID) && sm_p->hex_ready && (dt > HEX_PERIOD)){
    
    //Check if HEX is controlling any Zernikes
    zernike_control = 0;
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      if(sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_HEX)
	zernike_control = 1;
    
    //Run Zernike control
    if(zernike_control){
      // - run Zernike PID
      shk_hex_zernpid(&shkevent, hex_delta.zcmd, FUNCTION_NO_RESET);
      
      // - zero out uncontrolled Zernikes
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(sm_p->state_array[state].shk.zernike_control[i] != ACTUATOR_HEX)
	  hex_delta.zcmd[i] = 0;

      // - convert Zernike deltas to axis deltas
      hex_zern2hex_alt(hex_delta.zcmd, hex_delta.acmd);

      // - add Zernike PID output deltas to HEX command
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_HEX)
	  hex_try.zcmd[i] += hex_delta.zcmd[i];

      // - add axis deltas to HEX command
      for(i=0;i<HEX_NAXES;i++)
	hex_try.acmd[i] += hex_delta.acmd[i];
    }

    //Run HEX calibration
    if(shkevent.hed.hex_calmode != HEX_CALMODE_NONE)
      sm_p->hex_calmode = hex_calibrate(shkevent.hed.hex_calmode,&hex_try,&shkevent.hed.hex_calstep,SHKID,FUNCTION_NO_RESET);
 
    //Send command to HEX
    if(hex_send_command(sm_p,&hex_try,SHKID)){
      // - copy command to current position
      memcpy(&hex,&hex_try,sizeof(hex_t));
    }

    //Reset time
    memcpy(&hex_last,&start,sizeof(struct timespec));
  }


  /*************************************************************/
  /*******************  ALPAO DM Control Code  *****************/
  /*************************************************************/
  
  //Get last ALP command
  alp_get_command(sm_p,&alp);
  memcpy(&alp_try,&alp,sizeof(alp_t));

  //Check if we will send a command
  if((sm_p->state_array[state].alp_commander == SHKID) && sm_p->alp_ready){
    //Check if ALP is controlling any Zernikes
    zernike_control = 0;
    for(i=0;i<LOWFS_N_ZERNIKE;i++)
      if(sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_ALP)
	zernike_control = 1;
    
    //Run Zernike control
    if(zernike_control){
      // - run Zernike PID
      shk_alp_zernpid(&shkevent, alp_delta.zcmd, FUNCTION_NO_RESET);

      // - zero out uncontrolled Zernikes
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(sm_p->state_array[state].shk.zernike_control[i] != ACTUATOR_ALP)
	  alp_delta.zcmd[i] = 0;

      // - convert zernike deltas to actuator deltas
      alp_zern2alp(alp_delta.zcmd,alp_delta.acmd,FUNCTION_NO_RESET);

      // - add Zernike PID output deltas to ALP command
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_ALP)
	  alp_try.zcmd[i] += alp_delta.zcmd[i];

      // - add actuator deltas to ALP command
      for(i=0;i<ALP_NACT;i++)
	alp_try.acmd[i] += alp_delta.acmd[i];
    }

    //Check if ALP is controlling SHK cells
    if(sm_p->state_array[state].shk.cell_control == ACTUATOR_ALP){
      // - run cell PID
      shk_alp_cellpid(&shkevent, FUNCTION_NO_RESET);

      // - convert cell commands to actuator deltas
      shk_cells2alp(shkevent.cells,alp_delta.acmd,FUNCTION_NO_RESET);
      
      // - add actuator deltas to ALP command
      for(i=0;i<ALP_NACT;i++)
	alp_try.acmd[i] += alp_delta.acmd[i];
    }

    //Calibrate ALP
    if(shkevent.hed.alp_calmode != ALP_CALMODE_NONE)
      sm_p->alp_calmode = alp_calibrate(shkevent.hed.alp_calmode,&alp_try,&shkevent.hed.alp_calstep,SHKID,FUNCTION_NO_RESET);
     
    //Send command to ALP
    if(alp_send_command(sm_p,&alp_try,SHKID,n_dither)){
      // - copy command to current position
      memcpy(&alp,&alp_try,sizeof(alp_t));
    }
  }
  
  //Copy HEX command to shkevent
  memcpy(&shkevent.hex,&hex,sizeof(hex_t));
  
  //Copy ALP command to shkevent
  memcpy(&shkevent.alp,&alp,sizeof(alp_t));

  //Get end timestamp
  clock_gettime(CLOCK_REALTIME,&end);
  shkevent.hed.end_sec  = end.tv_sec;
  shkevent.hed.end_nsec = end.tv_nsec;
  
  //Write SHKEVENT to circular buffer
  if(sm_p->write_circbuf[BUFFER_SHKEVENT]){
    
    //Open SHKEVENT circular buffer
    shkevent_p=(shkevent_t *)open_buffer(sm_p,BUFFER_SHKEVENT);
    
    //Copy data
    memcpy(shkevent_p,&shkevent,sizeof(shkevent_t));;
    
    //Get final timestamp
    clock_gettime(CLOCK_REALTIME,&end);
    shkevent_p->hed.end_sec  = end.tv_sec;
    shkevent_p->hed.end_nsec = end.tv_nsec;
    
    //Close buffer
    close_buffer(sm_p,BUFFER_SHKEVENT);
    
    //Save end timestamps for full image code
    shkevent.hed.end_sec  = end.tv_sec;
    shkevent.hed.end_nsec = end.tv_nsec;
  }

  /*************************************************************/
  /**********************  SHK Packet Code  ********************/
  /*************************************************************/
  if(sm_p->write_circbuf[BUFFER_SHKPKT]){
    //Samples collected each time through
    for(i=0;i<SHK_BEAM_NCELLS;i++){
      shkpkt.cells[i].xorigin_deviation[sample] = shkevent.cells[i].xorigin_deviation;
      shkpkt.cells[i].yorigin_deviation[sample] = shkevent.cells[i].yorigin_deviation;
      shkpkt.cells[i].xtarget_deviation[sample] = shkevent.cells[i].xtarget_deviation;
      shkpkt.cells[i].ytarget_deviation[sample] = shkevent.cells[i].ytarget_deviation;
      shkpkt.cells[i].xcommand[sample]          = shkevent.cells[i].xcommand;
      shkpkt.cells[i].ycommand[sample]          = shkevent.cells[i].ycommand;
    }
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      shkpkt.zernike_measured[i][sample]        = shkevent.zernike_measured[i];
      shkpkt.alp_zcmd[i][sample]                = shkevent.alp.zcmd[i];
    }
    for(i=0;i<ALP_NACT;i++){
      shkpkt.alp_acmd[i][sample]                = shkevent.alp.acmd[i];
    }

    //Last sample, fill out rest of packet and write to circular buffer
    if(sample == SHK_NSAMPLES-1){
      //Header
      memcpy(&shkpkt.hed,&shkevent.hed,sizeof(pkthed_t));
      //Cells
      for(i=0;i<SHK_BEAM_NCELLS;i++){
	shkpkt.cells[i].spot_found              = shkevent.cells[i].spot_found;
	shkpkt.cells[i].spot_captured           = shkevent.cells[i].spot_captured;
	shkpkt.cells[i].maxval                  = shkevent.cells[i].maxval;
	shkpkt.cells[i].boxsize                 = shkevent.cells[i].boxsize;
	shkpkt.cells[i].intensity               = shkevent.cells[i].intensity;
	shkpkt.cells[i].background              = shkevent.cells[i].background;
	shkpkt.cells[i].xorigin                 = shkevent.cells[i].xorigin;
	shkpkt.cells[i].yorigin                 = shkevent.cells[i].yorigin;
	shkpkt.cells[i].xtarget                 = shkevent.cells[i].xtarget;
	shkpkt.cells[i].ytarget                 = shkevent.cells[i].ytarget;
      }
      //Zernike items
      for(i=0;i<LOWFS_N_ZERNIKE;i++){
	shkpkt.zernike_target[i] = shkevent.zernike_target[i];
	shkpkt.hex_zcmd[i]       = shkevent.hex.zcmd[i];
	for(j=0;j<LOWFS_N_PID;j++){
	  shkpkt.gain_alp_zern[i][j] = shkevent.gain_alp_zern[i][j];
	}
      }
      //Gains
      for(i=0;i<LOWFS_N_PID;i++){
	shkpkt.gain_alp_cell[i] = shkevent.gain_alp_cell[i];
	shkpkt.gain_hex_zern[i] = shkevent.gain_hex_zern[i];
      }
      //Hex Commands
      for(i=0;i<HEX_NAXES;i++){
	shkpkt.hex_acmd[i] = shkevent.hex.acmd[i];
      }
      //Other event items
      shkpkt.boxsize = shkevent.boxsize;
      shkpkt.wsp_pcmd = shkevent.wsp.pcmd;
      shkpkt.wsp_ycmd = shkevent.wsp.ycmd;

      //Open SHKPKT circular buffer
      shkpkt_p=(shkpkt_t *)open_buffer(sm_p,BUFFER_SHKPKT);

      //Copy data
      memcpy(shkpkt_p,&shkpkt,sizeof(shkpkt_t));;
    
      //Get final timestamp
      clock_gettime(CLOCK_REALTIME,&end);
      shkpkt_p->hed.end_sec  = end.tv_sec;
      shkpkt_p->hed.end_nsec = end.tv_nsec;
    
      //Close buffer
      close_buffer(sm_p,BUFFER_SHKPKT);
    }
  }
  /*************************************************************/
  /**********************  Full Image Code  ********************/
  /*************************************************************/
  if(sm_p->write_circbuf[BUFFER_SHKFULL]){
    if(timespec_subtract(&delta,&start,&full_last))
      printf("SHK: shk_process_image --> timespec_subtract error!\n");
    ts2double(&delta,&dt);
    if(dt > SHK_FULL_IMAGE_TIME){
      //Copy packet header
      memcpy(&shkfull.hed,&shkevent.hed,sizeof(pkthed_t));
      shkfull.hed.type = BUFFER_SHKFULL;
    
      //Fake data
      if(sm_p->w[SHKID].fakemode != FAKEMODE_NONE){
	if(sm_p->w[SHKID].fakemode == FAKEMODE_TEST_PATTERN)
	  for(i=0;i<SHKXS;i++)
	    for(j=0;j<SHKYS;j++)
	      shkfull.image.data[i][j]=fakepx++;
      }
      else{
	//Copy full image
	memcpy(&(shkfull.image.data[0][0]),buffer->pvAddress,sizeof(shk_t));
      }
    
      //Copy shkevent
      memcpy(&shkfull.shkevent,&shkevent,sizeof(shkevent_t));
    
      //Open SHKFULL circular buffer
      shkfull_p=(shkfull_t *)open_buffer(sm_p,BUFFER_SHKFULL);

      //Copy data
      memcpy(shkfull_p,&shkfull,sizeof(shkfull_t));;
    
      //Get final timestamp
      clock_gettime(CLOCK_REALTIME,&end);
      shkfull_p->hed.end_sec  = end.tv_sec;
      shkfull_p->hed.end_nsec = end.tv_nsec;
    
      //Close buffer
      close_buffer(sm_p,BUFFER_SHKFULL);
    
      //Reset time
      memcpy(&full_last,&start,sizeof(struct timespec));
    }
  }
}
