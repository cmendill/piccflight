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
/* SHK_XY2INDEX                                               */
/*  - Transform x,y to buffer index                           */
/**************************************************************/
uint64 shk_xy2index(int x, int y){
  return (uint64)x + ((uint64)y)*SHKXS;
}

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
  FILE *fd=NULL;
  int i;
  
  /* Open output file */
  //--create output folder if it does not exist
  check_and_mkdir(SHK_ORIGIN_FILE);
  //--open file
  if((fd = fopen(SHK_ORIGIN_FILE, "w")) == NULL){
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
  //Close file
  fclose(fd);

  printf("SHK: Wrote: %s\n",SHK_ORIGIN_FILE);

  return;
}

/***************************************************************/
/* SHK_LOADORIGIN                                              */
/*  - Loads cell origins from file                             */
/***************************************************************/
void shk_loadorigin(shkevent_t*shkevent){
  FILE *fd=NULL;
  uint64 fsize,rsize;
  int i;
  shkcell_t cells[SHK_BEAM_NCELLS];
  
  /* Open origin file */
  if((fd = fopen(SHK_ORIGIN_FILE,"r")) == NULL){
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
  printf("SHK: Read: %s\n",SHK_ORIGIN_FILE);

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
  int    x,y,blx,bly,trx,try,boxsize;
  uint64 px;
  double wave2surf = 1;
  double xcentroid=0,ycentroid=0,xdeviation=0,ydeviation=0;
  double value;
  
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
  
  //Calculate corners of centroid box (binned coordinates)
  blx = floor((cell->xtarget - boxsize)/SHKBIN);
  bly = floor((cell->ytarget - boxsize)/SHKBIN);
  trx = floor((cell->xtarget + boxsize)/SHKBIN);
  try = floor((cell->ytarget + boxsize)/SHKBIN);
  
  //Impose limits (binned coordinates)
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
      px = shk_xy2index(x,y);
      intensity += image[px];
      if(image[px] > maxval){
	maxval = image[px];
	xcentroid = ((double)x + 0.5)*SHKBIN; //unbinned coordinates
	ycentroid = ((double)y + 0.5)*SHKBIN; //unbinned coordinates
      }
    }
  }
  
  //Check if spot is above or below deadband threshold
  if(maxval > SHK_SPOT_UPPER_THRESH){
    cell->spot_found=1;
    //Centroid will be re-calculated below
  }
  if(maxval < SHK_SPOT_LOWER_THRESH){
    //Reset values
    cell->spot_found=0;
    cell->spot_captured=0;
    cell->xcentroid = cell->xtarget;
    cell->ycentroid = cell->ytarget;
    cell->xtarget_deviation = 0;
    cell->ytarget_deviation = 0;
    cell->xorigin_deviation = 0;
    cell->yorigin_deviation = 0;
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
  
    //Calculate corners of centroid box (binned coordinates)
    blx = floor((cell->xtarget - boxsize)/SHKBIN);
    bly = floor((cell->ytarget - boxsize)/SHKBIN);
    trx = floor((cell->xtarget + boxsize)/SHKBIN);
    try = floor((cell->ytarget + boxsize)/SHKBIN);

    //Impose limits (binned coordinates)
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
	px = shk_xy2index(x,y);
	value = (double)image[px] - cell->background;
	if(value > SHK_SPOT_UPPER_THRESH){
	  xhist[x]  += value;
	  yhist[y]  += value;
	  intensity += value;
	}
      }
    }
    
    //Weight histograms
    total = 0;
    xnum  = 0;
    ynum  = 0;
    for(x=blx;x<=trx;x++){
      xnum  += ((double)x+0.5) * xhist[x]; //binned coordinates
    }
    for(y=bly;y<=try;y++){
      ynum  += ((double)y+0.5) * yhist[y]; //binned coordinates
      total += yhist[y];
    }

    //Debugging
    /*
    if(blx > 200 && blx < 230 && bly > 200 && bly < 230){
      printf("SHK: ");
      for(x=blx;x<=trx;x++)
    	printf("%4.1f ",xhist[x]);
      printf("\n");
    }
    */
    
    //Calculate centroid
    if(total > 0){
      xcentroid = (xnum/total) * SHKBIN; //unbinned coordinates
      ycentroid = (ynum/total) * SHKBIN; //unbinned coordinates
    }
    
    //Save centroids
    cell->xcentroid = xcentroid;
    cell->ycentroid = ycentroid;
    
    //Save deviations
    cell->xtarget_deviation = xcentroid - cell->xtarget;
    cell->ytarget_deviation = ycentroid - cell->ytarget;
    cell->xorigin_deviation = xcentroid - cell->xorigin;
    cell->yorigin_deviation = ycentroid - cell->yorigin;
  }
  
  //Save boxsize
  cell->boxsize = boxsize;
  
  //Save max pixel
  cell->maxval = maxval;
  
  //Save total intensity (of final centroid box)
  cell->intensity  = intensity;
  
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
  int i,j;
  uint64 px;
  int npix=0;
  double background=0;
  
  //Calculate detector background
  for(i=20/SHKBIN;i<50/SHKBIN;i++){
    for(j=20/SHKBIN;j<50/SHKBIN;j++){
      px = i + j*SHKXS;
      background += (double)image[px];
      npix++;
    }
  }
  background /= npix;

  //Init # spot found and captured
  shkevent->nspot_found = 0;
  shkevent->nspot_captured = 0;
  
  //Centroid cells
  for(i=0;i<SHK_BEAM_NCELLS;i++){
    shkevent->cells[i].background = background;
    shk_centroid_cell(image,&shkevent->cells[i],shkevent->boxsize);
    shkevent->nspot_found    += shkevent->cells[i].spot_found;
    shkevent->nspot_captured += shkevent->cells[i].spot_captured;
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
  FILE *fd=NULL;
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

    //X-Derivatives
    /* 0*/ dz_dxdy[2*i+0+( 0*SHK_BEAM_NCELLS)] =      2.0 * (1)                                                                    *unit_conversion;
    /* 1*/ dz_dxdy[2*i+0+( 2*SHK_BEAM_NCELLS)] =      2.0 * (0)                                                                    *unit_conversion;
    /* 2*/ dz_dxdy[2*i+0+( 4*SHK_BEAM_NCELLS)] =  sqrt(3) * (4*x_1)                                                                *unit_conversion;
    /* 3*/ dz_dxdy[2*i+0+( 6*SHK_BEAM_NCELLS)] =  sqrt(6) * (2*y_1)                                                                *unit_conversion;
    /* 4*/ dz_dxdy[2*i+0+( 8*SHK_BEAM_NCELLS)] =  sqrt(6) * (2*x_1)                                                                *unit_conversion;
    /* 5*/ dz_dxdy[2*i+0+(10*SHK_BEAM_NCELLS)] =  sqrt(8) * (6*x_1*y_1)                                                            *unit_conversion;
    /* 6*/ dz_dxdy[2*i+0+(12*SHK_BEAM_NCELLS)] =  sqrt(8) * (9*x_2 + 3*y_2 - 2)                                                    *unit_conversion;
    /* 7*/ dz_dxdy[2*i+0+(14*SHK_BEAM_NCELLS)] =  sqrt(8) * (6*x_1*y_1)                                                            *unit_conversion;
    /* 8*/ dz_dxdy[2*i+0+(16*SHK_BEAM_NCELLS)] =  sqrt(8) * (3*x_2 - 3*y_2)                                                        *unit_conversion;
    /* 9*/ dz_dxdy[2*i+0+(18*SHK_BEAM_NCELLS)] =  sqrt(5) * (24*x_3 - 12*x_1 + 24*x_1*y_2)                                         *unit_conversion;
    /*10*/ dz_dxdy[2*i+0+(20*SHK_BEAM_NCELLS)] = sqrt(10) * (16*x_3 - 6*x_1)                                                       *unit_conversion;
    /*11*/ dz_dxdy[2*i+0+(22*SHK_BEAM_NCELLS)] = sqrt(10) * (24*x_2*y_1 - 6*y_1 + 8*y_3)                                           *unit_conversion;
    /*12*/ dz_dxdy[2*i+0+(24*SHK_BEAM_NCELLS)] = sqrt(10) * (4*x_3 - 12*x_1*y_2)                                                   *unit_conversion;
    /*13*/ dz_dxdy[2*i+0+(26*SHK_BEAM_NCELLS)] = sqrt(10) * (12*x_2*y_1 - 4*y_3)                                                   *unit_conversion;
    /*14*/ dz_dxdy[2*i+0+(28*SHK_BEAM_NCELLS)] = sqrt(12) * (50*x_4 - 36*x_2 + 60*x_2*y_2 - 12*y_2 + 10*y_4 + 3)                   *unit_conversion;
    /*15*/ dz_dxdy[2*i+0+(30*SHK_BEAM_NCELLS)] = sqrt(12) * (40*x_3*y_1 - 24*x_1*y_1 + 40*x_1*y_3)                                 *unit_conversion;
    /*16*/ dz_dxdy[2*i+0+(32*SHK_BEAM_NCELLS)] = sqrt(12) * (25*x_4 - 12*x_2 - 30*x_2*y_2 + 12*y_2 - 15*y_4)                       *unit_conversion;
    /*17*/ dz_dxdy[2*i+0+(34*SHK_BEAM_NCELLS)] = sqrt(12) * (60*x_3*y_1 - 24*x_1*y_1 + 20*x_1*y_3)                                 *unit_conversion;
    /*18*/ dz_dxdy[2*i+0+(36*SHK_BEAM_NCELLS)] = sqrt(12) * (5*x_4 - 30*x_2*y_2 + 5*y_4)                                           *unit_conversion;
    /*19*/ dz_dxdy[2*i+0+(38*SHK_BEAM_NCELLS)] = sqrt(12) * (20*x_3*y_1 - 20*x_1*y_3)                                              *unit_conversion;
    /*20*/ dz_dxdy[2*i+0+(40*SHK_BEAM_NCELLS)] = sqrt( 7) * (120*x_5 - 120*x_3 + 240*x_3*y_2 + 24*x_1 - 120*x_1*y_2 + 120*x_1*y_4) *unit_conversion;
    /*21*/ dz_dxdy[2*i+0+(42*SHK_BEAM_NCELLS)] = sqrt(14) * (150*x_4*y_1 + 180*x_2*y_3 - 120*x_2*y_1 + 12*y_1 - 40*y_3 + 30*y_5)   *unit_conversion;
    /*22*/ dz_dxdy[2*i+0+(44*SHK_BEAM_NCELLS)] = sqrt(14) * (90*x_5 + 60*x_3*y_2 - 80*x_3 + 12*x_1 - 30*x_1*y_4)                   *unit_conversion;

    //Y-Derivatives
    /* 0*/ dz_dxdy[2*i+1+( 0*SHK_BEAM_NCELLS)] =      2.0 * (0)                                                                    *unit_conversion;
    /* 1*/ dz_dxdy[2*i+1+( 2*SHK_BEAM_NCELLS)] =      2.0 * (1)                                                                    *unit_conversion;
    /* 2*/ dz_dxdy[2*i+1+( 4*SHK_BEAM_NCELLS)] =  sqrt(3) * (4*y_1)                                                                *unit_conversion;
    /* 3*/ dz_dxdy[2*i+1+( 6*SHK_BEAM_NCELLS)] =  sqrt(6) * (2*x_1)                                                                *unit_conversion;
    /* 4*/ dz_dxdy[2*i+1+( 8*SHK_BEAM_NCELLS)] =  sqrt(6) * (-2*y_1)                                                               *unit_conversion;
    /* 5*/ dz_dxdy[2*i+1+(10*SHK_BEAM_NCELLS)] =  sqrt(8) * (3*x_2 + 9*y_2 - 2)                                                    *unit_conversion;
    /* 6*/ dz_dxdy[2*i+1+(12*SHK_BEAM_NCELLS)] =  sqrt(8) * (6*x_1*y_1)                                                            *unit_conversion;
    /* 7*/ dz_dxdy[2*i+1+(14*SHK_BEAM_NCELLS)] =  sqrt(8) * (3*x_2 - 3*y_2)                                                        *unit_conversion;
    /* 8*/ dz_dxdy[2*i+1+(16*SHK_BEAM_NCELLS)] =  sqrt(8) * (-6*x_1*y_1)                                                           *unit_conversion;
    /* 9*/ dz_dxdy[2*i+1+(18*SHK_BEAM_NCELLS)] =  sqrt(5) * (24*x_2*y_1 - 12*y_1 + 24*y_3)                                         *unit_conversion;
    /*10*/ dz_dxdy[2*i+1+(20*SHK_BEAM_NCELLS)] = sqrt(10) * (6*y_1 - 16*y_3)                                                       *unit_conversion;
    /*11*/ dz_dxdy[2*i+1+(22*SHK_BEAM_NCELLS)] = sqrt(10) * (8*x_3 - 6*x_1 + 24*x_1*y_2)                                           *unit_conversion;
    /*12*/ dz_dxdy[2*i+1+(24*SHK_BEAM_NCELLS)] = sqrt(10) * (-12*x_2*y_1 + 4*y_3)                                                  *unit_conversion;
    /*13*/ dz_dxdy[2*i+1+(26*SHK_BEAM_NCELLS)] = sqrt(10) * (4*x_3 - 12*x_1*y_2)                                                   *unit_conversion;
    /*14*/ dz_dxdy[2*i+1+(28*SHK_BEAM_NCELLS)] = sqrt(12) * (40*x_3*y_1 -24*x_1*y_1 + 40*x_1*y_3)                                  *unit_conversion;
    /*15*/ dz_dxdy[2*i+1+(30*SHK_BEAM_NCELLS)] = sqrt(12) * (10*x_4 - 12*x_2 + 60*x_2*y_2 - 36*y_2 + 50*y_4 + 3)                   *unit_conversion;
    /*16*/ dz_dxdy[2*i+1+(32*SHK_BEAM_NCELLS)] = sqrt(12) * (-20*x_3*y_1 + 24*x_1*y_1 - 60*x_1*y_3)                                *unit_conversion;
    /*17*/ dz_dxdy[2*i+1+(34*SHK_BEAM_NCELLS)] = sqrt(12) * (15*x_4 - 12*x_2 + 30*x_2*y_2 + 12*y_2 - 25*y_4)                       *unit_conversion;
    /*18*/ dz_dxdy[2*i+1+(36*SHK_BEAM_NCELLS)] = sqrt(12) * (-20*x_3*y_1 + 20*x_1*y_3)                                             *unit_conversion;
    /*19*/ dz_dxdy[2*i+1+(38*SHK_BEAM_NCELLS)] = sqrt(12) * (5*x_4 - 30*x_2*y_2 + 5*y_4)                                           *unit_conversion;
    /*20*/ dz_dxdy[2*i+1+(40*SHK_BEAM_NCELLS)] = sqrt( 7) * (120*x_4*y_1 - 120*x_2*y_1 + 240*x_2*y_3 + 24*y_1 - 120*y_3 + 120*y_5) *unit_conversion;
    /*21*/ dz_dxdy[2*i+1+(42*SHK_BEAM_NCELLS)] = sqrt(14) * (30*x_5 - 40*x_3 + 180*x_3*y_2 + 12*x_1 - 120*x_1*y_2 + 150*x_1*y_4)   *unit_conversion;
    /*22*/ dz_dxdy[2*i+1+(44*SHK_BEAM_NCELLS)] = sqrt(14) * (30*x_4*y_1 - 60*x_2*y_3 - 12*y_1 + 80*y_3 - 90*y_5)                   *unit_conversion;

  }

  /* Write forward matrix to file */
  if(SHK_SAVE_ZMATRIX){
    if(write_file(SHKZER2SHKCEL_OUTFILE,dz_dxdy,sizeof(dz_dxdy))){
      printf("SHK: ERROR: Could not write Zernike matrix file: %s\n",SHKZER2SHKCEL_OUTFILE);
    }
    else{
      if(SHK_DEBUG) printf("SHK: Wrote Zernike matrix file: %s\n",SHKZER2SHKCEL_OUTFILE);
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
    if(write_file(SHKCEL2SHKZER_OUTFILE,dxdy_dz,sizeof(dxdy_dz))){
      printf("SHK: ERROR: Could not write Zernike matrix file: %s\n",SHKCEL2SHKZER_OUTFILE);
    }
    else{
      if(SHK_DEBUG) printf("SHK: Wrote Zernike matrix file: %s\n",SHKCEL2SHKZER_OUTFILE);
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
      if(shkevent->cells[i].spot_found){
	shk_xydev[2*i + 0] = shkevent->cells[i].xorigin_deviation;
	shk_xydev[2*i + 1] = shkevent->cells[i].yorigin_deviation;
      }
    }
    //Do Zernike fit matrix multiply
    if(shkevent->nspot_found >= SHK_ZFIT_MIN_CELLS){
      num_dgemv(shk2zern, shk_xydev, shkevent->zernike_measured, LOWFS_N_ZERNIKE, 2*SHK_BEAM_NCELLS);
    }else{
      //Zero out fit values
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	shkevent->zernike_measured[i]=0;
    }
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
  static int init=0;
  static double cells2alp_matrix[2*SHK_BEAM_NCELLS*ALP_NACT]={0};
  double shk_xydev[2*SHK_BEAM_NCELLS]={0};
  int i;
    
  /* Initialize */
  if(!init || reset){
    //Read matrix file
    if(read_file(SHKCEL2ALPACT_FILE,cells2alp_matrix,sizeof(cells2alp_matrix)))
      memset(cells2alp_matrix,0,sizeof(cells2alp_matrix));
    else
      printf("SHK: Read: %s\n",SHKCEL2ALPACT_FILE);
    
    //Set init flag
    init=1;
    //Return if reset
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
void shk_alp_cellpid(shkevent_t *shkevent, int pid_type, int reset){
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
    if(pid_type == PID_DOUBLE_INTEGRATOR){
      if(xint[i] > SHK_ALP_CELL_INT_MAX) xint[i]=SHK_ALP_CELL_INT_MAX;
      if(xint[i] < SHK_ALP_CELL_INT_MIN) xint[i]=SHK_ALP_CELL_INT_MIN;
      if(yint[i] > SHK_ALP_CELL_INT_MAX) yint[i]=SHK_ALP_CELL_INT_MAX;
      if(yint[i] < SHK_ALP_CELL_INT_MIN) yint[i]=SHK_ALP_CELL_INT_MIN;
    }
    //Calculate command delta
    shkevent->cells[i].xcommand = shkevent->gain_alp_cell[0] * shkevent->cells[i].xtarget_deviation + shkevent->gain_alp_cell[1] * xint[i];
    shkevent->cells[i].ycommand = shkevent->gain_alp_cell[0] * shkevent->cells[i].ytarget_deviation + shkevent->gain_alp_cell[1] * yint[i];
    //Zero command and integrators if spot not found
    if(!shkevent->cells[i].spot_found){
      shkevent->cells[i].xcommand = 0;
      shkevent->cells[i].ycommand = 0;
      xint[i] = 0;
      yint[i] = 0;
    }
  }
}

/**************************************************************/
/* SHK_ALP_ZERNPID                                            */
/*  - Run PID controller on measured Zernikes for ALP         */
/**************************************************************/
void shk_alp_zernpid(shkevent_t *shkevent, double *zernike_delta, int *zernike_switch, int pid_type, int reset){
  static int init = 0;
  static double zint[LOWFS_N_ZERNIKE] = {0};
  double zerr;
  int i,nfound=0;

  //Initialize
  if(!init || reset){
    memset(zint,0,sizeof(zint));
    init=1;
    if(reset) return;
  }
   
  //Run PID
  if(shkevent->nspot_found >= SHK_ZFIT_MIN_CELLS){
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      if(zernike_switch[i]){
	//Calculate error
	zerr = shkevent->zernike_measured[i] - shkevent->zernike_target[i];
	//Calculate integral
	zint[i] += zerr;
	//Fix windup
	if(pid_type == PID_DOUBLE_INTEGRATOR){
	  if(zint[i] > SHK_ALP_ZERN_INT_MAX) zint[i]=SHK_ALP_ZERN_INT_MAX;
	  if(zint[i] < SHK_ALP_ZERN_INT_MIN) zint[i]=SHK_ALP_ZERN_INT_MIN;
	}
	//Calculate command delta
	zernike_delta[i] = shkevent->gain_alp_zern[i][0] * zerr + shkevent->gain_alp_zern[i][1] * zint[i];
      }
    }
  }else{
    //Clear integrators and deltas
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      zernike_delta[i] = 0;
      zint[i] = 0;
    }
  }
}

/**************************************************************/
/* SHK_HEX_ZERNPID                                            */
/*  - Run PID controller on measured Zernikes for HEX         */
/**************************************************************/
void shk_hex_zernpid(shkevent_t *shkevent, double *zernike_delta, int *zernike_switch, int *offload_switch, int reset){
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
    //Offload ALP to HEX
    if(offload_switch[i]){
      zernike_delta[i] = -1.0 * shkevent->alp.zcmd[i] * shkevent->gain_hex_zern[0];
    }
    //Regular zernike control
    if(zernike_switch[i]){
      //Calculate error
      zerr = shkevent->zernike_measured[i] - shkevent->zernike_target[i];
      //Calculate integral
      zint[i] += zerr;
      //Calculate command
      zernike_delta[i] = shkevent->gain_hex_zern[0] * zerr + shkevent->gain_hex_zern[1] * zint[i];
    }
  }
}

/**************************************************************/
/* SHK_PROCESS_IMAGE                                          */
/*  - Main image processing function for SHK                  */
/**************************************************************/
int shk_process_image(stImageBuff *buffer,sm_t *sm_p){
  static shkfull_t shkfull;
  static shkevent_t shkevent;
  static shkpkt_t shkpkt;
  static calmode_t alpcalmodes[ALP_NCALMODES];
  static calmode_t hexcalmodes[HEX_NCALMODES];
  static calmode_t bmccalmodes[BMC_NCALMODES];
  static calmode_t tgtcalmodes[TGT_NCALMODES];
  static struct timespec start,end,delta,last,full_last,hex_last,pkt_last;
  static uint32 frame_number=0,sample=0;
  static int init=0;
  double dt;
  int i,j;
  uint16_t fakepx=0;
  int state;
  hex_t hex,hex_try,hex_delta;
  alp_t alp,alp_try,alp_delta={};
  static alp_t alp_first;
  static int pid_single_init=0;
  int zernike_control=0;
  int zernike_switch[LOWFS_N_ZERNIKE] = {0};
  int offload_switch[LOWFS_N_ZERNIKE] = {0};
  uint32_t n_dither=1;
  int reset_zernike=0;
  uint8 *image = (uint8 *)buffer->pvAddress;
    
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
    memset(&shkpkt,0,sizeof(shkpkt_t));
    //Init frame number & sample
    frame_number=0;
    sample=0;
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
    alp_alp2zern(NULL,NULL,FUNCTION_RESET);
    //Reset calibration routines
    alp_calibrate(sm_p,0,NULL,NULL,NULL,SHKID,FUNCTION_RESET);
    hex_calibrate(0,NULL,NULL,SHKID,FUNCTION_RESET);
    tgt_calibrate(sm_p,0,NULL,NULL,SHKID,FUNCTION_RESET);
    //Reset PID controllers
    shk_alp_cellpid(NULL,0,FUNCTION_RESET);
    shk_alp_zernpid(NULL,NULL,NULL,0,FUNCTION_RESET);
    shk_hex_zernpid(NULL,NULL,NULL,NULL,FUNCTION_RESET);
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
    memcpy(&pkt_last,&start,sizeof(struct timespec));
    memcpy(&last,&start,sizeof(struct timespec));
    //Init single integrator PID
    pid_single_init=0;
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
  shkevent.hed.version       = PICC_PKT_VERSION;
  shkevent.hed.type          = BUFFER_SHKEVENT;
  shkevent.hed.frame_number  = frame_number;
  shkevent.hed.exptime       = sm_p->shk_exptime;
  shkevent.hed.frmtime       = sm_p->shk_frmtime;
  shkevent.hed.ontime        = dt;
  shkevent.hed.state         = state;
  shkevent.hed.alp_commander = sm_p->state_array[state].alp_commander;
  shkevent.hed.hex_commander = sm_p->state_array[state].hex_commander;
  shkevent.hed.bmc_commander = sm_p->state_array[state].bmc_commander;
  shkevent.hed.start_sec     = start.tv_sec;
  shkevent.hed.start_nsec    = start.tv_nsec;
  
  //Increment frame_number
  frame_number++;

  //Save temperature
  shkevent.ccd_temp         = sm_p->shk_ccd_temp;

  //Save gains
  memcpy(shkevent.gain_alp_cell,(void *)sm_p->shk_gain_alp_cell,sizeof(shkevent.gain_alp_cell));
  memcpy(shkevent.gain_hex_zern,(void *)sm_p->shk_gain_hex_zern,sizeof(shkevent.gain_hex_zern));
  for(i=0;i<LOWFS_N_ZERNIKE;i++)
    for(j=0;j<LOWFS_N_PID;j++)
      shkevent.gain_alp_zern[i][j] = sm_p->shk_gain_alp_zern[i][j] * sm_p->shk_zernike_control[i];
  
  //Save calmodes
  shkevent.hed.hex_calmode = sm_p->hex_calmode;
  shkevent.hed.alp_calmode = sm_p->alp_calmode;
  shkevent.hed.bmc_calmode = sm_p->bmc_calmode;
  shkevent.hed.tgt_calmode = sm_p->tgt_calmode;
  
  //Save zernike targets
  memcpy(shkevent.zernike_target,(void *)sm_p->shk_zernike_target,sizeof(shkevent.zernike_target));

  //Run target calibration
  if(sm_p->state_array[state].tgt_commander == SHKID)
    if(shkevent.hed.tgt_calmode != TGT_CALMODE_NONE)
      sm_p->tgt_calmode = tgt_calibrate(sm_p,shkevent.hed.tgt_calmode,shkevent.zernike_target,&shkevent.hed.tgt_calstep,SHKID,FUNCTION_NO_RESET);
  
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
  
  //Get HEX command for user control
  if(sm_p->state_array[state].hex_commander == WATID){
    if(hex_get_command(sm_p,&hex)){
      //Skip this image
      return 0;
    }
  }
  
  //Check if we will send a command
  if((sm_p->state_array[state].hex_commander == SHKID) && sm_p->hex_ready){
    
    //Get last HEX command -- everytime through for event packets
    if(hex_get_command(sm_p,&hex)){
      //Skip this image
      return 0;
    }
    memcpy(&hex_try,&hex,sizeof(hex_t));
    
    //Check time
    if(dt > HEX_PERIOD){
      
      //Check if HEX is controlling any Zernikes
      zernike_control = 0;
      memset(zernike_switch,0,sizeof(zernike_switch));
      memset(offload_switch,0,sizeof(offload_switch));
      for(i=0;i<LOWFS_N_ZERNIKE;i++){
	if(sm_p->shk_zernike_control[i] && sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_HEX){
	  zernike_switch[i] = 1;
	  zernike_control = 1;
	  shkpkt.zernike_control[i] = ACTUATOR_HEX;
	}
	if(sm_p->shk_zernike_control[i] && sm_p->state_array[state].shk.alp_zernike_offload[i] == ACTUATOR_HEX){
	  offload_switch[i] = 1;
	  zernike_control = 1;
	}
      }
      
      //Run Zernike control
      if(zernike_control){
	// - run Zernike PID
	shk_hex_zernpid(&shkevent, hex_delta.zcmd, zernike_switch, offload_switch, FUNCTION_NO_RESET);
      
	// - zero out uncontrolled Zernikes
	for(i=0;i<LOWFS_N_ZERNIKE;i++)
	  if(!zernike_switch[i] && !offload_switch[i])
	    hex_delta.zcmd[i] = 0;
	
	// - convert Zernike deltas to axis deltas
	hex_zern2hex_alt(hex_delta.zcmd, hex_delta.acmd);
	
	// - add Zernike PID output deltas to HEX command
	for(i=0;i<LOWFS_N_ZERNIKE;i++)
	  if(zernike_switch[i] || offload_switch[i])
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
	// - command failed
	// - do nothing for now
      }else{
	// - copy command to current position
	memcpy(&hex,&hex_try,sizeof(hex_t));
	printf("HEX: {%f,%f,%f,%f,%f,%f}\n",hex.acmd[0],hex.acmd[1],hex.acmd[2],hex.acmd[3],hex.acmd[4],hex.acmd[5]);
      }
      
      //Reset time
      memcpy(&hex_last,&start,sizeof(struct timespec));
    }
  }


  /*************************************************************/
  /*******************  ALPAO DM Control Code  *****************/
  /*************************************************************/
  
  //Get ALP command for user control
  if(sm_p->state_array[state].alp_commander == WATID){
    if(alp_get_command(sm_p,&alp)){
      //Skip this image
      return 0;
    }
  }
  
  //Check if we will send a command or use shk2lyt
  if(((sm_p->state_array[state].alp_commander == SHKID) || sm_p->state_array[state].shk.shk2lyt) && sm_p->alp_ready){
    
    //Get last ALP command
    if(alp_get_command(sm_p,&alp)){
      //Skip this image
      return 0;
    }
    memcpy(&alp_try,&alp,sizeof(alp_t));

    //Check if ALP is controlling any Zernikes
    zernike_control = 0;
    memset(zernike_switch,0,sizeof(zernike_switch));
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      if(sm_p->shk_zernike_control[i] && sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_ALP){
	zernike_switch[i] = 1;
	zernike_control = 1;
	shkpkt.zernike_control[i] = ACTUATOR_ALP;
      }
    }
    
    //Run Zernike control
    if(zernike_control){
      // - run Zernike PID
      shk_alp_zernpid(&shkevent, alp_delta.zcmd, zernike_switch, sm_p->shk_alp_pid_type, FUNCTION_NO_RESET);

      // - zero out uncontrolled Zernikes
      for(i=0;i<LOWFS_N_ZERNIKE;i++)
	if(!zernike_switch[i])
	  alp_delta.zcmd[i] = 0;

      // - convert zernike deltas to actuator deltas
      alp_zern2alp(alp_delta.zcmd,alp_delta.acmd,FUNCTION_NO_RESET);
    }

    //Check if ALP is controlling SHK cells
    if(sm_p->state_array[state].shk.cell_control == ACTUATOR_ALP){
      // - run cell PID
      shk_alp_cellpid(&shkevent, sm_p->shk_alp_pid_type,FUNCTION_NO_RESET);

      // - convert cell commands to actuator deltas
      shk_cells2alp(shkevent.cells,alp_delta.acmd,FUNCTION_NO_RESET);
      
      // - convert actuator deltas to zernike deltas
      alp_alp2zern(alp_delta.acmd,alp_delta.zcmd,FUNCTION_NO_RESET);
    }

    //Check if SHK is actually sending a command
    if(sm_p->state_array[state].alp_commander == SHKID){
      
      //Apply command according to PID type
      if(sm_p->shk_alp_pid_type == PID_SINGLE_INTEGRATOR){
	// - Initialize command
	if(!pid_single_init){
	  memcpy(&alp_first,&alp,sizeof(alp_t));
	  pid_single_init=1;
	  printf("SHK: PID Single Init\n");
	}
	// - SET Zernike PID output deltas to ALP command
	for(i=0;i<LOWFS_N_ZERNIKE;i++)
	  if(zernike_switch[i])
	    alp_try.zcmd[i] = alp_first.zcmd[i] + alp_delta.zcmd[i];
      
	// - SET actuator deltas to ALP command
	for(i=0;i<ALP_NACT;i++)
	  alp_try.acmd[i] = alp_first.acmd[i] + alp_delta.acmd[i];
      }
      if(sm_p->shk_alp_pid_type == PID_DOUBLE_INTEGRATOR){
	// - ADD Zernike PID output deltas to ALP command
	for(i=0;i<LOWFS_N_ZERNIKE;i++)
	  if(zernike_switch[i])
	    alp_try.zcmd[i] += alp_delta.zcmd[i];
	
	// - ADD actuator deltas to ALP command
	for(i=0;i<ALP_NACT;i++)
	  alp_try.acmd[i] += alp_delta.acmd[i];
      }

      //Calibrate ALP
      if(shkevent.hed.alp_calmode != ALP_CALMODE_NONE)
	sm_p->alp_calmode = alp_calibrate(sm_p,shkevent.hed.alp_calmode,&alp_try,&shkevent.hed.alp_calstep,shkevent.zernike_calibrate,SHKID,FUNCTION_NO_RESET);
      
      //Send command to LYT
      if(sm_p->state_array[state].shk.shk2lyt){
	if(alp_set_shk2lyt(sm_p,&alp_try)){
	  //error, do nothing
	}
      }
      
      //Send command to ALP
      if(sm_p->state_array[state].alp_commander == SHKID){
	if(alp_send_command(sm_p,&alp_try,SHKID,n_dither)==0){
	  // - copy command to current position
	  memcpy(&alp,&alp_try,sizeof(alp_t));
	}
      }
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
  if(sm_p->circbuf[BUFFER_SHKEVENT].write)
    write_to_buffer(sm_p,&shkevent,BUFFER_SHKEVENT);
  
  /*************************************************************/
  /**********************  SHK Packet Code  ********************/
  /*************************************************************/
  if(sm_p->circbuf[BUFFER_SHKPKT].write){
    //Samples, collected each time through
    for(i=0;i<SHK_BEAM_NCELLS;i++){
      shkpkt.cells[i].xtarget_deviation[sample] = shkevent.cells[i].xtarget_deviation;
      shkpkt.cells[i].ytarget_deviation[sample] = shkevent.cells[i].ytarget_deviation;
      shkpkt.cells[i].xcommand[sample]          = shkevent.cells[i].xcommand;
      shkpkt.cells[i].ycommand[sample]          = shkevent.cells[i].ycommand;
    }
    for(i=0;i<LOWFS_N_ZERNIKE;i++){
      shkpkt.zernike_measured[i][sample]        = shkevent.zernike_measured[i];
      shkpkt.alp_zcmd[i][sample]                = shkevent.alp.zcmd[i];
    }

    //Increment sample counter
    sample++;

    //Get time since last packet write
    if(timespec_subtract(&delta,&start,&pkt_last))
      printf("SHK: shk_process_image --> timespec_subtract error!\n");
    ts2double(&delta,&dt);
        
    //Last sample, fill out rest of packet and write to circular buffer
    if((sample == SHK_NSAMPLES) || (dt > SHK_SHKPKT_TIME)){
      //Header
      memcpy(&shkpkt.hed,&shkevent.hed,sizeof(pkthed_t));
      shkpkt.hed.type = BUFFER_SHKPKT;

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

      //Set SHKPKT zernike control flags
      for(i=0;i<LOWFS_N_ZERNIKE;i++){
	shkpkt.zernike_control[i] = 0;
	if(sm_p->shk_zernike_control[i] && sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_HEX)
	  shkpkt.zernike_control[i] = ACTUATOR_HEX;
	if(sm_p->shk_zernike_control[i] && sm_p->state_array[state].shk.zernike_control[i] == ACTUATOR_ALP)
	  shkpkt.zernike_control[i] = ACTUATOR_ALP;
	if(sm_p->state_array[state].shk.cell_control == ACTUATOR_ALP)
	  shkpkt.zernike_control[i] = ACTUATOR_ALP;
      }
      
      //Actuator commands
      for(i=0;i<ALP_NACT;i++){
	shkpkt.alp_acmd[i] = shkevent.alp.acmd[i];
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
      
      //CCD Temp
      shkpkt.ccd_temp = shkevent.ccd_temp;
      
      //Number of samples
      shkpkt.nsamples = sample;

      //Write SHKPKT to circular buffer
      write_to_buffer(sm_p,&shkpkt,BUFFER_SHKPKT);
      
      //Reset time
      memcpy(&pkt_last,&start,sizeof(struct timespec));
      
      //Reset sample counter
      sample = 0;
    }
  }
  
  /*************************************************************/
  /**********************  Full Image Code  ********************/
  /*************************************************************/
  if(sm_p->circbuf[BUFFER_SHKFULL].write){
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
	if(sm_p->w[SHKID].fakemode == FAKEMODE_IMREG){
	  memset(&shkfull.image,0,sizeof(shkfull.image));
	  shkfull.image.data[CAM_IMREG_X][CAM_IMREG_Y] = 1;
	}
      }
      else{
	//Copy full image
	for(i=0;i<SHKXS;i++)
	  for(j=0;j<SHKYS;j++)
	    shkfull.image.data[i][j] = image[shk_xy2index(i,j)];
      }
      
      //Copy shkevent
      memcpy(&shkfull.shkevent,&shkevent,sizeof(shkevent_t));

      //Write SHKFULL to circular buffer
      write_to_buffer(sm_p,&shkfull,BUFFER_SHKFULL);

      //Reset time
      memcpy(&full_last,&start,sizeof(struct timespec));
    }
  }

  return 0;
}
