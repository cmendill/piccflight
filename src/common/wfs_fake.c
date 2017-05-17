#include "../hed/user_header.h"
#include "../common/controller.h"
#include "../common/common_functions.h"

#include "../hed/gasdev.h"
#include "../hed/nrutil.h"
#include "../hed/nrutil.h"
#include "../common/pmath.h"
#include "../hed/wfs_fake.h"
/********************************************************/
/* Ben's code translation:                              */
/* IMAGE_SIZE_X -> WFSXS                                */
/* IMAGE_SIZE_Y -> WFSXS                                */
/* MAP -> cal_t                                         */
/* IMAGE -> wfs_t                                       */
/* ABCD -> frame_t                                      */
/* ******************************************************/

/* functions */

/* this makes a simple pupil mask function */

void makePupil(float xc, float yc, float radius, float cobs, wfs_t *pupil){
  
  int i,j;
  double  dist;
  
    for(i=0;i<WFSXS;i++)
    {
      for(j=0;j<WFSXS;j++)
	{
	  dist = sqrt(pow(i-xc,2) + pow(j-yc,2));
	  
	  
	  if(dist < radius && dist >= cobs)
	    pupil->data[i][j] = 1;
	  else
	    pupil->data[i][j] = 0;
  
	  if((i-(int)xc==j-(int)yc) ||
	     (i-(int)xc==j-(int)yc+1) ||
	     (i-(int)xc==j-(int)yc-1)) pupil->data[i][j] = 0;
	  if((i-(int)xc==WFSXS-j-(int)yc) ||
	     (i-(int)xc==WFSXS-j-(int)yc+1) ||
	     (i-(int)xc==WFSXS-j-(int)yc-1)) pupil->data[i][j] = 0;
	}      
    }
  
}

void makeKenny(float xc, float yc, float radius, float cobs, wfs_t *pupil){
  /* Read in saved pupil mask */
  int i,j;
  int fd;
  char   filename[MAX_FILENAME];
  sprintf(filename,FAKEFILE_WFS,WFSYS,"m");
  fd = open(filename,O_RDONLY);
  if(fd < 0){
    printf("CTU: error opening: %s\n",filename);
    close(fd);
  }
  else{
    read(fd,pupil,sizeof(wfs_t));    
    close(fd);
  }
  
  /* 
     for(i=0;i<WFSXS;i++)
     for(j=0;j<WFSXS;j++)
     pupil->data[i][j]=1;
  */
  /*
  wfs_t left_pupil, right_pupil;
  int i,j;
  makePupil(xc - (int)(cobs*0.8), yc, radius, cobs,  &left_pupil);
  makePupil(xc + (int)(cobs*0.8), yc, radius, cobs, &right_pupil);
  for(i=0;i<WFSXS;i++)
    for(j=0;j<WFSXS;j++)
      if(left_pupil.data[i][j] == 1 && right_pupil.data[i][j] == 1)
	pupil->data[i][j] = 1;
      else
	pupil->data[i][j] = 0;
  */
}


/* make an array with distances from the center */
/* utility function for the noise map function */

void calculateDistance(wfs_t *dist){
  int i,j,f,g;

  for(i=0;i<WFSXS;i++)
    {
      if(i <= WFSXS/2 )
	f = i;
      else
	f = WFSXS-i;

      for(j=0;j<WFSXS;j++)
	{
	  if(j <= WFSXS/2 )
	    g = j;
	  else
	    g = WFSXS-j;
	  
	  dist->data[i][j] = sqrt(f*f + g*g);

	}
    }
}

/* make noise map */
/* use for testing/simulation, not flight */

/* p is the amplitude of the noise */

void makeNoiseMap(float p, cal_t *nmap){
  int i,j;
  float mean, std, sum_x, sum_x2;
  wfs_t kmap;

  int idum = 66;

  (void) calculateDistance(&kmap);

  sum_x = 0;
  sum_x2 = 0;
  
  /* mean */
  for(i=0;i<WFSXS;i++)
    {
      for(j=0;j<WFSXS;j++)
	{
	  sum_x += kmap.data[i][j];
	  
	}
    }
  
  mean = sum_x / (WFSXS*WFSXS);

  /* std */
  for(i=0;i<WFSXS;i++)
    {
      for(j=0;j<WFSXS;j++)
	{
	  sum_x2 += pow(kmap.data[i][j]-mean,2);
	}
    }
  
  std = sum_x2 / (WFSXS*WFSXS -1);

  /*normalize and output*/
  // also make zero-mean
  for(i=0;i<WFSXS;i++)
    {
      for(j=0;j<WFSXS;j++)
	{
	  //  kmap.data[i][j] -=  mean;
	  nmap->data[i][j] = p*(kmap.data[i][j]-0*mean)/std;
	}
    }
}



/* add tip/tilt/piston to the phase map */
/* units are radians and rad/pixel respectively */
void addTTP(cal_t *map, double tip, double tilt, double piston,int xoff, int yoff){
  int i,j;

  float ic, jc;

  ic = MAPXS/2;
  jc = MAPYS/2;

  for(i=0;i<WFSXS;i++)
    {
      for(j=0;j<WFSXS;j++)
	{
	  map->data[i][j] -= piston + tip*((float)i-xoff+0.5-ic) + tilt*((float)j-yoff+0.5-jc);
	}
    }

}


/* image set generator */
void makeABCD(cal_t *phasemap, wfs_t *pupil, float Navg, float noise, float bkg, wfs_t *output,int iwfs){
  int i,j;
  double s, flux, phase;
  long idum = 123;
  double gain = 1.0;
  double pixel;
  
  for(i=0;i<WFSXS;i++){
    for(j=0;j<WFSXS;j++){
      flux = Navg*pupil->data[i][j];
      phase = phasemap->data[i][j];
      
      s = flux*(1.0 + cos(phase + iwfs*(TWOPI/4.0)));
      pixel = gain*(s + noise*gasdev(&idum) + bkg);
      if(pixel < 0)
	pixel = 0;
      if(pixel > CAMERA_SAT)
	pixel = CAMERA_SAT;
      
      output->data[i][j] = (uint16)pround(pixel);
    }
  }
}
