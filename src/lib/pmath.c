#include <math.h>
#include <stdio.h>
#include "pmath.h"

/******************************************************************************
        ABSOLUTE VALUE
******************************************************************************/
float pabs(float x){
  if(x == 0)
    return(0);
  if(x > 0)
    return(x);
  
  return(-1*x);
}

/******************************************************************************
        ROUND
******************************************************************************/
int pround(float x){
  if(pabs((x - (int)x)) >=0.5)
    return((int)x+1);
  else
    return((int)x);
}

/******************************************************************************
        FLOOR
******************************************************************************/
int pfloor(float x){
  if(x > 0)
    return((int)x);
  if(x < 0){
    if(x != (int)x)
      return((int)x-1);
    else
      return((int)x);
  }
  return(0);
}
/******************************************************************************
        CEILING
******************************************************************************/
int pceil(float x){
  if(x > 0){
    if(x!=(int)x)
      return((int)x+1);
    else
      return((int)x);
  }
  if(x < 0)
    return((int)x);
  return(0);
}
/******************************************************************************
        FACTORIAL
******************************************************************************/
float pfact(int x){
  static float y;
  static int i;
  if(x < 0){
    printf("can't take factorial of neg number: %d\n",x);
    return(1);
  }
  y=1;
  for(i=2;i<=x;i++)
    y*=i;
  return(y);

}
/******************************************************************************
        POWER
******************************************************************************/
float ppow(float x,int exp){
  static int i;
  static float y;
  y=x;
  if(exp == 0)
    return 1;
  for(i=0;i<exp-1;i++)
    y*=x;
  return(y);
  
}
/******************************************************************************
        SQUARE
******************************************************************************/
float psquare(float x){
  return(x * x);
}
/******************************************************************************
        COSINE
******************************************************************************/
float pcos(float x){
  static int i;
  static float z,lz;
  i=0;
  x=fmod(x,2*M_PI);
  z=0;
  while((pabs(lz-z)>0 && i < 20) || i == 0){ 
    lz=z;
    z+=ppow(-1,i)*ppow(x,2*i)/pfact(2*i);
    i++;
  }
  return(z);
}
/******************************************************************************
        SINE
******************************************************************************/
float psin(float x){
  static int i;
  static float z,lz;
  i=0;
  x=fmod(x,2*M_PI);
  z=0;
  while((pabs(lz-z)>0 && i < 20) || i == 0){ 
    lz=z;
    z+=ppow(-1,i)*ppow(x,2*i+1)/pfact(2*i+1);
    i++;
  }
  return(z);
}
/******************************************************************************
        ARCTAN
******************************************************************************/
float patan(float x){
  static int i;
  static float z,lz;
  i=0;
  if(x>=-1.2 && x<=1.2){
    z=0;
    while((pabs(lz-z)>0 && i < 20) || i == 0){ 
      lz=z;
      z+=(ppow(2,2*i)*ppow(pfact(i),2)*ppow(x,2*i+1))/(pfact(2*i+1)*ppow(1+ppow(x,2),i+1));
      i++;
    }
    return(z);
  }
  if(x>1.2){
    z=M_PI/2;
    while((pabs(lz-z)>0 && i < 50) || i == 0){ 
      lz=z;
      z-=ppow(-1,i)/((2*(float)i+1)*ppow(x,2*i+1));
      i++;
    }
    return(z);
  }
  if(x<-1.2){
    z=-1*M_PI/2;
    while((pabs(lz-z)>0 && i < 50) || i == 0){ 
      lz=z;
      z-=ppow(-1,i)/((2*(float)i+1)*ppow(x,2*i+1));
      i++;
    }
    return(z);
  }
  return(0);
}

/******************************************************************************
        ARCTAN 2
******************************************************************************/
float patan2(float y, float x){
  if(x>0)
    return(patan(y/x));
  if(y>=0 && x<0)
    return(M_PI+patan(y/x));
  if(y<0 && x<0)
    return(patan(y/x) -M_PI);
  if(y>0 && x==0)
    return(M_PI/2);
  if(y<0 && x==0)
    return(-1 * M_PI/2);
  if(y==0 && x==0){
    printf("can't take arctan(0/0)\n");
    return(1);
  }
  return(0);
}

/******************************************************************************
        ARCTAN 3
******************************************************************************/
float patan3(float y, float x){
  if(x>0)
    return(atan(y/x));
  if(y>=0 && x<0)
    return(M_PI+atan(y/x));
  if(y<0 && x<0)
    return(atan(y/x) -M_PI);
  if(y>0 && x==0)
    return(M_PI/2);
  if(y<0 && x==0)
    return(-1 * M_PI/2);
  if(y==0 && x==0){
    printf("can't take arctan(0/0)\n");
    return(1);
  }
  return(0);
}



/******************************************************************************
        EXP
******************************************************************************/
float pexp(float x){
  static int i;
  static float z,lz;
  i=0;
  z=0;
  while((pabs(lz-z)>0 && i < 20) || i == 0){ 
    lz = z;
    z+=ppow(x,i)/pfact(i); 
    i++;
  }
  return(z);
}

/******************************************************************************
        SQRT
******************************************************************************/
float psqrt(float n){
  static float x,lx;
  static int i;
  if(n==0)
    return(0);
  if(n<0){
    printf("can't take sqrt of neg number: %f\n",n);
    return(1);
  }
  x=10;
  i=0;
  while((pabs(lx-x)>0 && i < 20) || i == 0){ 
    lx = x;
    x = lx - (ppow(lx,2)-n)/(2*lx);
    i++;
    
  }
  return(x);
}

/******************************************************************************
        LOG
******************************************************************************/
float plog(float x){
  static float z,s,m,a,g,la,lg;
  int i;
  if(x<0){
    printf("can't take log of neg number: %f\n",x);
    return(1);
  }
  m=16;
  s=4/(x*ppow(2,m));
  i=0;
  
  a = 0.5*(1+s);
  g = psqrt(s);
  while((pabs(a-g)>0 && i < 20) || i == 0){ 
    la = a;
    lg = g;
    a = 0.5*(la+lg);
    g = psqrt(la*lg);
    i++;
  }
  z = M_PI/(2*a) - m*M_LN2;
  return(z);
}

/******************************************************************************
        MEAN
******************************************************************************/
float pmean(unsigned short *data,long num){
  long i;
  double mean=0;
  for(i=0;i<num;i++)
    mean += (double)data[i]/(double)num;

  return(mean);
      
}

/******************************************************************************
        FMEAN
******************************************************************************/
float pfmean(float *data,long num){
  long i;
  double mean=0;
  for(i=0;i<num;i++)
    mean += (double)data[i]/(double)num;

  return(mean);
      
}

/******************************************************************************
        DET2
******************************************************************************/
double pdet2(double a00,double a01,double a10,double a11){
  return a00*a11-a01*a10;
}

/******************************************************************************
        DET3
******************************************************************************/
double pdet3(double m[][3]){
  return m[0][0]*m[1][1]*m[2][2]
    -m[0][0]*m[1][2]*m[2][1]
    -m[0][1]*m[1][0]*m[2][2]
    +m[0][1]*m[1][2]*m[2][0]
    +m[0][2]*m[1][0]*m[2][1]
    -m[0][2]*m[1][1]*m[2][0];
}

/******************************************************************************
        INVERT3
******************************************************************************/
void pinvert3(double minv[][3],double m[][3],double det){
  
  minv[0][0]=pdet2(m[1][1],m[1][2],m[2][1],m[2][2])/det;
  minv[1][0]=pdet2(m[0][2],m[0][1],m[2][2],m[2][1])/det;
  minv[2][0]=pdet2(m[0][1],m[0][2],m[1][1],m[1][2])/det;
  
  minv[0][1]=pdet2(m[1][2],m[1][0],m[2][2],m[2][0])/det;
  minv[1][1]=pdet2(m[0][0],m[0][2],m[2][0],m[2][2])/det;
  minv[2][1]=pdet2(m[0][2],m[0][0],m[1][2],m[1][0])/det;
  
  minv[0][2]=pdet2(m[1][0],m[1][1],m[2][0],m[2][1])/det;
  minv[1][2]=pdet2(m[0][1],m[0][0],m[2][1],m[2][0])/det;
  minv[2][2]=pdet2(m[0][0],m[0][1],m[1][0],m[1][1])/det;
  
}
