#include <QuickUSB.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include "../src/common/controller.h"

#define NACT 79
QCHAR uname[10];
QHANDLE hDevice;
short wdata[96];

void iwc_ttlim(unsigned short* ttarray, unsigned short tbias, short* cdata){
  short highlim, lowlim;
  short a1, a2, a3;
  int i;
  a1=ttarray[0];
  a2=ttarray[1];
  a3=ttarray[2];
  highlim=tbias+tbias*0.2;
  lowlim=tbias-tbias*0.2;
  for (i=0; i<3; i++){
    if (a1 > highlim){
      printf("Setting a1 from %hi to high actuator saftey limit (%hi)\r\n", a1, highlim);
      a1 = highlim;
      cdata[76]=highlim;
    }
    if (a1 < lowlim){
      printf("Setting a1 from %hi to low actuator saftey limit (%hi)\r\n", a1, lowlim);
      a1 = lowlim;
      cdata[76]=lowlim;
    }
    if (a2 > highlim){
      printf("Setting a2 from %hi to high actuator saftey limit (%hi)\r\n", a2, highlim);
      a2 = highlim;
      cdata[77]=highlim;
    }
    if (a2 < lowlim){
      printf("Setting a2 from %hi to low actuator saftey limit (%hi)\r\n", a2, lowlim);
      a2 = lowlim;
      cdata[77]=lowlim;
    }
    if (a3 > highlim){
      printf("Setting a3 from %hi to high actuator saftey limit (%hi)\r\n", a3, highlim);
      a3 = highlim;
      cdata[78]=highlim;
    }
    if (a3 < lowlim){
      printf("Setting a3 from %hi to low actuator saftey limit (%hi)\r\n", a3, lowlim);
      a3 = lowlim;
      cdata[78]=lowlim;
    }
  }
}

int gh_findModule(void){
  QLONG res;
  QLONG listlen = 10;

  res = QuickUsbFindModules(uname, listlen);
  if (res == 0){
    unsigned long error;
    int lastE = QuickUsbGetLastError(&error);
    printf("'Find Module'  failed. Error #%lu\r\n", error);
  }else{
    printf("Device %s found.\r\n", uname);
  }
  return res;
}

int gh_getDriver(){
  QWORD major, minor, build;
  QLONG res;
  res = QuickUsbGetDriverVersion(&major, &minor, &build);

  if (res == 0){
    unsigned long error;
    QLONG lastE = QuickUsbGetLastError(&error);
    printf("'Find Driver'  failed. Error #%lu\r\n", error);
  }
  return 0;
}

int gh_openDev(){
  PQCHAR devName = uname;
  QLONG res;
  res = QuickUsbOpen(&hDevice, devName);
  if (res == 0){
    unsigned long error;
    QLONG lastE = QuickUsbGetLastError(&error);
    printf("'Open Device'  failed. Error #%lu\r\n", error);
  }else{
    printf("Device %s open.\r\n", uname);
  }
  return 0;
}

int gh_writeUsb(unsigned char* data, QULONG length){
  QLONG res;
  res = QuickUsbWriteData(hDevice, data, length);
  if (res == 0){
    unsigned long error;
    QLONG lastE = QuickUsbGetLastError(&error);
    printf("'Write Data'   failed. Error #%lu\r\n", error);
  }
  return 0;
}

int gh_readUsb(unsigned char* data, QULONG length){
  QLONG res;
  res = QuickUsbReadData(hDevice, data, &length);
  if (res == 0){
    unsigned long error;
    QLONG lastE = QuickUsbGetLastError(&error);
    printf("'Read Data'    failed. Error #%lu\r\n", error);
  }else{
    printf("Read  %lu bytes.\r\n", length);
  }
  return 0;
}

int gh_closeDev(){
  QLONG res;
  res = QuickUsbClose(hDevice);
  if (res == 0){
    unsigned long error;
    QLONG lastE = QuickUsbGetLastError(&error);
    printf("'Close Device' failed. Error #%lu\r\n", error);
  }else{
    printf("Device %s closed.\r\n", uname);
  }
  return 0;
}

void gh_map(short comarr[NACT]){
  FILE *mapFile = fopen("iwctest_files/act_map.bin", "rb");
  if (!mapFile){
    printf("Cannot open actuator map file.\n\r");
  }
	
  unsigned long mapLen;
  fseek(mapFile, 0, SEEK_END);
  mapLen=ftell(mapFile);

  unsigned short act_map[NACT];
  fseek(mapFile,0, SEEK_SET);
  if(fread(&act_map, mapLen, 1, mapFile) != 1)
    perror("fread()");
  fclose(mapFile);

  int i;
  for (i=0; i<NACT; i++){
    int ind = act_map[i];
    wdata[ind] = comarr[i];

    if (i < NACT-3){
      if (comarr[i] > 16383){
	comarr[i] = 16383;
      }
      if (comarr[i] < 0){
	comarr[i] = 0;
      }
    }
  }
}

int main(int argc, char* argv[]){
  short cdata[NACT]; //unsigned
  short full_stroke_adu = 16383;
  float bias_pct = 0.58; //0.580 seems to be flat for old setup? 0.6228 for new (May 2017)
  float poke_pct = 0.694;
  short bias = bias_pct*full_stroke_adu;
  short poke = poke_pct*full_stroke_adu;
  short probe = 6000; //usually 3000. changed feb28
  unsigned short tbias = 0;
  unsigned short tdata[] = {1,1,1};
  int i, j, act;
  char ch;

  //Make sure Xinetics controller is disabled in the flight code
  if(XIN_ENABLE){
    printf("Xinetics controller must be disabled in the flight code!\n");
    return 0;
  }
  
  float prd=0;
  float cmd=0;
  if(argc == 2){
    cmd = atof(argv[1]);
    if(cmd < 0 || cmd > 5){
      printf("Period must be between 0 and 5 (seconds)\n");
    }else{
      prd=cmd;
    }
  }
  if(argc == 1){
    prd = 1.0;
  }

  if(poke < 0 || poke > full_stroke_adu || bias < 0 || bias > full_stroke_adu){
    printf("Voltage set error!\n");
    poke = 0;
    bias = 0;
  }
  
  gh_findModule();
  gh_getDriver();
  gh_openDev();
  for(i=0; i<NACT; i++)
    cdata[i]=0;
  
  gh_map(cdata);
  printf("Setting all actuators to 0 ADU\r\n");
  gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
  printf("Period: %f seconds\tpoke: %hi ADUs\tbias: %hi\t tbias: %hi\n\r", prd, poke, bias, tbias);
  iwc_ttlim(tdata, tbias, cdata);
  gh_map(cdata);

  unsigned long count = 0;
  act = 32;

  struct timeval start, stop;
  double secs = 0;

  if(system("stty raw -echo"))
    perror("system()");
  
  while (1){
    ch = getchar();
    
    if(ch=='`'){
      if(system("stty cooked echo"))
	perror("system()");
      printf("\nExiting due to keyboard command.\n\r");
      printf("Setting all actuators to 0\r\n");
      for(i=0; i<NACT; i++){
	cdata[i]=(0.0*full_stroke_adu);
      }
      gh_map(cdata);
      gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
      
      break;
    }
    
    if (ch == '0'){
      printf("Setting all actuators to zero (0 ADU).\r\n");
      for(i=0; i<NACT; i++){
	cdata[i]=0;  //cdata
      }
      gh_map(cdata);
      gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
    }

    if (ch == '5'){
      printf("Setting all facesheet actuators to bias (%hi ADU).\r\n", bias);
      for(i=0; i<NACT-3; i++){
	cdata[i]=bias;  //cdata
      }
      gh_map(cdata);
      gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
    }

    if (ch == 'a'){
      for (i=0; i<NACT; i++){
	printf("Actuator\t%i\tat position:\t%hu\r\n", i, cdata[i]);
      }
    }

    if (ch == '\\'){  //Influence function images
      count=0;
      for(j=0; j<NACT-3; j++)
	cdata[j] = bias;
      for(j=0; j<NACT-3; j++){
	printf("Poking Actuator\t#%i\tat %d ADU.\r\n", j, probe);
	cdata[j] = bias+ probe;
	gh_map(cdata);
	gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
	usleep(1000000*prd);
	cdata[j] = bias;
	gh_map(cdata);
	gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
      }
    }

    if (ch == '1'){
      printf("Pattern: Random.\r\n");
      int i, n, j, index;
      unsigned short randint;
      time_t t;
      n = NACT;
      srand(12345);
      short rand_max = bias + probe;
      short rand_min = bias - probe;
      short rand_range = rand_max - rand_min;
      printf("rand_max: %hi\r\n", rand_max);
      printf("rand_min: %hi\r\n", rand_min);

      for(i=0; i<NACT-3; i++){
	cdata[i] = rand()%rand_range + rand_min;
      }
      gh_map(cdata);
      gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
    }

    if (ch == '2'){
      printf("Pattern: Random.\r\n");
      int i, n, j, index;
      unsigned short randint;
      time_t t;
      n = NACT;
      srand(67890);
      short rand_max = bias + probe;
      short rand_min = bias - probe;
      short rand_range = rand_max - rand_min;
      printf("rand_max: %hi\r\n", rand_max);
      printf("rand_min: %hi\r\n", rand_min);

      for(i=0; i<NACT-3; i++){
	cdata[i] = rand()%rand_range + rand_min;
      }

      gh_map(cdata);
      gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
    }

    if (ch == '3'){
      printf("Pattern: Random.\r\n");
      int i, n, j, index;
      unsigned short randint;
      time_t t;
      n = NACT;
      srand(27182);
      short rand_max = bias + probe;
      short rand_min = bias - probe;
      short rand_range = rand_max - rand_min;
      printf("rand_max: %hi\r\n", rand_max);
      printf("rand_min: %hi\r\n", rand_min);

      for(i=0; i<NACT-3; i++){
	cdata[i] = rand()%rand_range + rand_min;
      }

      gh_map(cdata);
      gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
    }

    if (ch == '4'){
      printf("Pattern: Random.\r\n");
      int i, n, j, index;
      unsigned short randint;
      time_t t;
      n = NACT;
      srand(11111);
      short rand_max = bias + probe;
      short rand_min = bias - probe;
      short rand_range = rand_max - rand_min;
      printf("rand_max: %hi\r\n", rand_max);
      printf("rand_min: %hi\r\n", rand_min);

      for(i=0; i<NACT-3; i++){
	cdata[i] = rand()%rand_range + rand_min;
      }

      gh_map(cdata);
      gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
    }

    if (ch == 'q'){
      printf("Pattern: Small Random.\r\n");
      int i, j, n, index;
      short randint;
      time_t t;
      // n = NACT;

      short rand_max = probe*0.1;
      short rand_min = probe*-0.1;
      short rand_range = rand_max - rand_min;
      printf("rand_max: %hi\r\n", rand_max);
      printf("rand_min: %hi\r\n", rand_min);
      printf("rand_range: %hi\r\n", rand_range);

      for(i=0; i<NACT-3; i++){
	cdata[i] = bias;
      }

      for(n=0; n<1; n++){
	for(j=0; j<NACT-3;j++){ //Another divide by 2...
	  // index=act_ind[j];
	  randint = (short)rand()%rand_range;// + rand_min;
	  cdata[j]=cdata[j]+(randint);
	  printf("Act Command #%d: %d (randint: %hi)\r\n", j, cdata[j], randint);
	}
	usleep(100000);
	gh_map(cdata);
	gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
	printf("Done with loop %i \r\n", n);
      }
    }

    if (ch == ']'){
      act = act + 1;
      if (act >75){
	act = 0;
      }
      for(i=0; i<NACT-3; i++){
	cdata[i]=bias; //cdata
      }
      cdata[act] = bias + probe; //cdata
      gh_map(cdata);
      gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
      // printf("Command Actuator #%i      \r", act);
      printf("Poking Actuator\t#%i\tat %d ADUs.\r\n", act, probe);
    }
    
    if (ch ==';'){
      act = act - 1;
      if (act < 0){
	act = 75;
      }
      for(i=0; i<NACT-3; i++){
	cdata[i]=bias; //cdata
      }
      cdata[act] = bias + probe; //cdata
      gh_map(cdata);
      gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
      // printf("Command Actuator #%i      \r", act);
      printf("Poking Actuator\t#%i\tat %d ADUs.\r\n", act, probe);
    }

    if (ch == '-'){
      for(i=0; i<NACT-3; i++){
	cdata[i] = cdata[i] - 50;
	if (cdata[i] <0){
	  printf("** Voltage too low! Now set to 0\r\n");
	  for(i=0; i<NACT-3; i++){
	    cdata[i] = 0;
	  }
	}
	printf("All actuators at %d               \r", cdata[i]);

      }
      gh_map(cdata);
      gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
    }

    if (ch == '='){
      for(i=0; i<NACT-3; i++){
	cdata[i] = cdata[i] + 50;
	if (cdata[act] >= 16383){
	  printf("** Voltage too high! Now set to 16380\r\n");
	  for(i=0; i<NACT-3; i++){
	    cdata[i] = 16380;
	  }
	}
	printf("All actuators at %d               \r", cdata[i]);
	
      }
      gh_map(cdata);
      gh_writeUsb((unsigned char *)wdata, sizeof(wdata));
    }
    
  }
  
  gh_closeDev();
  printf("Done.\n\n");
  return 0;
}

