#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <netdb.h>
#include <math.h>

#include <sys/time.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/select.h>

/* piccflight headers */
#include "controller.h"
#include "common_functions.h"



/******************************************************************************
        OPEN SHARED MEMORY
******************************************************************************/
sm_t *openshm(int *mainfd){
  sm_t *sm_p=NULL;
  
  /* open the shared memory segment */
  if ((*mainfd = shm_open("piccshm",O_RDWR | O_CREAT,0666)) < 0){
    perror("shm_open()");
    return NULL;
  }

  /* set the size of the shared memory segment */
  if(ftruncate(*mainfd,sizeof(sm_t)) < 0){
    perror("ftruncate()");
    return NULL;
  }
  
  /* mmap the shared memory segment */
  sm_p = (sm_t *) mmap(NULL,sizeof(sm_t), PROT_READ | PROT_WRITE, MAP_SHARED, *mainfd,0);
  if(sm_p == MAP_FAILED){
    perror("mmap()");
    return NULL;
  }

  /* on success, return the shared memory pointer */
  return sm_p;
  
}    
  

/******************************************************************************
        TIMESPEC_SUBTRACT
******************************************************************************/
int timespec_subtract(struct timespec *result,struct timespec *inputx,struct timespec *inputy){
  /* Calculates inputx-inputy */
  static struct timespec x,y;

  /* Copy input data */
  memcpy(&x,inputx,sizeof(x));
  memcpy(&y,inputy,sizeof(y));
  
  /* Perform the carry for the later subtraction by updating y. */
  if (x.tv_nsec < y.tv_nsec) {
    int nsec = (y.tv_nsec - x.tv_nsec) / ONE_BILLION + 1;
    y.tv_nsec -= ONE_BILLION * nsec;
    y.tv_sec += nsec;
  }
  if (x.tv_nsec - y.tv_nsec > ONE_BILLION) {
    int nsec = (x.tv_nsec - y.tv_nsec) / ONE_BILLION;
    y.tv_nsec += ONE_BILLION * nsec;
    y.tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     tv_nsec is certainly positive. */
  result->tv_sec = x.tv_sec - y.tv_sec;
  result->tv_nsec = x.tv_nsec - y.tv_nsec;

  /* Return 1 if result is negative. */
  return x.tv_sec < y.tv_sec;
}

/******************************************************************************
        TS2DOUBLE
******************************************************************************/
void ts2double(volatile struct timespec *ts,volatile double *db){
  *db= (double)ts->tv_sec + (double)ts->tv_nsec / ONE_BILLION;

}
/******************************************************************************
        DOUBLE2TS
******************************************************************************/
void double2ts(volatile double *db,volatile struct timespec *ts){
  ts->tv_sec  = (time_t) *db;
  ts->tv_nsec = (long)(ONE_BILLION*(*db - (double)ts->tv_sec));

}
/******************************************************************************
        CHECK_FILE
******************************************************************************/
int check_file(char *filename){
  struct stat   buffer;   
  return (stat(filename, &buffer));
}
/******************************************************************************
        SORT
******************************************************************************/
int sort(const void *x, const void *y) {
  return (*(int*)x - *(int*)y);
}

   
/******************************************************************************
        WAIT FOR A PROCESS TO DIE
******************************************************************************/
int procwait(int pid,int timeout){
  int wpid,status,i,imax;
  imax = timeout*100;
  for(i=0;i<imax;i++){
    wpid = waitpid(pid,&status,WNOHANG);
    if (wpid == -1) {
      perror("waitpid");
      return(1);
    }
    if(wpid==pid)
      return(0);
    usleep(10000);
  }
  return(1);
}

  
/******************************************************************************
        CHECKIN WITH WATCHDOG
******************************************************************************/
void checkin(sm_t *sm_p,int id){
  sm_p->w[id].chk++; 
  return;
}    

/******************************************************************************
 CIRCULAR BUFFER NOTES
  read_from_buffer reads at read_offset and then increments read_offset
  write_to_buffer writes at write_offset and then increments write_offset
  No data is when read_offset == write_offset
  We start with read_offset == write_offset == 0
  Then we write to write_offset == 0 and increment to write_offset == 1
  Then we read at read_offset == 0 and increment to read_offset == 1
  A read won't start until write_offset > read_offset
  Reads happen until read_offset == write_offset
  If writes are happening more often than reads, write_offset will go around
  the circular buffer and come up behind read_offset
  Before writing at write_offset == N, write checks if read_offset == N+1
  If so, write increments read_offset to N+2, writes at N, and increments
  write_offset to N+1.
  write_offset will never lap read_offset or become equal to read_offset
  This always keeps read_offset ahead of write_offset and prevents reading and 
  writing to the same address at the same time (since we are using a buffer of 1
  between read_offset and write_offset).
  Since we are not using locks, the only issue is when read has already begun at
  N and write wants to write at N-1. In this case you may get a double increment
  of read_offset, one from write and one from read (after it is done reading).
  This is not a huge problem, you may loose an extra packet, but you were losing
  packets anyway. This saves us from the extra overhead of using locks.
  But, it does impose a different minimum buffer size of 3 instead of 2.
******************************************************************************/

/******************************************************************************
        CHECK A CIRCULAR BUFFER FOR DATA
******************************************************************************/
int check_buffer(sm_t *sm_p, int buf, int id){
  return(sm_p->circbuf[buf].write_offset != sm_p->circbuf[buf].read_offsets[id]);
}

/******************************************************************************
        READ FROM A CIRCULAR BUFFER
******************************************************************************/
int read_from_buffer(sm_t *sm_p, void *output, int buf, int id){
  unsigned long int offset;
  char *ptr; 
  
  //check for new data
  if(!check_buffer(sm_p,buf,id))
    return  0;
  
  //read data
  offset = (sm_p->circbuf[buf].read_offsets[id] % sm_p->circbuf[buf].bufsize) * sm_p->circbuf[buf].nbytes;
  ptr = (char *)sm_p->circbuf[buf].buffer + offset;
  memcpy(output, (void *)ptr, sm_p->circbuf[buf].nbytes);
    
  //increment read offset
  sm_p->circbuf[buf].read_offsets[id]++;
  
  return 1;
}

/******************************************************************************
        WRITE TO A CIRCULAR BUFFER
******************************************************************************/
int write_to_buffer(sm_t *sm_p, void *input, int buf){
  int i;
  char *ptr;
  unsigned long int offset;

  //for each client, if we are out of buffer space, remove trailing entry
  for (i=0;i<NCLIENTS;i++){
    if ( ((sm_p->circbuf[buf].write_offset+1)  % sm_p->circbuf[buf].bufsize) ==
	 ((sm_p->circbuf[buf].read_offsets[i]) % sm_p->circbuf[buf].bufsize) ){
      sm_p->circbuf[buf].read_offsets[i]++;                      
    }
  }
  
  //write data
  offset = (sm_p->circbuf[buf].write_offset % sm_p->circbuf[buf].bufsize) * sm_p->circbuf[buf].nbytes;
  ptr = (char *)sm_p->circbuf[buf].buffer + offset;
  memcpy((void *)ptr,input,sm_p->circbuf[buf].nbytes);

  //increment write offset
  sm_p->circbuf[buf].write_offset++;
  return 0;
}

/******************************************************************************
        OPEN A CIRCULAR BUFFER FOR WRITING
******************************************************************************/
void *open_buffer(sm_t *sm_p, int buf){
  int i;
  char *ptr;
  unsigned long int offset;

  //for each client, if we are out of buffer space, remove trailing entry
  for (i=0;i<NCLIENTS;i++){
    if ( ((sm_p->circbuf[buf].write_offset+1)  % sm_p->circbuf[buf].bufsize) ==
	 ((sm_p->circbuf[buf].read_offsets[i]) % sm_p->circbuf[buf].bufsize) ){
      sm_p->circbuf[buf].read_offsets[i]++;                      
    }
  }
  
  //get pointer
  offset = (sm_p->circbuf[buf].write_offset % sm_p->circbuf[buf].bufsize) * sm_p->circbuf[buf].nbytes;
  ptr = (char *)sm_p->circbuf[buf].buffer + offset;

  //return pointer
  return (void *)ptr;
}

/******************************************************************************
        CLOSE CIRCULAR BUFFER AFTER WRITING
******************************************************************************/
void close_buffer(sm_t *sm_p, int buf){
  //increment write offset
  sm_p->circbuf[buf].write_offset++;
}

/******************************************************************************
        READ NEWEST FROM A CIRCULAR BUFFER
******************************************************************************/
int read_newest_buffer(sm_t *sm_p, void *output, int buf, int id){
  //check for new data
  if(!check_buffer(sm_p,buf,id))
    return  0;

  //push read offset ahead to last buffer
  while(check_buffer(sm_p,buf,id) > 1)
    sm_p->circbuf[buf].read_offsets[id]++;
  
  //read data
  return read_from_buffer(sm_p, output, buf, id);
}


/******************************************************************************
        CONNECT TO A SOCKET - SEND
******************************************************************************/
int opensock_send(char *hostname,char *port){
  struct addrinfo hints, *res;
  int sockfd;
  
  // set protocols 
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET;  // use IPv4 or IPv6, whichever
  hints.ai_socktype = SOCK_STREAM;
  
  // fillout addr structs with getaddrinfo()
  if(getaddrinfo(hostname, port, &hints, &res) == -1){
    perror("getaddrinfo");
    return(-1);
  }
  
  // make a socket:
  sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if(sockfd == -1){
    perror("socket");
    return(-1);
  }
  
  //connect
  if(connect(sockfd, res->ai_addr, res->ai_addrlen) == -1){
    perror("connect");
    return(-1);
  }
  return(sockfd);
}



/******************************************************************************
        CONNECT TO A SOCKET - RECEIVE
******************************************************************************/
int opensock_recv(char *hostname,char *port){
  struct addrinfo hints, *res;
  int sockfd;
  int yes=1;
 
  // set protocols 
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET;  // use IPv4 or IPv6, whichever
  hints.ai_socktype = SOCK_STREAM;
  
  // fillout addr structs with getaddrinfo()
  if(getaddrinfo(hostname, port, &hints, &res) == -1){
    perror("getaddrinfo");
    return(-1);
  }
  
  // make a socket:
  sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if(sockfd == -1){
    perror("socket");
    return(-1);
  }

  // lose the pesky "Address already in use" error message
  if(setsockopt(sockfd,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(int)) == -1) {
    perror("setsockopt");
    return(-1);
  } 

  // bind it to the port we passed in to getaddrinfo():
  if(bind(sockfd, res->ai_addr, res->ai_addrlen) == -1){
    perror("bind");
    return(-1);
  }
  
  return(sockfd);
}

/******************************************************************************
        WRITE TO A SOCKET
******************************************************************************/
int write_to_socket(int s,void *buf,int num){
  int n;
  int m;
  n = 0;
  m = 0;
  while(m<num){
    n=send(s,buf+m,num-m,0);
    if(n==0){
#if ETH_DEBUG      
      printf("write_to_socket: socket %d hung up\n", s);
#endif      
      return(0);
    }
    if(n<0){
      perror("send");
      return(-1);
    }
    m+=n;
  }
  return(m);
}

/******************************************************************************
        READ FROM A SOCKET
******************************************************************************/
int read_from_socket(int s,void *buf,int num){
  int n;
  int m;
  n = 0;
  m = 0;
  while(m<num){
    n=recv(s,buf+m,num-m,0);
    if(n==0){
#if ETH_DEBUG      
      printf("read_from_socket: socket %d hung up\n", s);
#endif
      return(0);
    }
    if(n<0){
      perror("recv");
      return(-1);
    }
    m+=n;
  }
  return(m);
}
 



/******************************************************************************
        GET SOCKADDR, IPv4 or IPv6
******************************************************************************/
void *get_in_addr(struct sockaddr *sa){
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}


/******************************************************************************
        SEND DATA OVER ETHERNET
******************************************************************************/
int eth_send(char *addr,char *port,void *data,int nbytes){
  /* Open a Connection */
  int sockfd=-1;
  if((sockfd = opensock_send(addr,port)) < 0){
    printf("opensock_send failed!\n");
    close(sockfd);
    return(-1);
  }
  if(write_to_socket(sockfd,data,nbytes) < 0){
    printf("write_to_socket failed!\n");
    close(sockfd);
    return(-1);
  }
  close(sockfd);
  return(0);
  
}

/******************************************************************************
        RECURSIVE MKDIR
******************************************************************************/
void recursive_mkdir(const char *dir, mode_t mode) {
  char tmp[256];
  char *p = NULL;
  size_t len;

  snprintf(tmp, sizeof(tmp),"%s",dir);
  len = strlen(tmp);
  if(tmp[len - 1] == '/')
    tmp[len - 1] = 0;
  for(p = tmp + 1; *p; p++)
    if(*p == '/') {
      *p = 0;
      mkdir(tmp, mode);
      *p = '/';
    }
  mkdir(tmp, mode);
}

/**************************************************************/
/* RANDOM_ARRAY                                               */
/* - Create an array of random numbers                        */
/**************************************************************/
void random_array(double *array, long num, double amplitude){
  time_t t;
  long i;
  
  //Init random numbers
  srand((unsigned) time(&t));
  
  //Calculate zernike perturbation 
  for(i=0;i<num;i++)
    array[i] = (2*(rand() / (double) RAND_MAX) - 1) * amplitude;
}

/**************************************************************/
/* GETIRQ                                                     */
/* - Parse /proc/interrupts and return the irq of a driver    */
/**************************************************************/
int getirq(char *driver){
  FILE *fd=NULL;
  char filename[256];
  char line[256];
  char *irq;
  
  //Open file
  sprintf(filename,"/proc/interrupts");
  if((fd = fopen(filename,"r")) == NULL){
    perror("COM: getirq fopen");
    fclose(fd);
    return(-1);
  }
  //Read file line by line
  while(fgets(line, sizeof(line), fd)) {
    if(strstr(line,driver) != NULL){
      irq = strtok(line,":");
      fclose(fd);
      return(atoi(irq));
    }
  }

  printf("COM: getirq could not find driver %s\n",driver);
  
  //Close file
  fclose(fd);

  //Return error
  return(-1);
  
}

/**************************************************************/
/* SETIRQ_AFFINITY                                            */
/* - Set the processor affinity for a given irq               */
/**************************************************************/
int setirq_affinity(int irq, int proc){
  FILE *fd=NULL;
  char filename[256];
  
  //Open file
  sprintf(filename,"/proc/irq/%d/smp_affinity",irq);
  if((fd = fopen(filename,"w")) == NULL){
    perror("COM: setirq_affinity fopen");
    fclose(fd);
    return(-1);
  }

  //Write affinity
  if(fprintf(fd,"%d",proc)<0){
    perror("COM: setirq_affinity fprintf");
    fclose(fd);
    return(-1);
  }
  
  //Close file
  fclose(fd);

  //Return success
  return(0);
  
}


/**************************************************************/
/* TIMESTAMP                                                  */
/* - Generates a formatted timestamp string                   */
/**************************************************************/
void timestamp(char *ts){
  time_t ltime;
  ltime=time(NULL);
  struct tm *tm;
  tm=localtime(&ltime);
  sprintf(ts,"%04d%02d%02d_%02d%02d%02d", tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
}

/**************************************************************/
/* DITHERFILL                                                 */
/* - Fills an array (index) with indices in order of use      */
/*   to create an evenly spaced array of 1s & 0s              */
/**************************************************************/
int ditherfill(int *index, int length){
  int debug = 0; //set for debugging statements
  unsigned int i,j,k,l,in,fn;
  double delta;
  double *findex;
  int    *used;
  
  //malloc arrays
  if((findex = (double *)malloc(2*length * sizeof(double))) == NULL){
    printf("COM: malloc error ditherfill (findex)\n");
    return -1;
  }
  if((used = (int *)malloc(length * sizeof(int))) == NULL){
    printf("COM: malloc error ditherfill (used)\n");
    return -1;
  }
  
  //zero out arrays
  memset(findex,0,length * sizeof(double));
  memset(used,0,length * sizeof(int));

  //print header
  if(debug) printf("%4s %4s %4s %4s %8s %8s %8s %4s %4s\n","j","k","l","fn","base","del","find","iind","used");
  
  //fill the first two elements
  fn=0;
  in=0;
  findex[fn] = 0;
  index[in]  = floorl(findex[fn]);
  used[index[in]] = 1;
  fn++;
  in++;
  findex[fn] = length/2;
  index[in]  = floorl(findex[fn]);
  used[index[in]] = 1;
  fn++;
  in++;

  //fill remaining elements
  j=2;
  
  while(in < length){
    k=pow(2,j);
    delta = (double)length/(double)k;
    
    //minus deltas
    for(i=0;i<k/4;i++){
      l=(k/4)+i;
      findex[fn] = findex[l]-delta;
      index[in]  = floorl(findex[fn]);
      if(debug) printf("%4d %4d %4d %4d %8.4f %8.4f %8.4f %4d %4d\n",j,k,l,fn,findex[l],-delta,findex[fn],index[in],used[index[in]]);
      //check if this index has been used
      if(!used[index[in]]){
	used[index[in]] = 1;
	in++;
      }
      fn++;
    }
    //plus deltas
    for(i=0;i<k/4;i++){
      l=(k/4)+i;
      findex[fn] = findex[l]+delta;
      index[in]  = floorl(findex[fn]);
      if(debug) printf("%4d %4d %4d %4d %8.4f %8.4f %8.4f %4d %4d\n",j,k,l,fn,findex[l],-delta,findex[fn],index[in],used[index[in]]);
      //check if this index has been used
      if(!used[index[in]]){
	used[index[in]] = 1;
	in++;
      }
      fn++;
    }
    //increment j
    j++;
  }

  //print results
  if(debug){
    printf("\n");
    printf("---- Final Results ----\n");
    printf("%4s %4s %4s\n","num","ind","used");
    for(i=0;i<length;i++)
      printf("%4d %4d %4d\n",i,index[i],used[i]);
  }

  return 0;
}

/**************************************************************/
/* READ_UPLINK                                                */
/* - Read and parse command uplink packets                    */
/**************************************************************/
int read_uplink(char *cmd, int max, int fd){
  //ascii.dle, ID Byte, length, data, ascii.etx
  //ascii.dle = 0x10
  //ascii.etx = 0x03
  //  ID Byte = 0x14
  uint8_t dle;
  uint8_t id;
  uint8_t len;
  char data[255];
  uint8_t etx;
  uint8_t num;
  int n,m;
  
  //Read DLE (presync)
  if(read(fd,&dle,1) != 1){
    printf("CMD: Bad DLE read\n");
    return -1;
  }
  if(dle != 0x10){
    printf("CMD: Wrong DLE value 0x%x\n",dle);
    return -1;
  }
  
  //Read packet ID
  if(read(fd,&id,1) != 1){
    printf("CMD: Bad ID read\n");
    return -1;
  }
  
  //Check ID

  //Request Science Data
  if(id == 0x13){
    //Read ETX (postsync)
    if(read(fd,&etx,1) != 1){
      printf("CMD: Bad ETX read\n");
      return -1;
    }
    if(etx != 0x03){
      printf("CMD: Wrong ETX value 0x%x\n",etx);
      return -1;
    }
    
    //Return 0 to ignore
    return 0;
  }


  //Uplink Command
  if(id == 0x14){
    //Read data len
    if(read(fd,&len,1) != 1){
      printf("CMD: Bad LEN read\n");
      return -1;
    }
    
    //Check len
    if(len < 0 || len > 255){
      printf("CMD: Read LEN out of bounds 0x%x\n",len);
      return -1;
    }
    
    //Read data
    n=0; m=0;
    while(m < len){
      if((n=read(fd,&data[m],len-m)) < 0){
	printf("CMD: Bad DATA read\n");
	return -1;
      }
      m += n;
    }
    
    //Read ETX (postsync)
    if(read(fd,&etx,1) != 1){
      printf("CMD: Bad ETX read\n");
      return -1;
    }
    if(etx != 0x03){
      printf("CMD: Wrong ETX value 0x%x\n",etx);
      return -1;
    }
    
    //Copy data
    num = (len > max) ? max : len;
    memcpy(cmd,data,num);

    //Print command
    printf("CMD: Got uplink: %s\n",cmd);
        
    //Return number of bytes read up to max allowed
    return num;
  }

  //ID Not Found
  return 0;
  
}

/**************************************************************/
/* READ_FILE                                                  */
/* - Read entire file into pointer                            */
/**************************************************************/
int read_file(char *filename, void *dest, size_t nbytes){
  FILE   *fd=NULL;
  uint64 fsize;
  
  /****** READ FILE ******/
  //--open file
  if((fd = fopen(filename,"r")) == NULL){
    perror("ERR: read_file --> fopen");
    return 1;
  }

  //--check file size
  fseek(fd, 0L, SEEK_END);
  fsize = ftell(fd);
  rewind(fd);
  if(fsize != nbytes){
    printf("ERR: read_file --> incorrect file size %lu != %lu\n",fsize,nbytes);
    fclose(fd);
    return 1;
  }

  //--read file
  if(fread(dest,nbytes,1,fd) != 1){
    perror("ERR: read_file --> fread");
    fclose(fd);
    return 1;
  }

  fclose(fd);
  return 0;    
}
