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
int timespec_subtract(struct timespec *result,struct timespec *x,struct timespec *y){
  /* Perform the carry for the later subtraction by updating y. */
  if (x->tv_nsec < y->tv_nsec) {
    int nsec = (y->tv_nsec - x->tv_nsec) / ONE_BILLION + 1;
    y->tv_nsec -= ONE_BILLION * nsec;
    y->tv_sec += nsec;
  }
  if (x->tv_nsec - y->tv_nsec > ONE_BILLION) {
    int nsec = (x->tv_nsec - y->tv_nsec) / ONE_BILLION;
    y->tv_nsec += ONE_BILLION * nsec;
    y->tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     tv_nsec is certainly positive. */
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_nsec = x->tv_nsec - y->tv_nsec;

  /* Return 1 if result is negative. */
  return x->tv_sec < y->tv_sec;
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
  int wpid,status,i;
  for(i=0;i<timeout;i++){
    wpid = waitpid(pid,&status,WNOHANG);
    if (wpid == -1) {
      perror("waitpid");
      return(1);
    }
    if(wpid==pid)
      return(0);
    sleep(1);
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
        CHECK THE CIRCULAR BUFFER FOR DATA
******************************************************************************/
int check_buffer(sm_t *sm_p, int buf, int id){
  uint32 write_offset,read_offset;
  switch (buf) {
  case SCIBUF: write_offset = sm_p->sci_write_offset; read_offset = sm_p->sci_read_offsets[id]; break;
  case SHKBUF: write_offset = sm_p->shk_write_offset; read_offset = sm_p->shk_read_offsets[id]; break;
  case LYTBUF: write_offset = sm_p->lyt_write_offset; read_offset = sm_p->lyt_read_offsets[id]; break;
  case ACQBUF: write_offset = sm_p->acq_write_offset; read_offset = sm_p->acq_read_offsets[id]; break;
  default: return 0;
  }
  return(write_offset - read_offset);
}

/******************************************************************************
        READ FROM A CIRCULAR BUFFER
******************************************************************************/
int read_from_buffer(sm_t *sm_p, void *output, int buf, int id){
  uint32 nbytes;
  void *buffer;
  volatile uint32 *read_offsets;

  //check for new data
  if(!check_buffer(sm_p,buf,id))
    return  0;

  //setup for read
  switch (buf) {
  case SCIBUF:
    nbytes = sizeof(scievent_t);
    buffer= (void *)&(sm_p->sci_cirbuf[sm_p->sci_read_offsets[id] % SCIBUFSIZE]);
    read_offsets = sm_p->sci_read_offsets;
    break;
  case SHKBUF:
    nbytes = sizeof(shkevent_t);
    buffer= (void *)&(sm_p->shk_cirbuf[sm_p->shk_read_offsets[id] % SHKBUFSIZE]);
    read_offsets = sm_p->shk_read_offsets;
    break;
  case LYTBUF:
    nbytes = sizeof(lytevent_t);
    buffer= (void *)&(sm_p->lyt_cirbuf[sm_p->lyt_read_offsets[id] % LYTBUFSIZE]);
    read_offsets = sm_p->lyt_read_offsets;
    break;
  case ACQBUF:
    nbytes = sizeof(acqevent_t);
    buffer= (void *)&(sm_p->acq_cirbuf[sm_p->acq_read_offsets[id] % ACQBUFSIZE]);
    read_offsets = sm_p->acq_read_offsets;
    break;
  default: return 0;
  }

  //read data
  memcpy(output, buffer, nbytes);

  //increment read offset
  read_offsets[id]++;
  
  return 1;
}

/******************************************************************************
        WRITE TO A CIRCULAR BUFFER
******************************************************************************/
int write_to_buffer(sm_t *sm_p, void *input, int buf){
  int i;
  uint32 nbytes;
  void *buffer;
  volatile uint32 *write_offset;
  volatile uint32 *read_offsets;
  uint32 bufsize;
  
  //setup for write
  switch (buf) {
  case SCIBUF:
    bufsize = SCIBUFSIZE;
    nbytes  = sizeof(scievent_t);
    buffer  = (void *)&(sm_p->sci_cirbuf[sm_p->sci_write_offset % bufsize]);
    write_offset = &sm_p->sci_write_offset;
    read_offsets = sm_p->sci_read_offsets;
    break;
  case SHKBUF:
    bufsize = SHKBUFSIZE;
    nbytes  = sizeof(shkevent_t);
    buffer  = (void *)&(sm_p->shk_cirbuf[sm_p->shk_write_offset % bufsize]);
    write_offset = &sm_p->shk_write_offset;
    read_offsets = sm_p->shk_read_offsets;
    break;
  case LYTBUF:
    bufsize = LYTBUFSIZE;
    nbytes  = sizeof(lytevent_t);
    buffer  = (void *)&(sm_p->lyt_cirbuf[sm_p->lyt_write_offset % bufsize]);
    write_offset = &sm_p->lyt_write_offset;
    read_offsets = sm_p->lyt_read_offsets;
    break;
  case ACQBUF:
    bufsize = ACQBUFSIZE;
    nbytes  = sizeof(acqevent_t);
    buffer  = (void *)&(sm_p->acq_cirbuf[sm_p->acq_write_offset % bufsize]);
    write_offset = &sm_p->acq_write_offset;
    read_offsets = sm_p->acq_read_offsets;
    break;
  default: return 0;
  }

  //for each client, if we are out of buffer space, remove trailing entry
  for (i=0;i<NCLIENTS;i++)
    if (((*write_offset+1) % bufsize) == ((read_offsets[i]) % bufsize))
      read_offsets[i]++;                      

  //write data
  memcpy(buffer,input,nbytes);
  *write_offset++;
  return 0;
}

/******************************************************************************
        READ NEWEST FROM A CIRCULAR BUFFER
******************************************************************************/
int read_newest_buffer(sm_t *sm_p, void *output, int buf, int id){
  uint32 nbytes;
  void *buffer;
  volatile uint32 *read_offsets;

  //check for new data
  if(!check_buffer(sm_p,buf,id))
    return  0;

  //setup for read
  switch (buf) {
  case SCIBUF:
    while(check_buffer(sm_p,buf,id) > 1)
      sm_p->sci_read_offsets[id]++;
    nbytes = sizeof(scievent_t);
    buffer= (void *)&(sm_p->sci_cirbuf[sm_p->sci_read_offsets[id] % SCIBUFSIZE]);
    read_offsets = sm_p->sci_read_offsets;
    break;
  case SHKBUF:
    while(check_buffer(sm_p,buf,id) > 1)
      sm_p->shk_read_offsets[id]++;
    nbytes = sizeof(shkevent_t);
    buffer= (void *)&(sm_p->shk_cirbuf[sm_p->shk_read_offsets[id] % SHKBUFSIZE]);
    read_offsets = sm_p->shk_read_offsets;
    break;
  case LYTBUF:
    while(check_buffer(sm_p,buf,id) > 1)
      sm_p->lyt_read_offsets[id]++;
    nbytes = sizeof(lytevent_t);
    buffer= (void *)&(sm_p->lyt_cirbuf[sm_p->lyt_read_offsets[id] % LYTBUFSIZE]);
    read_offsets = sm_p->lyt_read_offsets;
    break;
  case ACQBUF:
    while(check_buffer(sm_p,buf,id) > 1)
      sm_p->acq_read_offsets[id]++;
    nbytes = sizeof(acqevent_t);
    buffer= (void *)&(sm_p->acq_cirbuf[sm_p->acq_read_offsets[id] % ACQBUFSIZE]);
    read_offsets = sm_p->acq_read_offsets;
    break;
  default: return 0;
  }

  //read data
  memcpy(output, buffer, nbytes);

  //increment read offset
  read_offsets[id]++;
  
  return 1;
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
        SEND DATA TO GSE
******************************************************************************/
int send2gse(void *data, int nbytes){
  return(eth_send(GSE_ADDR,GSE_PORT,data,nbytes));
}
