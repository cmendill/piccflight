#include <user_header.h>
#include <controller.h>
#include <common_functions.h>
#include <semun.h>
#include <pmath.h>
#include <watchdog.h>


/******************************************************************************
        OPEN SHARED MEMORY
******************************************************************************/
sm_t *openshm(int *mainfd){
  sm_t *sm_p=NULL;
  int count = 0;
  while((sm_p == NULL) && (count < SHMEM_RETRY)){
    if ((*mainfd = open("/dev/mem",O_RDWR)) < 0){
      printf("shared mem open failed on: /dev/mem (%d)\n",errno);
      switch (errno) {
      case EACCES:  printf ("Permission denied.\n"); break;
      case EMFILE:  printf ("No file handle available.\n"); break;
      case ENOENT:  printf ("File or path not found.\n"); break;
      default:      printf ("Unknown error.\n"); break;
      }
      printf("Will retry shared mem open...\n");
      sleep(SHMEM_SLEEP);
      count++;
    }
    else if ((sm_p = (sm_t *)mmap(0,sizeof(sm_t)
				  , PROT_READ | PROT_WRITE, MAP_SHARED, 
				  *mainfd, SHARED_BASE)) == MAP_FAILED){
      printf("shared mem mmap failed\n");
      printf("Will retry shared mem open...\n");
      close(*mainfd);
      sm_p = NULL;
      sleep(SHMEM_SLEEP);
      count++;
    }
    
  }
  return sm_p;
  
}    
  


/******************************************************************************
        TIMEVAL_SUBTRACT
******************************************************************************/
int timeval_subtract(struct timeval *result,struct timeval *x,struct timeval *y){
  /* Perform the carry for the later subtraction by updating y. */
  if (x->tv_usec < y->tv_usec) {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
    y->tv_usec -= 1000000 * nsec;
    y->tv_sec += nsec;
  }
  if (x->tv_usec - y->tv_usec > 1000000) {
    int nsec = (x->tv_usec - y->tv_usec) / 1000000;
    y->tv_usec += 1000000 * nsec;
    y->tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     tv_usec is certainly positive. */
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_usec = x->tv_usec - y->tv_usec;

  /* Return 1 if result is negative. */
  return x->tv_sec < y->tv_sec;
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
        CHECK THE SCI CIRCULAR BUFFER FOR DATA
******************************************************************************/
int check_sci_buffer(sm_t *sm_p, int id){
  return(sm_p->sci_write_offset - sm_p->sci_read_offsets[id]);
}

/******************************************************************************
        READ FROM THE SCI CIRCULAR BUFFER
******************************************************************************/
int read_from_sci_buffer(sm_t *sm_p, scievent_t *sci_p, int id){
  //check for new data
  if(!check_sci_buffer(sm_p,id))
    return  0;
  memcpy((void *)sci_p, (const void *)&(sm_p->sci_cirbuf[sm_p->sci_read_offsets[id] % CIRCBUFSIZE]), sizeof(scievent_t));
  sm_p->sci_read_offsets[id]++;
  return 1;
}

/******************************************************************************
        WRITE TO THE SCI IMAGE CIRCULAR BUFFER
******************************************************************************/
int write_to_sci_buffer(sm_t *sm_p, scievent_t *sci_p){
  int i;
  for (i=0;i<NCLIENTS;i++)                                  // For each read pointer
    {
      if (((sm_p->sci_write_offset+1) % CIRCBUFSIZE)      // If we are
	  == ((sm_p->sci_read_offsets[i]) % CIRCBUFSIZE)) // out of buffer space
	sm_p->sci_read_offsets[i]++;                      // remove trailing entry
    }
  memcpy((void *)&(sm_p->sci_cirbuf[sm_p->sci_write_offset % CIRCBUFSIZE]), (const void *)sci_p, sizeof(scievent_t));
  sm_p->sci_write_offset++;
  return 0;
}

/******************************************************************************
        READ NEWEST FROM THE SCI CIRCULAR BUFFER
******************************************************************************/
int read_newest_sci_buffer(sm_t *sm_p, scievent_t *sci_p, int id){
  //check for new data
  if(!check_sci_buffer(sm_p,id))
    return  0;
  
  //move read pointer to one before write pointer
  while(check_sci_buffer(sm_p,id) > 1)
    sm_p->sci_read_offsets[id]++;
  
  memcpy((void *)sci_p, (const void *)&(sm_p->sci_cirbuf[sm_p->sci_read_offsets[id] % CIRCBUFSIZE]), sizeof(scievent_t));
  sm_p->frame_read_offsets[id]++;
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
  return(eth_send(GSE_ADDR,JPL2GSE_PORT,data,nbytes));
}
