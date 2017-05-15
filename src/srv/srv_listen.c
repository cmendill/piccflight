#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>
#include <pthread.h>

#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "../common/controller.h"

#define LISTEN_PORT "14000"
#define CMD_SENDDATA  0x0ABACABB

/* Globals */
int listener;     // listening socket descriptor
int fdmax;        // maximum file descriptor number
extern volatile int clientfd;

/* Prototypes */
int eth_send(char *addr,char *port,void *data,int nbytes);
void *get_in_addr(struct sockaddr *sa);
int write_to_socket(int s,void *buf,int num);
int read_from_socket(int s,void *buf,int num);

void *listener_loop(void *t) {
  /* Start Networking Code */
  fd_set master;    // master file descriptor list
  fd_set read_fds;  // temp file descriptor list for select()
  int nbytes;
  int listener;     // listening socket descriptor
  int newfd;        // newly accept()ed socket descriptor
  struct sockaddr_storage remoteaddr; // client address
  socklen_t addrlen;
  char remoteIP[INET6_ADDRSTRLEN];
  
  int yes=1;        // for setsockopt() SO_REUSEADDR, below
  int i, rv;
  
  struct addrinfo hints, *ai, *p;
  struct timeval timeout;

  unsigned int recvcmd=0;
  
  //clear the master and temp sets
  FD_ZERO(&master);    
  FD_ZERO(&read_fds);
  
  //get a socket for the listener
  memset(&hints, 0, sizeof hints);
  hints.ai_family   = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags    = AI_PASSIVE;
  if ((rv = getaddrinfo(NULL, LISTEN_PORT, &hints, &ai)) != 0) {
    fprintf(stderr, "selectserver: %s\n", gai_strerror(rv));
    exit(2);
  }
  
  for(p = ai; p != NULL; p = p->ai_next) {
    listener = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
    if (listener < 0) { 
      continue;
    }
    
    //disable the "address already in use" error message 
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
    
    //bind socket
    if (bind(listener, p->ai_addr, p->ai_addrlen) < 0) {
      close(listener);
      continue;
    }
    
    break;
  }
  
  //if we got here, it means we didn't get bound
  if (p == NULL) {
    fprintf(stderr, "selectserver: failed to bind\n");
    exit(3);
  }
  
  freeaddrinfo(ai); // all done with this
  
  //start listening on listener
  if (listen(listener, 10) == -1) {
    perror("listen");
    exit(4);
  }
  
  //add the listener to the master set
  FD_SET(listener, &master);
  
  //keep track of the biggest file descriptor
  fdmax = listener; // so far, it's this only one
  /* main loop */
  while(1){
    //set timeout info
    timeout.tv_sec  = 5;
    timeout.tv_usec = 0;
    
    //copy master fd list
    read_fds = master; 
    
    //SELECT
    if (select(fdmax+1, &read_fds, NULL, NULL, &timeout) == -1) {
      perror("select");
      exit(5);
    }
    
    //run through the exitsting connections looking for data to read
    for(i = 0; i <= fdmax; i++) {
      if (FD_ISSET(i, &read_fds)) { // we got one!!
	if (i == listener) {
	  //handle new connections
	  addrlen = sizeof remoteaddr;
	  newfd = accept(listener,(struct sockaddr *)&remoteaddr,&addrlen);
	  
	  if (newfd == -1) {
	    perror("accept");
	  } 
	  else {
	    FD_SET(newfd, &master); // add to master set
	    if (newfd > fdmax) {    // keep track of the max
	      fdmax = newfd;
	    }
	    printf("selectserver: new connection from %s on socket %d\n",
		   inet_ntop(remoteaddr.ss_family, 
			     get_in_addr((struct sockaddr*)&remoteaddr),
			     remoteIP, INET6_ADDRSTRLEN),newfd);
	  }
	}
	else {
	  //handle data from a client
	  nbytes = read_from_socket(i,(void *)&recvcmd,sizeof(recvcmd));
	  if (nbytes <= 0) {
	    close(i); //close this socket
	    FD_CLR(i, &master); //remove from master set
	  } 
	  else {
	    // we got some data from a client
	    if(recvcmd == CMD_SENDDATA){
	      printf("Listener got CMD: Send Data\n");
	      clientfd = i;
	    }
	  }
	} // END handle data from client
      } // END got new incoming connection
    } // END looping through file descriptors
  } // END main loop
} //end of main

