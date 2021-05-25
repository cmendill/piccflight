#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>

/* Networking */
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "controller.h"
#include "common_functions.h"

/* Globals */
extern volatile int tcpfd;

void *tlm_listen(void *t) {
  /* Start Networking Code */
  //might need to be global
  int listenerfd;     // listening socket descriptor
  int fdmax;        // maximum file descriptor number
  fd_set master;    // master file descriptor list
  fd_set read_fds;  // temp file descriptor list for select()
  int nbytes;
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
  if ((rv = getaddrinfo(NULL, TLM_PORT, &hints, &ai)) != 0) {
    fprintf(stderr, "selectserver: %s\n", gai_strerror(rv));
    exit(2);
  }
  
  for(p = ai; p != NULL; p = p->ai_next) {
    listenerfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
    if (listenerfd < 0) { 
      continue;
    }
    
    //disable the "address already in use" error message 
    setsockopt(listenerfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
    
    //bind socket
    if (bind(listenerfd, p->ai_addr, p->ai_addrlen) < 0) {
      close(listenerfd);
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
  if (listen(listenerfd, 10) == -1) {
    perror("listen");
    exit(4);
  }
  
  //add the listener to the master set
  FD_SET(listenerfd, &master);
  
  //keep track of the biggest file descriptor
  fdmax = listenerfd; // so far, it's this only one
  /* main loop */
  while(1){
    //printf("listening\n");
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
	if (i == listenerfd) {
	  //handle new connections
	  addrlen = sizeof remoteaddr;
	  newfd = accept(listenerfd,(struct sockaddr *)&remoteaddr,&addrlen);
	  
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
	    printf("TLM: Closing socket %d\n",i);
	    if(i==tcpfd)
	      tcpfd=-1;
	    close(i); //close this socket
	    FD_CLR(i, &master); //remove from master set
	  } 
	  else {
	    // we got some data from a client
	    recvcmd = ntohl(recvcmd);
	    if(recvcmd == CMD_SENDDATA){
	      printf("TLM: Listener got CMD: Send Data\n");
	      tcpfd = i;
	    }
	  }
	} // END handle data from client
      } // END got new incoming connection
    } // END looping through file descriptors
  } // END main loop
} //end of main

