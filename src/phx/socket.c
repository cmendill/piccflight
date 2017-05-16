#include "socket.h"
#include <string.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdio.h>

#include <arpa/inet.h>

static int socket_fd;
static char connected = 0;
static struct sockaddr_in server_addr; /* Initialize server sockaddr_in structure */
static unsigned int addrlen;

int SERVER_init(int portNo) {
  /* initialize the socket server 
     return 1 if opened and bound successfully 
     return 0 if opened and bound fails */
  int opt;

  addrlen = sizeof(server_addr);
  memset(&server_addr, 0, addrlen);

  // Creating socket file descriptor
  if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
    /*perror("ERROR opening socket");*/
    return 0;
  }

//   opt = 2*1024*1024+sizeof(SocketFrame);
//  // Forcefully attaching socket to the port 8080
//   if (setsockopt(socket_fd, SOL_SOCKET, SO_SNDBUF, &opt, sizeof(opt))) {
//     // perror("setsockopt");
//     return 0;
//   }

   opt = 1;
  // Forcefully attaching socket to the port 8080
  if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    // perror("setsockopt");
    return 0;
  }
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(portNo);

  // Forcefully attaching socket to the port 8080
  if (bind(socket_fd, (struct sockaddr *)&server_addr, addrlen) < 0) {
    // perror("bind failed");
    return 0;
  }

  return 1;

}

int SERVER_uninit(SocketHandle* pSocket) {
  close(*pSocket);
  return 1;
}

int SERVER_connect(SocketHandle* pSocket) {
  /* wait for receiver to pick up
     return 1 if successfully connected to the receiver
     return 0 if not */

  printf("waiting for connection...\n");

  if (listen(socket_fd, 5) < 0) {
    // perror("listen");
    return 0;
  }

  *pSocket = accept(socket_fd, (struct sockaddr *)&server_addr, &addrlen); /* Accept actual connection from the client */
  if (*pSocket < 0) {
    /*perror("ERROR on accept");*/
    connected = 0;
    return 0;
  } else {
    connected = 1;
  }

  return 1;
}

int SERVER_disconnect(SocketHandle* pSocket) {
  if (shutdown(*pSocket, SHUT_RDWR) < 0) {
    /*perror("ERROR shutting down");*/
    return 0;
  }
  return 1;
}

int SERVER_rx(SocketHandle* pSocket, void* pBuffer, unsigned int bufferSize) {
  // return recv(*pSocket, pBuffer, bufferSize, 0);
  return read(*pSocket, pBuffer, bufferSize);
}

int SERVER_tx(SocketHandle* pSocket, void* pBuffer, unsigned int bufferSize) {
  // return send(*pSocket, pBuffer, bufferSize, 0);
  return write(*pSocket, pBuffer, bufferSize);
}

int SERVER_handshake(SocketHandle* pSocket, SocketHeader* pHeader) {
  SocketSignal sSignal;
  if(SERVER_tx(pSocket, pHeader, sizeof(SocketHeader))){
    if(SERVER_rx(pSocket, &sSignal, sizeof(sSignal))) {
      if(sSignal==ACK) {
        return 1;
      }
    }
  }
  return 0;
}

// int SERVER_frame(SocketHandle* pSocket, SocketFrame* psFrame, unsigned int sFrameSize, SocketSignal* psSignal) {
//   if(SERVER_tx(pSocket, psFrame, sFrameSize)){
//     if(SERVER_rx(pSocket, psSignal, sizeof(SocketSignal))) {
//         return 1;
//     }
//   }
//   return 0;
// }

int SERVER_frame(SocketHandle* pSocket, unsigned short* pFrameNumber, unsigned short* pPixels, unsigned int n_pixels, SocketSignal* psSignal) {
  unsigned tx_data = 0;
  unsigned int n_data = sizeof(unsigned short)*n_pixels;
  char* pPixelData = (char*)pPixels;

  tx_data = SERVER_tx(pSocket, pFrameNumber, sizeof(unsigned short));
  if (!(tx_data==sizeof(unsigned short))) {
    return 0;
  }

  tx_data = 0;
  while(n_data > 0) {
    tx_data = SERVER_tx(pSocket, pPixelData, n_data);
    if (tx_data<=0){
      return 0;
    } else {
      pPixelData += tx_data;
      n_data -= tx_data;
    }
  }

  tx_data = 0;
  if (n_data == 0) {
    tx_data = SERVER_rx(pSocket, psSignal, sizeof(SocketSignal));
    if (!(tx_data==sizeof(SocketSignal))) {
      return 0;
    } else {
      return 1;
    }
  }
  return 0;

}

















int CLIENT_init(int portNo) {

  int opt;
  addrlen = sizeof(server_addr);
  memset(&server_addr, 0, addrlen);

  if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    // printf("\n Socket creation error \n");
    return 0;
  }
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(portNo);

//   opt = 2*1024*1024+sizeof(SocketFrame);
//  // Forcefully attaching socket to the port 8080
//   if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &opt, sizeof(opt))) {
//     // perror("setsockopt");
//     return 0;
//   }

  return 1;

}

int CLIENT_uninit(void) {
  close(socket_fd);
  return 1;
}

int CLIENT_connect(int server_ip[4]) {

  char server_ip_str[16];
  sprintf(server_ip_str, "%d.%d.%d.%d", server_ip[0], server_ip[1], server_ip[2], server_ip[3]);

  // Convert IPv4 and IPv6 addresses from text to binary form
  if (inet_pton(AF_INET, server_ip_str, &server_addr.sin_addr) <= 0) {
    printf("Invalid address or Address not supported...\n");
    return 0;
  }

  if (connect(socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    printf("Connection Failed...\n");
    return 0;
  }
  return 1;
}

int CLIENT_disconnect(void) {
  if (shutdown(socket_fd, SHUT_RDWR) < 0) {
    /*perror("ERROR shutting down");*/
    return 0;
  }
  return 1;
}

int CLIENT_rx(void* pBuffer, unsigned int bufferSize) {
  // return recv(socket_fd, pBuffer, bufferSize, 0);
  return read(socket_fd, pBuffer, bufferSize);
}

int CLIENT_tx(void* pBuffer, unsigned int bufferSize) {
  // return send(socket_fd, pBuffer, bufferSize, 0);
  return write(socket_fd, pBuffer, bufferSize);
}


int CLIENT_handshake(SocketHeader* pHeader) {
  SocketSignal sSignal = ACK;
  if(CLIENT_rx(pHeader, sizeof(SocketHeader))){
    if(CLIENT_tx(&sSignal, sizeof(sSignal))) {
      return 1;
    }
  }
  return 0;
}

// int CLIENT_frame(SocketFrame* psFrame, unsigned int sFrameSize, SocketSignal* psSignal) {
//   if(CLIENT_rx(psFrame, sFrameSize)){
//     if(CLIENT_tx(psSignal, sizeof(SocketSignal))) {
//       return 1;
//     }
//   }
//   return 0;
// }

int CLIENT_frame(unsigned short* pFrameNumber, unsigned short* pPixels, unsigned int n_pixels, SocketSignal* psSignal) {
  unsigned int rx_data = 0;
  unsigned int n_data = sizeof(unsigned short)*n_pixels;
  char* pPixelData = (char*)(pPixels);

  rx_data = CLIENT_rx(pFrameNumber, sizeof(unsigned short));
  if (!(rx_data==sizeof(unsigned short))) {
    return 0;
  }

  rx_data = 0;
  while(n_data > 0) {
    rx_data = CLIENT_rx(pPixelData, n_data);
    if (rx_data<=0) {
      return 0;
    } else {
      pPixelData += rx_data;
      n_data -= rx_data;
    }
  }

  rx_data = 0;
  if (n_data == 0) {
    rx_data = CLIENT_tx(psSignal, sizeof(SocketSignal));
    if(!(rx_data==sizeof(SocketSignal))) {
      return 0;
    } else {
      return 1;
    }
  }
  return 0;
  
}