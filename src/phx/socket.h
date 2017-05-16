#ifndef _SOCKET
  #define _SOCKET

  typedef int SocketHandle;

  typedef enum {
    ACK = 104,
    NACK = 101,
    STOP = 100
  } SocketSignal;

  typedef struct {
    unsigned short width;
    unsigned short height;
    unsigned short exposure_us;
  } SocketHeader;

  int SERVER_init(int);
  int SERVER_uninit(SocketHandle*);
  int SERVER_connect(SocketHandle*);
  int SERVER_disconnect(SocketHandle*);
  int SERVER_rx(SocketHandle*, void*, unsigned int);
  int SERVER_tx(SocketHandle*, void*, unsigned int);
  int SERVER_handshake(SocketHandle*, SocketHeader*);
  int SERVER_frame(SocketHandle*, unsigned short*, unsigned short*, unsigned int, SocketSignal*);


  int CLIENT_init(int);
  int CLIENT_uninit(void);
  int CLIENT_connect(int[4]);
  int CLIENT_disconnect(void);
  int CLIENT_rx(void*, unsigned int);
  int CLIENT_tx(void*, unsigned int);
  int CLIENT_handshake(SocketHeader*);
  int CLIENT_frame(unsigned short*, unsigned short*, unsigned int, SocketSignal*);


#endif  /* _SOCKET */
