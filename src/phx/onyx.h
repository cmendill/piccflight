#ifndef _ONYX
	#define _ONYX
  
  #define J4 0x3E0
  #define J5 0x3D0
  #define J6 0x3B0
  #define J7 0x370
  #define J8 0x2F0
  #define J9 0x1F0

  #define GROUP1 0
  #define GROUP2 4

  #define PORTA 0
  #define PORTB 1
  #define PORTC 2
  #define CONFIG 3

  #define BASEPORT (J8 & J7 & J6 & J4)

  #define PORTA1 (BASEPORT+GROUP1+PORTA)
  #define PORTB1 (BASEPORT+GROUP1+PORTB)
  #define PORTC1 (BASEPORT+GROUP1+PORTC)
  #define CONFIG1 (BASEPORT+GROUP1+CONFIG)

  #define PORTA2 (BASEPORT+GROUP2+PORTA)
  #define PORTB2 (BASEPORT+GROUP2+PORTB)
  #define PORTC2 (BASEPORT+GROUP2+PORTC)
  #define CONFIG2 (BASEPORT+GROUP2+CONFIG)

  #define LENGTH 8

	#define OP 0
	#define IP 1

  void ONYX_set_group_io(short, short, short, short);
  void ONYX_output(short, short);
  void ONYX_init(void);
  void ONYX_uninit(void);

#endif   /* _ONYX */