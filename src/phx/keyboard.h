#ifndef _KBOARD
  #define _KBOARD

  #include <unistd.h>
  #include <termios.h>

  typedef enum {CAP_A_KEY=97, CAP_C_KEY=99, UP_KEY=65, DOWN_KEY=66, LEFT_KEY=68, RIGHT_KEY=67, ESC_KEY=27} KBoardKey;

  void KBOARD_init(void);
  void KBOARD_uninit(void);
  void KBOARD_flush(void);
  int KBOARD_key(KBoardKey* key);

#endif   /* _KBOARD */
