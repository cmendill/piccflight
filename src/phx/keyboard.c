#include "keyboard.h"

static int peek_character = -1;
static struct termios initial_settings, new_settings;

void KBOARD_flush(void) {
  tcflush(STDIN_FILENO, TCIFLUSH);
}

int KBOARD_key(KBoardKey* pkbKey) {
  char ch=0;
  new_settings.c_cc[VMIN] = 0;
  tcsetattr(0, TCSANOW, &new_settings);
  while (read(0, &ch, 1))
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(0, TCSANOW, &new_settings);
  *pkbKey = (KBoardKey)ch;
  return ch;
}

void KBOARD_uninit(void) {
  tcsetattr(0, TCSANOW, &initial_settings);
}

void KBOARD_init(void) {
  tcgetattr(0, &initial_settings);
  new_settings = initial_settings;
  new_settings.c_lflag    &= ~ICANON;
  new_settings.c_lflag    &= ~ECHO;
  new_settings.c_lflag    &= ~ISIG;
  new_settings.c_cc[VMIN]  = 1;
  new_settings.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &new_settings);
}