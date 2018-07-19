#include <stdio.h>

#include "dm7820_library.h"
#include "rtdalpao_library.h"

int main(int argc, char const *argv[]) {

  DM7820_Board_Descriptor* p_rtd_board;
  DM7820_Error dm7820_status;

  //Open driver
  printf("rtd_open\n");
  if((dm7820_status = rtd_open(0, &p_rtd_board)))
    perror("rtd_open");

  //Reset board
  printf("rtd_reset\n");
  if((dm7820_status = rtd_reset(p_rtd_board)))
    perror("rtd_reset");

  //Clear all
  printf("rtd_clear_all\n");
  if((dm7820_status = rtd_clear_all(p_rtd_board)))
    perror("rtd_clear_all");

  //Init ALPAO interface
  printf("rtdalpao_init\n");
  if((dm7820_status = rtdalpao_init(p_rtd_board,1)))
    perror("rtdalpao_init");

  //Print ALPAO info
  rtdalpao_print_info();

  //Cleanup ALPAO interface
  printf("Open\n");
  if((dm7820_status = rtdalpao_clean(p_rtd_board)))
    perror("rtdalpao_clean");

  //Close driver
  printf("Open\n");
  if((dm7820_status = rtd_close(p_rtd_board)))
    perror("rtd_close");

  return 0;
}
