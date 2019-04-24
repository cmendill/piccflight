#ifndef _RTD_FUNCTIONS
#define _RTD_FUNCTIONS

/* Function Prototypes */
int rtd_open(unsigned long minor_number, DM7820_Board_Descriptor** p_p_rtd_board);
int rtd_reset(DM7820_Board_Descriptor* p_rtd_board);
int rtd_close(DM7820_Board_Descriptor* p_rtd_board);
int rtd_alp_cleanup(DM7820_Board_Descriptor* p_rtd_board);
int rtd_tlm_cleanup(DM7820_Board_Descriptor* p_rtd_board);
int rtd_init_alp(DM7820_Board_Descriptor* p_rtd_board, int dithers_per_frame);
int rtd_init_tlm(DM7820_Board_Descriptor* p_rtd_board, uint32_t dma_size);
int rtd_send_alp(DM7820_Board_Descriptor* p_rtd_board, double *cmd);
int rtd_send_tlm(DM7820_Board_Descriptor* p_rtd_board, char *buf, uint32_t num, int flush);


#endif
