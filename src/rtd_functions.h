#ifndef _RTD_FUNCTIONS
#define _RTD_FUNCTIONS

/* Function Prototypes */
DM7820_Error rtd_open(unsigned long minor_number, DM7820_Board_Descriptor** p_p_rtd_board);
DM7820_Error rtd_reset(DM7820_Board_Descriptor* p_rtd_board);
DM7820_Error rtd_close(DM7820_Board_Descriptor* p_rtd_board);
DM7820_Error rtd_alp_cleanup(DM7820_Board_Descriptor* p_rtd_board);
DM7820_Error rtd_tlm_cleanup(DM7820_Board_Descriptor* p_rtd_board);
DM7820_Error rtd_install_isr(DM7820_Board_Descriptor* p_rtd_board, void *isr_pass);
DM7820_Error rtd_start_alp_clock(DM7820_Board_Descriptor* p_rtd_board);
DM7820_Error rtd_stop_alp_clock(DM7820_Board_Descriptor* p_rtd_board);
DM7820_Error rtd_init_alp(DM7820_Board_Descriptor* p_rtd_board, uint32_t dithers_per_frame);
DM7820_Error rtd_init_tlm(DM7820_Board_Descriptor* p_rtd_board, uint32_t dma_size);
DM7820_Error rtd_send_alp(DM7820_Board_Descriptor* p_rtd_board, double *cmd, volatile uint64_t *dma_done_count );
DM7820_Error rtd_send_tlm(DM7820_Board_Descriptor* p_rtd_board, char *buf, uint32_t num, volatile uint64_t *dma_done_count);


#endif
