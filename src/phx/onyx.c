#include <stdio.h>
#include <stdlib.h>
#include <sys/io.h>

#include "onyx.h"

void ONYX_set_group_io(short group, short io_a, short io_b, short io_c) { /* set the ports of the group to ip or op*/
	outb( (0x80|(io_a<<4)|(io_c<<3)|(io_b<<1)|io_c), BASEPORT+group+CONFIG);
}

void ONYX_output(short value, short port) {
	outb(value, port);
}

void ONYX_init(void) {
	if (ioperm(BASEPORT, 8, 1)) {perror("ioperm"); exit(1);}
}

void ONYX_uninit(void) {
	if (ioperm(BASEPORT, 8, 0)) {perror("ioperm"); exit(1);}
}