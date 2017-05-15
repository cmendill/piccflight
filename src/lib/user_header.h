
#define _XOPEN_SOURCE 500
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <error.h>
#include <time.h>
#include <math.h>
#include <ctype.h>
#if HAVE_MALLOC_H
# include <malloc.h>
#endif 
#include <pthread.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <sys/time.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/select.h>

