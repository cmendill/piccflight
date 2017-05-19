#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/ipc.h>
#include <sys/sem.h>

sm_t *openshm(int *mainfd);
int  timespec_subtract(struct timespec *result,struct timespec *x,struct timespec *y);
void ts2double(volatile struct timespec *ts,volatile double *db);
void double2ts(volatile double *db,volatile struct timespec *ts);
int  check_file(char *filename);
int sort(const void *x, const void *y);
int  procwait(int pid,int timout);
void checkin(sm_t *sm_p,int id);
int check_buffer(sm_t *sm_p, int buf, int id);
int read_from_buffer(sm_t *sm_p, void *output, int buf, int id);
int write_to_buffer(sm_t *sm_p, void *input, int buf);
int read_newest_buffer(sm_t *sm_p, void *output, int buf, int id);
int  opensock_send(char *hostname,char *port);
int  opensock_recv(char *hostname,char *port);
int  write_to_socket(int s,void *buf,int num);
int  read_from_socket(int s,void *buf,int num);
void *get_in_addr(struct sockaddr *sa);
int  eth_send(char *addr,char *port,void *data,int nbytes);
int send2gse(void *data, int nbytes);


