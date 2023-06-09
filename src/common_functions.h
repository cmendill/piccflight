#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>

#ifndef _COMMON_FUNCTIONS
#define _COMMON_FUNCTIONS

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
void write_to_buffer(sm_t *sm_p, void *input, int buf);
void *open_buffer(sm_t *sm_p, int buf);
void close_buffer(sm_t *sm_p, int buf);
int read_newest_buffer(sm_t *sm_p, void *output, int buf, int id);
int  opensock_send(char *hostname,char *port);
int  opensock_recv(char *hostname,char *port);
int  write_to_socket(int s,void *buf,int num);
int  read_from_socket(int s,void *buf,int num);
int write_to_socket_udp(int s,struct addrinfo *ai,void *buf,int num);
void *get_in_addr(struct sockaddr *sa);
int  eth_send(char *addr,char *port,void *data,int nbytes);
void recursive_mkdir(const char *dir, mode_t mode);
void check_and_mkdir(char *filename);
void random_array(double *array, long num, double amplitude);
int getirq(char *driver);
int setirq_affinity(int irq, int proc);
void timestamp(char *ts);
int ditherfill(int *index, int length);
int read_uplink(char *cmd, int max, int fd);
int read_file(char *filename, void *dst, size_t nbytes);
int write_file(char *filename, void *src, size_t nbytes);
void parabola_vertex(double *x, double *y, double *v);

#endif

