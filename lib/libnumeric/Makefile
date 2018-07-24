CC=gcc
DEBUG_FLAGS=
INCLUDE_FLAGS=-Iinclude -I/opt/intel/mkl/include
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall

CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

all: libnumeric.a

libnumeric.a: libnumeric.o
	ar -cr libnumeric.a libnumeric.o

libnumeric.o: libnumeric.c
	$(CC) $(CFLAGS) -c libnumeric.c 

clean:
	rm -f *.o *.a *~
