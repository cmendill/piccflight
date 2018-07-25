CC=gcc
DEBUG_FLAGS=
INCLUDE_FLAGS=-Iinclude -I/opt/intel/mkl/include
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall

CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

all: libnum.a

libnum.a: libnum.o
	ar -cr libnum.a libnum.o

libnum.o: libnum.c
	$(CC) $(CFLAGS) -c libnum.c 

clean:
	rm -f *.o *.a *~
