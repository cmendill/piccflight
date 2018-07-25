CC=gcc
DEBUG_FLAGS=
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall

LIBRTD_INCLUDE_FLAGS=-I../librtd/include

LIBALP_INCLUDE_FLAGS=-Iinclude

INCLUDE_FLAGS:=$(LIBRTD_INCLUDE_FLAGS) $(LIBALP_INCLUDE_FLAGS)

CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

all: libalp.a

libalp.a: libalp.o
	ar -cr libalp.a libalp.o

libalp.o: libalp.c
	$(CC) $(CFLAGS) -c libalp.c

clean:
	rm -f *.o *.a *~
