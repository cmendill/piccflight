CC=gcc
DEBUG_FLAGS=
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall

LIBRTD_INCLUDE_FLAGS=-I../rtd/include

LIBRTDALPAO_INCLUDE_FLAGS=-Iinclude

INCLUDE_FLAGS:=$(LIBRTD_INCLUDE_FLAGS) $(LIBRTDALPAO_INCLUDE_FLAGS)

CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

all: clean librtdalpao.a

librtdalpao.a: librtdalpao.o
	ar -cr librtdalpao.a librtdalpao.o

librtdalpao.o: librtdalpao.c
	$(CC) $(CFLAGS) -c librtdalpao.c

clean:
	rm -f *.o *.a *~
