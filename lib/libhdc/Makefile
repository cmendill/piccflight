CC=gcc
DEBUG_FLAGS=-g

INCLUDE_FLAGS=

LIBRARY_FLAGS=

OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations
CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

all: libhdc.a

libhdc.a: libhdc.o
	ar -cr libhdc.a libhdc.o

libhdc.o: libhdc.c
	$(CC) $(CFLAGS) -c libhdc.c $(LIBRARY_FLAGS)

clean:
	rm -f *.o *.a
