CC=gcc
DEBUG_FLAGS=-g

INCLUDE_FLAGS=-I/usr/include/libusb-1.0

LIBRARY_FLAGS=-lusb-1.0 -lm

OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations
CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

all: libbmc.a

libbmc.a: libbmc.o libbmc_auxiliary.o
	ar -cr libbmc.a libbmc.o libbmc_auxiliary.o

libbmc.o: libbmc.c
	$(CC) $(CFLAGS) -c libbmc.c $(LIBRARY_FLAGS)

libbmc_auxiliary.o: libbmc_auxiliary.c
	$(CC) $(CFLAGS) -c libbmc_auxiliary.c $(LIBRARY_FLAGS)

clean:
	rm -f *.o *.a
