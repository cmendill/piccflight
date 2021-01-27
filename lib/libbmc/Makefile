CC=gcc
DEBUG_FLAGS=-g
LIBRARY_FLAGS=-L/usr/local/lib -lusb-1.0 -lm 
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations
INCLUDE_FLAGS=-I/usr/include/libusb-1.0
CFLAGS:=$(DEBUG_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS) $(INCLUDE_FLAGS)

all: libbmc.a

libbmc.a: libbmc.o
	ar -cr libbmc.a libbmc.o

libbmc.o: libbmc.c $(wildcard *.h)
	$(CC) $(CFLAGS) -c libbmc.c $(LIBRARY_FLAGS)

libbmc_auxiliary.o: libbmc_auxiliary.c $(wildcard *.h)
	$(CC) $(CFLAGS) -c libbmc_auxiliary.c $(LIBRARY_FLAGS)
clean:
	rm -f *.o *.a
