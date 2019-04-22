CC=gcc
DEBUG_FLAGS=-g

LIBRARY_FLAGS=-lusb-1.0 -lm

OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations
CFLAGS:=$(DEBUG_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

all: libbmc.a

libbmc.a: libbmc.o libbmc_auxiliary.o
	ar -cr libbmc.a libbmc.o libbmc_auxiliary.o

libbmc.o: libbmc.c libbmc.h libbmc_auxiliary.h libbmc_round_map.h libbmc_square_map.h
	$(CC) $(CFLAGS) -c libbmc.c $(LIBRARY_FLAGS)

libbmc_auxiliary.o: libbmc_auxiliary.c libbmc.h libbmc_auxiliary.h libbmc_round_map.h libbmc_square_map.h
	$(CC) $(CFLAGS) -c libbmc_auxiliary.c $(LIBRARY_FLAGS)

clean:
	rm -f *.o *.a
