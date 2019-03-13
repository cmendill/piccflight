CC=gcc
DEBUG_FLAGS=-g

INCLUDE_FLAGS=

LIBRARY_FLAGS=-lm

OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations
CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

all: libbmp.a

libbmp.a: libbmp.o 
	ar -cr libbmp.a libbmp.o 

libbmp.o: libbmp.c
	$(CC) $(CFLAGS) -c libbmp.c $(LIBRARY_FLAGS)

clean:
	rm -f *.o *.a
