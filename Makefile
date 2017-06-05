#MKL OPTIONS
MKLROOT = /opt/intel/mkl
MKL_INCLUDE_DIR = $(MKLROOT)/include
MKLLINKLINE = -Wl,--start-group $(MKLROOT)/lib/intel64/libmkl_intel_lp64.a $(MKLROOT)/lib/intel64/libmkl_sequential.a $(MKLROOT)/lib/intel64/libmkl_core.a -Wl,--end-group -ldl
MKLOPTS = -I$(MKL_INCLUDE_DIR)

#COMPILER OPTIONS
CC = gcc
USER_CFLAGS = -Wall -Wno-unused -O6 -m64 -D_PHX_LINUX $(MKLOPTS)
LINK = -Lsrc/phx/lib -lphx -lpfw -lpbu -lm -lpthread -lrt $(MKLLINKLINE)

#FILES
TARGET  = ./bin/
SOURCE  = $(wildcard ./src/*/*.c)
OBJECT  = $(patsubst %.c,%.o,$(SOURCE))
DIASRC  = $(wildcard ./dia/*.c)
DIABIN  = $(patsubst %.c,%,$(DIASRC))

#DEPENDANCIES
COMDEP  = Makefile $(wildcard ./src/*/*.h) $(wildcard ./src/*/*/*.h)

#ALL
MAKEALL = $(TARGET)watchdog 
all: $(MAKEALL) $(DIABIN)

#WATCHDOG
$(TARGET)watchdog: $(OBJECT) 
	$(CC) $(USER_CFLAGS) -o $(TARGET)watchdog $(OBJECT) $(LINK)

#USERSPACE OBJECTS
%.o: %.c  $(COMDEP)
	$(CC) $(USER_CFLAGS) -o $@ -c $< 

#DIAGNOSTIC EXECUTABLES
%: %.c  $(COMDEP)
	$(eval TAR = $(TARGET)$(notdir $(basename $@)))
	$(CC) $(USER_CFLAGS) -o $(TAR) $< src/common/common_functions.c $(LINK)

#SCRIPTS
scripts:
	cp $(SCRIPTS) $(TARGET)

#CLEAN
clean:
	rm -f ./src/*/*.o $(MAKEALL) 

#REMOVE *~ files
remove_backups:
	rm -f ./*~ */*~ */*/*~
