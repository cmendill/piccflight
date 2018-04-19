#MKL OPTIONS
MKLROOT = /opt/intel/mkl
MKL_INCLUDE_DIR = $(MKLROOT)/include
MKLLINKLINE = -Wl,--start-group $(MKLROOT)/lib/intel64/libmkl_intel_lp64.a $(MKLROOT)/lib/intel64/libmkl_sequential.a $(MKLROOT)/lib/intel64/libmkl_core.a -Wl,--end-group -ldl
MKLOPTS = -I$(MKL_INCLUDE_DIR)

#COMPILER OPTIONS
CC = gcc
INCLUDE_FLAGS = -Ilib/phx/include -Ilib/rtd/include -Ilib/alpao/include -Ilib/hex/include -Ilib/quickusb/include
USER_CFLAGS = -Wall -Wno-unused -O6 -m64 -D_PHX_LINUX $(MKLOPTS) $(INCLUDE_FLAGS)
LINK = -Llib/phx -Llib/hex -Llib/alpao -Llib/quickusb -Llib/rtd -lasdk -lphx -lpfw -lpbu -lm -lpthread -lrt -lquickusb -lrtd-dm7820 -lpi_pi_gcs2 $(MKLLINKLINE)

#FILES
TARGET  = bin/
SOURCE  = $(wildcard ./src/*.c)
OBJECT  = $(patsubst %.c,%.o,$(SOURCE))

#DEPENDANCIES
COMDEP  = Makefile $(wildcard ./src/*.h)

#WATCHDOG
$(TARGET)watchdog: $(OBJECT) 
	$(CC) $(USER_CFLAGS) -o $(TARGET)watchdog $(OBJECT) $(LINK)


#USERSPACE OBJECTS
%.o: %.c  $(COMDEP)
	$(CC) $(USER_CFLAGS) -o $@ -c $< 

#CLEAN
clean:
	rm -f ./src/*.o $(TARGET)watchdog

#REMOVE *~ files
remove_backups:
	rm -f ./*~ */*~ */*/*~
