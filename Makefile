#MKL OPTIONS
MKLROOT = /opt/intel/mkl
MKL_INCLUDE_DIR = $(MKLROOT)/include
MKLLINKLINE = -Wl,--start-group $(MKLROOT)/lib/intel64/libmkl_intel_lp64.a $(MKLROOT)/lib/intel64/libmkl_sequential.a $(MKLROOT)/lib/intel64/libmkl_core.a -Wl,--end-group -ldl
MKLOPTS = -I$(MKL_INCLUDE_DIR)

#COMPILER OPTIONS
CC = gcc
INCLUDE_FLAGS = -Ilib/libfli -Ilib/libphx/include -Ilib/librtd/include -Ilib/libhex/include -Ilib/libalp/include -Ilib/libnum/include -I/usr/local/include/libuvc
CFLAGS = -Wall -Wno-unused -O6 -m64 -D_PHX_LINUX $(MKLOPTS) $(INCLUDE_FLAGS)
LFLAGS = -L/usr/local/lib -Llib/libhex -Llib/libphx -Llib/librtd -Llib/libfli -Llib/libalp -Llib/libnum -Llib/libuvc/build -lalp -lasdk -lphx -lpfw -lpbu -lfli -lm -lpthread -lrt -lrtd-dm7820 -lnum -lpi_pi_gcs2 -luvc -lusb-1.0 $(MKLLINKLINE)

#DEPENDANCIES
COMDEP  = Makefile $(wildcard ./src/*.h)

#FILES
TARGET   = bin/
SOURCE   = $(wildcard ./src/*.c)
OBJECT   = $(patsubst %.c,%.o,$(SOURCE))
LIBRARY  = libfli libalp libnum librtd


#WATCHDOG
$(TARGET)watchdog: $(OBJECT) $(LIBRARY)
	$(CC) $(CFLAGS) -o $(TARGET)watchdog $(OBJECT) $(LFLAGS) 


#USERSPACE OBJECTS
%.o: %.c  $(COMDEP)
	$(CC) $(CFLAGS) -o $@ -c $<

#LIBRARIES
libfli:
	cd lib/libfli && make
libalp:
	cd lib/libalp && make
libnum:
	cd lib/libnum && make
librtd:
	cd lib/librtd && make


#CLEAN
clean:
	rm -f ./src/*.o $(TARGET)watchdog

#REMOVE *~ files
remove_backups:
	rm -f ./*~ */*~ */*/*~
