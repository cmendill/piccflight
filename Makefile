#MKL OPTIONS
MKLROOT = /opt/intel/mkl
MKL_INCLUDE_DIR = $(MKLROOT)/include
MKLLINKLINE = -Wl,--start-group $(MKLROOT)/lib/intel64/libmkl_intel_lp64.a $(MKLROOT)/lib/intel64/libmkl_sequential.a $(MKLROOT)/lib/intel64/libmkl_core.a -Wl,--end-group -ldl
MKLOPTS = -I$(MKL_INCLUDE_DIR)

#COMPILER OPTIONS
CC = gcc

INCLUDE_FLAGS = -Ilib/libfli -Ilib/libphx/include -Ilib/librtd/include -Ilib/libhex/include -Ilib/libuvc/build/include
CFLAGS = -Wall -Wno-unused -O6 -m64 -D_PHX_LINUX $(MKLOPTS) $(INCLUDE_FLAGS) 
LFLAGS = -L/usr/local/lib -Llib/libhex -Llib/libphx -Llib/librtd -Llib/libfli -Llib/libuvc/build -lphx -lpfw -lpbu -lfli -lm -lpthread -lrt -lrtd-dm7820 -lpi_pi_gcs2 -luvc -lusb-1.0 $(MKLLINKLINE) -Wl,-rpath $(shell pwd)/lib/libhex -Wl,-rpath $(shell pwd)/lib/libuvc/build

#DEPENDANCIES
COMDEP  = Makefile $(wildcard ./src/*.h)

#FILES
TARGET   = bin/
SOURCE   = $(wildcard ./src/*.c)
OBJECT   = $(patsubst %.c,%.o,$(SOURCE))


#WATCHDOG
$(TARGET)watchdog: $(OBJECT) 
	$(CC) $(CFLAGS) -o $(TARGET)watchdog $(OBJECT) $(LFLAGS) 


#USERSPACE OBJECTS
%.o: %.c  $(COMDEP)
	$(CC) $(CFLAGS) -o $@ -c $<

#LIBRARIES
libs: libfli librtd libuvc

libfli:
	make -C lib/libfli
librtd:
	make -C lib/librtd
libuvc:
	cd lib/libuvc/build && cmake ../ && make
	cp lib/libuvc/include/libuvc/libuvc.h lib/libuvc/build/include/


#CLEAN
clean:
	rm -f ./src/*.o $(TARGET)watchdog

#REMOVE *~ files
remove_backups:
	rm -f ./*~ */*~ */*/*~
