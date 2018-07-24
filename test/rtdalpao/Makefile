#MKL OPTIONS
MKLROOT = /opt/intel/mkl
MKL_INCLUDE_DIR = $(MKLROOT)/include
MKLLINKLINE = -Wl,--start-group $(MKLROOT)/lib/intel64/libmkl_intel_lp64.a $(MKLROOT)/lib/intel64/libmkl_sequential.a $(MKLROOT)/lib/intel64/libmkl_core.a -Wl,--end-group -ldl
MKLOPTS = -I$(MKL_INCLUDE_DIR)

CC=gcc
DEBUG_FLAGS=
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall
TARGET=bin/

LIBNUMERIC_INCLUDE_FLAGS=-I../../lib/libnumeric/include
LIBNUMERIC_LIBRARY_FLAGS=-L../../lib/libnumeric -l:libnumeric.a 

LIBRTD_INCLUDE_FLAGS=-I../../lib/rtd/include
LIBRTD_LIBRARY_FLAGS=-L../../lib/rtd -l:librtd-dm7820.a 

LIBRTDALPAO_INCLUDE_FLAGS=-I../../lib/librtdalpao/include
LIBRTDALPAO_LIBRARY_FLAGS=-L../../lib/librtdalpao -l:librtdalpao.a

INCLUDE_FLAGS:=$(LIBRTD_INCLUDE_FLAGS) $(LIBRTDALPAO_INCLUDE_FLAGS) $(LIBNUMERIC_INCLUDE_FLAGS)

CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS) $(MKLOPTS)
LFLAGS:=$(LIBRTDALPAO_LIBRARY_FLAGS) $(LIBRTD_LIBRARY_FLAGS) $(LIBNUMERIC_LIBRARY_FLAGS) -lm -lpthread $(MKLLINKLINE)

all:	test_rtdalpao_info \
	test_rtdalpao_output \
	test_rtdalpao_tilt

test_rtdalpao_info: test_rtdalpao_info.c
	$(CC) $(CFLAGS) -o $(TARGET)test_rtdalpao_info test_rtdalpao_info.c $(LFLAGS)

test_rtdalpao_output: test_rtdalpao_output.c
	$(CC) $(CFLAGS) -o $(TARGET)test_rtdalpao_output test_rtdalpao_output.c $(LFLAGS)

test_rtdalpao_tilt: test_rtdalpao_tilt.c
	$(CC) $(CFLAGS) -o $(TARGET)test_rtdalpao_tilt test_rtdalpao_tilt.c $(LFLAGS)

clean:
	rm -f *.o *~ $(TARGET)test*
