CC=gcc
DEBUG_FLAGS=
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall
TARGET=bin/

LIBRTD_INCLUDE_FLAGS=-I../../lib/rtd/include
LIBRTD_LIBRARY_FLAGS=-L../../lib/rtd -l:librtd-dm7820.a 

LIBRTDALPAO_INCLUDE_FLAGS=-I../../lib/librtdalpao/include
LIBRTDALPAO_LIBRARY_FLAGS=-L../../lib/librtdalpao -l:librtdalpao.a

INCLUDE_FLAGS:=$(LIBRTD_INCLUDE_FLAGS) $(LIBRTDALPAO_INCLUDE_FLAGS)

CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)
LFLAGS:=$(LIBRTDALPAO_LIBRARY_FLAGS) $(LIBRTD_LIBRARY_FLAGS) -lm -lpthread

test_dither: test_dither_rtdalpao_info \
             test_dither_rtdalpao_output \
             test_dither_rtdalpao_periodic \
             test_dither_rtdalpao_wave

test_dither_rtdalpao_info: test_rtdalpao_info.c
	$(CC) $(CFLAGS) -DRTDALPAO_DITHER -o $(TARGET)test_rtdalpao_info test_rtdalpao_info.c $(LFLAGS)

test_dither_rtdalpao_output: test_rtdalpao_output.c
	$(CC) $(CFLAGS) -DRTDALPAO_DITHER -o $(TARGET)test_rtdalpao_output test_rtdalpao_output.c $(LFLAGS)

test_dither_rtdalpao_periodic: test_rtdalpao_periodic.c
	$(CC) $(CFLAGS) -DRTDALPAO_DITHER -o $(TARGET)test_rtdalpao_periodic test_rtdalpao_periodic.c $(LFLAGS)

test_dither_rtdalpao_wave: test_rtdalpao_wave.c
	$(CC) $(CFLAGS) -DRTDALPAO_DITHER -o $(TARGET)test_rtdalpao_wave test_rtdalpao_wave.c $(LFLAGS)






test_nodither: test_nodither_rtdalpao_info \
	       test_nodither_rtdalpao_output \
	       test_nodither_rtdalpao_periodic \
	       test_nodither_rtdalpao_wave

test_nodither_rtdalpao_info: test_rtdalpao_info.c
	$(CC) $(CFLAGS) -o $(TARGET)test_rtdalpao_info test_rtdalpao_info.c $(LFLAGS)

test_nodither_rtdalpao_output: test_rtdalpao_output.c
	$(CC) $(CFLAGS) -o $(TARGET)test_rtdalpao_output test_rtdalpao_output.c $(LFLAGS)

test_nodither_rtdalpao_periodic: test_rtdalpao_periodic.c
	$(CC) $(CFLAGS) -o $(TARGET)test_rtdalpao_periodic test_rtdalpao_periodic.c $(LFLAGS)

test_nodither_rtdalpao_wave: test_rtdalpao_wave.c
	$(CC) $(CFLAGS) -o $(TARGET)test_rtdalpao_wave test_rtdalpao_wave.c $(LFLAGS)


clean:
	rm -f *.o *~ $(TARGET)test_dither* $(TARGET)test_nodither*

