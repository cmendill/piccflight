#COMPILER OPTIONS
CC = gcc
USER_CFLAGS = -Wall -Wno-unused -O2

#FILES
TARGET  = ./bin/
SOURCE  = $(wildcard ./src/*/*.c)
OBJECT  = $(patsubst %.c,%.o,$(SOURCE))
DIASRC  = $(wildcard ./dia/*.c)
DIABIN  = $(patsubst %.c,%,$(DIASRC))

#DEPENDANCIES
COMDEP  = Makefile $(wildcard ./src/common/*.h)

#ALL
MAKEALL = $(TARGET)watchdog $(DIABIN)
all: $(MAKEALL)

#WATCHDOG
$(TARGET)watchdog: $(OBJECT) 
	$(CC) $(USER_CFLAGS) -o $(TARGET)watchdog $(OBJECT) -lm -lpthread

#USERSPACE OBJECTS
%.o: %.c  $(COMDEP)
	$(CC) $(USER_CFLAGS) -o $@ -c $< 

#USERSPACE EXECUTABLES
%: %.c  $(COMDEP)
	$(CC) $(USER_CFLAGS) -o $@ $< 

#SCRIPTS
scripts:
	cp $(SCRIPTS) $(TARGET)

#CLEAN
clean:
	rm -f ./src/*/*.o $(MAKEALL) ./rtl/*/*.o

#REMOVE *~ files
remove_backups:
	rm -f ./*~ */*~ */*/*~
