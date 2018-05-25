#Makefile for example.c

#COMPILER OPTIONS
CC = gcc
INCLUDE_FLAGS = -I/usr/include/libusb-1.0 -I/usr/local/include/libuvc
USER_FLAGS = -Wall $(INCLUDE_FLAGS)
LINK = -L/usr/local/lib -luvc -lpthread -lusb-1.0

example: example.c
	$(CC) -o example example.c $(USER_FLAGS) $(LINK)

clean:
	rm example
