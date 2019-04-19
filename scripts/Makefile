CC = gcc
CCFLAGS = -Wall -Wextra
LDFLAGS = -liw

all: scan_aps

scan_aps: scan_aps.o
	$(CC) -o $@ $^ $(LDFLAGS) 

%.o: %.c
	$(CC) -c -o $@ $< $(CCFLAGS)
