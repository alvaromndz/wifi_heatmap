CC = gcc
CCFLAGS = -Wall -Wextra
LDFLAGS = -liw

all: scan_aps

scan_aps: scan_aps.o
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(CC) -c -o $@ $< $(CCFLAGS)
