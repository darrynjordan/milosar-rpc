CC=arm-linux-gnueabihf-gcc
#HAD TO CHANGE AWAY FROM GNUEABI

#Default location for h files is ./source
CFLAGS= -std=gnu99 -Wall -Werror -I./src -L lib -lm -lpthread -lrp

#h files used go here
DEPS= rp.h colour.h imu.h controller.h mon.h binary.h uart.h

#c files used go here (with .o extension)
OBJ = src/main.o src/ini.o src/controller.o src/colour.o src/imu.o src/mon.o src/binary.o src/uart.o

#name of generated binaries
BIN = rpc

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFlags)

rpc: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f *.o src/*.o
