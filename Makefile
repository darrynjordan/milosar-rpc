CC=arm-linux-gnueabihf-gcc
#HAD TO CHANGE AWAY FROM GNUEABI

#Default location for h files is ./source
CFLAGS= -std=gnu99 -Wall -Werror -I./source -L lib -lm -lpthread -lrp

#h files used go here
DEPS= includes.h rp.h colour.h imu.h controller.h mon.h

#c files used go here (with .o extension)
OBJ = source/main.o source/ini.o source/controller.o source/colour.o source/imu.o source/mon.o

#name of generated binaries
BIN = rpc

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFlags)

rpc: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f *.o source/*.o
