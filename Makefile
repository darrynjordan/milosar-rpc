CC=arm-linux-gnueabi-gcc
#HAD TO CHANGE AWAY FROM GNUEABIHF

#Default location for h files is ./source
CFLAGS= -std=gnu99 -Wall -Werror -I./source -I./source/rpad/src -L lib -lm -lpthread -lrp

#h files used go here
DEPS= includes.h rp.h colour.h mon.h scope.h options.h imu.h controller.h transfer.h

#c files used go here (with .o extension)
OBJ = source/main.o source/mon.o source/options.o source/transfer.o source/scope.o source/ini.o source/controller.o source/colour.o source/imu.o

#name of generated binaries
BIN = rpc

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFlags)

rpc: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f *.o source/*.o
