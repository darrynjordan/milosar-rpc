#ifndef UM7_UART_H
#define UM7_UART_H

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>		
#include <fcntl.h>			
#include <termios.h>		
#include <errno.h>
#include "includes.h"

#define CREG_COM_SETTINGS 	0x00
#define CREG_COM_RATES1 	0x01
#define CREG_COM_RATES2 	0x02
#define CREG_COM_RATES3 	0x03
#define CREG_COM_RATES4 	0x04
#define CREG_COM_RATES5 	0x05
#define CREG_COM_RATES6 	0x06

typedef struct UM7_packet_struct
{
  uint8_t address;
  uint8_t packet_type;
  uint16_t checksum; 
  uint8_t data[30];
  uint8_t n_data_bytes;
} UM7_packet;


uint8_t parse_serial_data( uint8_t* rx_data, uint8_t rx_length, UM7_packet* packet );
int uartInit();
int releaseConnection();
int uartRead(int size);
int uartReadRaw(uint8_t* buffer, int size);
int uartWrite();
int getFirmwareVersion(void);
int flashCommit(void);
int factoryReset(void);
int zeroGyros(void);
int setHomePosition(void);
int setMagReference(void);
int resetEKF(void);

void writeRegister(uint8_t address, uint8_t data[4]);
#endif
