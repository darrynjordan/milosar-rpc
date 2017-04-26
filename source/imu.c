#include "imu.h"

/* File descriptor definition */
int uart_fd = -1;
//Global UM7 Packet for data reception
UM7_packet global_packet;

//Parse the serial data obtained through the UART interface and fit to a general packet structure
uint8_t parse_serial_data( uint8_t* rx_data, uint8_t rx_length, UM7_packet* packet )
{
	uint8_t index;
	// Make sure that the data buffer provided is long enough to contain a full packet
	// The minimum packet length is 7 bytes
	if( rx_length < 7 )
	{
		return 1;
		//printf("packet lenth too short\n");
	}
	
	// Try to find the 'snp' start sequence for the packet
	for(index = 0; index < (rx_length - 2); index++)
    {
		// Check for 'snp'. If found, immediately exit the loop
		if( rx_data[index] == 's' && rx_data[index+1] == 'n' && rx_data[index+2] == 'p' )
		{
			//printf("Found SNP\n");
			break;
		}
    }    
	uint8_t packet_index = index;
	
	// Check to see if the variable 'packet_index' is equal to (rx_length - 2). If it is, then the above
	// loop executed to completion and never found a packet header.
	if( packet_index == (rx_length - 2) )
	{
		//printf("Didn't find SNP\n");
		return 2;
    }
    
	// If we get here, a packet header was found. Now check to see if we have enough room
	// left in the buffer to contain a full packet. Note that at this point, the variable 'packet_index'
	// contains the location of the 's' character in the buffer (the first byte in the header)
	if( (rx_length - packet_index) < 7 )
	{
		//printf("not enough room after 's' for a full packet\n");
		return 3;
    }
    
	// We've found a packet header, and there is enough space left in the buffer for at least
	// the smallest allowable packet length (7 bytes). Pull out the packet type byte to determine
	// the actual length of this packet
	uint8_t PT = rx_data[packet_index + 3];

	// Do some bit-level manipulation to determine if the packet contains data and if it is a batch
	// We have to do this because the individual bits in the PT byte specify the contents of the
	// packet.
	uint8_t packet_has_data = (PT >> 7) & 0x01; // Check bit 7 (HAS_DATA)
	uint8_t packet_is_batch = (PT >> 6) & 0x01; // Check bit 6 (IS_BATCH)
	uint8_t batch_length = (PT >> 2) & 0x0F; // Extract the batch length (bits 2 through 5)

	// Now finally figure out the actual packet length
	uint8_t data_length = 0;
	if( packet_has_data )
    {
		if( packet_is_batch )
		{
			// Packet has data and is a batch. This means it contains 'batch_length' registers, each
			// of which has a length of 4 bytes
			data_length = 4*batch_length;
			//printf("Packet is batch, length = %i\n", (int)(data_length));
		}
		else // Packet has data but is not a batch. This means it contains one register (4 bytes)   
		{
			data_length = 4;
		}
    }
	else // Packet has no data
    {
		data_length = 0;
    }
    
	// At this point, we know exactly how long the packet is. Now we can check to make sure
	// we have enough data for the full packet.
	if( (rx_length - packet_index) < (data_length + 5) )
    {
		//printf("not enough data for full packet!\n");
		//printf("rx_length %d, packet_index %d, data_length+5 %d\n", rx_length, packet_index, data_length+5); 
		return 3;
    }
    
	// If we get here, we know that we have a full packet in the buffer. All that remains is to pull
	// out the data and make sure the checksum is good.
	// Start by extracting all the data
	packet->address = rx_data[packet_index + 4];
	
	//printf("packet address = %i\n", (int)(packet->Address));
	
	packet->packet_type = PT;

	// Get the data bytes and compute the checksum all in one step
	packet->n_data_bytes = data_length;
	uint16_t computed_checksum = 's' + 'n' + 'p' + packet->packet_type + packet->address;	
	
	for( index = 0; index < data_length; index++ )
    {
		// Copy the data into the packet structure's data array
		packet->data[index] = rx_data[packet_index + 5 + index];
		// Add the new byte to the checksum
		computed_checksum += packet->data[index];		
    }    
   
	// Now see if our computed checksum matches the received checksum
	// First extract the checksum from the packet
	uint16_t received_checksum = (rx_data[packet_index + 5 + data_length] << 8);

	received_checksum |= rx_data[packet_index + 6 + data_length];
	// Now check to see if they don't match
	if( received_checksum != computed_checksum )
    {
		//printf("checksum bad!\n");
		return 4;
    }
    
    //printf("checksum good!\n");
	packet->checksum = computed_checksum;
	// At this point, we've received a full packet with a good checksum. It is already
	// fully parsed and copied to the packet structure, so return 0 to indicate that a packet was
	// processed.
	return 0;
}



int uartInit()
{
	uart_fd = open("/dev/ttyPS1", O_RDWR | O_NOCTTY | O_NDELAY);

	if(uart_fd == -1)
	{
		cprint("[!!] ", BRIGHT, RED);
		fprintf(stderr, "Failed to init uart.\n");
		return -1;
	}
	
	struct termios settings;
	tcgetattr(uart_fd, &settings);

	/*  CONFIGURE THE UART
	*  The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):

	*	Baud rate:- B1200, B2400, B4800, B9600, B19200,
	*	B38400, B57600, B115200, B230400, B460800, B500000,
	*	B576000, B921600, B1000000, B1152000, B1500000,
	*	B2000000, B2500000, B3000000, B3500000, B4000000
	*	CSIZE:- CS5, CS6, CS7, CS8
	*	CLOCAL - Ignore modem status lines
	* 	CREAD - Enable receiver
	*	IGNPAR = Ignore characters with parity errors
	*	ICRNL - Map CR to NL on input (Use for ASCII comms
	*	where you want to auto correct end of line characters
	*	- don't us e for bianry comms!)
	*	PARENB - Parity enable
	*	PARODD - Odd parity (else even) */

	/* Set baud rate to 115200 */
	speed_t baud_rate = B115200;

	/* Baud rate fuctions
	* cfsetospeed - Set output speed
	* cfsetispeed - Set input speed
	* cfsetspeed  - Set both output and input speed */

	cfsetspeed(&settings, baud_rate);

	settings.c_cflag &= ~PARENB; /* no parity */
	settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
	settings.c_cflag &= ~CSIZE;
	settings.c_cflag |= CS8 | CLOCAL; /* 8 bits */
	settings.c_lflag = ICANON; /* canonical mode */
	settings.c_oflag &= ~OPOST; /* raw output */

	/* Setting attributes */
	tcflush(uart_fd, TCIFLUSH);
	tcsetattr(uart_fd, TCSANOW, &settings);
	
	cprint("[OK] ", BRIGHT, GREEN);
	printf("UART initialised.\n");

	return 0;
}

int uartReadRaw(uint8_t* buffer, int size)
{
	/* Don't block serial read */
	fcntl(uart_fd, F_SETFL, FNDELAY); 
  
	if(uart_fd == -1)
	{
		cprint("[!!] ", BRIGHT, RED);
		fprintf(stderr, "Failed to read from UART.\n");
		return -1;
	}
		
	if(read(uart_fd, (void*)buffer, size) < 0)
	{
		//No data yet avaliable
		return -1;
	}
	else
	{
		buffer[size] = '\0';		
		return 0;
	}  
	
	tcflush(uart_fd, TCIFLUSH); //flush the uart buffer
	
}

int uartRead(int size)
{
	/* Don't block serial read */
	fcntl(uart_fd, F_SETFL, FNDELAY); 
  
	while(1)
	{
		if(uart_fd == -1)
		{
			cprint("[!!] ", BRIGHT, RED);
			fprintf(stderr, "Failed to read from UART.\n");
			return -1;
		}
		
		unsigned char rx_buffer[size];
		
		int rx_length = read(uart_fd, (void*)rx_buffer, size);
		
		if (rx_length < 0)
		{
			/* No data yet avaliable, check again */
			if(errno == EAGAIN)
			{
				//fprintf(stderr, "AGAIN!\n");
				continue;
				/* Error differs */
			} 
			else
			{
				fprintf(stderr, "Error!\n");
				return -1;
			}
		  
		}
		else if (rx_length == 0)
		{
		//no data waiting
		}
		else
		{
			rx_buffer[rx_length] = '\0';
			if(parse_serial_data(rx_buffer,rx_length,&global_packet) == 0)
			{
				if (global_packet.address == 0xfd)
				{
					return -2; //bad checksum
				}
				else
				{
					return 0; //All good!
				}
			}
			else
			{
				return -3; //No data received
			}	
	
			break;		
		}
	} 
	
	tcflush(uart_fd, TCIFLUSH);  
}

 int uartWrite(UM7_packet* packet){
  
	/* Write some sample data into UART */
	/* ----- TX BYTES ----- */
	int msg_len = packet->n_data_bytes + 7;

	int count = 0;
	char tx_buffer[msg_len+1];
	//Add header to buffer
	tx_buffer[0] = 's';
	tx_buffer[1] = 'n';
	tx_buffer[2] = 'p';
	tx_buffer[3] = packet->packet_type;
	tx_buffer[4] = packet->address;
	
	//Calculate checksum and add data to buffer
	uint16_t checksum = 's' + 'n' + 'p' + tx_buffer[3] + tx_buffer[4];
	int i = 0;
	
	for (i = 0; i < packet->n_data_bytes;i++)
	{
	  tx_buffer[5+i] = packet->data[i];
	  checksum+= packet->data[i];
	}
	
	tx_buffer[5+i] = checksum >> 8;
	tx_buffer[6+i] = checksum & 0xff;
	tx_buffer[msg_len++] = 0x0a; //New line numerical value

	if(uart_fd != -1)
	{
		count = write(uart_fd, &tx_buffer, (msg_len)); //Transmit
	}
	
	if(count < 0)
	{
		cprint("[!!] ", BRIGHT, RED);
		fprintf(stderr, "UART TX error.\n");
		return -1;
	}
	
	return 0;
}

int releaseConnection()
{
	tcflush(uart_fd, TCIFLUSH);
	close(uart_fd);

	return 0;
}


int getFirmwareVersion(void)
{
	UM7_packet txpacket;
	txpacket.address = 0xAA;
	txpacket.packet_type = 0x00;
	txpacket.n_data_bytes = 0;

	if(uartWrite(&txpacket) < 0)
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Uart write error\n");
		return -1;
	}

	//If reveived data was bad or wrong address, repeat transmission and reception
	while((uartRead(20) < 0)||(global_packet.address!=txpacket.address))
	{
		if(uartWrite(&txpacket) < 0)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("Uart write error\n");
			return -1;
		}
	}
	
	char FWrev[5];
	FWrev[0] = global_packet.data[0];
	FWrev[1] = global_packet.data[1];
	FWrev[2] = global_packet.data[2];
	FWrev[3] = global_packet.data[3];
	FWrev[4] = '\0'; //Null-terminate string

	cprint("[OK] ", BRIGHT, GREEN);
	printf("Firmware version: %s\n", FWrev);
	
	return 0;
}


int flashCommit(void)
{
	UM7_packet txpacket;
	txpacket.address = 0xAB;
	txpacket.packet_type = 0x00;
	txpacket.n_data_bytes = 0;

	if(uartWrite(&txpacket) < 0)
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Uart write error\n");
		return -1;
	}

	//If reveived data was bad or wrong address, repeat transmission and reception
	while((uartRead(20) < 0)||(global_packet.address!=txpacket.address))
	{
		if(uartWrite(&txpacket) < 0)
		{
			printf("Uart write error\n");
			return -1;
		}
	}

	cprint("[OK] ", BRIGHT, GREEN);
	printf("Flash Commit.\n");
	return 0;
}


int factoryReset(void)
{
	UM7_packet txpacket;
	txpacket.address = 0xAC;
	txpacket.packet_type = 0x00;
	txpacket.n_data_bytes = 0;

	if(uartWrite(&txpacket) < 0)
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Uart write error\n");
		return -1;
	}

	//If reveived data was bad or wrong address, repeat transmission and reception
	while((uartRead(20) < 0)||(global_packet.address!=txpacket.address))
	{
		if(uartWrite(&txpacket) < 0)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("Uart write error\n");
			return -1;
		}
	}

	cprint("[OK] ", BRIGHT, GREEN);
	printf("Factory Reset\n");
	return 0;
}

int zeroGyros(void)
{
	UM7_packet txpacket;
	txpacket.address = 0xAD;
	txpacket.packet_type = 0x00;
	txpacket.n_data_bytes = 0;

	if(uartWrite(&txpacket) < 0)
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Uart write error\n");
		return -1;
	}

	//If reveived data was bad or wrong address, repeat transmission and reception
	while((uartRead(20) < 0)||(global_packet.address!=txpacket.address))
	{
		if(uartWrite(&txpacket) < 0)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("Uart write error\n");
			return -1;
		}
	}

	cprint("[OK] ", BRIGHT, GREEN);
	printf("Zero gyros\n");
	return 0;
}

int setHomePosition(void)
{
	UM7_packet txpacket;
	txpacket.address = 0xAE;
	txpacket.packet_type = 0x00;
	txpacket.n_data_bytes = 0;

	if(uartWrite(&txpacket) < 0)
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Uart write error\n");
		return -1;
	}

	//If reveived data was bad or wrong address, repeat transmission and reception
	while((uartRead(20) < 0)||(global_packet.address!=txpacket.address))
	{
		if(uartWrite(&txpacket) < 0)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("Uart write error\n");
			return -1;
		}
	}
	
	cprint("[OK] ", BRIGHT, GREEN);
	printf("Set home position.\n");
	return 0;
}
int setMagReference(void)
{
	UM7_packet txpacket;
	txpacket.address = 0xB0;
	txpacket.packet_type = 0x00;
	txpacket.n_data_bytes = 0;

	if(uartWrite(&txpacket) < 0)
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Uart write error\n");
		return -1;
	}

	//If reveived data was bad or wrong address, repeat transmission and reception
	while((uartRead(20) < 0)||(global_packet.address != txpacket.address))
	{
		if(uartWrite(&txpacket) < 0)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("Uart write error\n");
			return -1;
		}
	}

	cprint("[OK] ", BRIGHT, GREEN);
	printf("Set mag reference.\n");
	return 0;
}

int resetEKF(void)
{
	UM7_packet txpacket;
	txpacket.address = 0xB3;
	txpacket.packet_type = 0x00;
	txpacket.n_data_bytes = 0;

	if(uartWrite(&txpacket) < 0)
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Uart write error\n");
		return -1;
	}

	/* //If reveived data was bad or wrong address, repeat transmission and reception */
	/* while((uart_read(20) < 0)||(packet0.Address!=txpacket.Address)){ */
	/*   if(uart_write(&txpacket) < 0){ */
	/*     printf("Uart write error\n"); */
	/*     return -1; */
	/*   } */
	/* } */

	cprint("[OK] ", BRIGHT, GREEN);
	printf("Reset EKF\n");
	return 0;
}


// write to a 32 bit register
void writeRegister(uint8_t address, uint8_t *data)
{
	UM7_packet packet;
	
	packet.packet_type = 0b10000000; 	// has data
	packet.address = address;		    // register address
	packet.n_data_bytes = 4;			// 32-bit register
	packet.data[0] = data[0];		
	packet.data[1] = data[1];
	packet.data[2] = data[2];
	packet.data[3] = data[3];	

	if(uartWrite(&packet) < 0)
		printf("UART baud rate write error\n");	
	
	int attempt = 0;	
	//If reveived data was bad or wrong address, repeat transmission and reception
	while((uartRead(25) < 0)||(global_packet.address != packet.address))
	{
		if(uartWrite(&packet) < 0)
			printf("UART write error\n");
			
		if(attempt++ == 100)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("No reply from imu register %i.\n", packet.address);
			break;
		}
	}
}








