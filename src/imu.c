#include "imu.h"

// UART file discriptor
int uart_fd = -1; 

// global zero buffer
uint8_t zero_buffer[4] = {0, 0, 0, 0};

// global UM7 Packet for data reception
UM7_packet global_packet;

// Parse the serial data obtained through the UART interface and fit to a general packet structure
uint8_t parseSerialData(uint8_t* rx_data, uint8_t rx_length, UM7_packet* packet)
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

void initIMU(void)
{
	initUART();		
	getFirmwareVersion();
	resetEKF();	
	zeroGyros();
	setMagReference();
	setHomePosition();	
	//factoryReset();
	
	uint8_t zero[4] = {0, 0, 0, 0};
	uint8_t all_proc[4] = {0, 0, 0, 255};
	//uint8_t position[4] = {0, 0, 255, 0};
	
	writeRegister(CREG_COM_RATES1, 4, zero);		// raw gyro, accel and mag rate	
	writeRegister(CREG_COM_RATES2, 4, zero);		// temp rate and all raw data rate		
	writeRegister(CREG_COM_RATES3, 4, zero);		// proc accel, gyro, mag rate		
	writeRegister(CREG_COM_RATES4, 4, all_proc);	// all proc data rate	
	writeRegister(CREG_COM_RATES5, 4, zero);		// quart, euler, position, velocity rate
	writeRegister(CREG_COM_RATES6, 4, zero);		// heartbeat rate
}

void initUART(void)
{
	releaseConnection();
	
	uart_fd = open("/dev/ttyPS1", O_RDWR | O_NOCTTY | O_NDELAY);

	if(uart_fd == -1)
	{
		cprint("[!!] ", BRIGHT, RED);
		fprintf(stderr, "Failed to init UART.\n");
		exit(EXIT_FAILURE);
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

	/* Baud rate functions
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
	printf("UART Initialised.\n");
}

int rxPacket(int size)
{
	tcflush(uart_fd, TCIFLUSH); 
	
	// don't block serial read 
	fcntl(uart_fd, F_SETFL, FNDELAY); 
	
	unsigned char rx_buffer[size];
  
	while(1)
	{
		if(uart_fd == -1)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("UART has not been initialized.\n");
			return -1;
		}
		
		// perform uart read
		int rx_length = read(uart_fd, (void*)rx_buffer, size);
		
		if (rx_length == -1)
		{
			//printf("No UART data available yet, check again.\n");
			if(errno == EAGAIN)
			{
				// an operation that would block was attempted on an object that has non-blocking mode selected.
				//printf("UART read blocked, try again.\n");
				continue;
			} 
			else
			{
				printf("Error reading from UART.\n");
				return -1;
			}
		  
		}
		else if (rx_length == size)
		{	
			if(parseSerialData(rx_buffer, rx_length, &global_packet) == 0)
			{
				//printf("All good.\n");
				return 0; 
			}
			else
			{
				//printf("No useable packet received.\n");
				return -2; 		
			}		
		}
	}  
}


int txPacket(UM7_packet* packet)
{  
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
		tx_buffer[5 + i] = packet->data[i];
		checksum += packet->data[i];
	}
	
	tx_buffer[5 + i] = checksum >> 8;
	tx_buffer[6 + i] = checksum & 0xff;
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


int releaseConnection(void)
{
	tcflush(uart_fd, TCIFLUSH);
	close(uart_fd);

	return 0;
}


void getFirmwareVersion(void)
{
	writeRegister(GET_FW_REVISION, 0, zero_buffer);
	checkCommandSuccess("Check Firmware");
	
	char FWrev[5];
	FWrev[0] = global_packet.data[0];
	FWrev[1] = global_packet.data[1];
	FWrev[2] = global_packet.data[2];
	FWrev[3] = global_packet.data[3];
	FWrev[4] = '\0'; //Null-terminate string

	cprint("[**] ", BRIGHT, CYAN);
	printf("Firmware Version: %s\n", FWrev);
}


void flashCommit(void)
{
	writeRegister(FLASH_COMMIT, 0, zero_buffer);	
	checkCommandSuccess("Flash Commit");
}


void factoryReset(void)
{
	writeRegister(RESET_TO_FACTORY, 0, zero_buffer);
	checkCommandSuccess("Factory Reset");
}


void zeroGyros(void)
{
	writeRegister(ZERO_GYROS, 0, zero_buffer);
	checkCommandSuccess("Zero Gyros");
}


void setHomePosition(void)
{	
	writeRegister(SET_HOME_POSITION, 0, zero_buffer);	
	checkCommandSuccess("Set GPS Home");
}


void setMagReference(void)
{
	writeRegister(SET_MAG_REFERENCE, 0, zero_buffer);
	checkCommandSuccess("Set Mag Reference");
}


void resetEKF(void)
{
	writeRegister(RESET_EKF, 0, zero_buffer);
	checkCommandSuccess("Reset EKF");
}


void writeRegister(uint8_t address, uint8_t n_data_bytes, uint8_t *data)
{
	UM7_packet packet;		

	packet.address = address;
	packet.packet_type = 0x00;
	packet.n_data_bytes = n_data_bytes;	
	
	if (n_data_bytes != 0)
	{
		// packet contains data
		packet.packet_type |= PT_HAS_DATA; 					
	}	
	
	// populate packet data
	for (int i = 0; i < n_data_bytes; i++)
	{
		packet.data[i] = data[i];
	}		

	if (txPacket(&packet) < 0)
	{
		printf("UART baud rate write error\n");	
	}	
		
	int attempt_n = 0;	
		
	//If reveived data was bad or wrong address, repeat transmission and reception
	while ((rxPacket(20) < 0) || (global_packet.address != packet.address))
	{
		if (txPacket(&packet) < 0)
		{
			printf("UART baud rate write error\n");	
		}	
			
		if (attempt_n++ == 100)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("No response from UM7_R%i after 100 attempts.\n", packet.address);
			break;
		}
	}	
}


void checkCommandSuccess(char* command_name)
{
	if (global_packet.packet_type & PT_CF)
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("%s Error.\n", command_name);
		exit(EXIT_FAILURE);
	}
	else
	{
		cprint("[OK] ", BRIGHT, GREEN);
		printf("%s.\n", command_name);
	}
}


void readRegister(uint8_t address)
{
	rxPacket(60);
	//writeRegister(address, 0, zero_buffer);
	//printf("IMU Register %i: %f\n", address, bit32ToFloat(bit8ArrayToBit32(global_packet.data)));	
	
	if ((global_packet.address == address) && (global_packet.packet_type &= PT_IS_BATCH))
	{				
		system("clear\n");
		printf("UM7_R%i: %f\n", global_packet.address+0, bit32ToFloat(bit8ArrayToBit32(&global_packet.data[0])));
		printf("UM7_R%i: %f\n", global_packet.address+1, bit32ToFloat(bit8ArrayToBit32(&global_packet.data[4])));
		printf("UM7_R%i: %f\n", global_packet.address+2, bit32ToFloat(bit8ArrayToBit32(&global_packet.data[8])));
		printf("UM7_R%i: %f\n", global_packet.address+3, bit32ToFloat(bit8ArrayToBit32(&global_packet.data[12])));
	}
	else if (global_packet.address == address)
	{
		printf("UM7_R%i: %f\n", global_packet.address, bit32ToFloat(bit8ArrayToBit32(&global_packet.data[0])));
	}
	
}


float bit32ToFloat(uint32_t bit32)
{
	//https://en.wikipedia.org/wiki/Single-precision_floating-point_format	
	
	int sign = 1;
	float fraction = 1;	
	int exponent = -127;		
	
	float singleFloat = 0;		
	
	if (bit32 & (uint32_t)(1 << 31))
	{
		sign = -1;
	}
	
	for (int i = 1; i < 24; i++)
	{
		if (bit32 & (uint32_t)(1 << (23 - i)))
		{
			fraction += pow(2, -i);
		}
	}
	
	for (int i = 0; i < 8; i++)
	{
		if (bit32 & (uint32_t)(1 << (23 + i)))
		{
			exponent += pow(2, i);
		}
	}		
	
	singleFloat = sign*fraction*pow(2, exponent);
	
	return singleFloat;
}


void procHealth(UM7_packet* healthPacket)
{
	uint32_t healthBit32 = bit8ArrayToBit32(healthPacket->data);
	
	if (healthBit32 & (uint32_t)(1 << 0)) 
	{
		cprint("[**] ", BRIGHT, RED);
		printf("No GPS data for 2 seconds.\n");
	}
	
	if (healthBit32 & (uint32_t)(1 << 1)) 
	{
		cprint("[**] ", BRIGHT, RED);
		printf("Mag failed to init on startup.\n");
	}		
	
	if (healthBit32 & (uint32_t)(1 << 2))
	{
		cprint("[**] ", BRIGHT, RED);
		printf("Gyro failed to init on startup.\n");
	}		 
	 
	if (healthBit32 & (uint32_t)(1 << 3)) 
	{
		cprint("[**] ", BRIGHT, RED);
		printf("Acc failed to init on startup.\n");
	}		
	
	if (healthBit32 & (uint32_t)(1 << 4)) 
	{
		cprint("[**] ", BRIGHT, RED);
		printf("Acc norm exceeded - aggressive acceleration detected.\n");
	}	
	
	if (healthBit32 & (uint32_t)(1 << 5)) 
	{
		cprint("[**] ", BRIGHT, RED);
		printf("Mag norm exceeded - bad calibration.\n");
	}
	
	if (healthBit32 & (uint32_t)(1 << 8))
	{
		cprint("[**] ", BRIGHT, RED);
		printf("UART overflow - reduce broadcast rates.\n");
	}
	 
	
	int satInView = 0;
	int satUsed = 0;
	
	for (int i = 0; i < 6; i++)
	{
		if (healthBit32 & (uint32_t)(1 << (10 + i)))
		{
			satInView += pow(2, i);
		}
		
		if (healthBit32 & (uint32_t)(1 << (26 + i)))
		{
			satUsed += pow(2, i);
		}
	}
	
	cprint("[**] ", BRIGHT, CYAN);
	printf("Satellites currently in view: %i\n", satInView);	
	
	cprint("[**] ", BRIGHT, CYAN);
	printf("Satellites used in position calculation: %i\n", satUsed);
}


uint32_t bit8ArrayToBit32(uint8_t *data)
{
	uint32_t bit32 = 0;
	
	bit32 = (uint32_t)(data[3] << 0);
	bit32 += (uint32_t)(data[2] << 8);
	bit32 += (uint32_t)(data[1] << 16);
	bit32 += (uint32_t)(data[0] << 24);
	
	return bit32;
}





