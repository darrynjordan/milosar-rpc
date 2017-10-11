#include "imu.h"

// global zero buffer
uint8_t zero_buffer[4] = {0, 0, 0, 0};

packet global_packet;

// global healthbeat struct
heartbeat beat;

// Parse the serial data obtained through the UART interface and fit to a general packet structure
uint8_t parseUART(uint8_t* rx_data, uint8_t rx_length)
{
	uint8_t index;
	// Make sure that the data buffer provided is long enough to contain a full packet
	// The minimum packet length is 7 bytes
	if( rx_length < 7 )
	{
		return -1;
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
		return -2;
    }
    
	// If we get here, a packet header was found. Now check to see if we have enough room
	// left in the buffer to contain a full packet. Note that at this point, the variable 'packet_index'
	// contains the location of the 's' character in the buffer (the first byte in the header)
	if ((rx_length - packet_index) < 7)
	{
		//printf("not enough room after 's' for a full packet\n");
		return -3;
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
		return -4;
    }
    
	// If we get here, we know that we have a full packet in the buffer. All that remains is to pull
	// out the data and make sure the checksum is good.
	// Start by extracting all the data
	global_packet.address = rx_data[packet_index + 4];	
	global_packet.packet_type = PT;

	// Get the data bytes and compute the checksum all in one step
	global_packet.n_data_bytes = data_length;
	uint16_t computed_checksum = 's' + 'n' + 'p' + global_packet.packet_type + global_packet.address;	
	
	for( index = 0; index < data_length; index++ )
    {
		// Copy the data into the packet structure's data array
		global_packet.data[index] = rx_data[packet_index + 5 + index];
		// Add the new byte to the checksum
		computed_checksum += global_packet.data[index];		
    }    
   
	// Now see if our computed checksum matches the received checksum
	// First extract the checksum from the packet
	uint16_t received_checksum = (rx_data[packet_index + 5 + data_length] << 8);

	received_checksum |= rx_data[packet_index + 6 + data_length];
	// Now check to see if they don't match
	if (received_checksum != computed_checksum)
    {
		//printf("checksum bad!\n");
		return -6;
    }
    
    //printf("checksum good!\n");
	global_packet.checksum = computed_checksum;
	// At this point, we've received a full packet with a good checksum. It is already
	// fully parsed and copied to the packet structure, so return 0 to indicate that a packet was
	// processed.
	return 1;
}


void initIMU(Experiment *experiment)
{
	//baud rate of the UM7 main serial port = 115200
	//baud rate of the UM7 auxiliary serial port = 57600

	uint8_t com_settings[4] = {4 + (5 << 4), 0, 1, 0};	
	uint8_t all_proc[4] = {0, 0, 0, 255};
	uint8_t health[4] = {0, 6, 0, 0};
	//uint8_t position[4] = {0, 0, 255, 0};	
	
	writeRegister(CREG_COM_SETTINGS, 4, com_settings);		// baud rates, auto transmission		
	writeRegister(CREG_COM_RATES1, 4, zero_buffer);			// raw gyro, accel and mag rate	
	writeRegister(CREG_COM_RATES2, 4, zero_buffer);			// temp rate and all raw data rate		
	writeRegister(CREG_COM_RATES3, 4, zero_buffer);			// proc accel, gyro, mag rate		
	writeRegister(CREG_COM_RATES4, 4, all_proc);			// all proc data rate	
	writeRegister(CREG_COM_RATES5, 4, zero_buffer);			// quart, euler, position, velocity rate
	writeRegister(CREG_COM_RATES6, 4, health);				// heartbeat rate
	writeRegister(CREG_COM_RATES7, 4, zero_buffer);			// CHR NMEA-style packets
	
	if (experiment->is_debug_mode)
	{
		writeCommand(GET_FW_REVISION);
		
		char FWrev[5];
		FWrev[0] = global_packet.data[0];
		FWrev[1] = global_packet.data[1];
		FWrev[2] = global_packet.data[2];
		FWrev[3] = global_packet.data[3];
		FWrev[4] = '\0'; //Null-terminate string

		cprint("[**] ", BRIGHT, CYAN);
		printf("Firmware Version: %s\n", FWrev);
	}
	
	writeCommand(RESET_EKF);
	writeCommand(ZERO_GYROS);
	
	getHeartbeat(25);
	
	writeCommand(SET_MAG_REFERENCE);
	writeCommand(SET_HOME_POSITION);
}


int txPacket(packet* tx_packet)
{  
	int msg_len = tx_packet->n_data_bytes + 7;

	int count = 0;
	char tx_buffer[msg_len + 1];
	//Add header to buffer
	tx_buffer[0] = 's';
	tx_buffer[1] = 'n';
	tx_buffer[2] = 'p';
	tx_buffer[3] = tx_packet->packet_type;
	tx_buffer[4] = tx_packet->address;
	
	//Calculate checksum and add data to buffer
	uint16_t checksum = 's' + 'n' + 'p' + tx_buffer[3] + tx_buffer[4];
	int i = 0;
	
	for (i = 0; i < tx_packet->n_data_bytes;i++)
	{
		tx_buffer[5 + i] = tx_packet->data[i];
		checksum += tx_packet->data[i];
	}
	
	tx_buffer[5 + i] = checksum >> 8;
	tx_buffer[6 + i] = checksum & 0xff;
	tx_buffer[msg_len++] = 0x0a; //New line numerical value

	count = write(getFileID(), &tx_buffer, (msg_len)); //Transmit
	
	if(count < 0)
	{
		cprint("[!!] ", BRIGHT, RED);
		fprintf(stderr, "UART TX error.\n");
		return -1;
	}
	
	return 1;
}

//searches for the first valid paket within 'size' samples of the UART buffer
int rxPacket(int size)
{
	if(parseUART(getUART(size), size))
	{
		return 1; 
	}
	else
	{
		//printf("No useable packet received.\n");
		return -1; 		
	}		
}


//saves the data within the provided packet
int svPacket(packet* sv_packet)
{
	/*if ((sv_packet.address == DREG_ALL_PROC) && (sv_packet.packet_type &= PT_IS_BATCH))
	{		
		for (int i = 0; i < 12; i++)
		{
			float data = bit32ToFloat(bit8ArrayToBit32(&global_packet.data[4*i]));
			fwrite(&data, sizeof(float), 1, imuFile);
		}					
	}*/
	return 1;
}


int writeRegister(uint8_t address, uint8_t n_data_bytes, uint8_t *data)
{
	packet tx_packet;	

	tx_packet.address = address;
	tx_packet.packet_type = 0x00;
	tx_packet.n_data_bytes = n_data_bytes;	
	
	if (n_data_bytes != 0)
	{
		// packet contains data
		tx_packet.packet_type |= PT_HAS_DATA; 					
	}	
	
	// populate packet data
	for (int i = 0; i < n_data_bytes; i++)
	{
		tx_packet.data[i] = data[i];
	}			
		
	int i = 0;	
		
	//If reveived data was bad or wrong address, repeat transmission and reception
	do 
	{
		if (i++ == 150)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("No response from ");
			
			switch (tx_packet.address)
			{
				case 0: 
					printf("CREG_COM_SETTINGS.\n");
					break;
				case 1: 
					printf("CREG_COM_RATES1.\n");
					break;	
				case 2:
					printf("CREG_COM_RATES2.\n");
					break;	
				case 3:
					printf("CREG_COM_RATES3.\n");
					break;	
				case 4:
					printf("CREG_COM_RATES4.\n");
					break;
				case 5:	
					printf("CREG_COM_RATES5.\n");
					break;
				case 6:
					printf("CREG_COM_RATES6.\n");
					break;
				case 7:
					printf("CREG_COM_RATES7.\n");
					break;
				case 8:
					printf("CREG_MISC_SETTINGS.\n");
					break;
				case 85:
					printf("DREG_HEALTH.\n");
					break;
				case 179:
					printf("RESET_EKF.\n");
					break;
				default:
					printf("UM7_R%i.\n", tx_packet.address);
					break;
			}
	
			return -1;
		}
		
		txPacket(&tx_packet);
		rxPacket(25);
	} 
	while (global_packet.address != address);
	
	return 1;
}


void writeCommand(int command)
{
	if(writeRegister(command, 0, zero_buffer))
	{
		if (global_packet.packet_type & PT_CF)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("%i Error.\n", command);
			exit(EXIT_FAILURE);
		}
		else
		{
			cprint("[OK] ", BRIGHT, GREEN);			
			
			switch (command)
			{
				case 170: 
					printf("Received firmware version.\n");
					break;
				case 171: 
					printf("Flash committed.\n");
					break;	
				case 172:
					printf("Reset to factory settings.\n");
					break;	
				case 173:
					printf("Gyros zero.\n");
					break;	
				case 174:
					printf("GPS home position set.\n");
					break;
				case 176:	
					printf("Mag reference set.\n");
					break;
				case 179:
					printf("Extended Kalman filter reset.\n");
					break;
			}			
		}				
	}
}


void readRegister(uint8_t address)
{
	writeRegister(address, 0, zero_buffer);
	//printf("IMU Register %i: %f\n", address, bit32ToFloat(bit8ArrayToBit32(rx_packet.data)));	
	
	if (global_packet.packet_type &= PT_IS_BATCH)
	{				
		system("clear\n");
		printf("UM7_R%i: %f\n", global_packet.address+0, bit8ArrayToFloat(&global_packet.data[0]));
		printf("UM7_R%i: %f\n", global_packet.address+1, bit8ArrayToFloat(&global_packet.data[4]));
		printf("UM7_R%i: %f\n", global_packet.address+2, bit8ArrayToFloat(&global_packet.data[8]));
		printf("UM7_R%i: %f\n", global_packet.address+3, bit8ArrayToFloat(&global_packet.data[12]));
	}
	else 
	{
		printf("UM7_R%i: ", global_packet.address);
		for (int i = 0; i < 4; i++)
			printf(" %i", global_packet.data[i]);
		printf("\n");
	}	
}


void getHeartbeat(int size)
{
	//wait until valid health packet is received
	while((global_packet.address != DREG_HEALTH) && parseUART(getUART(size), size));
	
	uint32_t health = bit8ArrayToBit32(global_packet.data);
	int satsView = 0;
	int satsUsed = 0;
	
	beat.gps_fail = checkBit(health, 0);
	beat.mag_fail = checkBit(health, 1);
	beat.gyro_fail = checkBit(health, 2);
	beat.acc_fail = checkBit(health, 3);
	beat.acc_norm = checkBit(health, 4);
	beat.mag_norm = checkBit(health, 5);
	beat.uart_fail = checkBit(health, 8);
	
	for (int i = 0; i < 6; i++)
	{
		if (health & (uint32_t)(1 << (10 + i)))
		{
			satsView += pow(2, i);
		}
		
		if (health & (uint32_t)(1 << (26 + i)))
		{
			satsUsed += pow(2, i);
		}
	}
	
	beat.sats_used = satsUsed;	
	beat.sats_view = satsView;
}


void showHeartbeat(Experiment *experiment)
{
	if (experiment->is_debug_mode)
	{
		if (beat.gps_fail) 
		{
			cprint("[**] ", BRIGHT, RED);
			printf("No GPS data for 2 seconds.\n");
		}
		
		if (beat.mag_fail) 
		{
			cprint("[**] ", BRIGHT, RED);
			printf("Mag failed to init on startup.\n");
		}		
		
		if (beat.gyro_fail)
		{
			cprint("[**] ", BRIGHT, RED);
			printf("Gyro failed to init on startup.\n");
		}	
	 
		if (beat.acc_fail) 
		{
			cprint("[**] ", BRIGHT, RED);
			printf("Acc failed to init on startup.\n");
		}		
		
		if (beat.acc_norm) 
		{
			cprint("[**] ", BRIGHT, RED);
			printf("Acc norm exceeded - aggressive acceleration detected.\n");
		}	
		
		if (beat.mag_norm) 
		{
			cprint("[**] ", BRIGHT, RED);
			printf("Mag norm exceeded - bad calibration.\n");
		}
		
		if (beat.uart_fail)
		{
			cprint("[**] ", BRIGHT, RED);
			printf("UART overflow - reduce broadcast rates.\n");
		}		
		
		cprint("[**] ", BRIGHT, CYAN);
		printf("Satellites in view: %i\n", beat.sats_view);	

		cprint("[**] ", BRIGHT, CYAN);
		printf("Satellites used: %i\n", beat.sats_used);
	}
	else
	{
		int imu_sum = (beat.mag_fail + beat.mag_fail + beat.gyro_fail + beat.acc_fail + beat.acc_norm + beat.mag_norm);
		
		char* gps_status = (beat.gps_fail) ? "NO" : "OK";
		char* uart_status = (beat.uart_fail) ? "NO" : "OK";
		char* imu_status = (imu_sum) ? "NO" : "OK";
		
		printf("| GPS | IMU | UART | SATS |\n");
		printf("---------------------------\n");
		printf("| %s  | %s  | %s   | %3i  |\n", gps_status, imu_status, uart_status, beat.sats_view);	
	}
	
	printf("\033[%iA\n", 4);
}








