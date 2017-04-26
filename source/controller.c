#include "controller.h"

//load ramp parameters from ini files
void getParameters(Synthesizer *synth)
{
	//ensure that the register array is cleared
	memset(synth->binaryRegisterArray, 0, sizeof(synth->binaryRegisterArray));
	
	//reset all ramp parameters
	for(int i = 0; i < MAX_RAMPS; i++)
	{
		synth->ramps[i].number = i;  
		synth->ramps[i].bandwidth = 0;  
		synth->ramps[i].next = 0;    
		synth->ramps[i].trigger = 0;
		synth->ramps[i].reset = 0;
		synth->ramps[i].flag = 0;	
		synth->ramps[i].doubler = 0;
		synth->ramps[i].length = 0;
		synth->ramps[i].increment = 0;
	}	
	
	char* filename = (char*)malloc(50*sizeof(char));
	strcpy(filename, "ramps/");
	strcat(filename, synth->parameterFile);	
	
	if (ini_parse(filename, handler, synth) < 0) 
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Could not open %s. Check that the file name is correct.\n", filename);
		exit(EXIT_FAILURE);
    }   
}


//handler function called for every element in the ini file
//current implementation is inefficient, but no alternative could be found
int handler(void* pointer, const char* section, const char* attribute, const char* value)
{			
	char* rampSection = (char*)malloc(50*sizeof(char));
	Synthesizer* synth = (Synthesizer*)pointer;
	
	#define MATCH(s, n) strcmp(section, s) == 0 && strcmp(attribute, n) == 0
	
	if (MATCH("setup", "frac_num")) synth->fractionalNumerator = atoi(value);
	
	for(int i = 0; i < MAX_RAMPS; i++)
	{
		sprintf(rampSection, "ramp%i", i);

		if (MATCH(rampSection, "length")) 			synth->ramps[i].length = atof(value); 
		else if (MATCH(rampSection, "bandwidth")) 	synth->ramps[i].bandwidth = atof(value);  
		else if (MATCH(rampSection, "increment")) 	synth->ramps[i].increment = atof(value);
		else if (MATCH(rampSection, "next")) 		synth->ramps[i].next = atoi(value);    
		else if (MATCH(rampSection, "trigger")) 	synth->ramps[i].trigger = atoi(value);
		else if (MATCH(rampSection, "reset")) 		synth->ramps[i].reset = atoi(value);
		else if (MATCH(rampSection, "flag")) 		synth->ramps[i].flag = atoi(value);	
		else if (MATCH(rampSection, "doubler")) 	synth->ramps[i].doubler = atoi(value);	
	}					    
    return 1;	
}


void calculateRampParameters(Synthesizer *synth, Experiment *experiment)
{	
	if (experiment->is_debug_mode) 
	{
		cprint("\n[**] ", BRIGHT, CYAN);	
		printf("Synthesizer %i loaded with %s:\n", synth->number, synth->parameterFile);
		printf("Frequency Offset: %f [MHz]\n", vcoOut(synth->fractionalNumerator));
		printf("Fractional Numerator: %d\n", synth->fractionalNumerator);		
		printf("| NUM | NXT | RST | DBL |   LEN |            INC |      BNW |\n");
	}
	
	float phase_detector_frequency = 100.0; // after doubler and divider [Hz]
	
	for (int i = 0; i < 8; i++)
	{
		if (synth->ramps[i].length > pow(2, 16) - 1)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("Synth %i, ramp %i length set to maximum.\n", synth->number, i);
			synth->ramps[i].length = (uint16_t)(pow(2, 16) - 1);			
		}
		
		//calculate RAMPx_INC = (bandwidth [Hz] * 2^24)/(phase detector frequency [MHz] * ramp_length)
		if ((synth->ramps[i].length != 0) && (synth->ramps[i].increment == 0))
		{
			synth->ramps[i].increment = (synth->ramps[i].bandwidth*4*pow(2, 24))/(phase_detector_frequency*synth->ramps[i].length*pow(10,6));			
		}
		
		if (synth->ramps[i].increment > pow(2, 30) - 1)
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("Synth %i, ramp %i increment set to maximum.\n", synth->number, i);
			synth->ramps[i].length = (uint16_t)(pow(2, 30) - 1);			
		}		
		
		//perform two's complement for negative values: 2^30 - value
		if (synth->ramps[i].increment < 0.0)
		{
			synth->ramps[i].increment = pow(2, 30) + synth->ramps[i].increment;
		}		
		
		//set bit 31 if doubler key is true
		if (synth->ramps[i].doubler)
		{
			synth->ramps[i].increment = (uint64_t)synth->ramps[i].increment | (uint64_t)pow(2, 31);	
		}
		
		//scheme to generate for R92, R96, R103 etc.. 
		synth->ramps[i].nextTriggerReset = 0;
		
		synth->ramps[i].nextTriggerReset += (synth->ramps[i].next << 5) & 0xFF;
		synth->ramps[i].nextTriggerReset += (synth->ramps[i].trigger << 3) & 0xFF;
		synth->ramps[i].nextTriggerReset += (synth->ramps[i].reset << 2) & 0xFF;
		synth->ramps[i].nextTriggerReset += (synth->ramps[i].flag << 0) & 0xFF;
		
		if ((experiment->is_debug_mode) && (synth->ramps[i].next + synth->ramps[i].length + synth->ramps[i].increment + synth->ramps[i].reset != 0))
		{
			printf("|   %i |   %i |   %i |   %i | %5i | %14.3f | %8.3f |\n", 
			synth->ramps[i].number, synth->ramps[i].next, synth->ramps[i].reset, synth->ramps[i].doubler, synth->ramps[i].length, 
			synth->ramps[i].increment, bnwOut(synth->ramps[i].increment, synth->ramps[i].length));
		}		
	}	
	if (experiment->is_debug_mode) printf("\n");
}


void generateHexValues(Synthesizer *synth)
{	
	for (int i = 0; i < 8; i++)
	{	
		synth->ramps[i].hexIncrement     = (char*)malloc(8*sizeof(char));
		synth->ramps[i].hexLength        = (char*)malloc(4*sizeof(char));	
		synth->ramps[i].hexNextTrigReset = (char*)malloc(2*sizeof(char));
		
		memset(synth->ramps[i].hexIncrement, 0, 8*sizeof(char));
		memset(synth->ramps[i].hexLength, 0, 4*sizeof(char));
		memset(synth->ramps[i].hexNextTrigReset, 0, 2*sizeof(char));		
		
		sprintf(synth->ramps[i].hexIncrement, "%08X", (int)synth->ramps[i].increment);	
		sprintf(synth->ramps[i].hexLength, "%04X", (int)synth->ramps[i].length);		
		sprintf(synth->ramps[i].hexNextTrigReset, "%02X", (int)synth->ramps[i].nextTriggerReset);	
	}	
}


void generateBinValues(Synthesizer *synth)
{
	synth->binFractionalNumerator = (int*)malloc(24*sizeof(int));
	memset(synth->binFractionalNumerator, 0, 24*sizeof(int));
	decimalToBinary(synth->fractionalNumerator, synth->binFractionalNumerator);	
	
	for (int i = 0; i < 8; i++)
	{	
		synth->ramps[i].binIncrement     = (int*)malloc(32*sizeof(int));
		synth->ramps[i].binLength        = (int*)malloc(16*sizeof(int));	
		synth->ramps[i].binNextTrigReset = (int*)malloc(8*sizeof(int));		
		
		memset(synth->ramps[i].binIncrement, 	 0, 32*sizeof(int));
		memset(synth->ramps[i].binLength, 		 0, 16*sizeof(int));
		memset(synth->ramps[i].binNextTrigReset, 0,  8*sizeof(int));		
		
		decimalToBinary(synth->ramps[i].increment, synth->ramps[i].binIncrement);
		decimalToBinary(synth->ramps[i].length, synth->ramps[i].binLength);
		decimalToBinary(synth->ramps[i].nextTriggerReset, synth->ramps[i].binNextTrigReset);	
	}
}


void printBinary(int* binaryValue, int paddedSize)
{
	for (int i = paddedSize - 1; i >= 0; i--)
	{
		printf("%d", binaryValue[i]);
	}
}


void decimalToBinary(uint64_t decimalValue, int* binaryValue)
{
	int i = 0;
	
	while(decimalValue > 0)
	{
		binaryValue[i] = decimalValue%2;
		decimalValue = decimalValue/2;
		i++;
	}
}


void readTemplateFile(const char* filename, Synthesizer *synth)
{
	FILE *templateFile;
	char line[86][15];
	char trash[15];
	
	//Open specified file
	templateFile = fopen(filename,"r");
	
	//Check if file correctly opened
	if (templateFile == 0)
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Could not open %s. Check that the file name is correct.\n", filename);
		exit(0);
	}
	else
	{
		for (int l = 0; l < 86; l++)
		{
			fscanf(templateFile, "%s",trash);
			fscanf(templateFile, "%s",line[l]);
			
			//get hex values and convert to decimal
			char hexValue[] = {line[l][6], line[l][7]};			
			int decimalValue = strtoul(hexValue, NULL, 16);			
			
			//convert decimal to binary and store in register array
			decimalToBinary(decimalValue, synth->binaryRegisterArray[85 - l]);					
		}
	}
	//close file
	fclose(templateFile);
}


void printRegisterValues(Synthesizer *synth)
{	
	for (int i = 141; i >= 0; i--)
	{		
		printf("R%03i : ", i);
		printBinary(synth->binaryRegisterArray[i], 8);	
		printf("\n");
	}	
	printf("\n");
}


void insertRampParameters(Synthesizer *synth)
{
	
	//======================================================================================
	//for every ramp
	for (int i = 7; i >= 0; i--)
	{
		//for every bit
		for (int j = 7; j >= 0; j--)
		{
			synth->binaryRegisterArray[92 + 7*i][j] = synth->ramps[i].binNextTrigReset[j];
		}
	}
	//======================================================================================
	
	
	
	//======================================================================================
	//for every ramp
	for (int i = 7; i >= 0; i--)
	{
		//for every bit
		for (int j = 15; j >= 8; j--)
		{
			synth->binaryRegisterArray[91 + 7*i][j - 8] = synth->ramps[i].binLength[j];
		}
	}	
	
	//for every ramp
	for (int i = 7; i >= 0; i--)
	{
		//for every bit
		for (int j = 7; j >= 0; j--)
		{
			synth->binaryRegisterArray[90 + 7*i][j] = synth->ramps[i].binLength[j];
		}
	}	
	//======================================================================================
	
	
	
	//======================================================================================
	//for every ramp
	for (int i = 7; i >= 0; i--)
	{
		//for every bit
		for (int j = 31; j >= 24; j--)
		{
			synth->binaryRegisterArray[89 + 7*i][j - 24] = synth->ramps[i].binIncrement[j];
		}
	}
	
	//for every ramp
	for (int i = 7; i >= 0; i--)
	{
		//for every bit
		for (int j = 23; j >= 16; j--)
		{
			synth->binaryRegisterArray[88 + 7*i][j - 16] = synth->ramps[i].binIncrement[j];
		}
	}
	
	//for every ramp
	for (int i = 7; i >= 0; i--)
	{
		//for every bit
		for (int j = 15; j >= 8; j--)
		{
			synth->binaryRegisterArray[87 + 7*i][j - 8] = synth->ramps[i].binIncrement[j];
		}
	}
	
	//for every ramp
	for (int i = 7; i >= 0; i--)
	{
		//for every bit
		for (int j = 7; j >= 0; j--)
		{
			synth->binaryRegisterArray[86 + 7*i][j] = synth->ramps[i].binIncrement[j];
		}
	}
	//======================================================================================	
	
	
	//======================================================================================		
	//for every bit
	for (int j = 23; j >= 16; j--)
	{
		synth->binaryRegisterArray[21][j - 16] = synth->binFractionalNumerator[j];
	}
	
	//for every bit
	for (int j = 15; j >= 8; j--)
	{
		synth->binaryRegisterArray[20][j - 8] = synth->binFractionalNumerator[j];
	}
	
	//for every bit
	for (int j = 7; j >= 0; j--)
	{
		synth->binaryRegisterArray[19][j] = synth->binFractionalNumerator[j];
	}
	//======================================================================================	
}


void initPins(Synthesizer *synth)
{
	int offset = 4*(synth->number - 1);
	
	//Initialize pins	
	synth->latchPin = offset + 0 + RP_DIO0_N;
	synth->dataPin  = offset + 1 + RP_DIO0_N;
	synth->clockPin = offset + 2 + RP_DIO0_N;
	synth->trigPin  = offset + 3 + RP_DIO0_N;
	
	/*if (DEBUG_MODE)
	{
		printf("Synth %i latch pin: \tRP_DIO%i_N\n", synth->number, offset + 0);
		printf("Synth %i data pin: \tRP_DIO%i_N\n", synth->number, offset + 1);
		printf("Synth %i clock pin: \tRP_DIO%i_N\n", synth->number, offset + 2);
		printf("Synth %i trig pin: \tRP_DIO%i_N\n", synth->number, offset + 3);
	}*/
	
	//Set directions of pins
	rp_DpinSetDirection(synth->latchPin, RP_OUT);
	rp_DpinSetDirection(synth->dataPin, RP_OUT);
	rp_DpinSetDirection(synth->clockPin, RP_OUT);
	rp_DpinSetDirection(synth->trigPin, RP_OUT);
	
	//Pull trigger pin low
	rp_DpinSetState(synth->trigPin, RP_LOW);
}


void setRegister(Synthesizer *synth, int registerAddress, int registerValue)
{
	int binAddress[16];
	int binRegister[8];
	
	memset(binAddress, 0, 16*sizeof(int));
	memset(binRegister, 0, 8*sizeof(int));
	
	decimalToBinary(registerAddress, binAddress);	
	decimalToBinary(registerValue, binRegister);
	
	//Latch enable high
	rp_DpinSetState(synth->latchPin, RP_HIGH);
	usleep(1);

	//Clock high
	rp_DpinSetState(synth->clockPin, RP_HIGH);
	usleep(1);

	//latch enable low
	rp_DpinSetState(synth->latchPin, RP_LOW);
	usleep(1);

	//data low
	rp_DpinSetState(synth->dataPin, RP_LOW);
	usleep(1);

	//clock setup time
	usleep(1000);

	//clock high
	rp_DpinSetState(synth->clockPin, RP_HIGH);
	usleep(1);

	//clock low
	rp_DpinSetState(synth->clockPin,RP_LOW);
	usleep(1);

	for (int j = 15; j >= 0 ; j--)
	{
		//Assert address bits on data line
		if (binAddress[j] == 1)
		{
			rp_DpinSetState(synth->dataPin, RP_HIGH);
		}
		else
		{
			rp_DpinSetState(synth->dataPin, RP_LOW);
		}
		
		//clock high
		rp_DpinSetState(synth->clockPin, RP_HIGH);
		usleep(1);
		
		//clock low
		rp_DpinSetState(synth->clockPin, RP_LOW);				
		usleep(1);
	}			
	
	//Only do this the first loop iteration, set addressFlag
	synth->addressFlag = 1;	

	//Write register data
	for(int j = 7; j >= 0; j--)
	{
		if (binRegister[j] == 1)
		{
			rp_DpinSetState(synth->dataPin, RP_HIGH);
		}
		else
		{
			rp_DpinSetState(synth->dataPin, RP_LOW);
		}
		
		//clock high
		rp_DpinSetState(synth->clockPin, RP_HIGH);
		usleep(1);
		
		//clock low
		rp_DpinSetState(synth->clockPin, RP_LOW);
		usleep(1);
	}
	
	//Latch enable high
	rp_DpinSetState(synth->latchPin, RP_HIGH);
	usleep(1);

	//data low
	rp_DpinSetState(synth->dataPin, RP_LOW);	
}


void updateRegisters(Synthesizer *synth)
{
	int startAddress = 141;
	int binAddress[16];
	memset(binAddress, 0, 16*sizeof(int));
	decimalToBinary(startAddress, binAddress);
	
	synth->addressFlag = 0;
	
	for (int i = 141; i >= 0; i--)
	{
		if (synth->addressFlag == 0)
		{
			//Latch enable high
			rp_DpinSetState(synth->latchPin, RP_HIGH);
			usleep(1);

			//Clock high
			rp_DpinSetState(synth->clockPin, RP_HIGH);
			usleep(1);

			//latch enable low
			rp_DpinSetState(synth->latchPin, RP_LOW);
			usleep(1);

			//data low
			rp_DpinSetState(synth->dataPin, RP_LOW);
			usleep(1);

			//clock setup time
			usleep(1000); 

			//clock high
			rp_DpinSetState(synth->clockPin, RP_HIGH);
			usleep(1);

			//clock low
			rp_DpinSetState(synth->clockPin,RP_LOW);
			usleep(1);

			for (int j = 15; j >= 0 ; j--)
			{
				//Assert address bits on data line
				if (binAddress[j] == 1)
				{
					rp_DpinSetState(synth->dataPin, RP_HIGH);
				}
				else
				{
					rp_DpinSetState(synth->dataPin, RP_LOW);
				}
				
				//clock high
				rp_DpinSetState(synth->clockPin, RP_HIGH);
				usleep(1);
				
				//clock low
				rp_DpinSetState(synth->clockPin, RP_LOW);				
				usleep(1);
			}			
			
			//Only do this the first loop iteration, set addressFlag
			synth->addressFlag = 1;
		}

		//Write register data
		for(int j = 7; j >= 0; j--)
		{
			if (synth->binaryRegisterArray[i][j] == 1)
			{
				rp_DpinSetState(synth->dataPin, RP_HIGH);
				//rp_DpinSetState(RP_LED2, RP_HIGH);
			}
			else
			{
				rp_DpinSetState(synth->dataPin, RP_LOW);
				//rp_DpinSetState(RP_LED2, RP_LOW);
			}
			
			//clock high
			rp_DpinSetState(synth->clockPin, RP_HIGH);
			usleep(1);
			
			//clock low
			rp_DpinSetState(synth->clockPin, RP_LOW);
			usleep(1);
		}
	}
	
	//Latch enable high
	rp_DpinSetState(synth->latchPin, RP_HIGH);
	usleep(1);

	//data low
	rp_DpinSetState(synth->dataPin, RP_LOW);
}
 
 
void initRP(void)
{
	// Initialization of API
	if (rp_Init() != RP_OK) 
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Red Pitaya API initialization failed!\n");
		exit(EXIT_FAILURE);
	}
}


void releaseRP(void)
{
	rp_GenOutDisable(RP_CH_1);
	rp_Release();
}


void triggerSynthesizers(Synthesizer *synthOne, Synthesizer *synthTwo)
{
	//Trigger
	rp_DpinSetState(synthOne->trigPin, RP_LOW);
	rp_DpinSetState(synthTwo->trigPin, RP_LOW);	
	usleep(1);	
	rp_DpinSetState(synthTwo->trigPin, RP_HIGH);
	rp_DpinSetState(synthOne->trigPin, RP_HIGH);
	/*usleep(1);
	rp_DpinSetState(synthOne->trigPin, RP_LOW);
	rp_DpinSetState(synthTwo->trigPin, RP_LOW);*/	
}


void parallelTrigger(Synthesizer *synthOne, Synthesizer *synthTwo)
{		
	cprint("[??] ", BRIGHT, BLUE);
	printf("Press enter to trigger...\n");
	
	//Dirty, but it works...
	getchar();
	getchar();
	
	//Rising edge required
	setpins(synthOne->trigPin - RP_DIO0_N, 0, synthTwo->trigPin - RP_DIO0_N, 0, 0x4000001C);
	usleep(1);
	setpins(synthOne->trigPin - RP_DIO0_N, 1, synthTwo->trigPin - RP_DIO0_N, 1, 0x4000001C);
	/*usleep(1);
	setpins(synthOne->trigPin - RP_DIO0_N, 0, synthTwo->trigPin - RP_DIO0_N, 0, 0x4000001C);*/
	
	cprint("[OK] ", BRIGHT, GREEN);
	printf("Synthesizers triggered in parallel.\n");		
}


void configureVerbose(Experiment *experiment, Synthesizer *synthOne, Synthesizer *synthTwo)
{
	//read-write mode
	system("rw\n");

	//create time-stamped folder
	char syscmd[100];
	char foldername[100];
	char timestamp[100];
	
	time_t rawtime = time(NULL);
	struct tm tm = *localtime(&rawtime);
	
	sprintf(timestamp, "%02d_%02d_%02d_%02d_%02d", tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	sprintf(foldername, "%s/%s/", experiment->storageDir, timestamp);
	sprintf(syscmd, "mkdir %s/%s", experiment->storageDir, timestamp);		
	system(syscmd);
	
	char* ch1_out = (char*)malloc(100*sizeof(char));
	strcpy(ch1_out, foldername);
	strcat(ch1_out, "external.bin");
	
	char* ch2_out = (char*)malloc(100*sizeof(char));
	strcpy(ch2_out, foldername);
	strcat(ch2_out, "reference.bin");
	
	char* imu_out = (char*)malloc(100*sizeof(char));
	strcpy(imu_out, foldername);
	strcat(imu_out, "imu.bin");	
	
	char* summary = (char*)malloc(100*sizeof(char));
	strcpy(summary, foldername);
	strcat(summary, "summary.ini");	
	
	char* time_stamp = (char*)malloc(100*sizeof(char));
	strcpy(time_stamp, timestamp);
	
	experiment->ch1_filename = ch1_out;
	experiment->ch2_filename = ch2_out;
	experiment->imu_filename = imu_out;
	experiment->summary_filename = summary;
	experiment->timeStamp = time_stamp;
	
	FILE* summaryFile;
	summaryFile = fopen(experiment->summary_filename, "w");
	
	//Check if file correctly opened
	if (summaryFile == 0)
    {
		cprint("[!!] ", BRIGHT, RED);
		printf("Could not open summary file. Ensure you have read-write access\n");
    }
	else
    {
		//copy ini parameter files
		sprintf(syscmd, "cp ramps/%s %s", synthOne->parameterFile, foldername);
		system(syscmd);
		
		if (synthOne->parameterFile != synthTwo->parameterFile)
		{
			sprintf(syscmd, "cp ramps/%s %s", synthTwo->parameterFile, foldername); 
			system(syscmd);
		}
		
		//print summary file 
		fprintf(summaryFile, "[overview]\r\n");
		fprintf(summaryFile, "timestamp = %s\r\n", timestamp);
		fprintf(summaryFile, "rpc_version = %s\r\n", VERSION);
		
		cprint("[??] ", BRIGHT, BLUE);
		printf("Experiment comment [140]: ");
		char userin[140];
		scanf("%[^\n]s", userin);
		
		fprintf(summaryFile, "comment = %s\r\n", userin);
		
		fprintf(summaryFile, "\n[dataset]\r\n");
		fprintf(summaryFile, "storage_directory = %s\r\n", experiment->ch1_filename);
		fprintf(summaryFile, "decimation_factor = %d\r\n", experiment->decFactor);
		fprintf(summaryFile, "sampling_rate =  %.2f\r\n", 125e6/experiment->decFactor);
		fprintf(summaryFile, "duration = %i\r\n", 1024*experiment->recDuration);			
		
		fprintf(summaryFile, "\n[synth_one]\r\n");
		fprintf(summaryFile, "frequency_offset = %.3f\r\n", vcoOut(synthOne->fractionalNumerator));
		fprintf(summaryFile, "fractional_numerator = %d\r\n", synthOne->fractionalNumerator);		
		fprintf(summaryFile, "| NUM | NXT | RST | DBL |   LEN |            INC |      BNW |\r\n");		
		
		for (int k = 0; k<8; k++)
		{
			if ((synthOne->ramps[k].next + synthOne->ramps[k].length + synthOne->ramps[k].increment + synthOne->ramps[k].reset != 0))
			{
				fprintf(summaryFile, "|   %i |   %i |   %i |   %i | %5i | %14.3f | %8.3f |\r\n", 
				synthOne->ramps[k].number, synthOne->ramps[k].next, synthOne->ramps[k].reset, synthOne->ramps[k].doubler, synthOne->ramps[k].length, 
				synthOne->ramps[k].increment, bnwOut(synthOne->ramps[k].increment, synthOne->ramps[k].length));
			}
		}
		
		fprintf(summaryFile, "\n[synth_two]\r\n");
		fprintf(summaryFile, "frequency_offset = %.3f\r\n", vcoOut(synthTwo->fractionalNumerator));
		fprintf(summaryFile, "fractional_numerator = %d\r\n", synthTwo->fractionalNumerator);		
		fprintf(summaryFile, "| NUM | NXT | RST | DBL |   LEN |            INC |      BNW |\r\n");
		
		for (int j = 0; j<8; j++)
		{
			if ((synthTwo->ramps[j].next + synthTwo->ramps[j].length + synthTwo->ramps[j].increment + synthTwo->ramps[j].reset != 0))
			{
				fprintf(summaryFile, "|   %i |   %i |   %i |   %i | %5i | %14.3f | %8.3f |\r\n", 
				synthTwo->ramps[j].number, synthTwo->ramps[j].next, synthTwo->ramps[j].reset, synthTwo->ramps[j].doubler, synthTwo->ramps[j].length, 
				synthTwo->ramps[j].increment, bnwOut(synthTwo->ramps[j].increment, synthTwo->ramps[j].length));	
			}	
		}      

		fclose(summaryFile);
	}	
}


void generateClock(void)
{
	system("generate 1 1 50000000 sine");
	
	/*rp_GenReset();
	rp_GenOutEnable(RP_CH_1);
	rp_GenAmp(RP_CH_1, 1.0f);  	// 2*0.5  = 1 Vpp
	rp_GenFreq(RP_CH_1, 50000000);
	rp_GenWaveform(RP_CH_1, RP_WAVEFORM_SINE);
	rp_GenMode(RP_CH_1, RP_GEN_MODE_CONTINUOUS);*/
}


void getExperimentParameters(Experiment *experiment)
{	
	char userin;
	do
	{  
		cprint("[??] ", BRIGHT, BLUE);
		printf("Decimation factor: ");	    
	} while (((scanf("%d%c", &experiment->decFactor, &userin)!=2 || userin!='\n') && clean_stdin()));

	do
	{  
		cprint("[??] ", BRIGHT, BLUE);
		printf("Recording duration [s]: ");	    
	} while (((scanf("%d%c", &experiment->recDuration, &userin)!=2 || userin!='\n') && clean_stdin()));
	
	experiment->recSize = (125e6*16*experiment->recDuration)/(experiment->decFactor*8*1000);		
}


int continuousAcquire(int channel, int kbytes, int dec, char* filename_ch1, char* filename_ch2, char* filename_imu, int is_imu_en)
{	
	cprint("[OK] ", BRIGHT, GREEN);
	printf("Acquisition on channel %i initiated.\n", channel);
	
	static option_fields_t g_options =
    {
		/* Setting defaults */
		.address = "",
		.port = 14000,
		.port2 = 14001,
		.tcp = 1,
		.mode = file,
		.kbytes_to_transfer = 32,
		.fname_ch1 = "/media/storage/channel_1.dat",
		.fname_ch2 = "/media/storage/channel_2.dat",
		.fname_imu = "/media/storage/imu.dat",
		.imu_en = 0,
		.report_rate = 1,
		.scope_chn = 0, //0 = ch1, 1 = ch2, 2 = ch1+ch2 (only for network mode)
		.scope_dec = 8,
		.scope_equalizer = 1,
		.scope_hv = 0,
		.scope_shaping = 1,
    };

	//Writes file of specified size to 
	int retval;
	int sock_fd = -1;
	int sock_fd2 = -1;
	struct scope_parameter param;

	//Part of function that returns usage instructions for invalid input
	//Also sets up options
	if(0 != handle_options(channel, kbytes, dec, filename_ch1, filename_ch2, filename_imu, is_imu_en, &g_options))
	{
		//usage(argv[0]);
		return 1;
	}

	//If anything goes wrong with initialising scope or options, go to cleanup
	signal_init();

	if (scope_init(&param, &g_options)) 
	{
		retval = 2;
		cprint("[!!] ", BRIGHT, RED);
		printf("Scope initialisation failed, went to cleanup\n");
		goto cleanup;
	}

	if (g_options.mode == client || g_options.mode == server) 
	{
		if (connection_init(&g_options)) 
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("Connection initialisation failed, went to cleanup_scope\n");
			retval = 3;
			goto cleanup_scope;
		}
	}

	//Anything after this is successful operation
	retval = 0;
	while (!transfer_interrupted())
	{
		if (g_options.mode == client || g_options.mode == server) 
		{
			if (connection_start(&g_options, &sock_fd, &sock_fd2) < 0) 
			{
				fprintf(stderr, "%s: problem opening connection.\n", __func__);
				continue;
			}
		}
      
		retval = transfer_data(sock_fd, sock_fd2, &param, &g_options);
		
		if (retval && !transfer_interrupted())
			fprintf(stderr, "%s: problem transferring data.\n", __func__);

		if (g_options.mode == client || g_options.mode == server)
			connection_stop();
      
		if (g_options.mode == file)
			break;
    }
  
	connection_cleanup();
	
	cleanup_scope:
	scope_cleanup(&param);
	cleanup:
	signal_exit();

	return retval;	
}


int clean_stdin()
{
	while (getchar()!='\n');
	return 1;
}


void initIMU()
{	
	if (uartInit() < 0)
		exit(EXIT_FAILURE);
	
	if (getFirmwareVersion())
		exit(EXIT_FAILURE);
		
	if (zeroGyros())
		exit(EXIT_FAILURE);
		
	if (setMagReference())
		exit(EXIT_FAILURE);
		
	if (setHomePosition())
		exit(EXIT_FAILURE);
		
	if (resetEKF())
		exit(EXIT_FAILURE);		
	
	uint8_t* regData = (uint8_t*)malloc(4*sizeof(uint8_t*));
	uint8_t* zeros = (uint8_t*)malloc(4*sizeof(uint8_t*));	
	memset(zeros, 0, 4);	
	
	writeRegister(CREG_COM_RATES1, zeros);		// raw gyro, accel and mag rate
	usleep(1000);
	writeRegister(CREG_COM_RATES2, zeros);		// raw temp rate and all raw data rate	
	usleep(1000);
	
	regData[0] = 255;			// proc accel rate
	regData[1] = 0;				// proc gyro rate
	regData[2] = 0;				// proc mag rate
	regData[3] = 0;				// reserved
	
	writeRegister(CREG_COM_RATES3, regData);	// proc data rate
	usleep(1000);
	//writeRegister(CREG_COM_RATES4, zeros);		// all proc data rate	
	//usleep(1000);
	writeRegister(CREG_COM_RATES5, zeros);		// quart, euler, position, velocity rate
	usleep(1000);
	
	regData[0] = 0;					// pose rate
	regData[1] = 0;					// res
	regData[2] = 4;					// health rate
	regData[3] = 0;					// res
	
	writeRegister(CREG_COM_RATES6, zeros);		// heartbeat rate
	usleep(1000);
}


double vcoOut(uint32_t fracNum)
{
	return 25*(96 + fracNum/pow(2, 24)) - 2400;
}

double bnwOut(double rampInc, uint16_t rampLen)
{
	if(rampInc < pow(2, 24))
		return (rampInc*rampLen*100)/pow(2, 26);
	else
		return ((rampInc - pow(2, 30))*rampLen*100)/pow(2, 26);
}








