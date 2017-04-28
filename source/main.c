#include "includes.h"
#include <unistd.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void splash(Experiment *experiment);
void help(void);

//Global UM7 receive packet
extern UM7_packet global_packet;

int main(int argc, char *argv[])
{
	int opt;
	int is_synth_one = 0;
	int is_synth_two = 0;

	//declare structs
	Experiment experiment;
	Synthesizer synthOne;
	Synthesizer synthTwo;
	
	//initialise synth number
	synthOne.number = 1;
	synthTwo.number = 2;
	
	experiment.storageDir = "/media/storage";
	experiment.is_debug_mode = 0;
	experiment.adc_channel = 0;

	// Retrieve the options:
    while ((opt = getopt(argc, argv, "dib:c:t:l:rh")) != -1 )
    {
        switch (opt)
        {
            case 'd':
                experiment.is_debug_mode = 1;
                break;
            case 'r':
                experiment.storageDir = "/tmp";
                break;       
            case 'i':
                experiment.is_imu = 1;
                break;
			case 'c':
				experiment.adc_channel = atoi(optarg);
				break;
			case 'b':
				synthOne.parameterFile = optarg;
				synthTwo.parameterFile = optarg;
				is_synth_one = 1;
				is_synth_two = 1;
				break;
			case 'l':
				is_synth_one = 1;
				synthOne.parameterFile = optarg;
				break;
			case 't':
				is_synth_two = 1;
				synthTwo.parameterFile = optarg;
				break;
			case 'h':
				help();
				break;
            case '?':
				if (optopt == 'c')
					fprintf (stderr, "Option -%c requires an argument.\n", optopt);
				else
				fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
        }
    }

    if (is_synth_one + is_synth_two != 2)
    {
		cprint("[!!] ", BRIGHT, RED);
		printf("A .ini parameter file must be provided for each synthesizer.\n");
		exit(EXIT_FAILURE);
	}

	splash(&experiment);

	//get parameters for ini files
	getParameters(&synthOne);
	getParameters(&synthTwo);

	//calculate additional ramp parameters
	calculateRampParameters(&synthOne, &experiment);
	calculateRampParameters(&synthTwo, &experiment);

	//convert necessary values to binary
	generateBinValues(&synthOne);
	generateBinValues(&synthTwo);

	//import register values from template file
	readTemplateFile("template/register_template.txt", &synthOne);
	readTemplateFile("template/register_template.txt", &synthTwo);

	//insert calculated ramp parameters into register array
	insertRampParameters(&synthOne);
	insertRampParameters(&synthTwo);

	//initialise the red pitaya and configure pins
	initRP();
	initPins(&synthOne);
	initPins(&synthTwo);

	//red pitaya provides 50 MHz reference signal for synth's
	generateClock();

	//software reset all synth register values
	setRegister(&synthOne, 2, 0b00000100);
	setRegister(&synthTwo, 2, 0b00000100);

	//initialise IMU and configure update rates
	if (experiment.is_imu) initIMU();

	//send register array values to synths
	updateRegisters(&synthOne);
	updateRegisters(&synthTwo);
	
	//get user input for final experiment settings
	getExperimentParameters(&experiment);
	configureVerbose(&experiment, &synthOne, &synthTwo);
	
	if (experiment.n_ramps > 0)
	{
		//set number of ramps to generate
		setRegister(&synthOne, 83, 0b11111111);
		setRegister(&synthTwo, 83, 0b11111111);
		
		//enable ramp auto - clears ramp_en when target number of ramps finished
		setRegister(&synthOne, 84, 0b00111111);
		setRegister(&synthTwo, 84, 0b00111111);
	}
	
	//enable ramping now that ramps have been configured
	//note that synths will wait on ramp0 until triggered.
	setRegister(&synthOne, 58, 0b00100001);
	setRegister(&synthTwo, 58, 0b00100001);
	
	int n_flags = 0;
	FILE *binOutFile;	
	uint32_t bufferSize = 2048;
	int u_delay = ((float)(bufferSize*experiment.decFactor/(float)ADC_RATE)*1e6);
	int16_t* binOutBuffer = (int16_t*)malloc(bufferSize*sizeof(int16_t*));
	
	cprint("[OK] ", BRIGHT, GREEN);
	printf("ADC capture delay: %i\n", u_delay);
	
	memset(binOutBuffer, 0, bufferSize);	
	
	if (!(binOutFile = fopen(experiment.ch1_filename, "wb"))) 
	{
		fprintf(stderr, "file open failed, %s\n", strerror(errno));
		return EXIT_FAILURE;
	}	
	
	rp_AcqSetDecimation(RP_DEC_8);	
	rp_AcqSetTriggerDelay(-ADC_BUFFER_SIZE/2);
	experiment.trigger_source = RP_TRIG_SRC_EXT_PE;	
	
	struct timeval start_time, end_time;
	unsigned long duration = 0UL;	
	
	rp_AcqStart();	
	usleep(u_delay);
	rp_AcqSetTriggerSrc(experiment.trigger_source);		

	//trigger synth's to begin generating ramps at the same time
	parallelTrigger(&synthOne, &synthTwo);	
	
	//very specific to the number of ramps chosen!
	while (n_flags < (8191-1)/4)
	{
		//get the state of the ADC trigger source
		rp_AcqGetTriggerSrc(&experiment.trigger_source);
		
		if (experiment.trigger_source == 0)
		{
			gettimeofday(&start_time, NULL);	
			
			//new flag detected
			n_flags += 1;				
			
			//transfer data from ADC buffer to RAM
			rp_AcqGetLatestDataRaw(RP_CH_1, &bufferSize, binOutBuffer);		
			
			//restart adc 
			rp_AcqStart();
			
			//ensure adc buffer contains new samples
			usleep(u_delay);
			
			//transfer data to SD, add additional delay 
			fwrite(binOutBuffer, sizeof(int16_t), bufferSize, binOutFile);
		
			//set state of ADC trigger back to external pin rising edge.
			rp_AcqSetTriggerSrc(RP_TRIG_SRC_EXT_PE);	
			
			gettimeofday(&end_time, NULL);	
			
			duration = end_time.tv_usec - start_time.tv_usec;
			printf("Time: %lu us \n", duration);	
		}
	}		
	
	fclose(binOutFile);	
	
	cprint("[OK] ", BRIGHT, GREEN);
	printf("Ramp Count: %i\n", n_flags);	
	
	cprint("[OK] ", BRIGHT, GREEN);
	printf("File Size: %i\n", experiment.recSize);
	
	cprint("[OK] ", BRIGHT, GREEN);
	printf("Storage location: %s/%s\n", experiment.storageDir, experiment.timeStamp);

	releaseRP();

	return EXIT_SUCCESS;
}


void splash(Experiment *experiment)
{
	system("clear\n");
	printf("UCT RPC: %s\n", VERSION);
	printf("--------------\n");

	if (experiment->is_debug_mode)
	{
		cprint("[OK] ", BRIGHT, GREEN);
		printf("Debug mode enabled.\n");
	}
	else
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Debug mode disabled.\n");
	}

	if (experiment->is_imu)
	{
		cprint("[OK] ", BRIGHT, GREEN);
		printf("IMU mode enabled.\n");
		
		cprint("\n[**] ", BRIGHT, CYAN);
		printf("RP UART Pins: https://wiki.redpitaya.com/tmp/Extension_connector.png\n");
	}
	else
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("IMU mode disabled.\n");
	}
}

void help(void)
{
	system("clear\n");
	printf("UCT RPC: %s\n", VERSION);
	printf("--------------\n");
	printf(" -h: display this help screen\n");
	printf(" -d: enable debug mode\n");
	printf(" -i: enable imu mode\n");
	printf(" -l: name of local oscillator (lo) synth parameter file\n");
	printf(" -t: name of radio frequency (rf) synth parameter file\n");
	printf(" -r: write output files to /tmp\n");
	printf(" -c: input adc channel \t(0 or 1)\n");	
	exit(EXIT_SUCCESS);	
}
