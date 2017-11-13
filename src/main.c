#include <unistd.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include "controller.h"
#include "rp.h"
#include "colour.h"
#include "imu.h"

void splash(void);
void help(void);
void parse_uart(void);
void parse_options(int argc, char *argv[]);

extern heartbeat beat;
extern uint8_t* uart_buffer;

//global experiment active flag
int is_experiment_active = false;
int is_imu_allowed = false;

//declare global structs
Experiment experiment;
Synthesizer synthOne;
Synthesizer synthTwo;

int main(int argc, char *argv[])
{
	pthread_t imu_thread;
	
	//initialise default values
	synthOne.number = 1;
	synthTwo.number = 2;
	experiment.storageDir = "/media/storage";
	experiment.is_debug_mode = 0;
	experiment.adc_channel = 0;

	//parse command line options
	parse_options(argc, argv);
	
	//display splash screen
	splash();

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

	//send register array values to synths
	updateRegisters(&synthOne);
	updateRegisters(&synthTwo);
	
	//set experiment values
	experiment.ns_ext_buffer = 1280;	
	experiment.u_max_loop = 950; 	
	experiment.n_flags = 0;		
	experiment.n_corrupt = 0;		
	experiment.n_missed = 0;	
	experiment.trigger_source = RP_TRIG_SRC_EXT_PE;	
	experiment.decFactor = 8;
	
	//get user input for final experiment settings
	configureVerbose(&experiment, &synthOne, &synthTwo);
	
	/*if (experiment.n_ramps > 0)
	{
		//set number of ramps to generate
		setRegister(&synthOne, 83, 0b11111111);
		setRegister(&synthTwo, 83, 0b11111111);
		
		//enable ramp auto - clears ramp_en when target number of ramps finished
		setRegister(&synthOne, 84, 0b00111111);
		setRegister(&synthTwo, 84, 0b00111111);
	}*/
	
	//enable ramping now that ramps have been configured
	//note that synths will wait on ramp0 until triggered.
	setRegister(&synthOne, 58, 0b00100001);
	setRegister(&synthTwo, 58, 0b00100001);	
	
	FILE *extFile;

	struct timeval start_time, transfer_time, loop_time;	
	
	//time required to fill the adc buffer with fresh data [us]
	int u_adc_buffer = 1.1*experiment.ns_ext_buffer*((float)experiment.decFactor/(float)ADC_RATE)*1e6;

	//time used by the rp_AcqGetLatestDataRaw function to transfer data from fpga to cpu [us]
	double transfer_duration = 0;
	
	//total time used by the data capture loop used as indication for lost flags [us]
	double loop_duration = 0;
	
	//buffer used to store adc samples 
	int16_t* extBuffer = (int16_t*)malloc(experiment.ns_ext_buffer*sizeof(int16_t*));	
	memset(extBuffer, 0, experiment.ns_ext_buffer);
	
	rp_AcqSetDecimation(RP_DEC_8);	
	rp_AcqSetAveraging(false);
	
	//set how many samples are recorded after trigger occurs.
	//by default, ADC_BUFFER_SIZE/2 more samples are recorded.
	//thus, using rp_AcqSetTriggerDelay(-ADC_BUFFER_SIZE/2) results 
	//in no new samples being recorded
	rp_AcqSetTriggerDelay(-ADC_BUFFER_SIZE/2);
	
	if (experiment.is_debug_mode)
	{
		bool is_averaging;
		rp_AcqGetAveraging(&is_averaging);
		
		cprint("[**] ", BRIGHT, CYAN);		
		if (is_averaging)
			printf("Averaging is enabled.\n");
		else
			printf("Averaging is disabled.\n");
		
		cprint("[**] ", BRIGHT, CYAN);
		printf("Decimation factor: %i\n", experiment.decFactor);
		
		cprint("[**] ", BRIGHT, CYAN);
		printf("Capture delay: %i\n", u_adc_buffer);
	}		
	
	if (!(extFile = fopen(experiment.ch1_filename, "wb"))) 
	{
		fprintf(stderr, "ext file open failed, %s\n", strerror(errno));
		return EXIT_FAILURE;
	}	
	
	//initialise IMU and configure update rates
	if (experiment.is_imu) 
	{
		initUART(B115200);
		initIMU(&experiment);
		
		is_experiment_active = true;
		
		if (pthread_create(&imu_thread, NULL, (void*)parse_uart, NULL))
		{
			cprint("[!!] ", BRIGHT, RED);
			printf("Error launching imu thread.\n");
		}
		else
		{
			//start experiment
			is_experiment_active = true;
			cprint("[OK] ", BRIGHT, GREEN);
			printf("IMU active.\n");
		}
	}
	
	//start adc sampling
	rp_AcqStart();	
	
	//allow enough time for the adc buffer to fill with new data 
	usleep(u_adc_buffer);
	
	//set the source of the adc trigger
	rp_AcqSetTriggerSrc(experiment.trigger_source);		

	//trigger synth's to begin generating ramps at the same time
	parallelTrigger(&synthOne, &synthTwo);	
	
	//allow imu thread activity
	is_imu_allowed = true;
	
	//loop until the specified number of ramps have been detected
	while (experiment.n_flags < experiment.n_ramps) 								//(n_flags < (pow(2, 13) - 1 - 1)/4 - n_missed)
	{
		//get the latest state of the ADC trigger source
		rp_AcqGetTriggerSrc(&experiment.trigger_source);
		
		//trigger source set to zero implies that data capture is complete
		if (experiment.trigger_source == 0)
		{
			//disable imu thread activity
			is_imu_allowed = false;
			
			//get start time
			gettimeofday(&start_time, NULL);	
			
			//flag has been detected
			experiment.n_flags += 1;				
			
			//transfer data from ADC buffer to RAM
			rp_AcqGetLatestDataRaw(RP_CH_1, &experiment.ns_ext_buffer, extBuffer);		
			
			//restart adc sampling
			rp_AcqStart();
			
			//get transfer time
			gettimeofday(&transfer_time, NULL);				
			transfer_duration = elapsed_us(start_time, transfer_time);
			
			//check to see if there is enough time to fill the adc buffer with new data
			if (experiment.u_max_loop - transfer_duration < u_adc_buffer) 
			{				
				experiment.n_corrupt += 1;		
				printf("Data transfer took %.2f us\n", transfer_duration);
			}
			else
			{
				//allow enough time for the adc buffer to fill with new data
				//usleep(u_adc_buffer);		 - causes spurious lags!!! Use with caution.
			}	
			
			//transfer buffer to SD
			fwrite(extBuffer, sizeof(int16_t), experiment.ns_ext_buffer, extFile);
		
			//set state of ADC trigger back to external pin rising edge.
			rp_AcqSetTriggerSrc(RP_TRIG_SRC_EXT_PE);
			
			//get loop time
			gettimeofday(&loop_time, NULL);				
			loop_duration = elapsed_us(start_time, loop_time);	
			
			//enable imu thread activity
			is_imu_allowed = true;		

			//check to see if a flag could be lost
			if (loop_duration > experiment.u_max_loop) 
			{				
				//experiment.n_missed += loop_duration/experiment.u_max_loop;	
				printf("Loop took %.2f us\n", loop_duration);	
			}	
		}
	}		
	
	is_experiment_active = false;

	
	fclose(extFile);		

	if (experiment.is_imu) 
	{
		//join all threads
		pthread_join(imu_thread, NULL);		
		dnitUART();
	}
	
	if (experiment.n_missed > 0)
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Missed ramps: %i\n", experiment.n_missed);			
	}
	
	if (experiment.n_corrupt > 0)
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Corrupt ramps: %i\n", experiment.n_corrupt);			
	}
	
	cprint("[OK] ", BRIGHT, GREEN);
	printf("Ramp Count: %i\n", experiment.n_flags);	
	
	if (experiment.is_debug_mode)
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("File Size: %.2f MB\n", experiment.outputSize);
		
		cprint("[**] ", BRIGHT, CYAN);
		printf("Storage location: %s/%s\n", experiment.storageDir, experiment.timeStamp);
	}

	//disable ramping now that specified number of ramps have been synthesized
	setRegister(&synthOne, 58, 0b00100000);
	setRegister(&synthTwo, 58, 0b00100000);
	

	if (experiment.is_debug_mode)
	{
		cprint("[**] ", BRIGHT, CYAN);
		printf("Enter password to transfer files:\n");
		
		char syscmd[100];
		sprintf(syscmd, "scp -r %s/%s darryn@10.42.0.1:/home/darryn/Dropbox/Datasets/Temp", experiment.storageDir, experiment.timeStamp);		
		system(syscmd);
	}

	
	releaseRP();

	return EXIT_SUCCESS;
}


void splash(void)
{
	system("clear\n");
	printf("UCT RPC: %s\n", VERSION);
	printf("--------------\n");

	if (experiment.is_debug_mode)
	{
		cprint("[OK] ", BRIGHT, GREEN);
		printf("Debug mode enabled.\n");
	}
	else
	{
		cprint("[!!] ", BRIGHT, RED);
		printf("Debug mode disabled.\n");
	}

	if (experiment.is_imu)
	{
		cprint("[OK] ", BRIGHT, GREEN);
		printf("IMU mode enabled.\n");
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


void parse_uart(void)
{		
	FILE *imuFile;

	if (!(imuFile = fopen(experiment.imu_filename, "wb"))) 
	{		
		cprint("[!!] ", BRIGHT, RED);
		printf("IMU file open failed.\n");
		exit(EXIT_FAILURE);
	}

	//while experiment is active
	while (is_experiment_active)
	{
		//prevent intensive processing while is_imu_allowed is false
		//while(!is_imu_allowed);
		
		int rx_size = getUART();
		
		fwrite(uart_buffer, sizeof(uint8_t), rx_size, imuFile);
		
		usleep(1e3);
	}

	fclose(imuFile);
}


void parse_options(int argc, char *argv[])
{
	int opt;
	int is_synth_one = 0;
	int is_synth_two = 0;
	
	//retrieve command-line options
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
}
