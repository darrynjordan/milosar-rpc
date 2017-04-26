#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "includes.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include "rp.h"

#define VERSION "1.0.1"
#define MAX_RAMPS 8
#define NUM_REGISTERS 142

typedef struct 
{
	uint8_t number;
	uint8_t reset;
	uint8_t next;
	uint8_t trigger;
	uint8_t flag;
	uint8_t doubler;
	
	double bandwidth;
	
	int nextTriggerReset;
	double increment;
	uint16_t length;		

	char* hexIncrement;
	char* hexLength;
	char* hexNextTrigReset;
	
	int* binIncrement;
	int* binLength;
	int* binNextTrigReset;	
} Ramp;


typedef struct
{
	int   number;
	uint32_t fractionalNumerator;
	int*  binFractionalNumerator;
	int   addressFlag;
	int   binaryRegisterArray[NUM_REGISTERS][MAX_RAMPS];
	char* parameterFile;
	Ramp  ramps[MAX_RAMPS];
	rp_dpin_t latchPin, dataPin, clockPin, trigPin;
} Synthesizer;

typedef struct
{
	int is_imu;				//is the IMU connected?
	int is_debug_mode;		//is debug mode enabled
	int adc_channel;		//adc channel to record on
	int decFactor; 			//adc decimation factor
	int recDuration; 		//recording duration [s] 
	int recSize; 			//recoring size [kB]
	char* storageDir; 		//path to storage directory
	char* timeStamp;		//experiment timestamp
	char* ch1_filename; 	//filename of output data including path
	char* ch2_filename; 	//filename of output data including path
	char* imu_filename; 	//filename of output data including path
	char* summary_filename; //filename of summary file including path
} Experiment;

void clearTerminal(void);
void getParameters(Synthesizer *synth);
void getExperimentParameters(Experiment *experiment);
int  handler(void* user, const char* section, const char* name, const char* value);

void calculateRampParameters(Synthesizer *synth, Experiment *experiment);
void generateHexValues(Synthesizer *synth);
void generateBinValues(Synthesizer *synth);

void readTemplateFile(const char* filename, Synthesizer *synth);
void printRegisterValues(Synthesizer *synth);
void insertRampParameters(Synthesizer *synth);

void initRP(void);
void releaseRP(void);

void initPins(Synthesizer *synth);
void updateRegisters(Synthesizer *synth);
void triggerSynthesizers(Synthesizer *synthOne, Synthesizer *synthTwo);
void parallelTrigger(Synthesizer *synthOne, Synthesizer *synthTwo);
void configureVerbose(Experiment *experiment, Synthesizer *synthOne, Synthesizer *synthTwo);
void generateClock(void);
void setRegister(Synthesizer *synth, int registerAddress, int registerValue);

void decimalToBinary(uint64_t decimalValue, int* binaryValue);
void printBinary(int* binaryValue, int paddedSize);
int  continuousAcquire(int channel, int kbytes, int dec, char* filename_ch1, char* filename_ch2, char* filename_imu, int is_imu_en);
int  clean_stdin();
void initIMU(void);

double vcoOut(uint32_t fracNum);
double bnwOut(double rampInc, uint16_t);

#endif
