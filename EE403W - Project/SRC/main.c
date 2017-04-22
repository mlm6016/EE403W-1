#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_emc.h"
#include "pin_mux.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "eGFX.h"
#include "eGFX_Driver.h"
#include "FONT_5_7_1BPP.h"
#include "OCR_A_Extended__20px__Bold__SingleBitPerPixelGridFit_1BPP.h"
#include "pin_mux.h"
#include "fsl_device_registers.h"
#include "fsl_i2c.h"
#include "fsl_i2s.h"
#include "fsl_wm8904.h"
#include "Audio.h"
#include <math.h>
#include "Sprites_16BPP_565.h"
#include "Arial_Unicode_MS__48px__Regular__SystemDefault_1BPP.h"
//#include <fsl_ft5406.h>

//**********************************************************
#define  BUFFER_SIZE				480
#define	 DATABUFFER_SIZE 			11
#define	 SAMPLEINBUFFER_SIZE	4320
#define  SAMPLERATE					8000

// Sample Out
volatile uint32_t NextSampleOut = 0;

//Screen Refresh Control, 17fps
uint32_t BackgroundRightBufferIdx = 0;     
volatile uint32_t NextRightBufferReady = 0;

//Screen Output
volatile float * eBuffer;
volatile float * BackgroundeBuffer;
volatile float eBuffer1[BUFFER_SIZE];
volatile float eBuffer2[BUFFER_SIZE];
volatile uint32_t NexteBufferReady = 0;

//LP - Adapting Threshold
volatile float SampleMax = 0;
volatile float Threshold = 3500.0f;
volatile float threshold = 7500.0f;
uint32_t ThresholdCounter = 0;
volatile float * SampleDataBuffer;
volatile float SampleBuffer[SAMPLEINBUFFER_SIZE];
uint32_t SampleInCounter = 0;

//BP - Display to Screen
volatile float * SampleDataBufferD;
volatile float SampleBufferD[SAMPLEINBUFFER_SIZE];
volatile float DisplayScale = 1;

//IIR Cascade Filter - LP at 125Hz - Peak Detection
volatile float * InputDataBufferLP1;
volatile float * OutputDataBufferLP1;
volatile float InputBufferLP1[DATABUFFER_SIZE];
volatile float OutputBufferLP1[DATABUFFER_SIZE];
volatile float * InputDataBufferLP2;
volatile float * OutputDataBufferLP2;
volatile float InputBufferLP2[DATABUFFER_SIZE];
volatile float OutputBufferLP2[DATABUFFER_SIZE];
volatile float * InputDataBufferLP3;
volatile float * OutputDataBufferLP3;
volatile float InputBufferLP3[DATABUFFER_SIZE];
volatile float OutputBufferLP3[DATABUFFER_SIZE];
volatile float * InputDataBufferLP4;
volatile float * OutputDataBufferLP4;
volatile float InputBufferLP4[DATABUFFER_SIZE];
volatile float OutputBufferLP4[DATABUFFER_SIZE];

//IIR Cascade Filter - BP at 175hz, bandwidth of 250hz - Display
volatile float * InputDataBufferC1;
volatile float * OutputDataBufferC1;
volatile float InputBufferC1[DATABUFFER_SIZE];
volatile float OutputBufferC1[DATABUFFER_SIZE];
volatile float * InputDataBufferC2;
volatile float * OutputDataBufferC2;
volatile float InputBufferC2[DATABUFFER_SIZE];
volatile float OutputBufferC2[DATABUFFER_SIZE];

//Heart Rate
volatile float * HRDataBuffer;
volatile float HRBuffer[DATABUFFER_SIZE];
volatile uint32_t waitwidth = 3500;
uint32_t waitwidthcounter = 0;

//Peak Detection
uint32_t samplecounter = 0;
uint32_t sample1 = 0;
uint32_t sample2 = 0;
uint32_t time = 0;
uint32_t heartrate = 0;
bool flag = 0;
uint32_t waitsamples = 0;

//Sample In Data, Right and Left Channel
typedef union 
{
	uint32_t Data;
	int16_t Channel[2];
	
}I2S_FIFO_Data_t;

//Sorting for Median Filter
int medsort()
{ 
	int i,j,temp;
	int sortHRData[DATABUFFER_SIZE];
	for(i=0;i<DATABUFFER_SIZE;i++){
		sortHRData[i] = HRDataBuffer[i];
	}

	for(i=0;i<DATABUFFER_SIZE-1;i++) {
		for(j=0;j<DATABUFFER_SIZE-i-1;j++) {
			if(sortHRData[j]>sortHRData[j+1]){
				temp = sortHRData[j];
				sortHRData[j] = sortHRData[j+1];
				sortHRData[j+1] = temp;
			}
		}
	}
	return sortHRData[(DATABUFFER_SIZE-1)/2];
}

//Sample In Interrupt
void FLEXCOMM6_DriverIRQHandler(void)
{
    if (I2S0->FIFOINTSTAT & I2S_FIFOINTSTAT_TXLVL_MASK)
    {
		//NextSampleOut holds the last value from the I2S RX Interrupt, it is in packed FIFO format
		I2S0->FIFOWR = NextSampleOut;
		//Clear TX level interrupt flag
        I2S0->FIFOSTAT = I2S_FIFOSTAT_TXLVL(1U);
	}
}

//Sample Out Interrupt
void FLEXCOMM7_DriverIRQHandler(void)
{
	//Data processing only needed on Right Channel
	register float HeartRateDispPlay;
	register float HeartRateLP;
	I2S_FIFO_Data_t FIFO_Data;
		
	//IIR order 8 LPF at 125Hz
	float lp1num1 = 0.14434199518337648;
	float lp1num2 = -0.28442805478298855;
	float lp1num3 = 0.14434199518337648;
	float lp1den2 = -1.9444648617460958;
	float lp1den3 = 0.95027972334557886;
	float lp2num1 = 0.34572023710089611;
	float lp2num2 = -0.68671896389840603;
	float lp2num3 = 0.34572023710089617;
	float lp2den2 = -1.9841212834672031;
	float lp2den3 = 0.99407309369291874;
	float lp3num1 = 0.037716146370676197;
	float lp3num2 = -0.068178959024559713;
	float lp3num3 = 0.037716146370676197;
	float lp3den2 = -1.9171909542662215;
	float lp3den3 = 0.9197096295410857;
	float lp4num1 = 0.51931329908023227;
	float lp4num2 = -1.0300640852608061;
	float lp4num3 = 0.51931329908023227;
	float lp4den2 = -1.969216296572047;
	float lp4den3 = 0.9778779586925429;
		
	// IIR order 8 BP at 175hz, bw of 250hz
	float c1num1 = 0.052243778453137261;
	float c1num2 = -0.18573512775437154;
	float c1num3 = 0.26698671208673364;
	float c1num4 = -0.18573512775437162;
	float c1num5 = 0.052243778453137289;
	float c1den2 = -3.8422927499977213;
	float c1den3 = 5.5938777399753814;
	float c1den4 = -3.6598651286014814;
	float c1den5 = 0.90836119435345186;
	float c2num1 = 0.02982206986795672;
	float c2num2 = -0.070886084123522666;
	float c2num3 = 0.082129560263292795;
	float c2num4 = -0.070886084123522847;
	float c2num5 = 0.029822069867956765;
	float c2den2 = -3.7344487197601755;
	float c2den3 = 5.2455036887142263;
	float c2den4 = -3.2854173444609494;
	float c2den5 = 0.77443822040045551;	
	
	// Clear RX level interrupt flag
	I2S1->FIFOSTAT = I2S_FIFOSTAT_RXLVL(1U);	

	// Pulling in Data from audio codec
	FIFO_Data.Data = I2S1->FIFORD;
		
	//LP - Peak Detection
	//Cascade 1
	InputDataBufferLP1[0] = (float)(FIFO_Data.Channel[1]);
	OutputDataBufferLP1[0] = (float)(lp1num1*InputDataBufferLP1[0]+lp1num2*InputDataBufferLP1[1]+lp1num3*InputDataBufferLP1[2]-lp1den2*OutputDataBufferLP1[1]-lp1den3*OutputDataBufferLP1[2]);
	//Cascade 2
	InputDataBufferLP2[0] = (float)OutputDataBufferLP1[0];
	OutputDataBufferLP2[0] = (float)(lp2num1*InputDataBufferLP2[0]+lp2num2*InputDataBufferLP2[1]+lp2num3*InputDataBufferLP2[2]-lp2den2*OutputDataBufferLP2[1]-lp2den3*OutputDataBufferLP2[2]);
	//Cascade 3
	InputDataBufferLP3[0] = (float)OutputDataBufferLP2[0];
	OutputDataBufferLP3[0] = (float)(lp3num1*InputDataBufferLP3[0]+lp3num2*InputDataBufferLP3[1]+lp3num3*InputDataBufferLP3[2]-lp3den2*OutputDataBufferLP3[1]-lp3den3*OutputDataBufferLP3[2]);
	//Cascade 4
	InputDataBufferLP4[0] = (float)OutputDataBufferLP3[0];
	OutputDataBufferLP4[0] = (float)(lp4num1*InputDataBufferLP4[0]+lp4num2*InputDataBufferLP4[1]+lp4num3*InputDataBufferLP4[2]-lp4den2*OutputDataBufferLP4[1]-lp4den3*OutputDataBufferLP4[2]);

	//HeartRate Signal to use for peak detection
	HeartRateLP = (float)OutputDataBufferLP4[0];
		
	//BP - For LCD and Audio Playback
	//BP - Cascade 1
	InputDataBufferC1[0] = (float)(FIFO_Data.Channel[1]);
	OutputDataBufferC1[0] = (float)(c1num1*InputDataBufferC1[0] + c1num2*InputDataBufferC1[1] + c1num3*InputDataBufferC1[2] + c1num4*InputDataBufferC1[3] + c1num5*InputDataBufferC1[4] - c1den2*OutputDataBufferC1[1] - c1den3*OutputDataBufferC1[2] - c1den4*OutputDataBufferC1[3] - c1den5*OutputDataBufferC1[4]);
	//BP - Cascade 2
	InputDataBufferC2[0] = (float)OutputDataBufferC1[0];
	OutputDataBufferC2[0] =(float)(c2num1*InputDataBufferC2[0] + c2num2*InputDataBufferC2[1] + c2num3*InputDataBufferC2[2] + c2num4*InputDataBufferC2[3] + c2num5*InputDataBufferC2[4] - c2den2*OutputDataBufferC2[1] - c2den3*OutputDataBufferC2[2] - c2den4*OutputDataBufferC2[3] - c2den5*OutputDataBufferC2[4]);
	
	//RightChannel is used for the display on the LCD and for audio playback
	HeartRateDispPlay = (float)OutputDataBufferC2[0];
	
	//NextSampleOut constructs the processed signal on both Left and Right Channels for audio playback.
	NextSampleOut = (uint16_t)OutputDataBufferC2[0];
	NextSampleOut = ((uint32_t)(NextSampleOut)) | ((uint32_t)(NextSampleOut<<16));

	//After each sample in, shift the data down one index.
	for (int k = DATABUFFER_SIZE-1; k > 0; k--){   
			InputDataBufferC1[k]  = InputDataBufferC1[k-1];
			InputDataBufferC2[k]  = InputDataBufferC2[k-1];
			OutputDataBufferC1[k] = OutputDataBufferC1[k-1];
			OutputDataBufferC2[k] = OutputDataBufferC2[k-1];
			InputDataBufferLP1[k] = InputDataBufferLP1[k-1];
			OutputDataBufferLP1[k] = OutputDataBufferLP1[k-1];
			InputDataBufferLP2[k] = InputDataBufferLP2[k-1];
			OutputDataBufferLP2[k] = OutputDataBufferLP2[k-1];
			InputDataBufferLP3[k] = InputDataBufferLP3[k-1];
			OutputDataBufferLP3[k] = OutputDataBufferLP3[k-1];
			InputDataBufferLP4[k] = InputDataBufferLP4[k-1];
			OutputDataBufferLP4[k] = OutputDataBufferLP4[k-1];
	}
	
	//Buffering 4320 samples for Peak Detection, Display and Audio Playback
	SampleDataBuffer[SampleInCounter] = (float)(HeartRateLP);
	SampleDataBufferD[SampleInCounter] = (float)(HeartRateDispPlay);
	//SampleDataBufferD[SampleInCounter] = (float)(FIFO_Data.Channel[1]);
	SampleInCounter++;

	if (SampleInCounter == SAMPLEINBUFFER_SIZE)
	{
		//SampleMax is used to adjust the Amplitude Threshold for Peak Dection
		//SampleMax is the largest sample in the 4320 sample buffer
		
		for (int i=0;i<SAMPLEINBUFFER_SIZE;i++)
		{
			if (SampleDataBuffer[i]>SampleMax) {
				SampleMax = SampleDataBuffer[i];
			}
		}	
		SampleInCounter = 0;
		
		
		//Threshold Adaptation from 1,000 to 32,767
		
		if (SampleMax > Threshold)
		{
			Threshold += 2000.0f;
			//ThresholdCounter = 0;
		}
		if (SampleMax <= Threshold){
			//ThresholdCounter++;
			//If device is not being used, decrease threshold towards 1000.
			//if (ThresholdCounter == 2){
				//ThresholdCounter = 0;
				//Threshold = Threshold - 4000.0f;
				//if (Threshold < 1000.0f)
				Threshold -= 2000.0f;
		}
		if (Threshold < 2000.0f) {
			Threshold = 2000.0f;
		}
		
		
		//Bandpass Filter - Display
		for (int i=0;i<BUFFER_SIZE;i++)
		{
			//Downsample the 4320 sample display buffer by 9 to a Display buffer of 480
			//1 sample per LCD pixel
			//PingPong Buffers
			//Displays approximately 0.5s of data to the LCD
			DisplayScale=1;
			BackgroundeBuffer[i] = (DisplayScale*SampleDataBufferD[i*9])/32768.0f;
			if (BackgroundeBuffer == eBuffer1)
			{
				BackgroundeBuffer = eBuffer2;
				eBuffer = eBuffer1;
			}
			else
			{
				BackgroundeBuffer = eBuffer1;
				eBuffer = eBuffer2;
			}
			if(NexteBufferReady == 0)
			NexteBufferReady = 1;
		}
		SampleMax = 0;		 
	}
				
	//Control for screen refresh rate
	//Refresh the screen for every 480 samples received, 17fps. 
	BackgroundRightBufferIdx++;		
	if(BackgroundRightBufferIdx == BUFFER_SIZE)
	{
		//Set a flag to inidicate that the buffer is ready!
		if(NextRightBufferReady == 0)
			NextRightBufferReady = 1;
		//Reset our index that will fill up the background buffer
		BackgroundRightBufferIdx=0;
	}
		
	//Peak Detection for Heart Rate Calculation
	//Each Heart Rate Calculation is based on the knowledge of the sample rate and the amount of samples between each peak
	//Flag asserts "waitwidth" samples after peak has been detected to avoid multiple peak detects on the same recorded heart beat
	if ((HeartRateLP > Threshold)&&(flag==0))
	{	
		if(samplecounter > (waitsamples + waitwidth))
		{
			//Amount of samples between each heart beat peak
			sample1 = sample2;
			sample2 = samplecounter;
			time = sample2-sample1;
			//Shift the data once along the buffer for the next current heart rate calculation can be indexed at 0
			for (int k = DATABUFFER_SIZE-1; k > 0; k--){   
				HRDataBuffer[k]=HRDataBuffer[k-1];
			}
			HRDataBuffer[0] = (60*SAMPLERATE/time)+1;			
			//Median filter for outlier heart rate calculations
			heartrate = medsort();
			flag = 1;
			waitsamples = samplecounter;
		}
	}
	//Reset flag for the following heart rate calculation
	if((flag==1)&&(HeartRateLP<threshold)){
	flag = 0;
	}
	
	//Keeping track of current sample
	samplecounter++;
}

//Initilization of Buffers to store 4320 samples from the LP and BP filters
void SampleInBuffer()
{
	for (int i=1;i<SAMPLEINBUFFER_SIZE;i++)
	{
		SampleBuffer[i] = 0;
		SampleBufferD[i] = 0;
	}
	SampleDataBuffer = SampleBuffer;
	SampleDataBufferD = SampleBufferD;
}

//Initialization of BP and LP Cascade Filter Buffers
void DataInitBuffer()
{
	for (int i=0;i<DATABUFFER_SIZE;i++)
	{
		InputBufferC1[i] = 0;
		OutputBufferC1[i] = 0;
		InputBufferC2[i] = 0;
		OutputBufferC2[i] = 0;
		InputBufferLP1[i] = 0;
		OutputBufferLP1[i] = 0;
		InputBufferLP2[i] = 0;
		OutputBufferLP2[i] = 0;
		InputBufferLP3[i] = 0;
		OutputBufferLP3[i] = 0;
		InputBufferLP4[i] = 0;
		OutputBufferLP4[i] = 0;
		HRBuffer[i] = 0;
	}
	InputDataBufferC1 = InputBufferC1;
	OutputDataBufferC1 = OutputBufferC1;
	InputDataBufferC2 = InputBufferC2;
	OutputDataBufferC2 = OutputBufferC2;
	InputDataBufferLP1 = InputBufferLP1;
	OutputDataBufferLP1 = OutputBufferLP1;
	InputDataBufferLP2 = InputBufferLP2;
	OutputDataBufferLP2 = OutputBufferLP2;
	InputDataBufferLP3 = InputBufferLP3;
	OutputDataBufferLP3 = OutputBufferLP3;
	InputDataBufferLP4 = InputBufferLP4;
	OutputDataBufferLP4 = OutputBufferLP4;	
	HRDataBuffer = HRBuffer;	
	
}

//Initialization of Display Buffer
void InitBuffers()
{
	for (int i=0;i<BUFFER_SIZE;i++)
	{
		eBuffer1[i] = 0;
		eBuffer2[i] = 0;
	}
	
	eBuffer = eBuffer1;
	BackgroundeBuffer = eBuffer2;

}

int main(void)
{
	CLOCK_EnableClock(kCLOCK_InputMux);		
	CLOCK_EnableClock(kCLOCK_Iocon);	
	CLOCK_EnableClock(kCLOCK_Gpio0);  
	CLOCK_EnableClock(kCLOCK_Gpio1);
	//USART0 clock
	CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
	//Initialize the rest 
	BOARD_InitPins();	
	BOARD_BootClockRUN();  
	BOARD_InitDebugConsole();
	BOARD_InitSDRAM();
	eGFX_InitDriver();		
	InitBuffers();
	DataInitBuffer();
	SampleInBuffer();
	InitAudio_CODEC();		
	//Control for display color
	int a = 0;
	int b = 0;
	int c = 0;
		
	while(1)
	{
		//Draw to the screen once the Display Buffer is ready
		while(NextRightBufferReady == 0 || NexteBufferReady == 0)
		{
		}		
		NextRightBufferReady = 0;
		NexteBufferReady = 0;
			
		//Clear the BackBuffer for new data
		eGFX_ImagePlane_Clear(&eGFX_BackBuffer);
					
		for(int i=1;i<eGFX_PHYSICAL_SCREEN_SIZE_X;i++)
		{
			//Green
			if(heartrate<=100)
			{
				 a = 0;
				 b = 255;
				 c = 0;
			//Yellow
			}else if(heartrate>100&&heartrate<=120)
			{
				a = 255;
				b = 255;
				c = 0;
			//Orange
			}else if(heartrate>120&&heartrate<=140)
			{
				a = 255;
				b = 150;
				c = 0;
			}
			//Red
			else{
				a = 255;
				b = 0;
				c = 0;
			}			

		//Points used to draw lines on the display based on the DisplayBuffer 480 Samples
		float Point1e = (eBuffer[i-2]*(eGFX_PHYSICAL_SCREEN_SIZE_Y/2.0));
		float Point2e = (eBuffer[i]*(eGFX_PHYSICAL_SCREEN_SIZE_Y/2.0));
								
		//Draw the lines to the backbuffer		
		eGFX_DrawLine(&eGFX_BackBuffer,                       
					i-1		,	Point1e+eGFX_PHYSICAL_SCREEN_SIZE_Y/2.0,      
					i		,   Point2e+eGFX_PHYSICAL_SCREEN_SIZE_Y/2.0, 
								eGFX_RGB888_TO_RGB565(a,b,c)
					);
		
		}
		
		eGFX_Blit(&eGFX_BackBuffer,0,0,&Sprite_16BPP_565_403W_Logo_flat);
		
		//Write the heart rate to the BackBuffer
		//eGFX_printf(&eGFX_BackBuffer,100,100,&Arial_Unicode_MS__48px__Regular__SystemDefault_1BPP,"%d",heartrate);
		eGFX_printf_HorizontalCentered_Colored(&eGFX_BackBuffer,
											224, 
											&Arial_Unicode_MS__48px__Regular__SystemDefault_1BPP,   
											eGFX_RGB888_TO_RGB565(255,0,0),
											"%d",heartrate);
		/*
		eGFX_printf(&eGFX_BackBuffer,200,25,&FONT_5_7_1BPP,"Heart Rate = %d",heartrate);
		eGFX_printf(&eGFX_BackBuffer,200,45,&FONT_5_7_1BPP,"Heart Rate = %f",HRDataBuffer[0]);
		eGFX_printf(&eGFX_BackBuffer,200,45,&FONT_5_7_1BPP,"Heart Rate = %f",HRDataBuffer[1]);		
		eGFX_printf(&eGFX_BackBuffer,200,65,&FONT_5_7_1BPP,"Heart Rate = %f",HRDataBuffer[2]);
		eGFX_printf(&eGFX_BackBuffer,200,85,&FONT_5_7_1BPP,"Heart Rate = %f",HRDataBuffer[3]);
		eGFX_printf(&eGFX_BackBuffer,200,105,&FONT_5_7_1BPP,"Heart Rate = %f",HRDataBuffer[4]);
		eGFX_printf(&eGFX_BackBuffer,200,125,&FONT_5_7_1BPP,"Heart Rate = %f",HRDataBuffer[5]);
		eGFX_printf(&eGFX_BackBuffer,200,145,&FONT_5_7_1BPP,"Heart Rate = %f",HRDataBuffer[6]);
		eGFX_printf(&eGFX_BackBuffer,200,165,&FONT_5_7_1BPP,"Heart Rate = %f",HRDataBuffer[7]);
		eGFX_printf(&eGFX_BackBuffer,200,185,&FONT_5_7_1BPP,"Heart Rate = %f",HRDataBuffer[8]);
		eGFX_printf(&eGFX_BackBuffer,200,205,&FONT_5_7_1BPP,"Heart Rate = %f",HRDataBuffer[9]);
		eGFX_printf(&eGFX_BackBuffer,200,225,&FONT_5_7_1BPP,"Heart Rate = %f",HRDataBuffer[10]);
		eGFX_printf(&eGFX_BackBuffer,200,245,&FONT_5_7_1BPP,"Heart Rate = %f",HRDataBuffer[11]);*/
		//eGFX_printf(&eGFX_BackBuffer,200,265,&FONT_5_7_1BPP,"Heart Rate = %f",Threshold);
		
		
		
								
		//Dump the BackBuffer to the screen for display
		eGFX_Dump(&eGFX_BackBuffer);
	}
}
