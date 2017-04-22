/***
 *      _        _    ____    _____ 
 *     | |      / \  | __ )  |___ / 
 *     | |     / _ \ |  _ \    |_ \ 
 *     | |___ / ___ \| |_) |  ___) |
 *     |_____/_/   \_\____/  |____/ 
 *                                  
 */

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
//**********************************************************
#define  BUFFER_SIZE  									 1024
#define  ALPHA													 0.9


volatile float * RightBuffer;						//This is a pointer to Data we can use
volatile float * BackgroundRightBuffer; //This is a pointer to a buffer that the DMIC IRQ routine is filling up

uint32_t BackgroundRightBufferIdx = 0;     //This variable is what we use to track where we are in the background buffer

volatile float RightBuffer1[BUFFER_SIZE];  //We need 2 buffers in memory to work with.  One to store data an one to have for working on in the main loop
volatile float RightBuffer2[BUFFER_SIZE];
volatile uint32_t NextRightBufferReady = 0;

volatile float * LeftBuffer;						//This is a pointer to Data we can use
volatile float * BackgroundLeftBuffer; //This is a pointer to a buffer that the DMIC IRQ routine is filling up

uint32_t BackgroundLeftBufferIdx = 0;     //This variable is what we use to track where we are in the background buffer

volatile float LeftBuffer1[BUFFER_SIZE];  //We need 2 buffers in memory to work with.  One to store data an one to have for working on in the main loop
volatile float LeftBuffer2[BUFFER_SIZE];
volatile uint32_t NextLeftBufferReady = 0;

volatile float * eBuffer;						//This is a pointer to Data we can use
volatile float * BackgroundeBuffer; //This is a pointer to a buffer that the DMIC IRQ routine is filling up

uint32_t BackgroundeBufferIdx = 0;     //This variable is what we use to track where we are in the background buffer

volatile float eBuffer1[BUFFER_SIZE];  //We need 2 buffers in memory to work with.  One to store data an one to have for working on in the main loop
volatile float eBuffer2[BUFFER_SIZE];
volatile uint32_t NexteBufferReady = 0;
//************************************************************

/*

In this lab,   you will test the onboard audio CODEC.   If you look at the Schematic for the LPCXpresso54608 
board,   you will find that there is a CODEC (combined ADC and DAC) hooked to microcontroller through 2 I2S ports.

Some Data converters are easy to set up and use as they only require a few hardware configuration pins.   The WM8904
has a more complicated startup procedure.   In addition to the I2S ports for the ADC and DAC,   there is a connection 
to the micocontroller via an I2C connection.   I2C is a standard prototcol for inter chip communication.   The
Wikipedia page is a good place to start.

This Lab will give you an initialization function for the CODEC.   The intialization function will be commented so you can
follow along.

Once the CODEC is initialized,   you will have 2 interrupts handlers to work with.  One will be used for the ADC and the other for the DAC.
The CODEC will be used in such a way that we can do per-sample real time processing.

*/


volatile uint32_t NextSampleOut = 0;

/*
	This union will be used to split 32-bit fifo data into 2 int16_t samples.
	Unions are uses to overlay several variable across the same memory.
*/
typedef union 
{
	uint32_t Data;
	int16_t Channel[2];
	
}I2S_FIFO_Data_t;


/*
	Special Note!!  The routines betwen are the interrupt handlers
for Flexcomm 6 and 7.    We are directly processing the data in the interrupts.
I commented out the default handlers in fsl_flexcomm.c   The NXP I2s routines for
transmitting and recieving data are quiet bulky.  We are going define our own interrupt handlers here.

*/

/***
 *      ___ ____  ____    _______  __  ___       _                             _   
 *     |_ _|___ \/ ___|  |_   _\ \/ / |_ _|_ __ | |_ ___ _ __ _ __ _   _ _ __ | |_ 
 *      | |  __) \___ \    | |  \  /   | || '_ \| __/ _ \ '__| '__| | | | '_ \| __|
 *      | | / __/ ___) |   | |  /  \   | || | | | ||  __/ |  | |  | |_| | |_) | |_ 
 *     |___|_____|____/    |_| /_/\_\ |___|_| |_|\__\___|_|  |_|   \__,_| .__/ \__|
 *                                                                      |_|        
 */
 
void FLEXCOMM6_DriverIRQHandler(void)
{
    if (I2S0->FIFOINTSTAT & I2S_FIFOINTSTAT_TXLVL_MASK)
    {
        /*
					NextSampleOut Holds the last value from the I2S RX Interrupt.
				  It is also ready in the "packed" FIFO format
			  */
				I2S0->FIFOWR = NextSampleOut;
		
				 /* Clear TX level interrupt flag */
        I2S0->FIFOSTAT = I2S_FIFOSTAT_TXLVL(1U);
		}
}

/***
 *      ___ ____  ____    ____  __  __  ___       _                             _   
 *     |_ _|___ \/ ___|  |  _ \ \ \/ / |_ _|_ __ | |_ ___ _ __ _ __ _   _ _ __ | |_ 
 *      | |  __) \___ \  | |_) | \  /   | || '_ \| __/ _ \ '__| '__| | | | '_ \| __|
 *      | | / __/ ___) | |  _ <  /  \   | || | | | ||  __/ |  | |  | |_| | |_) | |_ 
 *     |___|_____|____/  |_| \_\/_/\_\ |___|_| |_|\__\___|_|  |_|   \__,_| .__/ \__|
 *                                                                       |_|        
 */
void FLEXCOMM7_DriverIRQHandler(void)
{
		register float LeftChannel;
		register float RightChannel;
	  I2S_FIFO_Data_t FIFO_Data; 
	
     /* Clear RX level interrupt flag */
     I2S1->FIFOSTAT = I2S_FIFOSTAT_RXLVL(1U);
	
	   /*
				Read the Recieve FIFO.   Data is packed as two samples in one 32-bit word.  We will immediately store the data
				in a variable that is used is the transmit routine to send incoming data back out.
		 */
	    FIFO_Data.Data = I2S1->FIFORD;
	    NextSampleOut = FIFO_Data.Data; //dump the data back out!
	
	  /*
			In the configuration for this lab,  2 channels of data are packed
			in one 32-bit word.  The Right Channel is in the upper 16-bits and the Left-Channel in the lower
		  Notice between we can use a "union" (I2S_FIFO_Data_t) to read the data in as 32-bit and access it as two 16-bit signed numbers.
	  */
	   
	   LeftChannel = (float)(FIFO_Data.Channel[0])/32768.0f;
	   RightChannel = (float)(FIFO_Data.Channel[1])/32768.0f;

		/*
			Do something with the Left and Right channel here
		
		*/
		//*****************************Added Code************************************
	
		
		BackgroundRightBuffer[BackgroundRightBufferIdx] = RightChannel;
		BackgroundLeftBuffer[BackgroundLeftBufferIdx] = LeftChannel;
		BackgroundeBuffer[BackgroundeBufferIdx] = (float)(1.0-ALPHA)*fabs(RightChannel) + ALPHA*BackgroundeBuffer[BackgroundeBufferIdx-1];
		
		BackgroundeBufferIdx++;
		BackgroundRightBufferIdx++;
		BackgroundLeftBufferIdx++;

		if(BackgroundeBufferIdx == BUFFER_SIZE) //Envelope buffer swap
		{
			if(BackgroundeBuffer == eBuffer1)
				{
					BackgroundeBuffer = eBuffer2;
					eBuffer = eBuffer1;
				}
				else
				{
					BackgroundeBuffer = eBuffer1;
					eBuffer = eBuffer2;
				}

				/*Set a flag to inidicate that the buffer is ready!*/
				if(NexteBufferReady == 0)
					NexteBufferReady = 1;
				
				/*Reset our index that will fill up the background buffer*/
	  		BackgroundeBufferIdx=0;
		}

		if(BackgroundRightBufferIdx == BUFFER_SIZE)
		{
			  
			/***
			 *      ____                       _____ _          
			 *     / ___|_      ____ _ _ __   |_   _| |__   ___ 
			 *     \___ \ \ /\ / / _` | '_ \    | | | '_ \ / _ \
			 *      ___) \ V  V / (_| | |_) |   | | | | | |  __/
			 *     |____/ \_/\_/ \__,_| .__/    |_| |_| |_|\___|
			 *      ____         __  _|_|                       
			 *     | __ ) _   _ / _|/ _| ___ _ __ ___           
			 *     |  _ \| | | | |_| |_ / _ \ '__/ __|          
			 *     | |_) | |_| |  _|  _|  __/ |  \__ \          
			 *     |____/ \__,_|_| |_|  \___|_|  |___/          
			 *                                                  
			 */

   /*If the backgroundbuffer is filled, flop the pointers

			Note that there are many other ways to accomplish the same "ping-pong" approach but this will give you a place to
			start.   Some ping-pong uses an array of pointers (more than 2 buffers) and use a variable to track which is active and which one is
			being filled.
			
			This approach just looks at what the pointers are pointing too and swaps them.
   */			

			if(BackgroundRightBuffer == RightBuffer1)
				{
					BackgroundRightBuffer = RightBuffer2;
					RightBuffer = RightBuffer1;
				}
				else
				{
					BackgroundRightBuffer = RightBuffer1;
					RightBuffer = RightBuffer2;
				}

				/*Set a flag to inidicate that the buffer is ready!*/
				if(NextRightBufferReady == 0)
					NextRightBufferReady = 1;
				
				/*Reset our index that will fill up the background buffer*/
	  		BackgroundRightBufferIdx=0;
	  }

		if(BackgroundLeftBufferIdx == BUFFER_SIZE)
		{
			  
			/***
			 *      ____                       _____ _          
			 *     / ___|_      ____ _ _ __   |_   _| |__   ___ 
			 *     \___ \ \ /\ / / _` | '_ \    | | | '_ \ / _ \
			 *      ___) \ V  V / (_| | |_) |   | | | | | |  __/
			 *     |____/ \_/\_/ \__,_| .__/    |_| |_| |_|\___|
			 *      ____         __  _|_|                       
			 *     | __ ) _   _ / _|/ _| ___ _ __ ___           
			 *     |  _ \| | | | |_| |_ / _ \ '__/ __|          
			 *     | |_) | |_| |  _|  _|  __/ |  \__ \          
			 *     |____/ \__,_|_| |_|  \___|_|  |___/          
			 *                                                  
			 */

   /*If the backgroundbuffer is filled, flop the pointers

			Note that there are many other ways to accomplish the same "ping-pong" approach but this will give you a place to
			start.   Some ping-pong uses an array of pointers (more than 2 buffers) and use a variable to track which is active and which one is
			being filled.
			
			This approach just looks at what the pointers are pointing too and swaps them.
   */			

			if(BackgroundLeftBuffer == LeftBuffer1)
				{
					BackgroundLeftBuffer = LeftBuffer2;
					LeftBuffer = LeftBuffer1;
				}
				else
				{
					BackgroundLeftBuffer = LeftBuffer1;
					LeftBuffer = LeftBuffer2;
				}

				/*Set a flag to inidicate that the buffer is ready!*/
				if(NextLeftBufferReady == 0)
					NextLeftBufferReady = 1;
				
				/*Reset our index that will fill up the background buffer*/
	  		BackgroundLeftBufferIdx=0;
	  }
		
		//*****************************END Added Code************************************
}

void InitBuffers()
{
	for (int i=0;i<BUFFER_SIZE;i++)
	{
		RightBuffer1[i] = 0;
		RightBuffer2[i] = 0;
		LeftBuffer1[i] = 0;
		LeftBuffer2[i] = 0;
		eBuffer1[i] = 0;
		eBuffer2[i] = 0;
	}

	RightBuffer = RightBuffer1;
	LeftBuffer = LeftBuffer1;
	eBuffer = eBuffer1;

	BackgroundRightBuffer = RightBuffer2;
	BackgroundLeftBuffer = LeftBuffer2;
	BackgroundeBuffer = eBuffer2;

	NextRightBufferReady = 0;
	NextLeftBufferReady = 0;
	NexteBufferReady = 0;
}

int main(void)
{

    CLOCK_EnableClock(kCLOCK_InputMux);
		
    CLOCK_EnableClock(kCLOCK_Iocon);
	
    CLOCK_EnableClock(kCLOCK_Gpio0);
  
    CLOCK_EnableClock(kCLOCK_Gpio1);

  	/* USART0 clock */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* Initialize the rest */
    BOARD_InitPins();
	
    BOARD_BootClockRUN();
  
    BOARD_InitDebugConsole();

		BOARD_InitSDRAM();

		eGFX_InitDriver();
		
		InitBuffers();

      /*
				This function initializes the WM8904 CODEC and two I2S ports to send and recieve audio.
				Read through the comments in the function. (See Audio.c)
			*/

   	InitAudio_CODEC();
		
		PRINTF("\r\nLab 2 r\n");
	
		while(1)
		{
				/*
						Audio Data is processed in the IRQ routine.   Do what you want here to display data, etc.
				*/
			
			while(NextRightBufferReady == 0)
			{
			}
		
			NextRightBufferReady = 0;
			
			while(NextLeftBufferReady == 0)
			{
			}
			
			NextLeftBufferReady = 0;

						while(NexteBufferReady == 0)
			{
			}
			
			NexteBufferReady = 0;
			
		   eGFX_ImagePlane_Clear(&eGFX_BackBuffer);
			
			for(int i=1;i<eGFX_PHYSICAL_SCREEN_SIZE_X;i++)
							{
  
								/*To draw a line we need 2 points.  The loop will grab x[n] and x[n-1]
			
								  Let's draw to the screen.   There are a bunch of "primitive" functions (lines, circles, etc) in eGFX.h.
									this example should get you started
									*/
			
								
							/*	Remember that the DMIC IRQ routine store data in the buffer as a normalized floating point number (-1.0 to 1.0)
								
							 	We are going to rescale those numbers to something that makes sense for our screen.   Since our screen is 272 pixels tall, we will rescale
								 the amplide to half that.   The reason for use 1/2 of the screen size is that we are going to center the waveform at the half way point in 
								the y direction.
								
								*/
								float Point1E = (eBuffer[i-1]*(eGFX_PHYSICAL_SCREEN_SIZE_Y/6));
								
								float Point2E = (eBuffer[i]*(eGFX_PHYSICAL_SCREEN_SIZE_Y/6));
								
								float Point1R = (RightBuffer[i-1]*(eGFX_PHYSICAL_SCREEN_SIZE_Y/6));

								float Point2R = (RightBuffer[i]*(eGFX_PHYSICAL_SCREEN_SIZE_Y/6));
								
								float Point1L = (RightBuffer[i-1]*(eGFX_PHYSICAL_SCREEN_SIZE_Y/6));

								float Point2L = (RightBuffer[i]*(eGFX_PHYSICAL_SCREEN_SIZE_Y/6));
							
								
								eGFX_DrawLine(&eGFX_BackBuffer,                       //We always have to tell the graphics functions what we are drawing to
												i-1		,		Point1R+eGFX_PHYSICAL_SCREEN_SIZE_Y/3.0,       //X and Y coordinate of starting point.   Our Y value is the scaled value of the microphone sample + 1/2 of the screen height
												i		  ,   Point2R+eGFX_PHYSICAL_SCREEN_SIZE_Y/3.0, 			//X and Y coordinate of end point
												eGFX_RGB888_TO_RGB565(255,0,0)              /*The color.   The LCD uses 16-bit color is 5-6-5 form 
																																		5 bits for red, 5 bits for blue and 6 bits for green.
								
																																			eGFX_RGB888_TO_RGB565 is a macro that converts RGB values of 0-255 (RGB888) to RGB565.
																																    	eGFX_RGB888_TO_RGB565(0,255,0) would be solid green
								
																																		*/
												);
								
						/*	eGFX_DrawLine(&eGFX_BackBuffer,                       //We always have to tell the graphics functions what we are drawing to
												i-1		,		Point1L+eGFX_PHYSICAL_SCREEN_SIZE_Y*2.0/3.0,       //X and Y coordinate of starting point.   Our Y value is the scaled value of the microphone sample + 1/2 of the screen height
												i		  ,   Point2L+eGFX_PHYSICAL_SCREEN_SIZE_Y*2.0/3.0, 			//X and Y coordinate of end point
												eGFX_RGB888_TO_RGB565(0,255,0)              /*The color.   The LCD uses 16-bit color is 5-6-5 form 
																																		5 bits for red, 5 bits for blue and 6 bits for green.
								
																																			eGFX_RGB888_TO_RGB565 is a macro that converts RGB values of 0-255 (RGB888) to RGB565.
																																    	eGFX_RGB888_TO_RGB565(0,255,0) would be solid green
								
																																		*/
												);
												
							eGFX_DrawLine(&eGFX_BackBuffer,                       //We always have to tell the graphics functions what we are drawing to
												i-1		,		Point1E+eGFX_PHYSICAL_SCREEN_SIZE_Y/3.0,       //X and Y coordinate of starting point.   Our Y value is the scaled value of the microphone sample + 1/2 of the screen height
												i		  ,   Point2E+eGFX_PHYSICAL_SCREEN_SIZE_Y/3.0, 			//X and Y coordinate of end point
												eGFX_RGB888_TO_RGB565(255,255,0)              /
												);
							
							eGFX_DrawLine(&eGFX_BackBuffer,                       //We always have to tell the graphics functions what we are drawing to
												i-1		,		eGFX_PHYSICAL_SCREEN_SIZE_Y/3.0-Point1E,       //X and Y coordinate of starting point.   Our Y value is the scaled value of the microphone sample + 1/2 of the screen height
												i		  ,   eGFX_PHYSICAL_SCREEN_SIZE_Y/3.0-Point2E, 			//X and Y coordinate of end point
												eGFX_RGB888_TO_RGB565(255,255,0)              
												);*/
							
							}
			
			/* eGFX_printf(&eGFX_BackBuffer,
										200,250,   //The x and y coordinate of where to draw the text.
										&FONT_5_7_1BPP,   //Long font name!
									  "Poinnt2E = %f", Point2E);*/
							
			
			eGFX_Dump(&eGFX_BackBuffer);
		}
}



