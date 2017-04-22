#include "fsl_device_registers.h"
#include "fsl_i2c.h"
#include "fsl_i2s.h"
#include "fsl_wm8904.h"
#include "fsl_debug_console.h"

#define I2C2_MASTER_CLOCK_FREQUENCY (12000000)

#define SAMPLE_RATE			 (8000U)									//This is what we want the Macro to be
#define SAMPLE_RATE_REG  kWM8904_SampleRate8kHz  //This enum gets sent to the codec
#define OVERSAMPLE_RATE  kWM8904_FsRatio1024X    //Our PLL Rate is 24576000.   So, the oversample rate is 24576000/32000 which is 768
#define I2S_CLOCK_DIVIDER (CLOCK_GetAudioPllOutFreq() / SAMPLE_RATE / 16U / 2U)  //THis computes the clock rate for the I2S bit clock.

/*

	These  data structures are used to initialize the I2S units.
	We are going to use the SDK drivers to set up the I2S peripherals.  The LPC54608
	has a bunch of "Flexcomm" units that can be configured to be several different peripheral
	types.   In the case of I2S,   Flexcomm channels 6 and 7 can be configured for I2S
	
	UM10912 (LPC5460x User Manual) Chapter 21 describes the Flexomm

*/

static i2s_config_t s_TxConfig;
static i2s_config_t s_RxConfig;

/*This function hides the complicated initialization of the onboard CODEC*/
void InitAudio_CODEC()
{

   /*
			Before we set the Flexcom channels for I2S,  we have to set up the
		external WM8904 CODEC.  It has many configuration settings.   Get the datasheet for the WM8904 and you can see 
	  all of the different options.  Page 6 of the WM8904 has a block diagram of the internal functions of the CODEC.
	
		To configure the WM8904,  we have to use an I2C interface.  I2C is a two-wire serial bus
	  that is  common used for inter chip communication.  The WM8904 has an I2C interface which is used
	  to configure the CODEC.   There are registers exposed over this interface.    This init fuction will
	  take care of this intialization. 
	
	  To talk I2C,  we are going to use the NXP SDK drivers and an NXP driver for the WM8904.  The structures below
	   are used by the drivers to configure the I2C interface and the WM8904
	 */
	
   	i2c_master_config_t i2cConfig;

    wm8904_config_t codecConfig;
  
    wm8904_handle_t codecHandle;
	
	 /*
				The external CODEC requires a master clock.  This is used by the sigma-delta converters in the CODEC.
				The LPC54608 has a phased locked loop (PLL) clock synthesizer dedicated for generating audio
				clocks.   The structures below are used by the NXP drivers for the Audio PLL
	 */
	 
	  pll_config_t audio_pll_config = 
		{
        .desiredRate = 8192000U, .inputRate = 12000000U,
    };
		
		pll_setup_t audio_pll_setup;
		
		/*
				The I2C signal that are connected to the WM8902 is from FlexComm 2. (See the schematic for the LPCXpresso54608 to see this connection.
			  Enable the clock for the I2C
	  */
    
		CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);
		
    /* reset FLEXCOMM for I2C */
    RESET_PeripheralReset(kFC2_RST_SHIFT_RSTn);
		
    /* Initialize AUDIO PLL clock */
    CLOCK_SetupAudioPLLData(&audio_pll_config, &audio_pll_setup);
    audio_pll_setup.flags = PLL_SETUPFLAG_POWERUP | PLL_SETUPFLAG_WAITLOCK;
    CLOCK_SetupAudioPLLPrec(&audio_pll_setup, audio_pll_setup.flags);

    /*
			Flexcomm 6 and 7 are used the I2S channels (one for transmit and one for recieve)
   		I2S clocks will be derived from the Audio PLL.
		*/
    CLOCK_AttachClk(kAUDIO_PLL_to_FLEXCOMM6);
    CLOCK_AttachClk(kAUDIO_PLL_to_FLEXCOMM7);
		
		/* Attach PLL clock to MCLK for I2S, no divider */
    CLOCK_AttachClk(kAUDIO_PLL_to_MCLK);
    SYSCON->MCLKDIV = SYSCON_MCLKDIV_DIV(0U);
		
    /*
		 *  Initialize the I2C to talk to the WM8904
		 *
     * enableMaster = true;
     * baudRate_Bps = 100000U;
     * enableTimeout = false;
     */
    I2C_MasterGetDefaultConfig(&i2cConfig);
    i2cConfig.baudRate_Bps = WM8904_I2C_BITRATE;
    I2C_MasterInit(I2C2, &i2cConfig, I2C2_MASTER_CLOCK_FREQUENCY);


    /*
			Now that the I2C is setup, we can talk to the WM8904
			We are using the NXP provided provided drivers
     */
    WM8904_GetDefaultConfig(&codecConfig);
		codecConfig.format.fsRatio = OVERSAMPLE_RATE;     //These Macros are set at the top of the file.
		codecConfig.format.sampleRate = SAMPLE_RATE_REG;
		
		/*
		Tell the driver what I2C Channel we are use and then initialize the CODEC
		*/
    
		codecHandle.i2c = I2C2;
    if (WM8904_Init(&codecHandle, &codecConfig) != kStatus_Success)
    {
        PRINTF("WM8904_Init failed!\r\n");
    }
		
    /* Adjust output volume it to your needs, 0x0006 for -51 dB, 0x0039 for 0 dB etc. */
    /* Page 154 of the WM8904 data sheet has the volume settings*/
		WM8904_SetVolume(&codecHandle, 0x0039, 0x0039);
  
    /*
			The WM8904 is now configured.   We have to set the I2S channels
		  to transmit and recieve audio data
		
		*/
		
    /* Reset FLEXCOMM for I2S.  Flexcomm 6 is the transmit interface and FlexComm7 is the receive*/
    RESET_PeripheralReset(kFC6_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kFC7_RST_SHIFT_RSTn);
		
		
		/*
			We are using the NXP Driver to setup the I2S.   We must populate
		  the configuration structures and their drivers will hit the registers for us.
		
		  The Transmitter is setup to by an I2S master and the receiver as an I2S slave
			
		*/
		/*
     * masterSlave = kI2S_MasterSlaveNormalMaster;
     * mode = kI2S_ModeI2sClassic;
     * rightLow = false;
     * leftJust = false;
     * pdmData = false;
     * sckPol = false;
     * wsPol = false;
     * divider = 1;
     * oneChannel = false;
     * dataLength = 16;
     * frameLength = 32;
     * position = 0;
     * watermark = 4;
     * txEmptyZero = true;
     * pack48 = false;
     */
    I2S_TxGetDefaultConfig(&s_TxConfig);
    s_TxConfig.divider = I2S_CLOCK_DIVIDER;  //This macro is set at the top of the file.  It is a divder to generate the bit clock

    /*
     * masterSlave = kI2S_MasterSlaveNormalSlave;
     * mode = kI2S_ModeI2sClassic;
     * rightLow = false;
     * leftJust = false;
     * pdmData = false;
     * sckPol = false;
     * wsPol = false;
     * divider = 1;
     * oneChannel = false;
     * dataLength = 16;
     * frameLength = 32;
     * position = 0;
     * watermark = 4;
     * txEmptyZero = false;
     * pack48 = false;
     */

    I2S_RxGetDefaultConfig(&s_RxConfig);
		
		
		/*
			The only setting we will change is the FIFO Watermark.
			The incoming and outgoing I2S data go through a set of FIFO buffers.
			The watermark level is the point at which the I2S will generate an interrupt.
			
			We are are going to interrupt when there are at least 2 samples available or less than 2 to transmit.
		  This will give us some to to prep a new sample while another is being transmitted or recieved.
		*/
		
		s_RxConfig.watermark = 2;
		s_TxConfig.watermark = 2;
		
		/*
			Pass our configuration structures to the driver
		*/
    I2S_TxInit(I2S0, &s_TxConfig);
    I2S_RxInit(I2S1, &s_RxConfig);
		
		/*
			Enable Interrupts to the Flexcomms and set to the highest priority
			We are going to trigger interrupts on the FIFO level (watermark) flags 
		*/
		NVIC_SetPriority(FLEXCOMM6_IRQn,0);
		NVIC_SetPriority(FLEXCOMM7_IRQn,0);
				
		NVIC_EnableIRQ(FLEXCOMM6_IRQn);
		NVIC_EnableIRQ(FLEXCOMM7_IRQn);
		
		/*
			  We are going to put in some data to transmit FIFO to get it Started!
				THis will ensure there is always some data to be sent out.   Out Rx IRQ will be putting a sample into the output
				queue	everytime it gets a sample in from the input queue
		*/
		I2S0->FIFOWR = 0xFFFFFFFF;
		I2S0->FIFOWR = 0xFFFFFFFF;
	
    I2S_EnableInterrupts(I2S0, kI2S_TxLevelFlag);
    I2S_EnableInterrupts(I2S1, kI2S_RxLevelFlag);
    
    I2S_Enable(I2S0);
		I2S_Enable(I2S1);


	  /* At this point, the interrupts will start firing*/

}
