/* ###################################################################
** 	   Application : NXP_CUP CAR UNIZA
**     Created by  : Tomáš Ivan
**     Abstract    : This application was created to control and actuation an intelligent car designed for NXP_CUP
**     Processor   : S32K144
**     Filename    : main.c
**     Date 	   : 12_March_2019
**     Last UpDate : 6_October_2019
**
**	   Control options :
**	   			- fully autonomous control of the car along the race track
** ###################################################################*/

/* Including necessary module. Cpu.h contains other modules needed for compiling.*/
#include "Cpu.h"
#include "pin_mux.h"
#include "clockMan1.h"
#include "pwrMan1.h"
#include "trgmux1.h"
#include "lpuart1.h"
#include "lpuart2.h"
#include "dmaController1.h"
#include "lpspiCom1.h"
#include "flexTimer_pwm3.h"
#include "pdb0.h"
#include "pdb1.h"
#include "adConv0.h"
#include "adConv1.h"

/* User includes (#include below this line is not maintained by Processor Expert) */
#include "peripherals_config.h"
#include "freemaster.h"
#include "actuate_s32k.h"
#include "meas_s32k.h"
#include "state_machine.h"
#include "motor_structure.h"

  volatile int exit_code = 0;

/*------------------------------------
* Application Functions
* ----------------------------------*/
void		DelayForStart();
void		CaptureReading();
int16_t   	CaptureProcessing();

/*------------------------------------
* CAR control variables
* ----------------------------------*/
// ******************************** CAR OPTION ********************************//
// UNIZA 1 - I tried the code on this car
	tFloat 	ServoLimit_Right 	= 1.9F;
	tFloat  ServoRotation 		= 1.4F;
	tFloat 	ServoLimit_Left 	= 1.15F;
	tFloat  ServoRotation_calib	= 0.0F;
	tFloat 	ServoRotation_Init 	= 1.45F;
	tFloat 	PWM_Max 			= 60.0F;
	tFloat 	PWM_Min 			= 40.0F;
	tFloat 	ThresholdShift 		= 0.0F;
//
////// UNIZA 2
//	tFloat 	ServoLimit_Right 	= 1.3F;
//	tFloat  ServoRotation 		= 1.0F;
//	tFloat 	ServoLimit_Left 	= 0.8F;
//	tFloat  ServoRotation_calib	= 0.0F;
//	tFloat 	ServoRotation_Init 	= 1.0F;
//	tFloat 	PWM_Max 			= 60.0F;
//	tFloat 	PWM_Min 			= 40.0F;
//	tFloat 	ThresholdShift 		= 400.0F;
//
////// UNIZA 3
//	tFloat 	ServoLimit_Right 	= 2.35F;
//	tFloat  ServoRotation 		= 2.0F;
//	tFloat 	ServoLimit_Left 	= 1.75F;
//	tFloat  ServoRotation_max	= 2.4F;
//	tFloat 	ServoRotation_min 	= 1.7F;
//	tFloat 	PWM_Max 			= 60.0F;
//	tFloat 	PWM_Min 			= 40.0F;
//	tFloat 	ThresholdShift 		= 400.0F;
//
////// UNIZA 4
//	tFloat 	ServoLimit_Right 	= 2.20F;
//	tFloat  ServoRotation 		= 1.85F;
//	tFloat 	ServoLimit_Left 	= 1.5F;
//	tFloat  ServoRotation_max	= 2.3F;
//	tFloat 	ServoRotation_min 	= 1.75F;
//	tFloat 	PWM_Max 			= 60.0F;
//	tFloat 	PWM_Min 			= 40.0F;
//	tFloat 	ThresholdShift 		= 400.0F;

/*------------------------------------
* Application State & Control Variables
* ----------------------------------*/
uint8_t    				appState = APP_INIT;
uint8_t    				rotationDir = ROTATION_DIR_CW;
volatile   uint8_t 		appSwitchState = 0, faultSwitchClear;
int16_t 				switchCounter[2], switchOffCounter = 1;
uint32_t   				ledCounter, whiteLedCounter, StartTimer_1 = 250, StartTimer_2 = 0;

/*------------------------------------
* Structures & Unions
* ----------------------------------*/
tDriveStatus 			driveStatus;
tFaultStatus 			faultStatus, faultStatusLatched;
tReadingCapture 		ReadingCapture;
tProcessingCapture 		ProcessingCapture;
tControlCarDirection	ControlCarDirection;
tControlCarVelocity		ControlCarVelocity;
tHWmeasuring			HWmeasuring;
tADCresults  			ADCResults;

/*------------------------------------
* Filters
* ----------------------------------*/
GDFLIB_FILTER_MA_T_FLT	Udcb_filt, Ubat_filt, Potentiometer_filt, Track_filt;
GDFLIB_FILTER_MA_T_FLT flttrMyMA = GDFLIB_FILTER_MA_DEFAULT_FLT;

/*-----------------------------------------------------------*
* Track processing, Threshold & Drive Control OPTIONs
* ----------------------------------------------------------*/
uint16_t 	Threshold_Option = 6U;
uint16_t	TrackCalculation_Option = 5U;



/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
int main(void)
{
	// MCU peripherals initialization
	McuClockConfig();
	McuCacheConfig();
	McuPowerConfig();
	McuIntConfig();
	McuTrigmuxConfig();
	McuPinsConfig();
	McuLpuartConfig();
	McuLpitConfig();
	McuAdcConfig();
	McuPdbConfig();
	McuFtmConfig();

	// FreeMASTER initialization
	FMSTR_Init();

	/* FILTER FOR VALUES - INIT */
	/* Initialize DC bus voltage moving average filter  */
	GDFLIB_FilterMAInit_FLT(&Udcb_filt);
	Udcb_filt.fltLambda = DCB_V_FILTER_MA_LAMBDA;

	/* Initialize BAT voltage moving average filter */
	GDFLIB_FilterMAInit_FLT(&Ubat_filt);
	Ubat_filt.fltLambda = BAT_V_FILTER_MA_LAMBDA;

	/* Initialize Potentiometer voltage moving average filter */
	GDFLIB_FilterMAInit_FLT(&Potentiometer_filt);
	Potentiometer_filt.fltLambda = POT_FILTER_MA_LAMBDA;

	/* Initialize TrackFilttering moving average filter */
	GDFLIB_FilterMAInit_FLT (&Track_filt);
	Track_filt.fltLambda = (tFloat)(0.07);

	/* INIT SETTINGS PINS FOR CAMMERA */
	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
	PINS_DRV_ClearPins(PTE, 1<<2);
	PINS_DRV_ClearPins(PTE, 1<<16);

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  /* For example: for(;;) { } */
    for(;;)
    {
    	/* FreeMASTER */
    	FMSTR_Poll();

        /* Call application state machine function */
    	AppStateMachine[appState]();
    	AppStateLed[appState]();

    	/* Buttons checking */
    	CheckSwitchState();
    }

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the Freescale S32K series of microcontrollers.
**
** ###################################################################
*/



/*******************************************************************************
* Interrupt Routines
*******************************************************************************/

/*******************************************************************************
*
* Function: 	PDB0_IRQHandler(void)
*
* Description:  PDB0 Interrupt Service Routine
*
*******************************************************************************/
void PDB0_IRQHandler(void)
{
	// Disable PDB0
	PDB_DRV_Disable(INST_PDB0);

	/* Clear PDB0 sequence errors */
	PDB_DRV_ClearAdcPreTriggerSeqErrFlags(INST_PDB0, 0, 0xFF);

	// Enable PDB0
	PDB_DRV_Enable(INST_PDB0);
}

/*******************************************************************************
*
* Function: 	PDB1_IRQHandler(void)
*
* Description:  PDB1 Interrupt Service Routine
*
*******************************************************************************/
void PDB1_IRQHandler(void)
{
	// Disable PDB1
	PDB_DRV_Disable(INST_PDB1);

	// Clear PDB1 sequence errors
	PDB_DRV_ClearAdcPreTriggerSeqErrFlags(INST_PDB1, 0, 0xFF);

	// Enable PDB1
	PDB_DRV_Enable(INST_PDB1);
}

/*******************************************************************************
*
* Function: 	ADC1_IRQHandler()
*
* Description:  ADC1 interrupt service routine
*
*******************************************************************************/
void ADC1_IRQHandler()
{
	/* Function, for delay after setting right button on yellow board */
	DelayForStart();

	// Set period for PWM servo control using LPIT0_Ch1 (50Hz)
	// ADC_IQR is in 20kHz loop, so we must divided it by 400.
	if (driveStatus.B.ServoAllowed == 1)
	{
		ControlCarDirection.pwm_counter++;

		if (ControlCarDirection.pwm_counter == 400)
		{
			LPIT_DRV_StartTimerChannels(INST_LPIT0, 2);
			PINS_DRV_SetPins(PTC, 1 << 10);
			ControlCarDirection.pwm_counter = 0;
		}
	}
	else
	{
		LPIT_DRV_StopTimerChannels(INST_LPIT0, 2);
	}

	/* Measuring from ADCs */
	// Battery voltage measurement
	MEAS_GetBATVoltage(&ADCResults.BATVVoltage);
	// DC Bus voltage measurement
	MEAS_GetDCBVoltage(&ADCResults.DCBVVoltage);
//  // Left motor current raw value measurement - NOTE !!! must set it for using/view Currents !!!
//	MEAS_GetMOT_LCurrent(&ADCResults.MOT_LIVoltageRAW);
	// Right motor current raw value measurement
	MEAS_GetValue_PotentiometerOnYelowBoard(&ADCResults.PotentiometerOnYellowBoardRAW);

	/* Filtering values from ADCs */
	// DC Bus voltage filter
	HWmeasuring.u_dc_bus_filt = GDFLIB_FilterMA(ADCResults.DCBVVoltage, &Udcb_filt);
	// Battery voltage filter
	HWmeasuring.u_bat_filt = GDFLIB_FilterMA(ADCResults.BATVVoltage, &Udcb_filt);
//	// Real Left motor current filter - NOTE !!! must set it for using/view Currents !!!
//	ADCResults.MOT_LIVoltage = GDFLIB_FilterMA(ADCResults.MOT_LIVoltageRAW, &ULI_filt);
	// Potentiometer on processor board filter
	ADCResults.PotentiometerOnYellowBoard = GDFLIB_FilterMA(ADCResults.PotentiometerOnYellowBoardRAW, &Potentiometer_filt);

    // Application variables record
    FMSTR_Recorder();
}

/*******************************************************************************
*
* Function: 	LPIT0_Ch0_IRQHandler()
*
* Description:  track reading & processing
*
*******************************************************************************/
void LPIT0_Ch0_IRQHandler()
{
    // Get capture from the camera & the track processing, if camera is allowed
	if(driveStatus.B.CameraAllowed == 1)
	{
		CaptureReading();
	}

	// clearing flag of 0.ch interrupt
    LPIT_DRV_ClearInterruptFlagTimerChannels(0, 0b1);
}

/*******************************************************************************
*
* Function: 	LPIT0_Ch1_IRQHandler()
*
* Description:  LPIT channel 0 time-out interrupt service routine
* 				(servo control)
*
*******************************************************************************/
void LPIT0_Ch1_IRQHandler()
{
	// set servo PWM, if servo is allowed
	if(driveStatus.B.ServoAllowed == 1)
	{
		LPIT_DRV_StopTimerChannels(INST_LPIT0, 2);
		PINS_DRV_ClearPins(PTC, 1 << 10);

		// saturation on servo limits
		if(ServoRotation <= ServoLimit_Left)
		{
			ServoRotation = ServoLimit_Left;
		}
		else if(ServoRotation >= ServoLimit_Right)
		{
			ServoRotation = ServoLimit_Right;
		}

		// scaling servo rotation to microseconds [us]
		ControlCarDirection.ServoRotation_Scale = (uint32_t)(MLIB_Mul(ServoRotation, 1000.0F));

		// setting servo PWM
		LPIT_DRV_SetTimerPeriodByUs(INST_LPIT0, 1, ControlCarDirection.ServoRotation_Scale);
	}

	// clearing flag of 1.ch interrupt
	LPIT_DRV_ClearInterruptFlagTimerChannels(0, 0b10);
}
/*******************************************************************************
*
* Function: 	LPIT0_Ch2_IRQHandler()
*
* Description:  LPIT channel 2 time-out interrupt service routine
* 				(speed and current control loop)
*
*******************************************************************************/
void LPIT0_Ch2_IRQHandler()
{
	// set DCmotors PWMs, if DCmotors are allowed
	if(driveStatus.B.DCMotorsAllowed == 1)
	{
		// using electric differential
		// (in the curve - outer wheel must be set to lower speed, than internal)
		// (on the straight - booth wheels must by set to maximum speed)

//		if(ProcessingCapture.MiddleOfLine < ProcessingCapture.MiddleLeft)
//		{
//			ControlCarVelocity.PWM1 = PWM_Max;
//			ControlCarVelocity.PWM2 = PWM_Min;
//		}
//		else if(ProcessingCapture.MiddleOfLine > ProcessingCapture.MiddleRight)
//		{
//			ControlCarVelocity.PWM1 = PWM_Min;
//			ControlCarVelocity.PWM2 = PWM_Max;
//		}
//		else
//		{
			ControlCarVelocity.PWM1 = ADCResults.PotentiometerOnYellowBoard; //PWM_Max;
			ControlCarVelocity.PWM2 = ADCResults.PotentiometerOnYellowBoard; //PWM_Max;
//		}

	}
	else
	{
		// if DCmotors PWMs are't allowed, Udc_motor must by 0V (50% duty)
		ControlCarVelocity.PWM1 = 50.0F;
		ControlCarVelocity.PWM2 = 50.0F;
	}

	// function with scaling and setting PWMs to DCmotors drivers
	ACTUATE_SetDutycycle(ControlCarVelocity.PWM1, 0, ControlCarVelocity.PWM2, 0);

	// clearing flag of 2.ch interrupt
	LPIT_DRV_ClearInterruptFlagTimerChannels(0, 0b100);
}
/******************************************************************************
*
* Function:		void AppInit(void)
*
* Description: 	Application INIT state function
*
*******************************************************************************/
void AppInit(void)
{
    driveStatus.B.ServoAllowed 				= 1;
	driveStatus.B.DCMotorsAllowed 			= 0;
	driveStatus.B.CameraAllowed 			= 1;
    driveStatus.B.Calib 					= 0;
    driveStatus.B.SwStart 					= 0;

	ControlCarDirection.ServoDiff 			= 0.0F;

	ReadingCapture.MultRatio				= 1.0F;

	ProcessingCapture.MiddleOfLine 			= 64U;
	ProcessingCapture.FinishLine  			= 0;
	ProcessingCapture.RightLine_Detect 		= 0U;
	ProcessingCapture.LeftLine_Detect 		= 0U;
	ProcessingCapture.DiffBorder 			= 0;
	ProcessingCapture.MiddleOfLineDiff 		= 0;
	ProcessingCapture.LeftLine_DetectOld 	= 0;
	ProcessingCapture.RightLine_DetectOld 	= 0;
	ProcessingCapture.CamBool		 		= 0;
	ProcessingCapture.LeftLine_End 			= 0;
	ProcessingCapture.LeftLine_Thick 		= 0;
	ProcessingCapture.LeftLine_Position 	= 0;
	ProcessingCapture.RightLine_End 		= 0;
	ProcessingCapture.RightLine_Thick 		= 0;
	ProcessingCapture.RightLine_Position 	= 0;
	ProcessingCapture.MiddleOfLine 			= 64U;
	ProcessingCapture.RightLine_Detect 		= 0U;
	ProcessingCapture.LeftLine_Detect 		= 0U;
	ProcessingCapture.DiffOfLineOld 		= 0U;
	ProcessingCapture.DiffOfLine	 		= 0U;

    // Disable all PWMs
    ACTUATE_DisableOutput(HW_INPUT_TRIG1);

    // Disable DCmotors drivers
    PINS_DRV_SetPins(PTA, 1 << 2); // disable U1
   	PINS_DRV_SetPins(PTA, 1 << 3); // disable U2

    appState = APP_STOP;
}

/*******************************************************************************
*
* Function: 	void AppStop(void)
*
* Description: 	Application STOP state function
*
*******************************************************************************/
void AppStop(void)
{
	if(appSwitchState == 1)
	{
		// Enable ACTUATE (PWMs) output
		ACTUATE_EnableOutput(HW_INPUT_TRIG1);

		driveStatus.B.ServoAllowed 		= 1;
		driveStatus.B.DCMotorsAllowed 	= 0;
		driveStatus.B.CameraAllowed 	= 1;
		driveStatus.B.Calib 			= 1;

		appState = APP_CALIB;
	}
}

/*******************************************************************************
*
* Function: 	void AppCalib(void)
*
* Description:  Application CALIB state function
*
*******************************************************************************/
void AppCalib(void)
{
	// Delay to start, after switching left button
    if(StartTimer_2 > 40000)
    {
		AppStopToAlignment();
    }
}

/*******************************************************************************
*
* Function: 	void AppStopToAlignment(void)
*
* Description:  Application STOP to ALIGN state transition function
*
*******************************************************************************/
void AppStopToAlignment(void)
{
    appState = APP_ALIGNMENT;
}

/*******************************************************************************
*
* Function: 	void AppAlignment(void)
*
* Description:  Application ALIGN state function
*
*******************************************************************************/
void AppAlignment(void)
{
    AppAlignmentToStart();
}

/*******************************************************************************
*
* Function: 	void AppAlignmentToStart(void)
*
* Description:  Application ALIGN to START state transition function
*
*******************************************************************************/
void AppAlignmentToStart(void)
{
    appState = APP_START;
}

/*******************************************************************************
*
* Function: 	void AppStart(void)
*
* Description:  Application START state function
*
*******************************************************************************/
void AppStart(void)
{
	AppStartToRun();
}

/*******************************************************************************
*
* Function: 	void AppStartToRun(void)
*
* Description:  Application START to RUN state transition function
*
*******************************************************************************/
void AppStartToRun(void)
{
	PINS_DRV_ClearPins(PTA, 1 << 2); // enable U1
	PINS_DRV_ClearPins(PTA, 1 << 3); // enable U2

    appState = APP_RUN;
}

/*******************************************************************************
*
* Function: 	void AppRun(void)
*
* Description:  Application RUN state function
*
*******************************************************************************/
void AppRun(void)
{
    if(appSwitchState == 0)
    {
    	// Disable actuator
		ACTUATE_DisableOutput(HW_INPUT_TRIG1);

		// Disable DCmotors
		driveStatus.B.DCMotorsAllowed = 0;

		appState = APP_INIT;
	}
    else
    {
    	// enable DCmotors
    	driveStatus.B.DCMotorsAllowed = 1;
    }
}

/*******************************************************************************
*
* Function: 	void AppFault(void)
*
* Description: 	Application FAULT state function
*
*******************************************************************************/
void AppFault(void)
{
	// stop in this AppState, when the faults are not cleared
    if(faultSwitchClear == 1)
    {
        driveStatus.B.Fault 	= 0;
        faultStatus.R 			= 0;
        faultStatusLatched.R 	= 0;

        faultSwitchClear = 0;
        appState = APP_INIT;
    }
}

/*******************************************************************************
*
* Function: 	void CheckFaults(void)
*
* Description:  Application fault detection function.
*
*******************************************************************************/
void CheckFaults(void)
{

}

/*******************************************************************************
*
* Function: 	void CheckSwitchState(void)
*
* Description:  User switch state detection function
* 					Check Buttons On Processor Board
*
*******************************************************************************/
void CheckSwitchState(void)
{
	// checking left button on yellow board
	// pressing the button throws the start-up status
	if((PINS_DRV_ReadPins(PTC) >> 13 & 1) == 1)
		driveStatus.B.SwStart = 1;
	else
		driveStatus.B.SwStart = 0;

	// checking right button on yellow board
	// pressing this button will reset the variable and start a new Threshold Buffer entry
	if((PINS_DRV_ReadPins(PTC) >> 12 & 1) == 1)
	{
		driveStatus.B.SwStart = 0;
		appSwitchState 		  = 0;
		appState 			  = 0;
	}
}

/***************************************************************************//*!
*
* Function:		void RGBLedOFF()
*
* Description:	This function turns RGB LED off
*
******************************************************************************/
void RGBLedOFF()
{
	PINS_DRV_SetPins(PTD, 1<<0);		// RGB Blue  Led OFF
	PINS_DRV_SetPins(PTD, 1<<15);		// RGB Red 	 Led OFF
	PINS_DRV_SetPins(PTD, 1<<16);		// RGB Green Led OFF
}

/***************************************************************************//*!
*
* Function:		void RGBLedRedON()
*
* Description:	This function turns RGB Red LED on
*
******************************************************************************/
void RGBLedRedON()
{
	PINS_DRV_SetPins(PTD, 1<<0);		// RGB Blue  Led OFF
	PINS_DRV_ClearPins(PTD, 1<<15);		// RGB Red 	 Led ON
	PINS_DRV_SetPins(PTD, 1<<16);		// RGB Green Led OFF
}

/***************************************************************************//*!
*
* Function:		void RGBLedGreenON()
*
* Description:	This function turns RGB Green LED on
*
******************************************************************************/
void RGBLedGreenON()
{
	PINS_DRV_ClearPins(PTD, 1<<16);		// RGB Green Led ON
	PINS_DRV_SetPins(PTD,   1<<0);		// RGB Blue  Led OFF
	PINS_DRV_SetPins(PTD,   1<<15);		// RGB Red 	 Led OFF
}

/***************************************************************************//*!
*
* Function:		void RGBLedWhiteFlash()
*
* Description:	This function flashes RGB White LED
*
******************************************************************************/
void RGBLedWhiteFlash()
{
	whiteLedCounter += 1;

	/* RGB White Led FLASHING */
	if(whiteLedCounter > WHITE_LED_FLASH_FREQ)
	{
		PINS_DRV_TogglePins(PTD, 1<<16);	// RGB Blue Led Toggle
		PINS_DRV_TogglePins(PTD, 1<<0); 	// RGB Blue Led Toggle
		PINS_DRV_TogglePins(PTD, 1<<15); 	// RGB Red  Led Toggle
		whiteLedCounter = 0;
	}
}

/***************************************************************************//*!
*
* Function:		void RGBLedWhiteON()
*
* Description:	This function turns RGB White LED
*
******************************************************************************/
void RGBLedWhiteON()
{
	PINS_DRV_ClearPins(PTD, 1<<16);		// RGB Green Led ON
	PINS_DRV_ClearPins(PTD, 1<<0);		// RGB Blue Led ON
	PINS_DRV_ClearPins(PTD, 1<<15);		// RGB Red Led ON
}



/***************************************************************************//*!
*
* Function:		void CaptureReading()
*
* Description:	This function performs reading ThresholdBuffer[], ActualCaptureBuffer[]
* 						& compare both values to setting BOOL functions (reading lines)
*
* PINS used for camera:
*				CAMERA_ADC   	- 	PTE6 /ADC1_SE11
*				CAMERA_CLOCK 	- 	PTE16
*				CAMERA_SI 		-	PTE2
*
******************************************************************************/
void   	CaptureReading()
{
	// calculate modulo (residue after division)
	ReadingCapture.ModuloPosition 	= ReadingCapture.BufferPosition%2;

	// start reading capture must by with set SI pin
	if(ReadingCapture.BufferPosition == 2)
	{
		// setting SI pin
		PINS_DRV_SetPins(PTE, 1<<2);
	}
	else if(ReadingCapture.BufferPosition == 4)
	{
		// cleaning SI pin
		PINS_DRV_ClearPins(PTE, 1<<2);
	}

	// only on leading edge (CAMERA_CLOCK - PTE16)
	if(ReadingCapture.ModuloPosition == 0)
	{
		// cutting of camera capture (because the camera capture is rounded and with noise) - getting pixels from limit <15;115>
		if((ReadingCapture.BufferPosition/2 >= 13) && (ReadingCapture.BufferPosition/2 <= 115))
		{
			// getting value from ADC (ADC1_11)
			ADC_DRV_GetChanResult(1, 0, &ReadingCapture.CamPixel_ADC);
			// waiting for conversion finish
			ADC_DRV_WaitConvDone(INST_ADCONV1);
			// masking ADC value & retyping to float
			ReadingCapture.CamPixel = (tFloat)(ReadingCapture.CamPixel_ADC & 0x0FFF);


//	 PROCESS to get the ThresholdCapture, ActualCameraCapture and BOOL values
//	 ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//	 1. STATE - getting Threshold (repeatedly catching, because I want the bigger values (due to noise from LightBulb) - CaptureDone <2;10>)
//				- I can get image from camera, or set Threshold to constant (less accurately) and insert it to ThresholdBuffer[128]
//	 2. STATE - getting Actual Capture (repeatedly catching)
//					1. STEP - insert actual pixel to ActualBuffer[128]
//					2. STEP - calculate average pixel
//					3. STEP - shift Threshold by value ThresholdShift
//					4. STEP - comparison ThresholdBuffer_Shift with ActualCapture and setting BOOL values
//					5. STEP - if BufferPosition is bigger like 256 (both CLOCK edges), entering to function CaptureProcessing(); and repeat 2.STATE again

			// 1. STATE - getting Threshold
			if(ReadingCapture.CaptureDone <= 2)
			{
				// inserting values to the ThresholdBuffer[]
				// 			NOTE !!!!!! - set only once option !!!!!!!!
				// set constant ThresholdBuffer[]
				ReadingCapture.ThresholdBuffer[ReadingCapture.BufferPosition/2] = 400.0F;
//				// set ThresholdBuffer[] considering to the camera capture
//				ReadingCapture.ThresholdBuffer[ReadingCapture.BufferPosition/2] = ReadingCapture.CamPixel;

				// getting maximal pixel (if necessary)
				if((tFloat)ReadingCapture.CamPixel > ReadingCapture.MaxPixel_Init)
					ReadingCapture.MaxPixel_Init = (tFloat)ReadingCapture.CamPixel;

				// getting average pixel (if necessary)
				ReadingCapture.CamPixel_Average = ReadingCapture.CamPixel_Average + ReadingCapture.CamPixel;

				// setting middle of line on absolute middle - only protect
				ProcessingCapture.MiddleOfLine = 64U;

			}

			// 2. STATE - getting Actual Capture
			else
			{
				// saturation in this state
				ReadingCapture.CaptureDone = 10;

				// inserting values to the ActualCaptureBuffer[]
				// 			NOTE !!!!!! - set only once option !!!!!!!!
//				// calculate with Ratio - preferably when the lighting is changing
//				ReadingCapture.ActualCapture[ReadingCapture.BufferPosition/2] = ReadingCapture.CamPixel/ReadingCapture.Ratio;
				// calculate without Ratio
				ReadingCapture.ActualCapture[ReadingCapture.BufferPosition/2] = ReadingCapture.CamPixel;

				// getting maximal pixel (if necessary)
				if((tFloat)ReadingCapture.CamPixel > ReadingCapture.MaxPixel)
					ReadingCapture.MaxPixel = (tFloat)ReadingCapture.CamPixel;

				// getting average pixel (if necessary)
				ReadingCapture.CamPixel_Average = ReadingCapture.CamPixel_Average + ReadingCapture.CamPixel;

				// shift Threshold by value ThresholdShift
				ReadingCapture.ThresholdBuffer_Shift[ReadingCapture.BufferPosition/2] = ReadingCapture.ThresholdBuffer[ReadingCapture.BufferPosition/2] - ThresholdShift;		// init pole

				// values for visualization in FMSTR
				ReadingCapture.ThresholdPixel_1 		= ReadingCapture.ThresholdBuffer_Shift[ReadingCapture.BufferPosition/2];
				ReadingCapture.ThresholdPixel_Shift 	= ReadingCapture.ThresholdBuffer_Shift[ReadingCapture.BufferPosition/2];

				// comparison ThresholdBuffer_Shift with ActualCapture and setting BOOL values
				if(ReadingCapture.ActualCapture[ReadingCapture.BufferPosition/2] < ReadingCapture.ThresholdBuffer_Shift[ReadingCapture.BufferPosition/2])			    									// ak je snimana hodnota vacsia ako prahova hodnota Treshold,
					ReadingCapture.BoolPosition = 1;	// black line
				else
					ReadingCapture.BoolPosition = 0;	// white plant

				// writing to BoolCaptureBuffer[], that is the main buffer for CaptureProcessing() function
				ReadingCapture.BoolCapture[ReadingCapture.BufferPosition/2] = ReadingCapture.BoolPosition;
			}
		}
		// cropped values (0;15) & (115;128) - setting to 0
		else
		{
			ReadingCapture.CamPixel = 0.0F;
			ReadingCapture.BoolPosition = 0;
			ReadingCapture.BoolCapture[ReadingCapture.BufferPosition/2] = ReadingCapture.BoolPosition;
		}

		// after overflow the buffers (when is picture fully capture)
		if(ReadingCapture.BufferPosition > 259)
		{
			// for 1. STATE
			if(ReadingCapture.CaptureDone == 1)
			{
				// calculate average - INIT
				ReadingCapture.AveragePixel_Init = ReadingCapture.CamPixel_Average/128.0F;
				ReadingCapture.CamPixel_Average = 0.0F;
			}
			// for 2. STATE
			else if(ReadingCapture.CaptureDone > 1)
			{
				// function to calculate track lines, and servo rotation
				CaptureProcessing();
				// calculate average - ACTUAL
				ReadingCapture.AveragePixel_Actual = ReadingCapture.CamPixel_Average/128.0F;

//				// setting MultRatio (if necessary)
//				if(ReadingCapture.AveragePixel_Actual < 500.0F)
//					ReadingCapture.MultRatio = 0.6F;
//				else if((ReadingCapture.AveragePixel_Actual >= 500.0F) && (ReadingCapture.AveragePixel_Actual < 800.0F))
//					ReadingCapture.MultRatio = 0.9F;
//				else if((ReadingCapture.AveragePixel_Actual >=800.0F) && (ReadingCapture.AveragePixel_Actual < 1100.0F))
//					ReadingCapture.MultRatio = 1.1F;
//				else if((ReadingCapture.AveragePixel_Actual >=1100.0F) && (ReadingCapture.AveragePixel_Actual < 2000.0F))
//					ReadingCapture.MultRatio = 1.2F;
//				else if(ReadingCapture.AveragePixel_Actual >= 2000.0F)
//					ReadingCapture.MultRatio = 1.4F;

				// calculate Ratio considering on average, initial pixel & MultRatio
				ReadingCapture.Ratio = (ReadingCapture.AveragePixel_Actual/ReadingCapture.AveragePixel_Init)*ReadingCapture.MultRatio;

				// resetting auxiliary variables
				ReadingCapture.CamPixel_Average = 0.0F;
				ReadingCapture.MaxPixel = 0.0F;
			}

			// resetting auxiliary variables
			ReadingCapture.MaxPixel = 0.0F;
			ReadingCapture.BufferPosition = 0;

			// increment fully captures done
			ReadingCapture.CaptureDone++;
		}
	}

	// increment BufferPosition (half pixel) + toggle CLOCK PIN (both edges)
	ReadingCapture.BufferPosition++;
	PINS_DRV_TogglePins(PTE, 1<<16); // clock pin
}


/***************************************************************************//*!
*
* Function:		int16_t CaptureProcessing()
*
* Description:	This function consists of several control options (TrackCalculation_Option).
* 				Function based on the BOOL values from CaptureReading():
* 					- calculates the middle of the track
*					- calculates the ServoRotation
*
******************************************************************************/
int16_t CaptureProcessing()
{
	// TrackCalculation_Option == 5
	// This option calculate with lines detect flags. When line ate detecting, I immediately move to the function for individually line
	if(TrackCalculation_Option == 5)
	{
		// if I only see the LEFT line
		if((ProcessingCapture.LeftLine_Detect == 1U) && (ProcessingCapture.RightLine_Detect == 0U))
		{
			// pass through the entire buffer
			for(ProcessingCapture.CamBool=0; ProcessingCapture.CamBool<128; ProcessingCapture.CamBool++)
			{
				// capturing the LEFT line thickness and position of LEFT line end based on the BOOL position
				if(ReadingCapture.BoolCapture[ProcessingCapture.CamBool] == 1)
				{
					ProcessingCapture.LeftLine_End = ProcessingCapture.CamBool;
					ProcessingCapture.LeftLine_Thick++;
				}
			}

			// ignoring the noise + calculate clean LEFT line position
			// if LEFT line thick is too small, clear LEFT line flag (LeftLine_Detect = 0U)
			if(ProcessingCapture.LeftLine_Thick >= 3)
			{
				ProcessingCapture.LeftLine_Position = ProcessingCapture.LeftLine_End - (ProcessingCapture.LeftLine_Thick/2);
				ProcessingCapture.LeftLine_Detect = 1U;
				ProcessingCapture.MiddleOfLine = ProcessingCapture.LeftLine_Position + 64;
			}
			else
			{
				ProcessingCapture.LeftLine_Position = 0;
				ProcessingCapture.LeftLine_Detect = 0U;
				ProcessingCapture.MiddleOfLine = 64;
			}
		}

		// if I only see the RIGHT line
		else if((ProcessingCapture.LeftLine_Detect == 0U) && (ProcessingCapture.RightLine_Detect == 1U))
		{
			// pass through the entire buffer
			for(ProcessingCapture.CamBool=128; ProcessingCapture.CamBool>0; ProcessingCapture.CamBool--)
			{
				// capturing the RIGHT line thickness and position of RIGHT line end based on the BOOL position
				if(ReadingCapture.BoolCapture[ProcessingCapture.CamBool] == 1)
				{
					ProcessingCapture.RightLine_End = ProcessingCapture.CamBool;
					ProcessingCapture.RightLine_Thick++;
				}
			}

			// ignoring the noise + calculate clean RIGHT line position
			// if RIGHT line thick is too small, clear RIGHT line flag (RightLine_Detect = 0U)
			if(ProcessingCapture.RightLine_Thick >= 3)
			{
				ProcessingCapture.RightLine_Position = ProcessingCapture.RightLine_End - (ProcessingCapture.RightLine_Thick/2);
				ProcessingCapture.RightLine_Detect = 1U;
				ProcessingCapture.MiddleOfLine = ProcessingCapture.RightLine_Position - 64;
			}
			else
			{
				ProcessingCapture.RightLine_Position = 128;
				ProcessingCapture.RightLine_Detect = 0U;
				ProcessingCapture.MiddleOfLine = 64;
			}
		}

		// if I did't see the LEFT line or the RIGHT line
		else //if((ProcessingCapture.LeftLine_Detect == 0U) && (ProcessingCapture.RightLine_Detect == 0U))
		{
			// pass through the LEFT HLAF buffer
			for(ProcessingCapture.CamBool=64; ProcessingCapture.CamBool>0; ProcessingCapture.CamBool--)
			{
				// capturing the LEFT line thickness and position of LEFT line end based on the BOOL position
				if(ReadingCapture.BoolCapture[ProcessingCapture.CamBool] == 1)
				{
					ProcessingCapture.LeftLine_End = ProcessingCapture.CamBool;
					ProcessingCapture.LeftLine_Thick++;
				}
			}

			// ignoring the noise + calculate clean LEFT line position
			// if LEFT line thick is too small, clear LEFT line flag (LeftLine_Detect = 0U)
			if(ProcessingCapture.LeftLine_Thick >= 3)
			{
				ProcessingCapture.LeftLine_Position = ProcessingCapture.LeftLine_End  + (ProcessingCapture.LeftLine_Thick/2);
				ProcessingCapture.LeftLine_Detect = 1U;
			}
			else
			{
				ProcessingCapture.LeftLine_Position = 0;
				ProcessingCapture.LeftLine_Detect = 0U;
			}

			// pass through the RIGHT HLAF buffer
			for(ProcessingCapture.CamBool=64; ProcessingCapture.CamBool<128; ProcessingCapture.CamBool++)
			{
				// capturing the RIGHT line thickness and position of RIGHT line end based on the BOOL position
				if(ReadingCapture.BoolCapture[ProcessingCapture.CamBool] == 1)
				{
					ProcessingCapture.RightLine_End = ProcessingCapture.CamBool;
					ProcessingCapture.RightLine_Thick++;
				}
			}

			// ignoring the noise + calculate clean RIGHT line position
			// if RIGHT line thick is too small, clear RIGHT line flag (RightLine_Detect = 0U)
			if(ProcessingCapture.RightLine_Thick >= 3)
			{
				ProcessingCapture.RightLine_Position = ProcessingCapture.RightLine_End - (ProcessingCapture.RightLine_Thick/2);
				ProcessingCapture.RightLine_Detect = 1U;
			}
			else
			{
				ProcessingCapture.RightLine_Position = 128;
				ProcessingCapture.RightLine_Detect = 0U;
			}

			// setting middle of line at absolute middle (64. pixel)
			ProcessingCapture.MiddleOfLine = 64;
		}

//	*NOTE !!!
//	this is the place to include PID regulator Rotation, based on middle of line
//	this is the place to use electric differential (it must be dependent on the position of the middle of line, not the sideways lines!!!)

		// resetting auxiliary variables
		ProcessingCapture.LeftLine_End 		= 0;
		ProcessingCapture.LeftLine_Thick 	= 0;
		ProcessingCapture.RightLine_End 	= 0;
		ProcessingCapture.RightLine_Thick	= 0;
		ProcessingCapture.CamBool			= 0;
	}

	// TrackCalculation_Option != 5
	// this option calculate with lines detect flags too, but positions of lines must by nearly middle (at track limit <50;78>)
	else
	{
		// if I only see the LEFT line
		if((ProcessingCapture.LeftLine_Detect == 1U) && (ProcessingCapture.RightLine_Detect == 0U))
		{
			// pass through the entire buffer
			for(ProcessingCapture.CamBool=0; ProcessingCapture.CamBool<128; ProcessingCapture.CamBool++)
			{
				// capturing the LEFT line thickness and position of LEFT line end based on the BOOL position
				if(ReadingCapture.BoolCapture[ProcessingCapture.CamBool] == 1)
				{
					ProcessingCapture.LeftLine_End = ProcessingCapture.CamBool;
					ProcessingCapture.LeftLine_Thick++;
				}
			}

			// ignoring the noise + calculate clean LEFT line position
			if(ProcessingCapture.LeftLine_Thick >= 1)
			{
				ProcessingCapture.LeftLine_Position = ProcessingCapture.LeftLine_End - (ProcessingCapture.LeftLine_Thick/2);
				ProcessingCapture.MiddleOfLine = ProcessingCapture.LeftLine_Position + 64;
			}
			else
			{
				ProcessingCapture.LeftLine_Position = 0;
				ProcessingCapture.MiddleOfLine = 64;
			}

			// if LEFT line position is too small, clear LEFT line flag (LeftLine_Detect = 0U)
			if(ProcessingCapture.LeftLine_Position < 50)
				ProcessingCapture.LeftLine_Detect = 0U;
			else
				ProcessingCapture.LeftLine_Detect = 1U;
		}
		// if I only see the RIGHT line
		else if((ProcessingCapture.LeftLine_Detect == 0U) && (ProcessingCapture.RightLine_Detect == 1U))
		{
			// pass through the entire buffer
			for(ProcessingCapture.CamBool=128; ProcessingCapture.CamBool>0; ProcessingCapture.CamBool--)
			{
				// capturing the RIGHT line thickness and position of RIGHT line end based on the BOOL position
				if(ReadingCapture.BoolCapture[ProcessingCapture.CamBool] == 1)
				{
					ProcessingCapture.RightLine_End = ProcessingCapture.CamBool;
					ProcessingCapture.RightLine_Thick++;
				}
			}

			// ignoring the noise + calculate clean RIGHT line position
			if(ProcessingCapture.RightLine_Thick >= 1)
			{
				ProcessingCapture.RightLine_Position = ProcessingCapture.RightLine_End - (ProcessingCapture.RightLine_Thick/2);
				ProcessingCapture.MiddleOfLine = ProcessingCapture.RightLine_Position - 64;
			}
			else
			{
				ProcessingCapture.RightLine_Position = 128;
				ProcessingCapture.MiddleOfLine = 64;
			}

			// if RIGHT line position is too big, clear RIGHT line flag (RightLine_Detect = 0U)
			if(ProcessingCapture.RightLine_Position > 78)
				ProcessingCapture.RightLine_Detect = 0U;
			else
				ProcessingCapture.RightLine_Detect = 1U;
		}

		// if I did't see the LEFT line or the RIGHT line
		else //if((ProcessingCapture.LeftLine_Detect == 0U) && (ProcessingCapture.RightLine_Detect == 0U))
		{
			// pass through the LEFT HLAF buffer
			for(ProcessingCapture.CamBool=64; ProcessingCapture.CamBool>0; ProcessingCapture.CamBool--)
			{
				// capturing the LEFT line thickness and position of LEFT line end based on the BOOL position
				if(ReadingCapture.BoolCapture[ProcessingCapture.CamBool] == 1)
				{
					ProcessingCapture.LeftLine_End = ProcessingCapture.CamBool;
					ProcessingCapture.LeftLine_Thick++;
				}
			}

			// ignoring the noise + calculate clean LEFT line position
			if(ProcessingCapture.LeftLine_Thick >= 2)
				ProcessingCapture.LeftLine_Position = ProcessingCapture.LeftLine_End  + (ProcessingCapture.LeftLine_Thick/2);
			else
				ProcessingCapture.LeftLine_Position = 0;

			// if LEFT line position is enough big, set LEFT line flag (LeftLine_Detect = 1)
			if(ProcessingCapture.LeftLine_Position > 50)
				ProcessingCapture.LeftLine_Detect = 1;
			else
				ProcessingCapture.LeftLine_Detect = 0;


			// pass through the RIGHT HLAF buffer
			for(ProcessingCapture.CamBool=64; ProcessingCapture.CamBool<128; ProcessingCapture.CamBool++)
			{
				// capturing the RIGHT line thickness and position of RIGHT line end based on the BOOL position
				if(ReadingCapture.BoolCapture[ProcessingCapture.CamBool] == 1)
				{
					ProcessingCapture.RightLine_End = ProcessingCapture.CamBool;
					ProcessingCapture.RightLine_Thick++;
				}
			}

			// ignoring the noise + calculate clean RIGHT line position
			if(ProcessingCapture.RightLine_Thick >= 2)
				ProcessingCapture.RightLine_Position = ProcessingCapture.RightLine_End - (ProcessingCapture.RightLine_Thick/2);
			else
				ProcessingCapture.RightLine_Position = 128;

			// if RIGHT line position is enough small, set RIGHT line flag (RightLine_Detect = 1)
			if(ProcessingCapture.RightLine_Position < 78)
				ProcessingCapture.RightLine_Detect = 1;
			else
				ProcessingCapture.RightLine_Detect = 0;

			// setting middle of line at middle calculated by average both lines
			ProcessingCapture.MiddleOfLine = MLIB_Div(MLIB_Add(ProcessingCapture.RightLine_Position,ProcessingCapture.LeftLine_Position),2);
		}

//	*NOTE !!!
//	this is the place to include PID regulator Rotation, based on middle of line
//	this is the place to use electric differential (it must be dependent on the position of the middle of line, not the sideways lines!!!)

		// resetting auxiliary variables
		ProcessingCapture.LeftLine_End 		= 0;
		ProcessingCapture.LeftLine_Thick 	= 0;
		ProcessingCapture.RightLine_End 	= 0;
		ProcessingCapture.RightLine_Thick	= 0;
		ProcessingCapture.CamBool			= 0;
	}


	// filtering middle of track, and calculate difference from the absolute middle
	ProcessingCapture.TrackDifference_Filt = ((uint16_t)GDFLIB_FilterMA((tFloat)ProcessingCapture.MiddleOfLine,&Track_filt)- (uint16_t)ProcessingCapture.MiddleOfTrack);

	// scaling of track difference to servo limits
	ControlCarDirection.ServoDiff 	= ((ServoLimit_Right + ServoLimit_Left)/2) + ((tFloat)ProcessingCapture.TrackDifference_Filt)*((ServoLimit_Right - ServoLimit_Left)/128.0F);    // hodnota_stredu_drahy *(max_prava-max_lava)/kroky;

	// calculating servo rotation, with constant (this constant is adjustable due to servo error)
	ServoRotation = ControlCarDirection.ServoDiff -0.5F;


	// resetting auxiliary variables
	ProcessingCapture.LeftLine_End 			= 0;
	ProcessingCapture.LeftLine_Thick 	    = 0;
	ProcessingCapture.RightLine_End 		= 0;
	ProcessingCapture.RightLine_Thick		= 0;
	ProcessingCapture.CamBool		 		= 0;

	return ProcessingCapture.LineOffset;
}



/***************************************************************************//*!
*
* Function:		void DelayForStart()
*
* Description:	This function makes the start delay after pressing the left button on processor board
*
******************************************************************************/
void DelayForStart()
{
	// TimeDelay after toggle START switch
	if(driveStatus.B.SwStart == 1)
	{
		// after start is StartTimer_1 = 250
		if(StartTimer_1 == 0)
		{
			StartTimer_2 = 1;
			appSwitchState = 1;
		}
		else
			StartTimer_1--;
	}
	else
		StartTimer_1 = 250;

	// TimeDelay to start after StartTimer_2 == 1
	if((StartTimer_2 > 0) && (StartTimer_2 < 100000))
		StartTimer_2 ++;
	else if(StartTimer_2 == 100000)
	{
		StartTimer_2 = 0;
		driveStatus.B.SwStart = 0;
	}
}
