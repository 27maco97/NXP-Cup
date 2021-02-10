/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     motor_structure.h
*
* @date     March-28-2017
*
* @brief    Header file for NXP_CUP
*
*******************************************************************************/
#ifndef MOTOR_STRUCTURE_H_
#define MOTOR_STRUCTURE_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "gdflib.h"
#include "gflib.h"
#include "gmclib.h"
#include "mlib.h"
#include "amclib.h"
#include "freemaster.h"
#include "SWLIBS_Config.h"

/******************************************************************************
| Defines and macros
-----------------------------------------------------------------------------*/
#define ROTATION_DIR_CW         		0
#define ROTATION_DIR_CCW        		1

/* DC Bus Voltage MA filter defined by Lambda */
#define DCB_V_FILTER_MA_LAMBDA    		0.25F
/* BATTERY Voltage MA filter defined by Lambda */
#define BAT_V_FILTER_MA_LAMBDA    		0.25F
/* BATTERY Voltage MA filter defined by Lambda */
#define POT_FILTER_MA_LAMBDA    		0.001F

/* User LED flashing period */
#define LED_FLASH_FREQ      			80000
/* User LED flashing period */
#define WHITE_LED_FLASH_FREQ      		80000


/******************************************************************************
| Typedefs and structures
-----------------------------------------------------------------------------*/
typedef union {
    uint16_t R;
    struct {
    	uint16_t ServoAllowed			:1;
    	uint16_t DCMotorsAllowed		:1;
    	uint16_t CameraAllowed			:1;
    	uint16_t CameraInit				:1;
    	uint16_t SwStart 				:1;
        uint16_t AdcSaved				:1;
        uint16_t CurrentLimiting		:1;
        uint16_t Fault					:1;
        uint16_t Calib					:1;
        uint16_t Reserved				:7;
    }B;
}tDriveStatus;

typedef union {
    uint8_t R;
    struct {
        uint8_t OverMotL_Current		:1;
        uint8_t OverMotR_Current		:1;
        uint8_t OverDCBusVoltage		:1;
        uint8_t UnderDCBusVoltage		:1;
        uint8_t PreDriverError			:1;
        uint8_t Reserved				:3;
    }B;
}tFaultStatus;

typedef struct {
	tFloat DCBVVoltage;
	tFloat BATVVoltage;
	tFloat MOT_LIVoltageRAW;
	tFloat MOT_LIVoltage;
	tFloat PotentiometerOnYellowBoardRAW;
	tFloat PotentiometerOnYellowBoard;
	tFloat MOT_LOffset;
	tFloat MOT_ROffset;
}tADCresults;

typedef struct {
	  tFloat 	u_dc_bus_filt;
	  tFloat 	u_bat_filt;
}tHWmeasuring;

typedef struct {
	  uint16_t 	ActualCapture[128];
	  uint16_t 	ActualCapture_2[128];
	  int16_t 	BoolCapture[128];
	  tFloat 	ThresholdBuffer[128];
	  uint16_t 	ThresholdBuffer_porovnanie[128];
	  tFloat 	ThresholdBuffer_Shift[128];
	  tFloat 	ThresholdBuffer_Shift2[128];
	  int16_t   BufferPosition;
	  int16_t   CamBool;
	  int16_t   ModuloPosition;
	  int16_t   PixelPosition;
	  int16_t   BoolPosition;
	  int16_t	BufferStart;
	  uint16_t	CamPixel_ADC;
	  uint16_t	CamPixel_ADC_old;
	  tFloat	CamPixel;
	  uint16_t 	CaptureDone;
	  uint16_t	ThresholdPixel_Shift;
	  tFloat	ThresholdPixel_1;
	  tFloat 	MaxPixel_Init;
	  tFloat	AveragePixel_Init;
	  tFloat	AveragePixel_Actual;
	  tFloat	Ratio;
	  tFloat	CamPixel_Average;
	  tFloat	MaxPixel;
	  tFloat 	MultRatio;
	  tFloat 	CLKPosition;
}tReadingCapture;

typedef struct {
	  int16_t   CamBool;
	  int16_t   MiddleOfTrack;
	  int16_t 	LeftLine_End;
	  int16_t	LeftLine_Thick;
	  int16_t	LeftLine_Position;
	  int16_t	RightLine_End;
	  int16_t	RightLine_Thick;
	  int16_t	RightLine_Position;
	  int16_t	MiddleOfLine;
	  int16_t	MiddleRight;
	  int16_t	MiddleLeft;
	  int16_t	MiddleOfLine_Old;
	  int16_t	LineOffset;
	  int16_t	LineOffset_Old;
	  int16_t   TrackDifference_Filt;
	  int16_t 	FinishLine;
	  int16_t	FinishLine_MinThick;
	  uint16_t  LeftLine_Detect;
	  uint16_t  RightLine_Detect;
	  uint16_t  LeftLine_DetectOld;
	  uint16_t  RightLine_DetectOld;
	  int16_t  	MiddleOfLineDiff;
	  int16_t  	MiddleOfLineOld;
	  uint16_t  DiffBorder;
	  int16_t   DiffOfLineOld;
	  int16_t   DiffOfLine;
	  tFloat	MiddleOfLineRamp;
}tProcessingCapture;

typedef struct {
	  uint32_t	ServoRotation_Scale;
	  tFloat 	PWM_servo;
	  tFloat 	ServoDiff;
	  uint16_t 	pwm_counter;
	  uint16_t 	lpit_threshold;
}tControlCarDirection;

typedef struct {
	  tFloat  	PWM1;
	  tFloat 	PWM2;
	  tFloat 	PWM_INIT;
	  tFloat 	PWM_scale;
	  tFloat	PWM_SET;
}tControlCarVelocity;


#endif /* MOTOR_STRUCTURE_H_ */
