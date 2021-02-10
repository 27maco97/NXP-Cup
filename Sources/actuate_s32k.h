/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     actuate_s32k.h
*
* @date     March-28-2017
*
* @brief    Header file for actuator module
*
*******************************************************************************/
#ifndef _ACTUATE_S32K_H_
#define _ACTUATE_S32K_H_

#include "peripherals_config.h"
#include "mlib.h"

/******************************************************************************
| Defines and macros            (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Typedefs and structures       (scope: module-local)
-----------------------------------------------------------------------------*/
typedef enum
{
	HW_INPUT_TRIG0,			/* FTM3 is triggered automatically by FTM0 init trigger through FTM3 signal input TRIG0 */
	HW_INPUT_TRIG1 			/* FTM3 is triggered manually by FTM3 SYNC bit through FTM3 signal input TRIG1 */
} ftm_hw_trigger_t;

/******************************************************************************
| Exported function prototypes
-----------------------------------------------------------------------------*/
extern tBool 	ACTUATE_EnableOutput(tBool ftmInputTrig);
extern tBool 	ACTUATE_DisableOutput(tBool ftmInputTrig);
extern tBool 	ACTUATE_SetDutycycle(tFloat dutyCycle_1, tFloat dutyCycle_2, tFloat dutyCycle_3, tFloat dutyCycle_4);

/******************************************************************************
| Inline functions
-----------------------------------------------------------------------------*/

#endif /* _ACTUATES_S32K_H_ */
