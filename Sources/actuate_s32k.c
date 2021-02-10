/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     actuate_s32k.c
*
* @date     March-28-2017
*
* @brief    Header file for actuator module
*
*******************************************************************************/
/******************************************************************************
| Includes
-----------------------------------------------------------------------------*/
#include "actuate_s32k.h"

/******************************************************************************
| External declarations
-----------------------------------------------------------------------------*/

/******************************************************************************
| Defines and macros            (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Typedefs and structures       (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Global variable definitions   (scope: module-exported)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Global variable definitions   (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Function prototypes           (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Function implementations      (scope: module-local)
-----------------------------------------------------------------------------*/

/******************************************************************************
| Function implementations      (scope: module-exported)
-----------------------------------------------------------------------------*/

/**************************************************************************//*!
@brief Unmask PWM output and set 0% dytucyle

@param[in,out]  

@return
******************************************************************************/
tBool ACTUATE_EnableOutput(tBool ftmInputTrig)
{
	uint16_t duty_cycle_1, duty_cycle_2, duty_cycle_3, duty_cycle_4;

    // Enable PWMs
	FTM_DRV_MaskOutputChannels(INST_FLEXTIMER_PWM3, 0x0, false);

	// Apply 50% duty cycle
	duty_cycle_1 = 50U;
	duty_cycle_2 = 50U;
	duty_cycle_3 = 50U;
	duty_cycle_4 = 50U;
	
	// Update duty cycle
	ACTUATE_SetDutycycle(duty_cycle_1, duty_cycle_2, duty_cycle_3, duty_cycle_4);

	return 1;
}

/**************************************************************************//*!
@brief Mask PWM output and set 0% dytucyle

@param[in,out]  

@return
******************************************************************************/
tBool ACTUATE_DisableOutput(tBool ftmInputTrig)
{
	uint16_t duty_cycle_1, duty_cycle_2, duty_cycle_3, duty_cycle_4;

    /* Disable PWM */
	FTM_DRV_MaskOutputChannels(INST_FLEXTIMER_PWM3, 0x3F, false);

	// Apply 50% duty cycle
	duty_cycle_1 = 50U;
	duty_cycle_2 = 50U;
	duty_cycle_3 = 50U;
	duty_cycle_4 = 50U;

	// Update duty cycle
	ACTUATE_SetDutycycle(duty_cycle_1, duty_cycle_2, duty_cycle_3, duty_cycle_4);

	return 1;
}

/**************************************************************************//*!
@brief Set PWM dytycyle, the dutycycle will by updated on next reload event

@param[in,out]  

@return
******************************************************************************/
tBool ACTUATE_SetDutycycle(tFloat dutyCycle_1, tFloat dutyCycle_2, tFloat dutyCycle_3, tFloat dutyCycle_4)
{
	tBool 				statePwm 	= true;
	uint16_t   			dutyTicks_1, dutyTicks_2, dutyTicks_3, dutyTicks_4;
	const uint8_t 		channels[4] = {0, 1, 2, 3};

    /* Duty cycle in clock ticks format */
	dutyTicks_1 = (uint16_t)MLIB_Mul(MLIB_Div(dutyCycle_1, 100.0F), HALF_PWM_MODULO);
	dutyTicks_2 = (uint16_t)MLIB_Mul(MLIB_Div(dutyCycle_2, 100.0F), HALF_PWM_MODULO);
	dutyTicks_3 = (uint16_t)MLIB_Mul(MLIB_Div(dutyCycle_3, 100.0F), HALF_PWM_MODULO);
	dutyTicks_4 = (uint16_t)MLIB_Mul(MLIB_Div(dutyCycle_4, 100.0F), HALF_PWM_MODULO);

	/* Set duty cycle for all PWM channels */
	uint16_t pwms[4] = {dutyTicks_1, dutyTicks_2, dutyTicks_3, dutyTicks_4};

    /* Update PWM duty cycle */
	FTM_DRV_FastUpdatePwmChannels(INST_FLEXTIMER_PWM3, 4, channels, pwms, true);

	statePwm = false;

	return(statePwm);
}

/* End of file */
