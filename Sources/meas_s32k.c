/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     meas_s32k.c
*
* @date     March-28-2017
*
* @brief    Header file for measurement module
*
*******************************************************************************/
/******************************************************************************
| Includes
-----------------------------------------------------------------------------*/
#include "meas_s32k.h"

/******************************************************************************
| External declarations
-----------------------------------------------------------------------------*/

/******************************************************************************
| Defines and macros            (scope: module-local)
-----------------------------------------------------------------------------*/
#define U_DCB_MAX                       (21.45F)
#define U_BAT_MAX						(21.45F)

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

/*******************************************************************************
*
* Function: 	tBool MEAS_GetBATVoltage(tFloat *BATVoltage)
*
* Description:  This function performs DC bus voltage measurement.
* 				Conversion complete interrupt is enabled.
*
* Param[in,out]: *DCBVoltage - pointer to a variable - DC bus voltage
*
*
* @return     	# true  - when measurement ended successfully
            	# false - when measurement is ongoing, or error occurred.
*
*******************************************************************************/
tBool MEAS_GetBATVoltage(tFloat *BATVoltage)
{
	uint16_t 			adcResult;

	ADC_DRV_GetChanResult(1, 0, &adcResult);
	*BATVoltage = MLIB_Mul(((tFloat)MLIB_Div((tFloat)(adcResult & 0x00000FFF), (tFloat)0x00000FFF)), U_BAT_MAX);

	return 1;
}

/*******************************************************************************
*
* Function: 	tBool MEAS_GetDCBVoltage(tFloat *DCBVoltage)
*
* Description:  This function performs DC bus voltage measurement.
* 				Conversion complete interrupt is enabled.
*
* Param[in,out]: *DCBVoltage - pointer to a variable - DC bus voltage
*
*
* @return     	# true  - when measurement ended successfully
            	# false - when measurement is ongoing, or error occurred.
*
*******************************************************************************/
tBool MEAS_GetDCBVoltage(tFloat *DCBVoltage)
{
	uint16_t 			adcResult;

	ADC_DRV_GetChanResult(1, 1, &adcResult);
	*DCBVoltage = MLIB_Mul(((tFloat)MLIB_Div((tFloat)(adcResult & 0x00000FFF), (tFloat)0x00000FFF)), U_DCB_MAX);

	return 1;
}

///*******************************************************************************
//*
//* Function: 	tBool MEAS_GetMOTCurrent_L(tFloat *getMOTCurrent)
//*
//* Description:  This function performs DC bus current measurement.
//* 				Conversion complete interrupt is disabled.
//*
//* Param[in,out]: *getDCBCurrent - pointer to a variable - DC bus current
//*
//*
//* @return     	# true  - when measurement ended successfully
//            	# false - when measurement is ongoing, or error occurred.
//*
//*******************************************************************************/
//tBool MEAS_GetMOT_LCurrent(tFloat *getMOT_LCurrent)
//{
//	uint16_t 			adcResult;
//
//	ADC_DRV_GetChanResult(0, 1, &adcResult);
//
//	*getMOT_LCurrent = MLIB_Mul(((tFloat)MLIB_Div((tFloat)(adcResult & 0x00000FFF), (tFloat)0x00000FFF)), I_MAX);
//
//	return 1;
//}

/*******************************************************************************
*
* Function: 	tBool MEAS_GetMOTCurrent_R(tFloat *getMOTCurrent)
*
* Description:  This function performs DC bus current measurement.
* 				Conversion complete interrupt is disabled.
*
* Param[in,out]: *getDCBCurrent - pointer to a variable - DC bus current
*
*
* @return     	# true  - when measurement ended successfully
            	# false - when measurement is ongoing, or error occurred.
*
*******************************************************************************/
tBool MEAS_GetValue_PotentiometerOnYelowBoard(tFloat *getMOT_RCurrent)
{
	uint16_t 			adcResult;

	ADC_DRV_GetChanResult(0, 0, &adcResult);

	*getMOT_RCurrent = MLIB_Mul(((tFloat)MLIB_Div((tFloat)(adcResult & 0x00000FFF), (tFloat)0x00000FFF)), 100.0F);

	return 1;
}

/* End of file */
