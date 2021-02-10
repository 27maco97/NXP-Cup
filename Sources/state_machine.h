/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     state_machine.h
*
* @date     March-28-2017
*
* @brief    Header file for StateMachineFrame "c" project
*
*******************************************************************************/
#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

/******************************************************************************
* Includes
******************************************************************************/
/******************************************************************************
* Defines and macros
******************************************************************************/
#define APP_INIT                0   /* Application states */
#define APP_CALIB               1
#define APP_ALIGNMENT           2
#define APP_START               3
#define APP_RUN                 4
#define APP_STOP                5
#define APP_FAULT               6

/******************************************************************************
* Constants
******************************************************************************/

/*******************************************************************************
* Types
*******************************************************************************/
typedef void (*tPointerFcn)(void);  /* pointer to a function */
typedef void (*tPointerStr)(void);  /* pointer to a structure */

/******************************************************************************
| Exported Variables
-----------------------------------------------------------------------------*/
/* Array with pointers to the state machine functions */
extern const tPointerFcn AppStateMachine[];
/* Array with pointers to the RGB Led state functions */
extern const tPointerFcn AppStateLed[];

/******************************************************************************
| Exported function prototypes
-----------------------------------------------------------------------------*/
extern void AppInit(void);
extern void AppCalib(void);
extern void AppAlignment(void);
extern void AppStart(void);
extern void AppRun(void);
extern void AppStop(void);
extern void AppFault(void);

extern void AppStopToAlignment(void);
extern void AppAlignmentToStart(void);
extern void AppStartToRun(void);

extern void CheckFaults(void);
extern void CheckSwitchState(void);
extern void StallCheck(void);

extern void RGBLedOFF(void);
extern void RGBLedRedON(void);
extern void RGBLedGreenON(void);
extern void RGBLedWhiteFlash(void);
extern void RGBLedWhiteON(void);

#endif /* STATE_MACHINE_H_ */
