/*******************************************************************************
*
* Copyright 2006-2015 Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
****************************************************************************//*!
*
* @file     peripherals_config.c
*
* @date     March-28-2017
*
* @brief    MCU Peripherals Configuration
*
*******************************************************************************/
/*******************************************************************************
* Includes
*******************************************************************************/
#include "peripherals_config.h"
#include "ftm_hw_access.h"
ftm_state_t statePwm3;
ftm_state_t stateIc1;

/*******************************************************************************
*
* Function: 	void McuClockConfig(void)
*
* Description:  This function installs the pre-defined clock configuration table
* 				to the clock manager. For details see clockMan1 configuration
* 				in Processor Expert.
*
*******************************************************************************/
void McuClockConfig(void)
{
	/* Clock configuration for MCU and MCU's peripherals */
	CLOCK_SYS_Init(g_clockManConfigsArr,
	               CLOCK_MANAGER_CONFIG_CNT,
	               g_clockManCallbacksArr,
	               CLOCK_MANAGER_CALLBACK_CNT);

	/* Clock configuration update */
	CLOCK_SYS_UpdateConfiguration(0, CLOCK_MANAGER_POLICY_FORCIBLE);
}

/*******************************************************************************
*
* Function: 	void McuPowerConfig(void)
*
* Description:  This function configures the Power manager for operation.
* 				For details see pwrMan1 configuration in Processor Expert.
*
*******************************************************************************/
void McuPowerConfig(void)
{
	/* Power mode configuration for RUN mode */
	POWER_SYS_Init(&powerConfigsArr, 0, &powerStaticCallbacksConfigsArr,0);
	/* Power mode configuration update */
	POWER_SYS_SetMode(0,POWER_MANAGER_POLICY_AGREEMENT);
}

/*******************************************************************************
*
* Function: 	void McuIntConfig(void)
*
* Description:  It enables an interrupt for a given IRQ number.
*
*******************************************************************************/
void McuIntConfig(void)
{
	INT_SYS_EnableIRQ(FTM3_Ovf_Reload_IRQn);			// Enable FTM3 timer overflow interrupt
	INT_SYS_EnableIRQ(PDB0_IRQn);						// Enable PDB0 interrupt
    INT_SYS_EnableIRQ(PDB1_IRQn);						// Enable PDB1 interrupt
	INT_SYS_EnableIRQ(ADC0_IRQn);						// Enable ADC0 interrupt
	INT_SYS_EnableIRQ(ADC1_IRQn);						// Enable ADC1 interrupt
	INT_SYS_EnableIRQ(LPIT0_Ch0_IRQn);					// Enable LPIT0 Ch0 interrupt
	INT_SYS_EnableIRQ(LPIT0_Ch1_IRQn);					// Enable LPIT0 Ch1 interrupt
	INT_SYS_EnableIRQ(LPIT0_Ch2_IRQn);					// Enable LPIT0 Ch2 interrupt
	INT_SYS_EnableIRQ(PORTD_IRQn);						// Enable PORTD interrupt
}

/*******************************************************************************
*
* Function: 	void McuTrigmuxConfig(void)
*
* Description:  This function configures the user defined target modules
* 				with the corresponding source triggers. For more details see
* 				configuration in Processor Expert.
*
*******************************************************************************/
void McuTrigmuxConfig(void)
{
	/* TRGMUX module initialization */
	TRGMUX_DRV_Init(INST_TRGMUX1, &trgmux1_InitConfig0);
}

/*******************************************************************************
*
* Function: 	void McuPinsConfig(void)
*
* Description:  This function configures MCU pins with the options provided
* 				in the provided structure. For more details see configuration
* 				in Processor Expert.
*
*******************************************************************************/
void McuPinsConfig(void)
{
	/* MCU Pins configuration */
	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

	/* Enable interrupt when rising edge is detected on PTD14
	 * to detect motor drivers faults */
	PINS_DRV_SetPinIntSel(PORTD, 14u, PORT_INT_RISING_EDGE);
}

/*******************************************************************************
*
* Function: 	void McuLpuartConfig(void)
*
* Description:  This function configures LPUART module. LPUART is used
* 				as a communication interface between MCU and FreeMASTER.
* 				For more details see configuration in Processor Expert.
*
* Note:			LPUART1 is used for USB and LPUART2 is used for Bluetooth
*
*******************************************************************************/
void McuLpuartConfig(void)
{
	/* LPUART module initialization */
	LPUART_DRV_Init(INST_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);
	LPUART_DRV_Init(INST_LPUART2, &lpuart2_State, &lpuart2_InitConfig0);

}

/*******************************************************************************
*
* Function: 	void McuLpitConfig(void)
*
* Description:  This function configures LPIT module.
* 				For more details see configuration in Processor Expert.
*
* Note: 		Speed control of the BLDC motor is executed in LPIT interrupt
* 				routine every 1ms.
*
*******************************************************************************/
void McuLpitConfig(void)
{
	/* LPIT module initialization */
	LPIT_DRV_Init(INST_LPIT0, &lpit0_InitConfig);

	/* LPIT channel0 initialization */
	LPIT_DRV_InitChannel(INST_LPIT0, 0, &lpit0_ChnConfig0);

	/* LPIT channel1 initialization */
	LPIT_DRV_InitChannel(INST_LPIT0, 1, &lpit0_ChnConfig1);

	/* LPIT channel2 initialization */
	LPIT_DRV_InitChannel(INST_LPIT0, 2, &lpit0_ChnConfig2);

	/* Start LPIT counter */
	LPIT_DRV_StartTimerChannels(INST_LPIT0, 0b101);



}

/*******************************************************************************
*
* Function: 	void McuFtmConfig(void)
*
* Description:  This function configures the FTM0 module to work as a simple
* 				up-counting timer to control 6-step commutation timing and
* 				FTM3 module to work as a complementary 6-channel center-aligned
* 				PWM generator. For more details see settings in Processor Expert.
*
* Note:         FTM0 module generates initial trigger to update
* 				PWM settings of the FTM3 module associated to new commutation sector.
*
* 				FTM3 even channels have inverted polarity due to the inverted
* 				high-side logic of the MC34GD3000.
* 				FTM3_CH6 generates triggers for PDBs in the middle of the PWM cycle.
*
*******************************************************************************/
void McuFtmConfig(void)
{
	/* FTM3 module initialized as PWM signals generator */
	FTM_DRV_Init(INST_FLEXTIMER_PWM3, &flexTimer_pwm3_InitConfig, &statePwm3);

	/* FTM3 module PWM initialization */
	FTM_DRV_InitPwm(INST_FLEXTIMER_PWM3, &flexTimer_pwm3_PwmConfig);

	/* Mask all FTM3 channels to disable PWM output */
	FTM_DRV_MaskOutputChannels(INST_FLEXTIMER_PWM3, 0x3F, false);

}

/*******************************************************************************
*
* Function: 	void McuPdbConfig(void)
*
* Description:  This function configures PDB0 and PDB1 module.
* 				For more details see configuration in Processor Expert.
*
* Note:			PDB0 CH0 pre-trigger0 delay set to sense BEMF voltage towards
* 				the end of the active PWM pulse.
* 				It is initially set as for minimal duty cycle 10%:
* 				0.9 x Half PWM period x 0.1 = 126
*				It is adapted according to the actual duty cycle, in runtime.
*
*				PDB1 CH0 pre-trigger1 delay set to sense DC bus voltage towards
* 				the end of the active PWM pulse.
* 				It is initially set as for minimal duty cycle 10%:
* 				0.9 x Half PWM period x 0.1 = 126
*				It is adapted according to the actual duty cycle, in runtime.
*
*******************************************************************************/
void McuPdbConfig(void)
{
	/* PDB0 module initialization */
	PDB_DRV_Init(INST_PDB0, &pdb0_InitConfig0);
	/* PDB1 module initialization */
	PDB_DRV_Init(INST_PDB1, &pdb1_InitConfig0);

	/* PDB0 CH0 pre-trigger0 initialization */
	PDB_DRV_ConfigAdcPreTrigger(INST_PDB0, 0, &pdb0_AdcTrigInitConfig0);
	/* PDB1 CH0 pre-trigger0 initialization */
	PDB_DRV_ConfigAdcPreTrigger(INST_PDB1,0, &pdb1_AdcTrigInitConfig0);
	/* PDB1 CH0 pre-trigger1 initialization */
    PDB_DRV_ConfigAdcPreTrigger(INST_PDB1,0, &pdb1_AdcTrigInitConfig1);

	/* PDB0 modulus value set to half of the PWM cycle */
	PDB_DRV_SetTimerModulusValue(INST_PDB0, HALF_PWM_MODULO);
    /* Set PDB1 modulus value set to half of the PWM cycle */
	PDB_DRV_SetTimerModulusValue(INST_PDB1, HALF_PWM_MODULO);

	/* PDB0 CH0 pre-trigger0 delay set to sense BEMF voltage towards the end of the active PWM pulse */
	/* Initially set as for minimal duty cycle 10%  -> 0.9 x Half PWM period x 0.1 = 180 */
	PDB_DRV_SetAdcPreTriggerDelayValue(INST_PDB0, 0, 0, PDB_DELAY_MIN);
	/* PDB1 CH0 pre-trigger0 delay set to sense DC bus current in the middle of the PWM cycle */
	PDB_DRV_SetAdcPreTriggerDelayValue(INST_PDB1, 0, 0, 0);
	/* PDB1 CH0 pre-trigger1 delay set to sense DC bus voltage towards the end of the active PWM pulse */
	/* Initially set as for minimal duty cycle 10%  -> 0.9 x Half PWM period x 0.1 = 126 */
	PDB_DRV_SetAdcPreTriggerDelayValue(INST_PDB1, 0, 1, PDB_DELAY_MIN);

	// Enable PDB0 prior to PDB0 load
	PDB_DRV_Enable(INST_PDB0);
	// Enable PDB1 prior to PDB1 load
	PDB_DRV_Enable(INST_PDB1);

	/* Load PDB0 configuration */
	PDB_DRV_LoadValuesCmd(INST_PDB0);
	/* Load PDB1 configuration */
	PDB_DRV_LoadValuesCmd(INST_PDB1);
}

/*******************************************************************************
*
* Function: 	void McuAdcConfig(void)
*
* Description:  This function configures ADC0 and ADC1 module.
* 				For mroe details see configuration in Processor Expert.
*
* Note:         ADC0 input channels are selected based on the actual
* 				commutation sector in runtime:
* 				sector 0,3 -> ADC0_SE2 -> Phase C voltage measurement
* 				sector 1,4 -> ADC0_SE5 -> Phase B voltage measurement
* 				sector 2,5 -> ADC0_SE4 -> Phase A voltage measurement
*
* 				ADC1_SE6 -> DC bus current measurement
* 				ADC1_SE7 -> DC bus voltage measurement
*
*******************************************************************************/
void McuAdcConfig(void)
{
	/* ADC0 module initialization */
	ADC_DRV_ConfigConverter(INST_ADCONV0, &adConv0_ConvConfig0);

	/* ADC0_SE2 input channel is initially selected for Phase C voltage sensing */
	ADC_DRV_ConfigChan(INST_ADCONV0, 0, &adConv0_ChnConfig0);

	/* ADC1 module initialization */
	ADC_DRV_ConfigConverter(INST_ADCONV1, &adConv1_ConvConfig0);

	/* ADC1_SE6 input channel is used for DC bus current sensing */
	ADC_DRV_ConfigChan(INST_ADCONV1, 0, &adConv1_ChnConfig0);

	/* ADC1_SE7 input channel is used for DC bus voltage sensing */
	ADC_DRV_ConfigChan(INST_ADCONV1, 1, &adConv1_ChnConfig1);
}

/*******************************************************************************
*
* Function: 	void CACHE_Init(void)
*
* Description:  This function enables Cache memory
*
*******************************************************************************/
void McuCacheConfig(void)
{
    // Enable Cache !
    // Flush and enable I cache and write buffer
    LMEM->PCCCR = LMEM_PCCCR_ENCACHE_MASK | LMEM_PCCCR_INVW1_MASK
    		    | LMEM_PCCCR_INVW0_MASK   | LMEM_PCCCR_GO_MASK;
}
