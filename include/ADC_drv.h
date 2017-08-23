/**************************************************************
* FILE:         ADC_drv.c
* DESCRIPTION:  A/D driver for piccolo devices
* AUTHOR:       Mitja Nemec
* DATE:         19.1.2012
*
****************************************************************/
#ifndef     __ADC_DRV_H__
#define     __ADC_DRV_H__

#include    "DSP28x_Project.h"

// which PWM module is the trigger source for ADC
#define     ADC_MODUL1      EPwm1Regs

// map the results registers to defines, for easier reconfiguration
#define     ADC_RESULT0     (AdcResult.ADCRESULT0)
#define     VOLTAGE_BEFORE  (AdcResult.ADCRESULT1)
#define     VOLTAGE_AFTER   (AdcResult.ADCRESULT2)
#define     CURRENT         (AdcResult.ADCRESULT3)

/**************************************************************
* initialize ADC
**************************************************************/
extern void ADC_init(void);

/**************************************************************
* waits for the ADC to finish with current sequence
* return: void
**************************************************************/
extern void ADC_wait(void);

#endif /* __ADC_DRV_H__ */
