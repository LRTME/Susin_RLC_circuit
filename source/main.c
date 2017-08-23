/****************************************************************
* FILE:         main.h
* DESCRIPTION:  initialization code
* AUTHOR:       Mitja Nemec
*
****************************************************************/
#include "main.h"

// Global variables

// function declaration

/**************************************************************
* main function only executes initialization code
**************************************************************/
void main(void)
{
	// local variables


	// initialize system clock
    InitSysCtrl();
    
    // default GPIO initialization
    InitGpio();

    // initialize PIE expansion unit
    InitPieCtrl();

    // populate vector table with dummy interrupt functions
    InitPieVectTable();

    // initialize ADC and PWM
    ADC_init();

    PWM_init();

    // initialize specific GPIO functions
    PCB_init();

    // initialize periodic interrupt function
    PER_int_setup();

    // enable interrupts
    EINT;
    ERTM;

    // start timer, which will trigger ADC and an interrupt
    PWM_start();

    // proceed to background loop code
    BACK_loop();
}   // end of main
