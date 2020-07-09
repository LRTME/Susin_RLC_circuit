/**************************************************************
* FILE:         PCB_util.c 
* DESCRIPTION:  PCB initialization & Support Functions
* AUTHOR:       Mitja Nemec
*
**************************************************************/
#include "PCB_util.h"

/**************************************************************
* turn the LED on control stick on
**************************************************************/
#pragma CODE_SECTION(PCB_LEDstick_on, "ramfuncs");
void PCB_LEDstick_on(void)
{
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;
}

/**************************************************************
* turn the LED on control stick off
**************************************************************/
#pragma CODE_SECTION(PCB_LEDstick_off, "ramfuncs");
void PCB_LEDstick_off(void)
{
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
}

/**************************************************************
* toggle the LED on control stick
**************************************************************/
#pragma CODE_SECTION(PCB_LEDstick_toggle, "ramfuncs");
void PCB_LEDstick_toggle(void)
{
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
}

/**************************************************************
* turn the load on
**************************************************************/
void PCB_load_on(void)
{
	GpioDataRegs.GPADAT.bit.GPIO1 = 1; // force output high (5 V)
}

/**************************************************************
* turn the load off
**************************************************************/
void PCB_load_off(void)
{
	GpioDataRegs.GPADAT.bit.GPIO1 = 0; // force output low (0 V)
}

/**************************************************************
* initialize specific GPIO functions
**************************************************************/
void PCB_init(void)
{
    EALLOW;

    /* OUTPUTS */
    // LED on controlSTICK-u
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioDataRegs.GPBDAT.bit.GPIO34 = 0;

    // LOAD
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0; // GPIO1 has GPIO function
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1; // GPIO1 is output (set to 1)
    GpioDataRegs.GPADAT.bit.GPIO1 = 0; // force output low (0 V)

    EDIS;

}
