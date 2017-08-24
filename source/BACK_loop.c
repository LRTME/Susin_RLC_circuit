/****************************************************************
* FILENAME:     BACK_loop.c
* DESCRIPTION:  background code
* AUTHOR:       Mitja Nemec
* START DATE:
*
****************************************************************/

#include "BACK_loop.h"

// module wide global variables


// declaration of external variables


// declaration of function prototypes

/**************************************************************
* Function which executes background loop code
**************************************************************/
#pragma CODE_SECTION(BACK_loop, "ramfuncs");
void BACK_loop(void)
{
    // local variables


    // infinite background loop
    while (1)
    {
    // once a second (when interrupt_cnt roles over) toggled LEDs on PCB
        if (interrupt_cnt == 0)
        {
            PCB_LEDstick_toggle();
            // wait for at least 100 us, in order prevent executing this code twice
            DELAY_US(100L);
        }
    }   // end of while(1)
}       // end of BACK_loop
