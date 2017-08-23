/**************************************************************
* FILE:         PCB_util.h 
* DESCRIPTION:  definitions for PCB Initialization & Support Functions
* AUTHOR:       Mitja Nemec
*
**************************************************************/
#ifndef   PCB_UTIL_H
#define   PCB_UTIL_H

#include    "DSP28x_Project.h"
#include    "define.h"

/**************************************************************
* turn the LED on control stick on
***************************************************************/
extern void PCB_LEDstick_on(void);
/**************************************************************
* turn the LED on control stick off
***************************************************************/
extern void PCB_LEDstick_off(void);
/**************************************************************
* toggle the LED on control stick
**************************************************************/
extern void PCB_LEDstick_toggle(void);
/**************************************************************
* turn the load off
**************************************************************/
extern void PCB_load_off(void);

/**************************************************************
* turn the load on
**************************************************************/
extern void PCB_load_on(void);

/**************************************************************
* initialize specific GPIO functions
**************************************************************/
extern void PCB_init(void);


#endif  // end of PCB_UTIL_H definition

