/****************************************************************
* FILENAME:     BACK_loop.h             
* DESCRIPTION:  background code header file
* AUTHOR:       Mitja Nemec
*
****************************************************************/
#ifndef     __BACK_LOOP_H__
#define     __BACK_LOOP_H__

#include    "DSP28x_Project.h"
#include    "IQmathLib.h"
#include    "define.h"
#include    "globals.h"

#include    "PWM_drv.h"
#include 	"PCB_util.h"
    
/**************************************************************
* Function which executes background loop code
**************************************************************/
extern void BACK_loop(void);
#endif // end of __BACK_LOOP_H__
