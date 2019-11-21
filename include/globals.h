/****************************************************************
* FILENAME:     globals.h
* DESCRIPTION:  project wide global variables
* AUTHOR:       Mitja Nemec
*
****************************************************************/
#ifndef     __GLOBALS_H__
#define     __GLOBALS_H__

#include    "DSP28x_Project.h"

#include    "define.h"

// interrupt counter
extern volatile long    	interrupt_cnt;

extern int					interrupt_cnt_min;
extern int					interrupt_cnt_s;

#endif // end of __GLOBALS_H__ definition
