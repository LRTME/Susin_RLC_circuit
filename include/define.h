/****************************************************************
* FILENAME:     define.h           
* DESCRIPTION:  file with global define macros
* AUTHOR:       Mitja Nemec
*
****************************************************************/
#ifndef     __DEFINE_H__
#define     __DEFINE_H__

// CPU speed [Hz]
#define     CPU_FREQ        80000000L

// switching frequency [Hz]
#define     SWITCH_FREQ     20000L

// ratio between switching and sampling frequency
#define     SAMP_PRESCALE   1

// sampling frequency [Hz]
#define     SAMPLE_FREQ     (SWITCH_FREQ/SAMP_PRESCALE)

// sampling period [s]
#define     SAMPLE_TIME     (1.0/SAMPLE_FREQ)

// periodical signal frequency [Hz]
#define		SIG_FREQ		50.0

// number of samples in one period of periodic signal
#define		SAMPLE_POINTS	(SAMPLE_FREQ/SIG_FREQ)

// math constants
#define     SQRT3           1.7320508075688772935274463415059
#define     SQRT2           1.4142135623730950488016887242097
#define     ZSQRT2          0.70710678118654752440084436210485
#define     PI              3.1415926535897932384626433832795

// RLC circuit resistance [Ohm]
#define		R_rlc			2.2
// RLC circuit inductance [H]
#define		L_rlc			150e-3
// RLC circuit capacitance [F]
#define		C_rlc			3.3e-6

// bool type definition
typedef enum {FALSE = 0, TRUE} bool;

// how peripherals behave on debug event
// 0 stop immediately, 1 stop when finished, 2 run free
#define     DEBUG_STOP      0

#endif // end of __DEFINE_H__ definition
