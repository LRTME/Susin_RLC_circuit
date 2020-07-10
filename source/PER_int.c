/****************************************************************
* FILENAME:     PER_int.c
* DESCRIPTION:  periodic interrupt code
* AUTHOR:       Mitja Nemec
*
****************************************************************/
#include    "PER_int.h"
#include    "TIC_toc.h"

// variables required for voltage measurement
long    	voltage_raw = 0;
long    	voltage_offset = 0;
float   	voltage_gain = (3.3/4096); // 12-bit ADC and 3.3V supply;
float   	voltage = 0.0;

// variables required for current measurement
long    	current_raw = 0;
long    	current_offset = 2048;
float   	current_gain = (3.3/4096) * (1.0/50.0) * (1.0/R_rlc); // 12-bit ADC and 3.3V supply; INA213 gain = 50; R = 2.2 Ohm
float   	current = 0.0;

// duty cycle
float   	duty = 0.0;

// variables for reference value generation and load toggling
bool		enable_load = FALSE;
float   	load_counter = 0;
float   	load_counter_prd = SAMPLE_FREQ/SIG_FREQ;
float   	load_counter_cmpr = 0.25*SAMPLE_FREQ;	 // 5000
float   	load_counter_load_on = 0.0*SAMPLE_FREQ/SIG_FREQ;  // 0
float   	load_counter_load_off = 0.05*SAMPLE_FREQ/SIG_FREQ; // 10000

float   	load_value = 0;
float   	load_value_high = 2.5;
float   	load_value_low = 0.25;

// variables for offset calibration 
bool    	offset_calibrated = FALSE;
long    	offset_counter = 0;
float   	offset_filter = 0.005;

// CPU load estimation
float   	cpu_load  = 0.0;
long    	interrupt_cycles = 0;

// counter of too long interrupt function executions
int     	interrupt_overflow_counter = 0;

// variables for the control algorithm
enum		{NONE, PI_only, PI_RES, PI_mRES, PI_REP, PI_DCT, PI_dual_DCT} select_controller = NONE;

float   	reference = 0.0;
float		angle_1Hz = 0.0;
float		angle_periodic = 0.0;
float		input_offset = 1.5;
float		input_amplitude = 1.0;
float		input_freq = SIG_FREQ;

// crossover frequency, where phase of F0 reaches -180° and is limited to fs/10 = 2e3
float		crossover_freq = SAMPLE_FREQ/10; // 2e3

int 		clear_REP_buffer_index = 0;
int 		clear_DCT_buffer_index = 0;
int			clear_dual_DCT_buffer_index = 0;

// second order system model for RLC circuit
float				T_2_order;
float				z_2_order;

float				multiple_res_reg_out = 0.0;
PI_float			voltage_PI_reg = PI_FLOAT_DEFAULTS;
RES_REG_float 		voltage_RES_reg = RES_REG_FLOAT_DEFAULTS;
RES_REG_float 		voltage_RES_reg2 = RES_REG_FLOAT_DEFAULTS;
RES_REG_float 		voltage_RES_reg3 = RES_REG_FLOAT_DEFAULTS;
RES_REG_float 		voltage_RES_reg4 = RES_REG_FLOAT_DEFAULTS;
RES_REG_float 		voltage_RES_reg5 = RES_REG_FLOAT_DEFAULTS;
RES_REG_float 		voltage_RES_reg6 = RES_REG_FLOAT_DEFAULTS;
RES_REG_float 		voltage_RES_reg7 = RES_REG_FLOAT_DEFAULTS;
RES_REG_float 		voltage_RES_reg8 = RES_REG_FLOAT_DEFAULTS;
// RES_REG_float 		voltage_RES_reg9 = RES_REG_FLOAT_DEFAULTS;
// RES_REG_float 		voltage_RES_reg10 = RES_REG_FLOAT_DEFAULTS;
REP_REG_float		voltage_REP_reg = REP_REG_FLOAT_DEFAULTS;
DCT_REG_float		voltage_DCT_reg = DCT_REG_FLOAT_DEFAULTS;
dual_DCT_REG_float	voltage_dual_DCT_reg = dual_DCT_REG_FLOAT_DEFAULTS;



/* create (declare) delay buffer array for the use of FPU FIR filter struct library within tok_grid_dct_reg */
// define the delay buffer for the FIR filter with specifed length - needed for DCT controller realization
float dbuffer1[FIR_FILTER_NUMBER_OF_COEFF];
// define the delay buffer for the FIR filter and place it in "firldb" section - needed for DCT controller realization
#pragma DATA_SECTION(dbuffer1, "firldb")
// align the delay buffer for max 1024 words (512 float variables) - needed for DCT controller realization
#pragma DATA_ALIGN (dbuffer1,0x400)

/* create (declare) FIR filter coefficient buffer array for the use of FPU FIR filter struct library within tok_grid_dct_reg */
// define the coeff buffer for the FIR filter with specifed length - needed for DCT controller realization
float coeff1[FIR_FILTER_NUMBER_OF_COEFF];
// define coefficient array and place it in "coefffilter" section - needed for DCT controller realization
#pragma DATA_SECTION(coeff1, "coefffilt");
// align the coefficent buffer for max 1024 words (512 float coeff) - needed for DCT controller realization
#pragma DATA_ALIGN (coeff1,0x400)


// define the delay buffer for the FIR filter with specifed length - needed for DCT controller realization
float dual_DCT_dbuffer1[FIR_FILTER_NUMBER_OF_COEFF];
// define the delay buffer for the FIR filter and place it in "firldb" section - needed for DCT controller realization
#pragma DATA_SECTION(dual_DCT_dbuffer1, "dual_DCT_firldb1")
// align the delay buffer for max 1024 words (512 float variables) - needed for DCT controller realization
#pragma DATA_ALIGN (dual_DCT_dbuffer1,0x400)

// define the coeff buffer for the FIR filter with specifed length - needed for DCT controller realization
float dual_DCT_coeff1[FIR_FILTER_NUMBER_OF_COEFF];
// define coefficient array and place it in "coefffilter" section - needed for DCT controller realization
#pragma DATA_SECTION(dual_DCT_coeff1, "dual_DCT_coefffilt1");
// align the coefficent buffer for max 1024 words (512 float coeff) - needed for DCT controller realization
#pragma DATA_ALIGN (dual_DCT_coeff1,0x400)

// define the delay buffer for the FIR filter with specifed length - needed for DCT controller realization
float dual_DCT_dbuffer2[FIR_FILTER_NUMBER_OF_COEFF];
// define the delay buffer for the FIR filter and place it in "firldb" section - needed for DCT controller realization
#pragma DATA_SECTION(dual_DCT_dbuffer2, "dual_DCT_firldb2")
// align the delay buffer for max 1024 words (512 float variables) - needed for DCT controller realization
#pragma DATA_ALIGN (dual_DCT_dbuffer2,0x400)

// define the coeff buffer for the FIR filter with specifed length - needed for DCT controller realization
float dual_DCT_coeff2[FIR_FILTER_NUMBER_OF_COEFF];
// define coefficient array and place it in "coefffilter" section - needed for DCT controller realization
#pragma DATA_SECTION(dual_DCT_coeff2, "dual_DCT_coefffilt1");
// align the coefficent buffer for max 1024 words (512 float coeff) - needed for DCT controller realization
#pragma DATA_ALIGN (dual_DCT_coeff2,0x400)


// temporary variables
float		temp1 = 0.0;
float		temp2 = 0.0;
float		temp3 = 0.0;

// lokalne funkcije
void	get_electrical(void);
void 	clear_controllers(void);

/**************************************************************
* interrupt function
**************************************************************/
#pragma CODE_SECTION(PER_int, "ramfuncs");
void interrupt PER_int(void)
{
    /// local variables

    // acknowledge interrupt within PWM module
    EPwm1Regs.ETCLR.bit.INT = 1;
    // acknowledge interrupt within PIE module
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

    // start CPU load stopwatch
    interrupt_cycles = TIC_time;
    TIC_start();

    // get previous CPU load estimate
    cpu_load = (float)interrupt_cycles * ((float)SAMPLE_FREQ/CPU_FREQ);

    // increase and wrap around interrupt counter every 1 second
    interrupt_cnt = interrupt_cnt + 1;
    if(interrupt_cnt >= SAMPLE_FREQ)
    {
        interrupt_cnt = 0;
        interrupt_cnt_s = interrupt_cnt_s + 1;
    }
    if(interrupt_cnt_s >= 60)
    {
    	interrupt_cnt_s = 0;
    	interrupt_cnt_min = interrupt_cnt_min + 1;
    }

    // wait for the ADC to finish with conversion
    ADC_wait();

    // get electrical measurements of voltage and current
    get_electrical();
    

    // calculate angle, which is changing its value in region [0,1) with a freq of 1 Hz
    angle_1Hz = angle_1Hz + 1.0 * 1.0/SAMPLE_FREQ;
    if (angle_1Hz > 1.0)
    {
    	angle_1Hz = angle_1Hz - 1.0;
    }
    if (angle_1Hz < 0.0)
    {
    	angle_1Hz = angle_1Hz + 1.0;
    }


    /* USER CODE */

    // calculate angle, which is changing its value in region [0,1)
    angle_periodic = angle_periodic + input_freq * 1.0/SAMPLE_FREQ;
    if (angle_periodic > 1.0)
    {
    	angle_periodic = angle_periodic - 1.0;
    }
    if (angle_periodic < 0.0)
    {
    	angle_periodic = angle_periodic + 1.0;
    }

    // PI controller
    if(select_controller != NONE)
    {
//     	// ref: vlak pulzov
//    	if(angle_periodic > 0.5)
//    	{
//    		voltage_PI_reg.Ref = input_offset + input_amplitude;
//    	}
//    	else
//    	{
//    		voltage_PI_reg.Ref = 0.0;
//    	}

    	// ref: sinus
//    	voltage_PI_reg.Ref = input_offset + input_amplitude*sin(2*PI*input_freq*interrupt_cnt/SAMPLE_FREQ);
    	voltage_PI_reg.Ref = input_offset + input_amplitude*sin(2*PI*angle_periodic);
    	voltage_PI_reg.Fdb = voltage;




    	PI_FLOAT_CALC(voltage_PI_reg);
    }

    // RES controller
    if(select_controller == PI_RES || select_controller == PI_mRES)
    {
    	voltage_RES_reg.Ref = voltage_PI_reg.Ref;
    	voltage_RES_reg2.Ref = voltage_PI_reg.Ref;
    	voltage_RES_reg3.Ref = voltage_PI_reg.Ref;
    	voltage_RES_reg4.Ref = voltage_PI_reg.Ref;
    	voltage_RES_reg5.Ref = voltage_PI_reg.Ref;
    	voltage_RES_reg6.Ref = voltage_PI_reg.Ref;
    	voltage_RES_reg7.Ref = voltage_PI_reg.Ref;
    	voltage_RES_reg8.Ref = voltage_PI_reg.Ref;
//    	voltage_RES_reg9.Ref = voltage_PI_reg.Ref;
//    	voltage_RES_reg10.Ref = voltage_PI_reg.Ref;

    	voltage_RES_reg.Fdb = voltage_PI_reg.Fdb;
    	voltage_RES_reg2.Fdb = voltage_PI_reg.Fdb;
    	voltage_RES_reg3.Fdb = voltage_PI_reg.Fdb;
    	voltage_RES_reg4.Fdb = voltage_PI_reg.Fdb;
    	voltage_RES_reg5.Fdb = voltage_PI_reg.Fdb;
    	voltage_RES_reg6.Fdb = voltage_PI_reg.Fdb;
    	voltage_RES_reg7.Fdb = voltage_PI_reg.Fdb;
    	voltage_RES_reg8.Fdb = voltage_PI_reg.Fdb;
//    	voltage_RES_reg9.Fdb = voltage_PI_reg.Fdb;
//    	voltage_RES_reg10.Fdb = voltage_PI_reg.Fdb;

    	voltage_RES_reg.Angle = angle_periodic; // integral fiksne frekvence f = 50 Hz --> ker gre od 0 do 1
    	voltage_RES_reg2.Angle = angle_periodic; // integral fiksne frekvence f = 50 Hz --> ker gre od 0 do 1
    	voltage_RES_reg3.Angle = angle_periodic; // integral fiksne frekvence f = 50 Hz --> ker gre od 0 do 1
    	voltage_RES_reg4.Angle = angle_periodic; // integral fiksne frekvence f = 50 Hz --> ker gre od 0 do 1
    	voltage_RES_reg5.Angle = angle_periodic; // integral fiksne frekvence f = 50 Hz --> ker gre od 0 do 1
    	voltage_RES_reg6.Angle = angle_periodic; // integral fiksne frekvence f = 50 Hz --> ker gre od 0 do 1
    	voltage_RES_reg7.Angle = angle_periodic; // integral fiksne frekvence f = 50 Hz --> ker gre od 0 do 1
    	voltage_RES_reg8.Angle = angle_periodic; // integral fiksne frekvence f = 50 Hz --> ker gre od 0 do 1
//    	voltage_RES_reg9.Angle = angle_periodic; // integral fiksne frekvence f = 50 Hz --> ker gre od 0 do 1
//    	voltage_RES_reg10.Angle = angle_periodic; // integral fiksne frekvence f = 50 Hz --> ker gre od 0 do 1


    	RES_REG_CALC(voltage_RES_reg);
    	RES_REG_CALC(voltage_RES_reg2);
    	RES_REG_CALC(voltage_RES_reg3);
    	RES_REG_CALC(voltage_RES_reg4);
    	RES_REG_CALC(voltage_RES_reg5);
    	RES_REG_CALC(voltage_RES_reg6);
    	RES_REG_CALC(voltage_RES_reg7);
    	RES_REG_CALC(voltage_RES_reg8);
//    	RES_REG_CALC(voltage_RES_reg9);
//    	RES_REG_CALC(voltage_RES_reg10);


    	multiple_res_reg_out = 	voltage_RES_reg.Out +   		\
								voltage_RES_reg2.Out +			\
								voltage_RES_reg3.Out +			\
								voltage_RES_reg4.Out +			\
								voltage_RES_reg5.Out +			\
								voltage_RES_reg6.Out +			\
								voltage_RES_reg7.Out +			\
								voltage_RES_reg8.Out; // +			\
								voltage_RES_reg9.Out +			\
								voltage_RES_reg10.Out;
    }

    // REP controller
    if(select_controller == PI_REP)
    {
		voltage_REP_reg.Ref = voltage_PI_reg.Ref;
		voltage_REP_reg.Fdb = voltage_PI_reg.Fdb;
		voltage_REP_reg.SamplingSignal = angle_periodic;

    	REP_REG_CALC(&voltage_REP_reg);
    }

    // DCT controller
    if(select_controller == PI_DCT)
    {
		voltage_DCT_reg.Ref = voltage_PI_reg.Ref;
		voltage_DCT_reg.Fdb = voltage_PI_reg.Fdb;
		voltage_DCT_reg.SamplingSignal = angle_periodic;

    	DCT_REG_CALC(&voltage_DCT_reg);
    }

    // dual DCT controller
    if(select_controller == PI_dual_DCT)
    {
		voltage_dual_DCT_reg.Ref = voltage_PI_reg.Ref;
		voltage_dual_DCT_reg.Fdb = voltage_PI_reg.Fdb;
		voltage_dual_DCT_reg.SamplingSignal = angle_periodic;

    	dual_DCT_REG_CALC(&voltage_dual_DCT_reg);
    }


    // select current controller
    switch(select_controller)
    {
    case NONE:
    	clear_controllers();
    	break;
    case PI_only:
    	duty = voltage_PI_reg.Out;
    	break;
    case PI_RES:
        duty = voltage_PI_reg.Out + voltage_RES_reg.Out;
        break;
    case PI_mRES:
        duty = voltage_PI_reg.Out + multiple_res_reg_out;
        break;
    case PI_REP:
        duty = voltage_PI_reg.Out + voltage_REP_reg.Out;
    	break;
    case PI_DCT:
    	duty = voltage_PI_reg.Out + voltage_DCT_reg.Out;
    	break;
    case PI_dual_DCT:
    	duty = voltage_PI_reg.Out + voltage_dual_DCT_reg.Out;
    	break;
    default:
    	duty = 0.0;
    	break;
    }
    if(select_controller == PI_only)
    {
    	duty = voltage_PI_reg.Out;
    }

	// send duty cycle to PWM module
	PWM_update(duty);


	// counter for load value and load toggling
	load_counter = load_counter + 1;
	if (load_counter >= load_counter_prd)
	{
		load_counter = 0;
	}

	// toggle the load
	if(enable_load != FALSE)
	{
		if (   (load_counter > load_counter_load_on)
				&& (load_counter < load_counter_load_off))
		{
			PCB_load_on();
		}
		if (   (load_counter < load_counter_load_on)
				|| (load_counter > load_counter_load_off))
		{
			PCB_load_off();
		}

		// generate load value
		if (load_counter > load_counter_cmpr)
		{
			load_value = load_value_low;
		}
		else
		{
			load_value = load_value_high;
		}

		reference = load_value;
	}

	/* END OF USER CODE */




    // store values for display within CCS or GUI
    DLOG_GEN_update();

    /* Test if new interrupt is already waiting.
     * If so, then something is seriously wrong.
     */
    if (EPwm1Regs.ETFLG.bit.INT == TRUE)
    {
        // count number of interrupt overflow events
        interrupt_overflow_counter = interrupt_overflow_counter + 1;

        /* if interrupt overflow event happened more than 10 times
         * stop the CPU
         *
         * Better solution would be to properly handle this event
         * (shut down the power stage, ...)
         */
        if (interrupt_overflow_counter >= 10)
        {
            asm(" ESTOP0");
        }
    }

    // stop the CPU load stopwatch
    TIC_stop();

}   // end of PWM_int




/**************************************************************
* Function, which calculates voltages and currents and
* calibrates the measuring probes.
**************************************************************/
void get_electrical(void)
{
	// at startup calibrate offset in current measurement
	if (offset_calibrated == FALSE)
	{
		current_raw = CURRENT;
		current_offset = (1.0 - offset_filter) * current_offset + offset_filter * current_raw;

		offset_counter = offset_counter + 1;
		if (offset_counter == SAMPLE_FREQ)
		{
			offset_calibrated = TRUE;
		}
	}
	// otherwise operate normally
	else
	{
		// calculate voltage measured from ADC data
		voltage_raw = VOLTAGE_AFTER;
		voltage = voltage_raw * voltage_gain;

		// calculate current measured from ADC data
		current_raw = CURRENT - current_offset;
		current = -current_raw * current_gain;
	}
}




/**************************************************************
* Function, which clears integral parts and outputs of
* voltage controllers
**************************************************************/
void clear_controllers(void)
{
	// clear all integral parts of controllers
    voltage_PI_reg.Ui = 0.0;

	// clear all outputs of PI controllers
    voltage_PI_reg.Out = 0.0;


	// clear all integral parts of resonant controllers
    voltage_RES_reg.Ui1 = 0.0;
    voltage_RES_reg.Ui2 = 0.0;
    voltage_RES_reg2.Ui1 = 0.0;
    voltage_RES_reg2.Ui2 = 0.0;
    voltage_RES_reg3.Ui1 = 0.0;
    voltage_RES_reg3.Ui2 = 0.0;
    voltage_RES_reg4.Ui1 = 0.0;
    voltage_RES_reg4.Ui2 = 0.0;
    voltage_RES_reg5.Ui1 = 0.0;
    voltage_RES_reg5.Ui2 = 0.0;
    voltage_RES_reg6.Ui1 = 0.0;
    voltage_RES_reg6.Ui2 = 0.0;
    voltage_RES_reg7.Ui1 = 0.0;
    voltage_RES_reg7.Ui2 = 0.0;
    voltage_RES_reg8.Ui1 = 0.0;
    voltage_RES_reg8.Ui2 = 0.0;
//    voltage_RES_reg9.Ui1 = 0.0;
//    voltage_RES_reg9.Ui2 = 0.0;
//    voltage_RES_reg10.Ui1 = 0.0;
//    voltage_RES_reg10.Ui2 = 0.0;

	// clear all outputs of resonant controllers
    voltage_RES_reg.Out = 0.0;
    voltage_RES_reg2.Out = 0.0;
    voltage_RES_reg3.Out = 0.0;
    voltage_RES_reg4.Out = 0.0;
    voltage_RES_reg5.Out = 0.0;
    voltage_RES_reg6.Out = 0.0;
    voltage_RES_reg7.Out = 0.0;
    voltage_RES_reg8.Out = 0.0;
//    voltage_RES_reg9.Out = 0.0;
//    voltage_RES_reg10.Out = 0.0;


	// clear all integral parts of repetitive controller
	// CAUTION: THE FACT IS THAT SOME TIME MUST BE SPEND TO CLEAR THE WHOLE BUFFER (ONE SAMPLE IN EACH ITERATION),
	//          WHICH IS TYPICAL LESS THAN 1 SEC!
    voltage_REP_reg.ErrSumHistory[clear_REP_buffer_index] = 0.0;

    voltage_REP_reg.ErrSum = 0.0;

    voltage_REP_reg.i = 0;
    voltage_REP_reg.i_prev = -1;

	clear_REP_buffer_index = clear_REP_buffer_index + 1;
	if(clear_REP_buffer_index >= voltage_REP_reg.BufferHistoryLength - 1)
	{
		clear_REP_buffer_index = 0;
	}

	// clear all outputs of repetitive controllers
	voltage_REP_reg.Out = 0.0;


	// clear all integral parts of DCT controller
	// CAUTION: THE FACT IS THAT SOME TIME MUST BE SPEND TO CLEAR THE WHOLE BUFFER (ONE SAMPLE IN EACH ITERATION),
	//          WHICH IS TYPICAL LESS THAN 1 SEC!
	voltage_DCT_reg.CorrectionHistory[clear_DCT_buffer_index] = 0.0;

	// CAUTION: THE FACT IS THAT SOME TIME MUST BE SPEND TO CLEAR THE WHOLE BUFFER (ONE SAMPLE IN EACH ITERATION),
	//          WHICH IS TYPICAL LESS THAN 1 SEC!
	dbuffer1[clear_DCT_buffer_index] = 0.0;

	voltage_DCT_reg.ErrSum = 0.0;

	voltage_DCT_reg.i = 0;
	voltage_DCT_reg.i_prev = -1;

	clear_DCT_buffer_index = clear_DCT_buffer_index + 1;
	if(clear_DCT_buffer_index >= voltage_DCT_reg.BufferHistoryLength - 1)
	{
		clear_DCT_buffer_index = 0;
	}

	// clear all outputs of DCT controllers
	voltage_DCT_reg.Out = 0.0;


	// clear all integral parts of dual DCT controller
	// CAUTION: THE FACT IS THAT SOME TIME MUST BE SPEND TO CLEAR THE WHOLE BUFFER (ONE SAMPLE IN EACH ITERATION),
	//          WHICH IS TYPICAL LESS THAN 1 SEC!
	dual_DCT_dbuffer1[clear_dual_DCT_buffer_index] = 0.0;
	dual_DCT_dbuffer2[clear_dual_DCT_buffer_index] = 0.0;


	voltage_dual_DCT_reg.ErrSum = 0.0;

	voltage_dual_DCT_reg.i = 0;
	voltage_dual_DCT_reg.i_prev = -1;

	clear_dual_DCT_buffer_index = clear_dual_DCT_buffer_index + 1;
	if(clear_dual_DCT_buffer_index >= voltage_dual_DCT_reg.BufferHistoryLength - 1)
	{
		clear_dual_DCT_buffer_index = 0;
	}

	// clear all outputs of DCT controllers
	voltage_dual_DCT_reg.Out = 0.0;
}




/**************************************************************
* Function, which initializes all required for execution of
* interrupt function
**************************************************************/
void PER_int_setup(void)
{

    // initialize data logger
    // specify trigger value
    dlog.trig_level = 0.01;
    // trigger on positive slope
    dlog.slope = Positive;                 
    // store every  sample
    dlog.downsample_ratio = 5;
    // Normal trigger mode
    dlog.mode = Normal;                    
    // number of calls to update function
    dlog.auto_time = 100;                  
    // number of calls to update function
    dlog.holdoff_time = 100;               

    // trigger signal
    dlog.trig = &angle_periodic; // &angle_periodic or &voltage

    // data signals
//    dlog.iptr1 = &voltage_PI_reg.Ref;
//    dlog.iptr2 = &voltage_PI_reg.Fdb;
//    dlog.iptr3 = &voltage_PI_reg.Err;
//    dlog.iptr4 = &voltage_PI_reg.Out;
//    dlog.iptr5 = &angle_periodic;
//    dlog.iptr6 = &voltage;
//    dlog.iptr7 = &current;
//    dlog.iptr8 = &current;

    dlog.iptr1 = &voltage; // &voltage or &voltage_PI_reg.Fdb
    dlog.iptr2 = &voltage_PI_reg.Err;
//
//    dlog.downsample_ratio = 5;

    /****************************************
    * Control algorithm initialization
    ****************************************/

    /* RLC circuit as a second order system */
	T_2_order = sqrt(L_rlc*C_rlc);
	z_2_order = R_rlc*C_rlc/(2*T_2_order);

    /* PI controller parameters initialization */
    voltage_PI_reg.Kp = 0.2; // 0.3 ali 0.2
    voltage_PI_reg.Ki = 100.0/SAMPLE_FREQ; // 100.0/SAMPLE_FREQ
    voltage_PI_reg.OutMax = 1.0;
    voltage_PI_reg.OutMin = 0.0;

    /* multiple RESonant controller parameters initialization */
    voltage_RES_reg.Harmonic = 1;
    voltage_RES_reg2.Harmonic = 2;
    voltage_RES_reg3.Harmonic = 3;
    voltage_RES_reg4.Harmonic = 4;
    voltage_RES_reg5.Harmonic = 5;
    voltage_RES_reg6.Harmonic = 6;
    voltage_RES_reg7.Harmonic = 7;
    voltage_RES_reg8.Harmonic = 8;
//    voltage_RES_reg9.Harmonic = 9;
//    voltage_RES_reg10.Harmonic = 10;
//    voltage_RES_reg.Kres = voltage_PI_reg.Ki * 													\
//    						(1.0 - (voltage_RES_reg.Harmonic*input_freq / crossover_freq) * 	\
//    						(voltage_RES_reg.Harmonic*input_freq / crossover_freq));
//    voltage_RES_reg2.Kres = voltage_RES_reg.Kres * 												\
//    						(1.0 - (voltage_RES_reg2.Harmonic*input_freq / crossover_freq) * 	\
//    						(voltage_RES_reg2.Harmonic*input_freq / crossover_freq));
//    voltage_RES_reg3.Kres = voltage_RES_reg.Kres * 												\
//    						(1.0 - (voltage_RES_reg3.Harmonic*input_freq / crossover_freq) * 	\
//    						(voltage_RES_reg3.Harmonic*input_freq / crossover_freq));
//    voltage_RES_reg4.Kres = voltage_RES_reg.Kres * 												\
//    						(1.0 - (voltage_RES_reg4.Harmonic*input_freq / crossover_freq) * 	\
//    						(voltage_RES_reg4.Harmonic*input_freq / crossover_freq));
//    voltage_RES_reg5.Kres = voltage_RES_reg.Kres * 												\
//    						(1.0 - (voltage_RES_reg5.Harmonic*input_freq / crossover_freq) * 	\
//    						(voltage_RES_reg5.Harmonic*input_freq / crossover_freq));
//    voltage_RES_reg6.Kres = voltage_RES_reg.Kres * 												\
//    						(1.0 - (voltage_RES_reg6.Harmonic*input_freq / crossover_freq) * 	\
//    						(voltage_RES_reg6.Harmonic*input_freq / crossover_freq));
//    voltage_RES_reg7.Kres = voltage_RES_reg.Kres * 												\
//    						(1.0 - (voltage_RES_reg7.Harmonic*input_freq / crossover_freq) * 	\
//    						(voltage_RES_reg7.Harmonic*input_freq / crossover_freq));
//    voltage_RES_reg8.Kres = voltage_RES_reg.Kres * 												\
//    						(1.0 - (voltage_RES_reg8.Harmonic*input_freq / crossover_freq) * 	\
//    						(voltage_RES_reg8.Harmonic*input_freq / crossover_freq));
    // Kres, ki delujejo ob ustrezni fazni komp.:
    // harmonik: 1,    2,    3,    4,    5,    6,    7,    8
    // Kres:    [1.0,  0.5,  0.5,  0.5  0.5,   x,    x,    x ] * voltage_PI_reg.Ki
    voltage_RES_reg.Kres =  1.0 * voltage_PI_reg.Ki; // 1.0 * 100.0/SAMPLE_FREQ
    voltage_RES_reg2.Kres = 0.5 * voltage_PI_reg.Ki; // 0.5 * 100.0/SAMPLE_FREQ
    voltage_RES_reg3.Kres = 0.5 * voltage_PI_reg.Ki; // 0.5 * 100.0/SAMPLE_FREQ
    voltage_RES_reg4.Kres = 0.5 * voltage_PI_reg.Ki; // 0.5 * 100.0/SAMPLE_FREQ
    voltage_RES_reg5.Kres = 0.5 * voltage_PI_reg.Ki; // 0.5 * 100.0/SAMPLE_FREQ
    voltage_RES_reg6.Kres = 0.25 * voltage_PI_reg.Ki; // 0.5 * 100.0/SAMPLE_FREQ
    voltage_RES_reg7.Kres = 0.25 * voltage_PI_reg.Ki; // 0.5 * 100.0/SAMPLE_FREQ
    voltage_RES_reg8.Kres = 0.25 * voltage_PI_reg.Ki; // 0.5 * 100.0/SAMPLE_FREQ
    //    voltage_RES_reg9.Kres = 0.0 * voltage_PI_reg.Ki; // 0.0 * 100.0/SAMPLE_FREQ
    //    voltage_RES_reg10.Kres = 0.0 * voltage_PI_reg.Ki; // 0.0 * 100.0/SAMPLE_FREQ
//    voltage_RES_reg.PhaseCompDeg = atan2(2*z_2_order*(2*PI*voltage_RES_reg.Harmonic*input_freq)*T_2_order,  (1 - (2*PI*voltage_RES_reg.Harmonic*input_freq)*T_2_order * (2*PI*voltage_RES_reg.Harmonic*input_freq)*T_2_order)) * 180.0/PI;
    voltage_RES_reg.PhaseCompDeg = 4.0;
	voltage_RES_reg2.PhaseCompDeg = atan2(2*z_2_order*(2*PI*voltage_RES_reg2.Harmonic*input_freq)*T_2_order,(1 - (2*PI*voltage_RES_reg2.Harmonic*input_freq)*T_2_order * (2*PI*voltage_RES_reg2.Harmonic*input_freq)*T_2_order)) * 180.0/PI;
	voltage_RES_reg3.PhaseCompDeg = atan2(2*z_2_order*(2*PI*voltage_RES_reg3.Harmonic*input_freq)*T_2_order,(1 - (2*PI*voltage_RES_reg3.Harmonic*input_freq)*T_2_order * (2*PI*voltage_RES_reg3.Harmonic*input_freq)*T_2_order)) * 180.0/PI;
	voltage_RES_reg4.PhaseCompDeg = atan2(2*z_2_order*(2*PI*voltage_RES_reg4.Harmonic*input_freq)*T_2_order,(1 - (2*PI*voltage_RES_reg4.Harmonic*input_freq)*T_2_order * (2*PI*voltage_RES_reg4.Harmonic*input_freq)*T_2_order)) * 180.0/PI;
	voltage_RES_reg5.PhaseCompDeg = atan2(2*z_2_order*(2*PI*voltage_RES_reg5.Harmonic*input_freq)*T_2_order,(1 - (2*PI*voltage_RES_reg5.Harmonic*input_freq)*T_2_order * (2*PI*voltage_RES_reg5.Harmonic*input_freq)*T_2_order)) * 180.0/PI;
	voltage_RES_reg6.PhaseCompDeg = atan2(2*z_2_order*(2*PI*voltage_RES_reg6.Harmonic*input_freq)*T_2_order,(1 - (2*PI*voltage_RES_reg6.Harmonic*input_freq)*T_2_order * (2*PI*voltage_RES_reg6.Harmonic*input_freq)*T_2_order)) * 180.0/PI;
	voltage_RES_reg7.PhaseCompDeg = atan2(2*z_2_order*(2*PI*voltage_RES_reg7.Harmonic*input_freq)*T_2_order,(1 - (2*PI*voltage_RES_reg7.Harmonic*input_freq)*T_2_order * (2*PI*voltage_RES_reg7.Harmonic*input_freq)*T_2_order)) * 180.0/PI;
	voltage_RES_reg8.PhaseCompDeg = atan2(2*z_2_order*(2*PI*voltage_RES_reg8.Harmonic*input_freq)*T_2_order,(1 - (2*PI*voltage_RES_reg8.Harmonic*input_freq)*T_2_order * (2*PI*voltage_RES_reg8.Harmonic*input_freq)*T_2_order)) * 180.0/PI;
//	voltage_RES_reg9.PhaseCompDeg = atan2(2*z_2_order*(2*PI*voltage_RES_reg9.Harmonic*input_freq)*T_2_order,(1 - (2*PI*voltage_RES_reg9.Harmonic*input_freq)*T_2_order * (2*PI*voltage_RES_reg9.Harmonic*input_freq)*T_2_order)) * 180.0/PI;
// 	voltage_RES_reg10.PhaseCompDeg = atan2(2*z_2_order*(2*PI*voltage_RES_reg10.Harmonic*input_freq)*T_2_order,(1 - (2*PI*voltage_RES_reg10.Harmonic*input_freq)*T_2_order * (2*PI*voltage_RES_reg10.Harmonic*input_freq)*T_2_order)) * 180.0/PI;
    voltage_RES_reg.OutMax = +0.5;	// +0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg2.OutMax = +0.5;	 // +0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg3.OutMax = +0.5;	 // +0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg4.OutMax = +0.5;	 // +0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg5.OutMax = +0.5;	 // +0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg6.OutMax = +0.5;	 // +0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg7.OutMax = +0.5;	 // +0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg8.OutMax = +0.5;	 // +0.5; // zaradi varnosti ne gre do 0.99
//    voltage_RES_reg9.OutMax = +0.5; // +0.5; // zaradi varnosti ne gre do 0.99
//    voltage_RES_reg10.OutMax = +0.5; // +0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg.OutMin = -0.5; // -0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg2.OutMin = -0.5;	 // -0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg3.OutMin = -0.5;	 // -0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg4.OutMin = -0.5;	 // -0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg5.OutMin = -0.5;	 // -0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg6.OutMin = -0.5;	 // -0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg7.OutMin = -0.5;	 // -0.5; // zaradi varnosti ne gre do 0.99
    voltage_RES_reg8.OutMin = -0.5;	 // -0.5; // zaradi varnosti ne gre do 0.99
//    voltage_RES_reg9.OutMin = -0.5; // -0.5; // zaradi varnosti ne gre do 0.99
//    voltage_RES_reg10.OutMin = -0.5; // -0.5; // zaradi varnosti ne gre do 0.99


    /* REPetitive controller parameters initialization */
    REP_REG_INIT_MACRO(voltage_REP_reg);
    voltage_REP_reg.BufferHistoryLength = SAMPLE_POINTS; // 400
    voltage_REP_reg.Krep = 0.4 * 1/2.0 * (voltage_RES_reg.Kres*SAMPLE_FREQ) / (2.0*SIG_FREQ); // 0.2 = 0.4 * 1/2.0 * 100.0/(2.0*SIG_FREQ)
    voltage_REP_reg.k = 10; // 10
    voltage_REP_reg.w0 = 0.2; // 0.2
    voltage_REP_reg.w1 = 0.2; // 0.2
    voltage_REP_reg.w2 = 0.2; // 0.2
    voltage_REP_reg.ErrSumMax = 0.6;
    voltage_REP_reg.ErrSumMin = -0.6;
    voltage_REP_reg.OutMax = 0.5; // 0.5
    voltage_REP_reg.OutMin = -0.5; // -0.5


    /* DCT controller parameters initialization */

    // initialize FPU library FIR filter pointers, which are pointing to the external FIR filter coefficient buffer and delay buffer
    // IMPORTANT: THOSE TWO POINTERS ARE USED TO CHANGE THE BUFFERS VALUES WITHIN STRUCTURE!
    //            INITIALZE THE POINTERS IN THE NEXT TWO LINES BEFORE CALLING ANY INITIZALIZING MACRO OR FUNCTION!
    voltage_DCT_reg.FIR_filter_float.coeff_ptr = coeff1;
    voltage_DCT_reg.FIR_filter_float.dbuffer_ptr = dbuffer1;

    // FPU library FIR filter initialization - necessary for the DCT filter realization
    voltage_DCT_reg.FIR_filter_float.cbindex = 0;
    voltage_DCT_reg.FIR_filter_float.order = FIR_FILTER_NUMBER_OF_COEFF - 1;
    voltage_DCT_reg.FIR_filter_float.input = 0.0;
    voltage_DCT_reg.FIR_filter_float.output = 0.0;
    voltage_DCT_reg.FIR_filter_float.init(&voltage_DCT_reg);

    // initialize current DCT controller
    DCT_REG_INIT_MACRO(voltage_DCT_reg); // initialize all arrays
    voltage_DCT_reg.Kdct = 0.6 * (voltage_RES_reg.Kres*SAMPLE_FREQ) / (4.0*SIG_FREQ); // 0.3 = 0.6 * 100.0/(4.0*SIG_FREQ)
    voltage_DCT_reg.k = 10; // 10
    voltage_DCT_reg.ErrSumMax = 0.6;
    voltage_DCT_reg.ErrSumMin = -0.6;
    voltage_DCT_reg.OutMax = 0.5; // 0.5
    voltage_DCT_reg.OutMin = -0.5; // -0.5
    DCT_REG_FIR_COEFF_INIT_MACRO(voltage_DCT_reg); // set coefficents of the DCT filter


    /* dual DCT controller parameters initialization */

    // initialize FPU library FIR filter pointers, which are pointing to the external FIR filter coefficient buffer and delay buffer
    // IMPORTANT: THOSE TWO POINTERS ARE USED TO CHANGE THE BUFFERS VALUES WITHIN STRUCTURE!
    //            INITIALZE THE POINTERS IN THE NEXT FOUR LINES BEFORE CALLING ANY INITIZALIZING MACRO OR FUNCTION!
    voltage_dual_DCT_reg.FIR_filter_float1.coeff_ptr = dual_DCT_coeff1;
    voltage_dual_DCT_reg.FIR_filter_float1.dbuffer_ptr = dual_DCT_dbuffer1;
    voltage_dual_DCT_reg.FIR_filter_float2.coeff_ptr = dual_DCT_coeff2;
    voltage_dual_DCT_reg.FIR_filter_float2.dbuffer_ptr = dual_DCT_dbuffer2;

    // FPU library FIR filter initialization - necessary for the DCT filter 1 realization
    voltage_dual_DCT_reg.FIR_filter_float1.cbindex = 0;
    voltage_dual_DCT_reg.FIR_filter_float1.order = FIR_FILTER_NUMBER_OF_COEFF2 - 1;
    voltage_dual_DCT_reg.FIR_filter_float1.input = 0.0;
    voltage_dual_DCT_reg.FIR_filter_float1.output = 0.0;
    voltage_dual_DCT_reg.FIR_filter_float1.init(&voltage_dual_DCT_reg);

    // FPU library FIR filter initialization - necessary for the DCT filter 2 realization
    voltage_dual_DCT_reg.FIR_filter_float2.cbindex = 0;
    voltage_dual_DCT_reg.FIR_filter_float2.order = FIR_FILTER_NUMBER_OF_COEFF2 - 1;
    voltage_dual_DCT_reg.FIR_filter_float2.input = 0.0;
    voltage_dual_DCT_reg.FIR_filter_float2.output = 0.0;
    voltage_dual_DCT_reg.FIR_filter_float2.init(&voltage_dual_DCT_reg);

    // initialize current dual DCT controller
    dual_DCT_REG_INIT_MACRO(voltage_dual_DCT_reg); // initialize all variables and coefficients
    voltage_dual_DCT_reg.Kdct = 1.0 * (voltage_RES_reg.Kres*SAMPLE_FREQ) / (4.0*SIG_FREQ); // 0.5 = 1.0 * 100.0/(4.0*SIG_FREQ)
    voltage_dual_DCT_reg.ErrSumMax = 10.0;
    voltage_dual_DCT_reg.ErrSumMin = -10.0;
    voltage_dual_DCT_reg.OutMax = 0.5; // 0.5
    voltage_dual_DCT_reg.OutMin = -0.5; // -0.5
    dual_DCT_REG_FIR_COEFF_INIT_MACRO(voltage_dual_DCT_reg); // set coefficents of the DCT filter


    // clear integral parts and outputs of all controllers
	clear_controllers();

    /****************************************
    * End of control algorithm initialization
    ****************************************/

    // setup interrupt trigger
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;
    EPwm1Regs.ETCLR.bit.INT = 1;
    EPwm1Regs.ETSEL.bit.INTEN = 1;

    // register the interrupt function
    EALLOW;
    PieVectTable.EPWM1_INT = &PER_int;
    EDIS;

    // acknowledge any spurious interrupts
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

    // enable interrupt within PIE
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

    // enable interrupt within CPU
    IER |= M_INT3;

    // enable interrupt in real time mode
    SetDBGIER(M_INT3);
}
