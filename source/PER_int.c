/****************************************************************
* FILENAME:     PER_int.c
* DESCRIPTION:  periodic interrupt code
* AUTHOR:       Mitja Nemec
*
****************************************************************/
#include    "PER_int.h"
#include    "TIC_toc.h"

// variables required for voltage measurement
long voltage_raw = 0;
long voltage_offset = 0;
float voltage_gain = ??;
float voltage = 0.0;

// variables required for current measurement
long current_raw = 0;
long current_offset = 2048;
float current_gain = ??;
float current = 0.0;

// duty cycle
float duty = 0.0;

// variables for reference value generation and load toggling
float ref_counter = 0;
float ref_counter_prd = SWITCH_FREQ;
float ref_counter_cmpr = 800;
float ref_counter_load_on = 350;
float ref_counter_load_off = 650;

float ref_value = 0;
float ref_value_high = 2.5;
float ref_value_low = 0.25;

// variables for offset calibration 
bool    offset_calib = TRUE;
long     offset_counter = 0;
float   offset_filter = 0.005;

// CPU load estimation
float   cpu_load  = 0.0;
long    interrupt_cycles = 0;

// counter of too long interrupt function executions
int interrupt_overflow_counter = 0;

// variables for the PID controller
float reference = 0.0;



/**************************************************************
* interrupt funcion
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
    cpu_load = (float)interrupt_cycles * ((float)SWITCH_FREQ/CPU_FREQ);

    // increase and wrap around interrupt counter every 1 second
    interrupt_cnt = interrupt_cnt + 1;
    if (interrupt_cnt >= SAMPLE_FREQ)
    {
        interrupt_cnt = 0;
    }

    // wait for the ADC to finish with conversion
    ADC_wait();

    // at startup calibrate offset in current measurement
    if (offset_calib == TRUE)
    {
        current_raw = CURRENT;
        current_offset = (1.0 - offset_filter) * current_offset + offset_filter * current_raw;

        offset_counter = offset_counter + 1;
        if ( offset_counter == SWITCH_FREQ)
        {
            offset_calib = FALSE;
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

        // counter for reference value and load toggling
        ref_counter = ref_counter + 1;
        if (ref_counter >= ref_counter_prd)
        {
            ref_counter = 0;
        }

        // toggle the load
        if (   (ref_counter > ref_counter_load_on)
            && (ref_counter < ref_counter_load_off))
        {
            PCB_load_on();
        }
        if (   (ref_counter < ref_counter_load_on)
            || (ref_counter > ref_counter_load_off))
        {
            PCB_load_off();
        }
        
        // generate reference value
        if (ref_counter > ref_counter_cmpr)
        {
            ref_value = ref_value_low;
        }
        else
        {
            ref_value = ref_value_high;
        }

        reference = ref_value;
        

        // place for PID controller

        
        // send duty cycle to PWM module
        PWM_update(duty);
        
    }
    
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
    	 * (shot down the power stage, ...)
    	 */
    	if (interrupt_overflow_counter >= 10)
    	{
    		asm(" ESTOP0");
    	}
    }

    // stop the sCPU load stopwatch
    TIC_stop();

}   // end of PWM_int

/**************************************************************
* Function which initializes all required for execution of
* interrupt function
**************************************************************/
void PER_int_setup(void)
{

    // initialize data logger
    dlog.trig_level = SWITCH_FREQ - 10;    // specify trigger value
    dlog.slope = Positive;                 // trigger on positive slope
    dlog.downsample_ratio = 1;             // store every  sample
    dlog.mode = Normal;                    // Normal trigger mode
    dlog.auto_time = 100;                  // number of calls to update function
    dlog.holdoff_time = 100;               // number of calls to update function

    dlog.trig = &ref_counter;
    dlog.iptr1 = &voltage;
    dlog.iptr2 = &current;

    // setup interrupt trigger
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    //sproži prekinitev na periodo
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;         //ob vsakem prvem dogodku
    EPwm1Regs.ETCLR.bit.INT = 1;                //clear possible flag
    EPwm1Regs.ETSEL.bit.INTEN = 1;              //enable interrupt

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
