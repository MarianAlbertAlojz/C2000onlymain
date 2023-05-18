#include "DSP28x_Project.h"     // DSP28x Headerfile
#include "sci_io.h"
#include "main.h"

void Init()
{
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
    //
    InitSysCtrl();

    Uint16 *SourceAddr = &RamfuncsLoadStart;
    Uint16* SourceEndAddr = &RamfuncsLoadEnd;
    Uint16* DestAddr = &RamfuncsRunStart;
    while(SourceAddr < SourceEndAddr)
    {
       *DestAddr++ = *SourceAddr++;
    }

    //
    // Step 2. Initalize GPIO:
    // This example function is found in the F2806x_Gpio.c file and
    // illustrates how to set the GPIO to its default state.
    //
     InitGpio();
    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;
    //
    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2806x_PieCtrl.c file.
    //
    InitPieCtrl();
    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;
    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2806x_DefaultIsr.c.
    // This function is found in F2806x_PieVect.c.
    //
    InitPieVectTable();

    EALLOW;
    PieVectTable.TINT0 = &cpu_timer0_isr;
    EDIS;

    InitSci();//SCIA 115200,N,1

    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 90, 100);//100us

    CpuTimer0Regs.TCR.all = 0x4010;    // Use write-only instruction to set TSS bit = 1
    IER |= M_INT1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;


    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM


    InitAdc();// Configure the ADC: Initialize the ADC
    InitEPwm(); // Configure the PWM
    CpuTimer0Regs.TCR.all = 0x4000;// Use write-only instruction to set TSS bit = 1

}
