#include "ADC.h"

int   ADCmode     = 1;
// ADC window modes
// 0 = No window mode
// 1 = ADC value > LowerLimit
// 2 = ADC value < UpperLimit
// 3 = ADC value is within the window defined by UpperLimit and LowerLimit
// 4 = ADC value is outside the window defined by UpperLimit and LowerLimit
int   LowerLimit  = 1000;
int   UpperLimit  = 2000;
int   RepeatCount = 6;
int   LastADCval  = 0;
void  (*ADCchangeFunc)(bool) = NULL;

// ADC change interrupt call back function
void ADCattachInterrupt(void (*isr)(bool))
{
  ADCchangeFunc = isr;
}

// This ADC ISR fires on every ADC conversion. This routine looks at the window flag 
// and will call the attached interrupt when the window condition changes. A repeat count
// filter is applied to filter false signals.
void ADC_Handler(void)
{
   volatile unsigned int static count  = 0;
   volatile unsigned int static countL = 0;
  
   volatile int intflag = ADC->INTFLAG.bit.WINMON;
   ADCsync;
   LastADCval = ADC->RESULT.reg;
   ADCsync;
   if(intflag == 1)
   {
      if(++countL >= RepeatCount)
      {
        // If here the limit has been exceeded for 
        // RepeatCount readings in a row
        count = 0;
        if((countL == RepeatCount) && (ADCchangeFunc != NULL)) ADCchangeFunc(true);
        if(countL > 2*RepeatCount) countL--;
      }    
   }
   else
   {
      if(++count >= RepeatCount)
      {
        // If here the ADC value is within the limit for 
        // RepeatCount readings in a row
        countL = 0;
        if((count == RepeatCount) && (ADCchangeFunc != NULL)) ADCchangeFunc(false);
        if(count > 2*RepeatCount) count--;
      }    
   }
}

// This function enables the ADC change detection system. The variables:
//   ADCmode
//   LowerLimit
//   UpperLimit
// need to be set before calling this function.  
int ADCchangeDet(void)
{
   pinPeripheral(A3, PIO_ANALOG);
   // Select the ADC channel
   ADCsync;
   ADC->CTRLA.bit.ENABLE = 1;
   ADCsync;
   ADC->INPUTCTRL.bit.MUXPOS = 7;
   ADCsync;
   ADC->CTRLB.bit.PRESCALER = 4;
   ADCsync;
   ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
   ADCsync;
   ADC->CTRLB.bit.FREERUN = 1;
   ADCsync;
   ADC->WINCTRL.bit.WINMODE = ADCmode;
   ADCsync;
   ADC->INTENSET.bit.RESRDY = 1;
   ADCsync;
   ADC->WINLT.reg = LowerLimit; 
   ADCsync;
   ADC->WINUT.reg = UpperLimit;
   // enable interrupts
   NVIC_EnableIRQ(ADC_IRQn);
   // Start adc
   ADCsync;
   ADC->SWTRIG.bit.START = 1;
}
