#ifndef ADC_h
#define ADC_h
#include <Arduino.h>

#define ADCsync while(ADC->STATUS.bit.SYNCBUSY == 1);

extern int   LowerLimit;
extern int   UpperLimit;
extern int   RepeatCount;
extern int   LastADCval;

int ADCchangeDet(void);
void ADCattachInterrupt(void (*isr)(bool));

#endif
