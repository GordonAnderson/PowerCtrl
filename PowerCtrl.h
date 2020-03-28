#ifndef PowerCtrl_h
#define PowerCtrl_h
#include "Hardware.h"

#define FILTER   0.1

#define SIGNATURE  0xAA55A5A5

#define ESC   27
#define ENQ   5

// TWI commands and constants
#define TWI_SERIAL          0x27      // This command enables the TWI port to process serial commands

#define TWI_READ_AVALIBLE   0x82      // Returns the number of bytes avalible in output buffer, 16 bit unsigned int

// This must match the structure in the MIPS host application
typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the board name, "FAIMSFB"
  int8_t        Rev;                    // Holds the board revision number
  int           Sdelay;                 // Startup delay in mS
  int           Ptime;                  // Putton press time
  bool          WasOn;                  // This flag is set if the system was on when power was removed
  ADCchan       V12;
  ADCchan       V24;
  //
  int           Signature;              // Must be 0xAA55A5A5 for valid data
} PowerCtrl;


void Read12V(void);
void Read24V(void);
void TurnOff(void);
void TurnOn(void);

#endif
