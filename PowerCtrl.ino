//
// PowerCtrl
//
// Adafruit Trinket M0
//
// This program is desiged to control the power control module for the e-msion ExD system desiged.
// The power control module monitors the system power state and will automatically power the system
// up when the 24 volt system power is applied. The power control module has a mode jumper to control
// the power up operation.
//  Jumper installed
//    If the jumper is installed then the system will remember its state when the power was removed
//    and return to that state when power is reapplied
//  Jumper not installed 
//    If the jumper is not installed then the system will power up every time when the system power 
//    is applied
//
// There are a number of user adjustable parameters to define the power up delay time and the length
// of time the buttons are pressed
//
// Gordon Anderson
// March 21, 2020
//
// Version 1.0, March 21, 2020
//  - First release
//
#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include "SERCOM.h"
#include <Thread.h>
#include <ThreadController.h>
#include <Adafruit_DotStar.h>

#include <Wire.h>
#include <SPI.h>
#include "Hardware.h"
#include "PowerCtrl.h"
#include "Errors.h"
#include "Serial.h"
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>
#include <SerialBuffer.h>

const char   Version[] PROGMEM = "PowerCtrl version 1.0, March 21, 2020";
PowerCtrl    powerctrl;
bool         ReturnAvalible = false;

Adafruit_DotStar strip = Adafruit_DotStar(1, INTERNAL_DS_DATA, INTERNAL_DS_CLK, DOTSTAR_BGR);

SerialBuffer sb;

// Reserve a portion of flash memory to store configuration
// Note: the area of flash memory reserved is lost every time
// the sketch is uploaded on the board.
FlashStorage(flash_powerctrl, PowerCtrl);

// ThreadController that will control all threads
ThreadController control = ThreadController();
//Threads
Thread SystemThread = Thread();

PowerCtrl Rev_1_PowerCtrl = {
                            sizeof(PowerCtrl),"PowerCtrl",1,
                            //
                            5000,
                            2000,
                            false,
                            V12,1907.5,0,
                            V24,1890.5,0,
                            SIGNATURE
                            };

extern void (*mySysTickHook)(void);
void (*mySysTickHook)(void) = msTimerIntercept;
void msTimerIntercept(void)
{

}

void setup() 
{    
  pinMode(PB,OUTPUT);
  digitalWrite(PB,LOW);
  pinMode(KILL,OUTPUT);
  digitalWrite(KILL,LOW);
  pinMode(MODE,INPUT_PULLUP);
  pinMode(LED,OUTPUT);
  strip.begin();
  strip.setPixelColor(0, 0, 0, 0);
  strip.show();
  LoadAltRev();
  // Read the flash config contents and test the signature
  powerctrl = flash_powerctrl.read();
  if(powerctrl.Signature != SIGNATURE) powerctrl = Rev_1_PowerCtrl;
  // Init serial communications
  SerialInit();
  analogReadResolution(12);
  // Configure Threads
  SystemThread.setName("Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(25);
  // Add threads to the controller
  control.add(&SystemThread);
  // Print the signon version message
  serial->println(Version);
// Power control logic
  delay(powerctrl.Sdelay);
  if(digitalRead(MODE) == HIGH)
  {
    // If here we always power up, but make sure its no already on!
    if(ReadADC(&powerctrl.V12) < 10)
    {
       digitalWrite(PB,HIGH);
       delay(powerctrl.Ptime);
       digitalWrite(PB,LOW);
    }  
  }
  else
  {
    if(powerctrl.WasOn)
    {
       // If system was on, turn it back on, but make sure its no already on!
       if(ReadADC(&powerctrl.V12) < 10)
       {
          digitalWrite(PB,HIGH);
          delay(powerctrl.Ptime);
          digitalWrite(PB,LOW);      
       }
    }
  }
}

// This function is called at 40 Hz
void Update(void)
{
  static int i = 40;

  if(i-- == 0) 
  {
    i = 40;
    if(digitalRead(LED) == LOW) digitalWrite(LED,HIGH);
    else digitalWrite(LED,LOW);
    // Here once per second
    if(digitalRead(MODE) == LOW)
    {
       // Make sure the saved state of WasOn matches the current state
       if(ReadADC(&powerctrl.V12) > 10)
       {
          if(!powerctrl.WasOn)
          {
            powerctrl.WasOn = true;
            // Flash LED ten time fast!
            for(int i=0;i<10;i++) {digitalWrite(LED,LOW); delay(100); digitalWrite(LED,HIGH); delay(100);}
            SaveSettings();
          }
       }
       else
       {
          if(powerctrl.WasOn)
          {
            powerctrl.WasOn = false;
            for(int i=0;i<10;i++) {digitalWrite(LED,LOW); delay(100); digitalWrite(LED,HIGH); delay(100);}
            SaveSettings();
          }      
       }
    }
  }
}

// This function process all the serial IO and commands
void ProcessSerial(bool scan = true)
{
  // Put serial received characters in the input ring buffer
  if (Serial.available() > 0)
  {
    PutCh(Serial.read());
  }
  if (!scan) return;
  // If there is a command in the input ring buffer, process it!
  if (RB_Commands(&RB) > 0) while (ProcessCommand() == 0); // Process until flag that there is nothing to do
}

void loop() 
{
  ProcessSerial();
  control.run();
  //strip.setPixelColor(0, 64, 0, 0); strip.show(); delay(1000); //red
  //strip.setPixelColor(0, 0, 64, 0); strip.show(); delay(1000); //green
  //strip.setPixelColor(0, 0, 0, 64); strip.show(); delay(1000); //blue
  //serial->println(analogRead(A3));
}

//
// Host command functions
//

void SaveSettings(void)
{
  powerctrl.Signature = SIGNATURE;
  flash_powerctrl.write(powerctrl);
  SendACK;
}

void RestoreSettings(void)
{
  static PowerCtrl pc;
  
  // Read the flash config contents and test the signature
  pc = flash_powerctrl.read();
  if(pc.Signature == SIGNATURE) powerctrl = pc;
  else
  {
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;
  }
  SendACK;  
}

void Software_Reset(void)
{
  NVIC_SystemReset();  
}

void FormatFLASH(void)
{
  flash_powerctrl.write(Rev_1_PowerCtrl);  
  SendACK;
}

void Read12V(void)
{
  serial->println(ReadADC(&powerctrl.V12));
}

void Read24V(void)
{
  serial->println(ReadADC(&powerctrl.V24));  
}

void TurnOff(void)
{
  // Pulse the kill switch
  digitalWrite(KILL,HIGH);
  delay(powerctrl.Ptime);
  digitalWrite(KILL,LOW);      
}

void TurnOn(void)
{
   // Pulse the push button if system is on  
   if(ReadADC(&powerctrl.V12) > 10)
   {
      digitalWrite(PB,HIGH);
      delay(powerctrl.Ptime);
      digitalWrite(PB,LOW);      
   }
}


void Debug(int i)
{
}
