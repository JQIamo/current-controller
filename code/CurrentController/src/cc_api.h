
#ifndef _CC_API_H
#define _CC_API_H

#include <EEPROM.h>
#include "LCD.h"

#ifndef GITVERSION
#define GITVERSION "unknown"
#endif

// these are rough, but good enough for now.
// eventually implement a calibration?
#define TRANSFER_SLOPE 426.311
#define TRANSFER_OFFSET 153.734

#define VREG_ON		4  // vreg enable pin

#define DAC_CS 		14 // chip select pin
#define DAC_MOSI 	15
#define DAC_SCK 	16

#define LCD_RS 		7
#define LCD_RST 	8
#define LCD_CS 		6
#define LCD_EN 		5

#define ENC_A 		10
#define ENC_B 		9
#define ENC_SW 		2
#define SW				3

#define EEPROM_INIT 0xBE

// remember! DACMIN sets the maximum current
#define DACVAL_OFF 65535  // DAC value for output -> 0mA




// Step sizes for current adjustment:
// Can toggle through these with the encoder pushbutton.

// If you add more step sizes, make sure to adjust ENCODER_TOTAL_STEPS!
// You also always get the "LSB" for free.
// ENCODER_TOTAL_STEPS should count the LSB size, too.
#define ENCODER_TOTAL_STEPS 5

// the first element of currentStepValues will actually be the LSB of the DAC; set elsewhere.
float currentStepValues[ENCODER_TOTAL_STEPS] = {1.0, 0.010, 0.025, 0.100, 1.000};

char * stepValues[ENCODER_TOTAL_STEPS] = {"LSB", "10 uA", "25 uA", "100 uA", "1 mA"};

int stepArray[ENCODER_TOTAL_STEPS]; // this will be initialized in the setupCalibration function,
                                    // to make sure it uses the right transfer function.

int stepIndex = 1;  // keeps track of which dacStepSize resolution you're at.
                    // defaults to smallest step value above LSB.

// dac step size, used to increment in ISR
int dacStepSize = 1;


uint16_t dacMinVal;
uint16_t dacVal;
int dacValAddr = 0;
int dacMinAddr = 0;

extern volatile long encPos;

extern LCD_ST7032 lcd;

// calculates delta DAC word for given current step
uint16_t uAtoDacVal(float current){
  // add 0.5 to round float, rather than truncate.
  return (uint16_t)(current*TRANSFER_SLOPE + 0.5);
}

// Calculates dacVal -> current in mA, for display
float toCurrent(unsigned int dacWord){
        //transferSlope = 427.024;
   //   transferOffset = 153.468;
  return (TRANSFER_OFFSET - dacWord/TRANSFER_SLOPE);
}

// prints stepsize to line 1 of LCD
void writeStepLCD(){
  lcd.setCursor(0, 0);
  lcd.print("r ");
  lcd.print(stepValues[stepIndex]);
  lcd.print("           ");
}

// prints current to line 2 of LCD
void writeCurrentLCD(float val){
  lcd.setCursor(0, 1);
  lcd.print(val, 3);
	lcd.print(" mA        ");
}

// bit bang DAC voltage
void writeDAC(unsigned int val){

 digitalWrite(DAC_CS, LOW);

	for(int i = 0; i < 16; i++){
		digitalWrite(DAC_SCK, LOW);
		digitalWrite(DAC_MOSI, !!(val & (1 << (15 - i))));
		digitalWrite(DAC_SCK, HIGH);
	}

 digitalWrite(DAC_CS, HIGH);

 dacVal = val;

}

void initFromEEPROM(){
  int addr = 0;
  uint8_t eepromInit;

  EEPROM.get(addr, eepromInit);

  if (eepromInit == EEPROM_INIT){
    // load registers from eeprom

    addr += sizeof(eepromInit);
    EEPROM.get(addr, dacMinVal);
    dacMinAddr = addr;

    addr += sizeof(dacMinVal);
    EEPROM.get(addr, encPos);

    dacValAddr = addr;  // for persisting encoder position/DAC val across power cycles
    dacVal = DACVAL_OFF; // start with off; just pull in last current to encoder

  } else {

    lcd.setCursor(0, 0);
    lcd.print("Init EEPROM");
    delay(2000);

    eepromInit = EEPROM_INIT;
    // set conservative max current @ quarter scale
    dacMinVal = 16384;

    // set to "off"
    dacVal = DACVAL_OFF;
    encPos = dacVal;

    addr = 0;
    EEPROM.put(addr, eepromInit); // set byte to show eeprom initialized

    addr += sizeof(eepromInit);
    EEPROM.put(addr, dacMinVal);
    dacMinAddr = addr;


    addr += sizeof(dacMinVal);
    EEPROM.put(addr, dacVal);
    dacValAddr = addr;
    
  }
}

#endif
