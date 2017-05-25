#include <EEPROM.h>
#include "LCD.h"

// these are rough, but good enough for now.
// eventually implement a calibration?
#define transferSlope 426.311
#define transferOffset 153.734


#define VREG_ON		4    // vreg enable pin

#define DAC_CS 		14 // A0         // chip select pin
#define DAC_MOSI 	15
#define DAC_SCK 	16

#define LCD_RS 		7
#define LCD_RST 	8
#define LCD_CS 		6
#define LCD_EN 		5

#define ENC_A 		10
#define ENC_B 		9
#define ENC_SW 		2
#define SW				3  // frontpanel switch


#define EEPROM_INIT 0xBE


// remember! DACMIN sets the maximum current
// these are EEPROM addresses
#define DACMIN_ADDR 1
#define DACMAX_ADDR 3
#define DACVAL_ADDR 5

uint16_t dacMinVal;
uint16_t dacMaxVal;
uint16_t dacVal;

extern volatile long encPos;

extern LCD_ST7032 lcd;



unsigned int uAtoDacVal(float current){
  unsigned int tmpDacVal;
  tmpDacVal = (current*transferSlope + 0.5); // add 0.5 to round float, rather than truncate.
  return tmpDacVal;
}

// Calculates dacVal -> current in mA, for display
float toCurrent(unsigned int dacWord){
        //transferSlope = 427.024;
   //   transferOffset = 153.468;
  return (transferOffset - dacWord/transferSlope);
}



void writeStepLCD(){
  lcd.setCursor(0, 0);
  lcd.print("r ");
  lcd.print(stepValues[stepIndex]);
  lcd.print("        ");
}

// prints current to LCD
void writeCurrentLCD(float val){
  lcd.setCursor(0,1);
  lcd.print(val, 3);
	lcd.print(" mA        ");
}


// bit bang it
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
  uint8_t eepromInit;// = 0xAA;
   EEPROM.get(0, eepromInit);
//
// lcd.setCursor(0,0);
// lcd.print("Zero addr: ");
// lcd.print(eepromInit);
// EEPROM.write(0, 0x45);
// delay(2000);
//
// lcd.setCursor(0,0);
// eepromInit = EEPROM.read(0);
// lcd.print("Zero addr: ");
// lcd.print(eepromInit);
// delay(2000);


  if (eepromInit == EEPROM_INIT){
    // load registers from eeprom

    //uint16_t eeprom_byte;

    //EEPROM.get(DACMIN_ADDR, eeprom_byte);
    //dacMinVal = eeprom_byte;
    //hardcode in for now
    dacMinVal = 26318;

    // EEPROM.get(DACMIN_ADDR + 1, eeprom_byte);
    // dacMinVal |= (eeprom_byte << 8);

    // max val
    //EEPROM.get(DACMAX_ADDR, eeprom_byte);
    dacMaxVal = 65535;  //eeprom_byte;
    // EEPROM.get(DACMAX_ADDR + 1, eeprom_byte);
    // dacMaxVal |= (eeprom_byte << 8);

    EEPROM.get(DACVAL_ADDR, encPos);
    dacVal = dacMaxVal; // start with off; just pull in last current to encoder
    //dacVal = eeprom_byte;
    //encPos = dacVal;

      // lcd.setCursor(0,0);
      // lcd.print("yesEEPROM");
      // lcd.setCursor(0,1);
      // lcd.print(dacVal);
      // delay(2000);

  } else {
    lcd.setCursor(0,0);
    lcd.print("NoEEPROM");
    delay(2000);
    //dacMinVal = 0;
    dacMinVal = 26318;
    dacMaxVal = 65535;
    dacVal = 65535;
    encPos = dacVal;

    EEPROM.put(DACMIN_ADDR, dacMinVal);
    EEPROM.put(DACMAX_ADDR, dacMaxVal);
    EEPROM.put(DACVAL_ADDR, dacVal);
    EEPROM.write(0, EEPROM_INIT); // set byte to show eeprom initialized
  }
}
