#include <EEPROM.h>

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

void initFromEEPROM(){
  uint8_t eepromInit;
  EEPROM.get(0, eepromInit);
  if (eepromInit == EEPROM_INIT){
    // load registers from eeprom

    uint16_t eeprom_byte;

    EEPROM.get(DACMIN_ADDR, eeprom_byte);
    dacMinVal = eeprom_byte;
    // EEPROM.get(DACMIN_ADDR + 1, eeprom_byte);
    // dacMinVal |= (eeprom_byte << 8);

    // max val
    EEPROM.get(DACMAX_ADDR, eeprom_byte);
    dacMaxVal = eeprom_byte;
    // EEPROM.get(DACMAX_ADDR + 1, eeprom_byte);
    // dacMaxVal |= (eeprom_byte << 8);

    EEPROM.get(DACVAL_ADDR, eeprom_byte);
    dacVal = eeprom_byte;
    encPos = dacVal;

  } else {
    dacMinVal = 0;
    dacMaxVal = 65535;
    dacVal = 65535;
    encPos = dacVal;

    EEPROM.put(DACMIN_ADDR, dacMinVal);
    EEPROM.put(DACMAX_ADDR, dacMaxVal);
    EEPROM.put(DACVAL_ADDR, dacVal);
    EEPROM.put(0, EEPROM_INIT); // set byte to show eeprom initialized
  }
}
