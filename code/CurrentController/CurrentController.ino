// // Current Controller Software v2.0
// // For hardware v2.0
//
//
// #include <SPI.h>
// #include <EEPROM.h>
// //#include <LiquidCrystal.h>
// #include "LCD.h"
//
// #include "src/PCInterrupt.h"
//
//
// // "serial number" for the current controller you're programming.
// // This is mostly for calibration of individual hardware...
// // There is a configureCalibration() function which initializes
// // relevant variables.
//
// //#define CONTROLLER_ID 10
//
// #define transferSlope 426.311
// #define transferOffset 153.734
//
// // #if CONTROLLER_ID == 0
// // 	#define transferSlope 426.562
// // 	#define transferOffset 153.620
// // #elif CONTROLLER_ID == 1
// // 	#define transferSlope 427.024
// // 	#define transferOffset 153.468
// // #elif CONTROLLER_ID == 2
// // 	#define transferSlope 426.311
// // 	#define transferOffset 153.734
// // #elif CONTROLLER_ID == 3
// // 	#define transferSlope 427.674
// // 	#define transferOffset 153.248
// // #elif CONTROLLER_ID == 4
// // 	#define transferSlope 426.559
// // 	#define transferOffset 153.646
// // #elif CONTROLLER_ID == 5
// // 	#define transferSlope 426.098
// // 	#define transferOffset 153.818
// // #elif CONTROLLER_ID == 6
// // 	#define transferSlope 426.963
// // 	#define transferOffset 153.504
// // #elif CONTROLLER_ID == 7
// // 	#define transferSlope 426.846
// // 	#define transferOffset 153.537
// // #elif CONTROLLER_ID == 8
// // 	#define transferSlope 427.594
// // 	#define transferOffset 153.270
// // #elif CONTROLLER_ID == 9
// // 	#define transferSlope 426.311
// // 	#define transferOffset 153.734
// // #elif CONTROLLER_ID == 10
// // 	#define transferSlope 422.904
// // 	#define transferOffset 154.956
// // #else
// // 	// just in case... some sensible values.
// // 	#define transferSlope 426.311
// // 	#define transferOffset 153.734
// // #endif
// //
//
//
//
//
// /***********************************
// CurrentController
// ************************************/
// #define SW_ENABLE 3  // frontpanel switch
// #define VREG_ON 4    // vreg enable pin
// #define DAC_CS 14 // A0         // chip select pin
// #define DAC_MOSI 15
// #define DAC_SCK 16
// // Analog input channels...
// // #define MON_12V A4
// // #define MON_M12V A5
// // #define MON_VREG A6
// // #define MON_CURR A7
//
// unsigned int dacVal; // global variable keeping track of dac value.
//                      // Future feature... store in EEPROM at powerdown?
//
// // Calibrated dacVal -> current in mA
// //float transferSlope;
// //float transferOffset;
// //#define transferSlope
//
// // dac Min and Max, will load actual values from eeprom during setup.
// long dacMin = 0;
// long dacMax = 65535;
//
// // two bytes for EEPROM current limit address. MSB-formatted.
// #define EEPROM_LIM_LOW 0
// #define EEPROM_LIM_HI 1
//
// // two bytes for EEPROM dac value. MSB-formatted.
// #define EEPROM_CURR_LOW 2
// #define EEPROM_CURR_HI 3
//
// // keeps track of whether or not dac is switched off.
// bool dacOff = 1;
// /***********************************
// Encoder
// ************************************/
// #define ENC_A 10
// #define ENC_B 9
// #define ENC_BUTTON 2
//
// // global variables for encoder
// volatile bool encPrevState = 0;
// volatile long encPos = 0;
// long encPosLast = -999;
//
// #define debounce 20  // debounce time
// #define holdTime 2000 // hold time in ms
//
// int encButton = LOW;  // encoder button value
// int encButtonLast = LOW;  // previous button value
// long encButtonDnTime;
// long encButtonUpTime;
// bool encWasHold;
// bool limitMenu = 0;  // keeps track of if you're setting the limit or the actual current.
// long savedEncPos;
// long eepromEncPos;
//
// int dacStepSize; // step size for current adjustment. will be initialized in setupConfiguration().
//
//
// // Step sizes for current adjustment:
// // Can toggle through these with the encoder pushbutton.
//
// // If you add more step sizes, make sure to adjust ENCODER_TOTAL_STEPS!
// // You also always get the "LSB" for free.
// // ENCODER_TOTAL_STEPS should count the LSB size, too.
// #define ENCODER_TOTAL_STEPS 5
//
// // the first element of currentStepValues will actually be the LSB of the DAC.
// float currentStepValues[ENCODER_TOTAL_STEPS] = {1.0, 0.010, 0.025, 0.100, 1.000}; // these should be in uA.
//
// char* stepValues[ENCODER_TOTAL_STEPS] = {"LSB", "10 uA", "25 uA", "100 uA", "1 mA"};
//
// int stepArray[ENCODER_TOTAL_STEPS]; // this will be initialized in the setupCalibration function,
//                                     // to make sure it uses the right transfer function.
//
// int stepIndex = 1;  // keeps track of which dacStepSize resolution you're at.
//                     // defaults to smallest step value above LSB.
//
//
//
// // interrupt routine for encoder
// // void encoder_interrupt(){
// //   bool currentState, dir;
// //
// //   currentState = digitalRead(ENC_A);
// //   dir = digitalRead(ENC_B);
// //
// //
// //   if (dir == LOW){
// //     if(currentState == LOW && encPrevState == HIGH){
// //      encPos += dacStepSize;
// //     }
// //     else if (currentState == HIGH && encPrevState == LOW){
// //       encPos -= dacStepSize;
// //     }
// //   } else{
// //         if(currentState == LOW && encPrevState == HIGH){
// //      encPos -= dacStepSize;
// //     }
// //     else if (currentState == HIGH && encPrevState == LOW){
// //       encPos += dacStepSize;
// //     }
// //   }
// //
// //   encPrevState = currentState;
// //
// //   // make sure not out of bounds
// //   if ((encPos < dacMin) && !limitMenu){
// //      encPos = dacMin;
// //   } else if (encPos > dacMax){
// //     encPos = dacMax;
// //   } else if(encPos < 0){
// //     encPos = 0;
// //   }
// // }
//
//
//
// /***********************************
// LCD
// ************************************/
// // #define LCD_DATA4 14  // PC0
// // #define LCD_DATA5 15  // PC1
// // #define LCD_DATA6 16  // PC2
// // #define LCD_DATA7 17  // PC3
// //
// // #define LCD_RS 7
// // #define LCD_EN 6
// //
// // LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_DATA4, LCD_DATA5, LCD_DATA6, LCD_DATA7);
//
// #define LCD_RS 7
// #define LCD_RST 8
// #define LCD_CS 6
// #define LCD_EN 5
//
// LCD_ST7032 lcd(LCD_RST, LCD_RS, LCD_CS);
//
// // generic LCD print function.
// void writeLCD(int line, char * buffer){
//   lcd.setCursor(0,line);
//   lcd.print(buffer);
//   lcd.print("        ");
// }
//
// // prints dacStepSize resolution to LCD
// void writeStepLCD(){
//   lcd.setCursor(0, 0);
//   lcd.print("r ");
//   lcd.print(stepValues[stepIndex]);
//   lcd.print("        ");
// }
//
// // prints current to LCD
// void writeCurrentLCD(float val){
//   lcd.setCursor(0,1);
//   //lcd.print(val, 3);
// lcd.print(val);
// 	lcd.print(" mA        ");
// }
//
//
//
// /***********************************
// Misc Functions
// ************************************/
//
// void writeDac(unsigned int val){
//
//  digitalWrite(DAC_CS, LOW);
//
//
// 	// shiftOut(DAC_MOSI, DAC_SCK, MSBFIRST, (val >> 8));
// 	// shiftOut(DAC_MOSI, DAC_SCK, MSBFIRST, val);
//  // SPI.transfer(lowByte(val >> 8));
//  // SPI.transfer(lowByte(val));
//  digitalWrite(DAC_CS, HIGH);
//
//  dacVal = val;
//
// }
//
// // Calculates dacVal -> current in mA, for display
// float toCurrent(unsigned int dacWord){
//         //transferSlope = 427.024;
//    //   transferOffset = 153.468;
//   return(transferOffset - dacWord/transferSlope);
//
// }
//
// // current in uA. must be integer.
// unsigned int uAtoDacVal(float current){
//   unsigned int tmpDacVal;
//   tmpDacVal = (current*transferSlope + 0.5); // add 0.5 to round float, rather than truncate.
//   return(tmpDacVal);
// }
//
//
//
// void encPressEvent(){
//
//   // if you're in the limit menu, set dac limit and exit out to normal function.
//   if(limitMenu){
//     dacMin = encPos;	// set dac limit (dacMin = current max)
//
// 	EEPROM.write(EEPROM_LIM_LOW, lowByte(dacMin));
//     EEPROM.write(EEPROM_LIM_HI, lowByte(dacMin >> 8));
//     limitMenu = 0;
//
// 	dacStepSize = stepArray[stepIndex];
//     writeStepLCD();
//
//     // reset encoder & display to saved value before current limit menu
//     encPos = savedEncPos;
//     encPosLast = savedEncPos;
//     float curr = toCurrent(savedEncPos);
//     writeCurrentLCD(curr);
//
//   }else{
//    stepIndex++;
//    if (stepIndex >= ENCODER_TOTAL_STEPS) stepIndex = 0;
//    dacStepSize = stepArray[stepIndex];
//    writeStepLCD();
//
//   }
// }
//
//
// void encHoldEvent(){
//   // temporarily set stepsize to 1 mA (largest value...)
//   savedEncPos = encPos;
//   encPos = dacMin;
//   encPosLast = dacMin;
//   dacStepSize = stepArray[ENCODER_TOTAL_STEPS - 1];	// largest step size...
//   limitMenu = 1;
//   writeLCD(0, "Set Max");
//
//   // write old max value to display
//   float curr = toCurrent(dacMin);
//   writeCurrentLCD(curr);
// }
//
//
// /***********************************
// Main program
// ************************************/
//
// volatile int currentState2;
// volatile int dir2;
// volatile int counter2 = 0;
// int oldCounter2 = -1;
// int oldCounter3 = 0;
// int oldmillis = 0;
//
// // SIGNAL(PCINT0_vect){
// // 	uint8_t p = PINB;
// // 	currentState2 = p & 0b0100;
// //
// // 	//currentState2 = digitalRead(ENC_A);
// // 	//dir2 = digitalRead(ENC_B);
// // 	dir2 = p & 0b0010;
// // 	counter2++;
// // }
//
// void setup(){
//   // optional serial COM...
//   //Serial.begin(9600);
// 	SPI.begin();
//
//   pinMode(DAC_CS, OUTPUT);
// 	pinMode(DAC_MOSI, OUTPUT);
// 	pinMode(DAC_SCK, OUTPUT);
//
//
//   pinMode(VREG_ON, OUTPUT);
//
//   pinMode(SW_ENABLE, INPUT_PULLUP);
//   pinMode(ENC_BUTTON, INPUT_PULLUP);
// 	 pinMode(ENC_A, INPUT_PULLUP);
// 	// pinMode(ENC_B, INPUT_PULLUP);
//
// 	pinMode(LCD_EN, OUTPUT);
//
//
//   // start with VREG turned off.
//   digitalWrite(VREG_ON, LOW);
//   digitalWrite(DAC_CS, HIGH);
//
// 	// turn on lcd
// 	digitalWrite(LCD_EN, HIGH);
//
//
//
//
//     // read current setpoint from memory
//     byte dacMinLowByte;
//     byte dacMinHiByte;
//
//     dacMinLowByte = EEPROM.read(EEPROM_LIM_LOW);
//     dacMinHiByte = EEPROM.read(EEPROM_LIM_HI);
//     dacMin += (long)dacMinLowByte;
//     dacMin += (((long)dacMinHiByte) << 8);
//
//    // lcd.print(dacMin);
//   //  delay(2000);
//
//
//
//     // initialize stepArray, then set dacStepSize accordingly.
//     stepArray[0] = 1;	// LSB step size
//
//     unsigned int convertedStepSize;
//     for (int i = 1; i < ENCODER_TOTAL_STEPS; i++){
// 	    convertedStepSize = uAtoDacVal(currentStepValues[i]);
// 	    stepArray[i] =  convertedStepSize;
//     }
//
//     dacStepSize = stepArray[stepIndex];
//
// 	byte encPosLowByte;
// 	byte encPosHiByte;
// 	long eepromEncPos;
//     encPosLowByte = EEPROM.read(EEPROM_CURR_LOW);
//     encPosHiByte = EEPROM.read(EEPROM_CURR_HI);
//     eepromEncPos += (long)encPosLowByte;
//     eepromEncPos += (((long)encPosHiByte) << 8);
// 	encPos = eepromEncPos;
// 	//encPosLast = encPos;
//
//   // attach interrupt routine for encoder
//   //attachInterrupt(0, encoder_interrupt, CHANGE);
// //PCattachInterrupt(ENC_A, encoder_interrupt2, CHANGE);
//
//   // SREG |= (1<<7); // enable global interrupts
//   // PCICR = 0x01; // set PCIE0 bit
//   // PCMSK0 = 0x04;  // set PCINT2, which is PB2 / ENC_A
// 	//
//
//
//   // set up SPI interface for communicating with DAC
//   //SPI.begin();
//   // SPI.setBitOrder(MSBFIRST);
//   // SPI.setClockDivider(SPI_CLOCK_DIV128);
//
//   // read EEPROM to set dac minimum value (== current maximum)
//  // dacMin = 0;
//
//   //encPos = dacMax;    // reset encoder position...
//   writeDac(dacMax);  // set dac to zero.
//
//
//   // start LCD
// 	delay(100);
// 	lcd.begin();
// 	delay(100);
//   writeLCD(0, stepValues[stepIndex]);
//
// }
//
//
//
//
//
//
// void loop(){
//
//     /**********************
//     Turns current on/off
//     **********************/
//     // Checks if switch is on.
//     int tmpEn = digitalRead(SW_ENABLE);
//
//
//     // if the switch is on but the dac was previously off...
//     if(tmpEn == HIGH && dacOff){
//       writeDac(encPos);
//       delay(20);
//       digitalWrite(VREG_ON, HIGH);
//       dacOff = 0;
//     }
//
//     // if the switch is off but the DAC is still on...
//     if (tmpEn == LOW && !dacOff){
//       writeDac(dacMax);  // set dac to zero
//
//       writeLCD(0, "WAIT...");
//       delay(2000);
//       digitalWrite(VREG_ON, LOW);
//       dacOff = 1;
//
// 	  // persist value to EEPROM, if different from before...
// 	  if (encPos != eepromEncPos){
// 		EEPROM.write(EEPROM_CURR_LOW, lowByte(encPos));
// 		EEPROM.write(EEPROM_CURR_HI, lowByte(encPos >> 8));
// 		eepromEncPos = encPos;
// 	  }
//
//       writeLCD(0,"OFF");
//       delay(2000);
//       writeStepLCD();
//
//     }
//
//
//
//
//     /**********************
//     Checks for button press or hold
//
//     our variables:
//
//     #define debounce 20  // debounce time
//     #define holdTime 2000 // hold time in ms
//
//     int encButton = 0;  // encoder button value
//     int encButtonLast = 0;  // previous button value
//     long encButtonDnTime;
//     long encButtonUpTime;
//     bool encWasHold;
//
//     **********************/
//
//     encButton = digitalRead(ENC_BUTTON);  // logical polarity is reversed for this button...
//
//     // tests for button pressed, and store the timestamp for the press event.
//     if(encButton == LOW && encButtonLast == HIGH && (millis() - encButtonUpTime) > long(debounce)){
//       encButtonDnTime = millis();
//     }
//
//      // tests for button released, and stores the timestamp for the release event
//     if(encButton == HIGH && encButtonLast == LOW && (millis() - encButtonDnTime) > long(debounce)){
//       if(encWasHold == false){
//         encPressEvent();
//       }else {
//         // if it was a hold event, we didn't triger encPressEvent(). But, now we reset
//         // the state so the next button press will have to check for a hold.
//         encWasHold = false;
//       }
//       encButtonUpTime = millis();
//     }
//
//     // test for button hold.
//     // if the button is held, trigger encHoldEvent(), and make sure not to trigger encPressEvent
//     // when button released (ie, set encWasHold = true).
//     if (encButton == LOW && (millis() - encButtonDnTime) > long(holdTime)){
//       encHoldEvent();
//       encWasHold = true;
//       encButtonDnTime = millis();
//     }
//
//     encButtonLast = encButton;
//
//     if (encPos != encPosLast){
// 		encPosLast = encPos;
// 		float curr = toCurrent(encPos);
// 		writeCurrentLCD(curr);
//
//
// 		// if the sucker is actually on, and you're not updating the current limit, update the DAC.
// 		if (tmpEn == HIGH && !limitMenu)  {
// 			writeDac(encPos);
// 			dacOff = 0;
// 		}
// 	}
//
// 	///////////////////////////////////////
// 	if (oldCounter2 != counter2){
//
// 		lcd.setCursor(0, 0);
// 		lcd.print(counter2);
// 		oldCounter2 = counter2;
// 	}
//
// if ((millis() - oldmillis) > 1000){
// 	oldmillis = millis();
// 	oldCounter3++;
// 	lcd.setCursor(0,1);
// 	lcd.print(oldCounter3);
// }
//
//
//
// }


#include <SPI.h>
#include "LCD.h"

#include "src/cc_interrupts.h"
#include "src/cc_api.h"

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

LCD_ST7032 lcd(LCD_RST, LCD_RS, LCD_CS);





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



void setup(){

	pinMode(LCD_EN, OUTPUT);
	digitalWrite(LCD_EN, HIGH);

	pinMode(ENC_A, INPUT_PULLUP);
	pinMode(ENC_B, INPUT_PULLUP);
	pinMode(ENC_SW, INPUT_PULLUP);

	pinMode(DAC_CS, OUTPUT);
	digitalWrite(DAC_CS, HIGH);

	pinMode(DAC_MOSI, OUTPUT);
	pinMode(DAC_SCK, OUTPUT);
	digitalWrite(DAC_SCK, HIGH);
	digitalWrite(DAC_MOSI, LOW);

	pinMode(VREG_ON, OUTPUT);
	digitalWrite(VREG_ON, HIGH);

	SREG |= (1<<7); // enable global interrupts
	PCICR = 0x01; // set PCIE0 bit
	PCMSK0 = 0x04;  // set PCINT2, which is PB2 / ENC_A

	// initialize stepArray, then set dacStepSize accordingly.
	stepArray[0] = 1;	// LSB step size

	unsigned int convertedStepSize;
	for (int i = 1; i < ENCODER_TOTAL_STEPS; i++){
	  convertedStepSize = uAtoDacVal(currentStepValues[i]);
	  stepArray[i] =  convertedStepSize;
	}

	dacStepSize = stepArray[stepIndex];


	initFromEEPROM();


	writeDAC(dacVal);

	SPI.begin();
	delay(1);
	lcd.begin();

	writeStepLCD();
	float curr = toCurrent(encPos);
	writeCurrentLCD(curr);

}


bool btnPressed;
uint16_t lastPressed;

bool inMaxMenu = 0;

void loop(){

	float curr;
bool isPressed;
	if (inMaxMenu){
		if (encLastPos != encPos){
			stepIndex = (ENCODER_TOTAL_STEPS - 1);
			dacStepSize = stepArray[stepIndex];
			curr = toCurrent(encPos);
			writeCurrentLCD(curr);
			encLastPos = encPos;
		}


		 isPressed = !digitalRead(ENC_SW);
		if (isPressed & !btnPressed){

			// cheap debounce
			delay(200);
			dacMinVal = encPos;

			writeStepLCD();
			encPos = dacVal;
			curr = toCurrent(encPos);
			writeCurrentLCD(curr);
		}
		btnPressed = isPressed;

		return;
	}

	// check if pressed
	isPressed = !digitalRead(ENC_SW);
	if (isPressed & !btnPressed){
		lastPressed = millis();
		// cheap debounce
		delay(200);
		stepIndex = ((stepIndex + 1) % ENCODER_TOTAL_STEPS);
		dacStepSize = stepArray[stepIndex];
		writeStepLCD();
	} else if (!isPressed & btnPressed & (millis() - lastPressed > 2000)){
			lcd.clear();
			lcd.setCursor(0,0);
			lcd.print("MAX CURRENT:");
			curr = toCurrent(encPos);
			writeCurrentLCD(curr);
			inMaxMenu = 1;
	}
	btnPressed = isPressed;


	if (encLastPos != encPos){
		writeDAC(encPos);
		curr = toCurrent(encPos);
		writeCurrentLCD(curr);
		encLastPos = encPos;
	}
}
