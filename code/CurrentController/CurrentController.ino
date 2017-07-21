// // Current Controller Software v2.0
// // For hardware v2.0


#include <SPI.h>
#include "LCD.h"

#include "src/cc_interrupts.h"
#include "src/cc_api.h"


LCD_ST7032 lcd(LCD_RST, LCD_RS, LCD_CS);

void setup(){

	pinMode(LCD_EN, OUTPUT);
	digitalWrite(LCD_EN, HIGH);

	pinMode(ENC_A, INPUT_PULLUP);
	pinMode(ENC_B, INPUT_PULLUP);
	pinMode(ENC_SW, INPUT_PULLUP);
	pinMode(SW, INPUT_PULLUP);

	pinMode(DAC_CS, OUTPUT);
	digitalWrite(DAC_CS, HIGH);

	pinMode(DAC_MOSI, OUTPUT);
	pinMode(DAC_SCK, OUTPUT);
	digitalWrite(DAC_SCK, HIGH);
	digitalWrite(DAC_MOSI, LOW);

	pinMode(VREG_ON, OUTPUT);
	digitalWrite(VREG_ON, LOW);

	SREG |= (1<<7); // enable global interrupts
	PCICR = 0x01; // set PCIE0 bit
	PCMSK0 = 0x04;  // set PCINT2, which is PB2 / ENC_A

	// interrupts for encoder button
	EICRA |= 1;	// trigger on any logic level change
	EIMSK |= 1;	// enable interrupt 0

	// initialize stepArray, then set dacStepSize accordingly.
	stepArray[0] = 1;	// LSB step size

	uint16_t convertedStepSize;
	for (int i = 1; i < ENCODER_TOTAL_STEPS; i++){
	  convertedStepSize = uAtoDacVal(currentStepValues[i]);
	  stepArray[i] =  convertedStepSize;
	}

	dacStepSize = stepArray[stepIndex];

	SPI.begin();
	delay(1);
	lcd.begin();


	initFromEEPROM();

	writeDAC(dacVal);

	writeStepLCD();
	float curr = toCurrent(encPos);
	writeCurrentLCD(curr);

}

int dacOff = 1;
int enSw = 0;
int menu = 0;	// default; menu == 1 if setting max current.

void output_onoff(){
	/**********************
	Turns current on/off
	**********************/

	// Checks if switch is on.
	// internal pullup, so negate
	enSw = !digitalRead(SW);

	// if the switch is on but the dac was previously off...
	if(enSw == HIGH && dacOff){
		writeDAC(encPos);
		delay(20);
		digitalWrite(VREG_ON, HIGH);
		dacOff = 0;
	}

	// if the switch is off but the DAC is still on...
	if (enSw == LOW && !dacOff){
		writeDAC(DACVAL_OFF);  // set dac to zero
		lcd.setCursor(0,0);
		lcd.print("WAIT...         ");
		delay(2000);
		digitalWrite(VREG_ON, LOW);
		dacOff = 1;

		// persist to eeprom
		EEPROM.put(dacValAddr, encPos);

		lcd.setCursor(0,0);
		lcd.print("OFF             ");
		delay(2000);
		writeStepLCD();
	}

}


uint16_t encPos_stashed = 0;

void loop(){

	output_onoff();

	float curr;
	if (menu == 0){
		// service button press:
		if (pushFlag == PUSH){
			stepIndex = ((stepIndex + 1) % ENCODER_TOTAL_STEPS);
			dacStepSize = stepArray[stepIndex];
			writeStepLCD();
			pushFlag = CLR;
		} else if (pushFlag == HOLD){
			menu = 1;
			pushFlag = CLR;
			encPos_stashed = encPos;
			encPos = dacMinVal;
			lcd.setCursor(0,0);
			lcd.print("Set MAX Current:");
			curr = toCurrent(encPos);
			writeCurrentLCD(curr);
		}


		if (encLastPos != encPos){
			if (enSw) {
				writeDAC(encPos);
			}
			curr = toCurrent(encPos);
			writeCurrentLCD(curr);
			encLastPos = encPos;
		}
	} else {
		// setting max current
		if (pushFlag == PUSH){
			// return to main menu
			menu = 0;
			dacMinVal = encPos;

			// persist to EEPROM
			EEPROM.put(dacMinAddr, encPos);

			writeStepLCD();
			encPos = encPos_stashed;

			pushFlag = CLR;
		}
		if (encLastPos != encPos){
			curr = toCurrent(encPos);
			writeCurrentLCD(curr);
			encLastPos = encPos;
		}

	}
}
