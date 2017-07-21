/*
  cc_interrupts.h

  ISR definitions, setup function, etc. for Atmega328.
  Annoyingly, encoder is routed to PCINT instead of INT;
  handling this without relying on arduino attachInterrupt.

*/
#include "cc_api.h"

// initial value is off (remember, dac full scale = 0 current)


volatile bool encPrevState = 0;
volatile long encPos;
long encLastPos = 65535;




int pushDownTime = 0;
enum push_state_t { CLR = 0x00, PUSH = 0x01, HOLD = 0x02 };
push_state_t pushFlag = 0;

ISR(INT0_vect){
	bool currentState = PIND & 0b0100;
	if (!currentState){
		pushDownTime = millis();
	} else {
		int now = millis();
		if  ((now - pushDownTime) > 2000){
			pushFlag = HOLD;
		} else if ((now - pushDownTime) > 40){
			pushFlag = PUSH;
		} else {
			pushFlag = CLR;
		}
	}
}


ISR(PCINT0_vect){
  // ENC_A is on PB2
  // ENC_B is on PB1
	uint8_t p = PINB;
	bool currentState =  p & 0b0100; // state of ENC_A
	bool dir = p & 0b0010;           // state of ENC_B

  if (dir == LOW){
    if(currentState == LOW && encPrevState == HIGH){
      encPos += dacStepSize;
    } else if (currentState == HIGH && encPrevState == LOW){
      encPos -= dacStepSize;
    }
	} else {
    if (currentState == LOW && encPrevState == HIGH){
      encPos -= dacStepSize;
    } else if (currentState == HIGH && encPrevState == LOW){
      encPos += dacStepSize;
    }
	}

  encPrevState = currentState;
	encPos = constrain(encPos, dacMinVal, DACVAL_OFF);
}
