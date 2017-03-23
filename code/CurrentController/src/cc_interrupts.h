/*
  cc_interrupts.h

  ISR definitions, setup function, etc. for Atmega328.
  Annoyingly, encoder is routed to PCINT instead of INT;
  handling this without relying on arduino attachInterrupt.

*/


// initial value is off (remember, dac full scale = 0 current)
extern uint16_t dacVal;
extern uint16_t dacMinVal;
extern uint16_t dacMaxVal;

volatile bool encPrevState = 0;
volatile long encPos;
long encLastPos = 65535;

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
	encPos = constrain(encPos, dacMinVal, dacMaxVal);
}
