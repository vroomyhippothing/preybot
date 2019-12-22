#include <Servo.h> //for ESC signals
#include <FastLED.h> //for LEDs
/////CONSTANTS
#define noSigOffTime 100000 //if there is no signal on a channel for this many microseconds, disable
#define LEDPeriod 20 //milliseconds between LED refreshes
#define yPercent 80
#define xPercent 55
#define sPercent 45

/////PINS
//RC input pins (must be within D8-D13)
#define xPin 9 //left-right channel
#define yPin 8 //forwards-backwards channel
#define zPin 10 //spin channel
#define sPin 11 //switch channel

//ESC pins
#define lfPin 2 // left front motor
#define rfPin 3 // right front motor
#define lbPin 4 // left back motor
#define rbPin 5 // right back motor

//LED pins
#define ledDataPin 6
#define ledClockPin 7

// drive vals. 0=-100% 100=0% 200=100%
uint8_t x = 100;
uint8_t y = 100;
uint8_t z = 100;

uint8_t s = 0; // value for switches

// motor disabling vars
bool lfOff = false;
bool rfOff = false;
bool lbOff = false;
bool rbOff = false;

// variables to be used in the interrupt routine to record pulse times
volatile uint32_t xRiseTime = 0;
volatile uint32_t yRiseTime = 0;
volatile uint32_t zRiseTime = 0;
volatile uint32_t sRiseTime = 0;

volatile uint32_t xFallTime = 0;
volatile uint32_t yFallTime = 0;
volatile uint32_t zFallTime = 0;
volatile uint32_t sFallTime = 0;

volatile bool xLast = false;
volatile bool yLast = false;
volatile bool zLast = false;
volatile bool sLast = false;

// pulse lengths (microseconds)
uint32_t xTime = 0;
uint32_t yTime = 0;
uint32_t zTime = 0;
uint32_t sTime = 0;

volatile byte newPulseDataFlag = 255;

unsigned long LEDTimerLastMillis = 0;

Servo lf;
Servo rf;
Servo lb;
Servo rb;

void setup() {
  lf.attach(lfPin);
  rf.attach(rfPin);
  lb.attach(lbPin);
  rb.attach(rbPin);

  lf.writeMicroseconds(1500);
  rf.writeMicroseconds(1500);
  lb.writeMicroseconds(1500);
  rb.writeMicroseconds(1500);

  // setup interrupt pins
  pciSetup(yPin);
  pciSetup(xPin);
  pciSetup(zPin);
  pciSetup(sPin);
}

void loop() {
  if (newPulseDataFlag != 255) {
    if (newPulseDataFlag == xPin) {
      noInterrupts(); // prevent an interrupt from changing a value while it's being used
      xTime = xFallTime - xRiseTime;
      interrupts();
    }
    if (newPulseDataFlag == yPin) {
      noInterrupts();
      xTime = yFallTime - yRiseTime;
      interrupts();
    }
    if (newPulseDataFlag == zPin) {
      noInterrupts();
      zTime = zFallTime - zRiseTime;
      interrupts();
    }
    if (newPulseDataFlag == sPin) {
      noInterrupts();
      sTime = sFallTime - sRiseTime;
      interrupts();
    }

    x = constrain(map(xTime, 1000, 2000, 0, 200), 0, 200);
    y = constrain(map(yTime, 1000, 2000, 0, 200), 0, 200);
    z = constrain(map(zTime, 1000, 2000, 0, 200), 0, 200);

    s = (constrain(sTime, 1000, 2000) - 1000) / 63; // convert sTime to 0-15

    //extract four bits (2^4=16)
    lfOff = s & B00000001 == 1;
    rfOff = (s & B00000010) >> 1 == 1;
    lbOff = (s & B00000100) >> 2 == 1;
    rbOff = (s & B00001000) >> 3 == 1;
  }

  if (micros() - xFallTime > noSigOffTime or micros() - yFallTime > noSigOffTime or micros() - zFallTime > noSigOffTime or micros() - sFallTime > noSigOffTime) {//signal loss timeout disable
    lf.writeMicroseconds(1500); // stop
    rf.writeMicroseconds(1500);
    lb.writeMicroseconds(1500);
    rb.writeMicroseconds(1500);
    delay(1);
  } else { // enabled
    if (newPulseDataFlag != 255) { // there's been a change to the controls
      if (!lfOff) { // left front wheel on
        //forward:+ right:- CCW:-
        lf.writeMicroseconds(constrain(1500 + (y - 100) * 5 * yPercent / 100 - (x - 100) * 5 * xPercent / 100 - (s - 100) * 5 * sPercent / 100, 1000, 2000));
      } else {
        lf.writeMicroseconds(1500); //turn off motor
      }

      if (!rfOff) { // right front wheel on
        //forward:+ right:+ CCW:+
        rf.writeMicroseconds(constrain(1500 + (y - 100) * 5 * yPercent / 100 + (x - 100) * 5 * xPercent / 100 + (s - 100) * 5 * sPercent / 100, 1000, 2000));
      } else {
        rf.writeMicroseconds(1500); //turn off motor
      }

      if (!lbOff) { // left back wheel on
        //forward:+ right:+ CCW:-
        lb.writeMicroseconds(constrain(1500 + (y - 100) * 5 * yPercent / 100 + (x - 100) * 5 * xPercent / 100 - (s - 100) * 5 * sPercent / 100, 1000, 2000));
      } else {
        lb.writeMicroseconds(1500); //turn off motor
      }

      if (!rbOff) { // right back wheel on
        //forward:+ right:- CCW:+
        rb.writeMicroseconds(constrain(1500 + (y - 100) * 5 * yPercent / 100 - (x - 100) * 5 * xPercent / 100 + (s - 100) * 5 * sPercent / 100, 1000, 2000));
      } else {
        rb.writeMicroseconds(1500); //turn off motor
      }
    }
  }//end of if enabled
  if (millis() - LEDTimerLastMillis > LEDPeriod) { // timer code to periodically run led code
    FastLED.show();
    LEDTimerLastMillis = millis();
  }
}

//this function gets called when any of the RC pins changes state
ISR (PCINT0_vect) { // Pin Change Interrupt Request 0 (pins D8 to D13)
  newPulseDataFlag = 255;
  if (digitalRead(xPin) == LOW) {
    if (xLast == HIGH) { // falling
      xFallTime = micros(); // record current microseconds since program start
      xLast = LOW; //save for next time
      newPulseDataFlag = xPin; // set flag that there was just a change on this pin.
    }
  } else { // xPin==HIGH
    if (xLast == LOW) { // rising
      xRiseTime = micros();
      xLast = HIGH;
    }
  }

  if (digitalRead(yPin) == LOW) {
    if (yLast == HIGH) { // falling
      yFallTime = micros(); // record current microseconds since program start
      yLast = LOW; //save for next time
      newPulseDataFlag = yPin; // set flag that there was just a change on this pin.
    }
  } else { // yPin==HIGH
    if (yLast == LOW) { // rising
      yRiseTime = micros();
      yLast = HIGH;
    }
  }

  if (digitalRead(zPin) == LOW) {
    if (zLast == HIGH) { // falling
      zFallTime = micros(); // record current microseconds since program start
      zLast = LOW; //save for next time
      newPulseDataFlag = zPin; // set flag that there was just a change on this pin.
    }
  } else { // zPin==HIGH
    if (zLast == LOW) { // rising
      zRiseTime = micros();
      zLast = HIGH;
    }
  }

  if (digitalRead(sPin) == LOW) {
    if (sLast == HIGH) { // falling
      sFallTime = micros(); // record current microseconds since program start
      sLast = LOW; //save for next time
      newPulseDataFlag = sPin; // set flag that there was just a change on this pin.
    }
  } else { // sPin==HIGH
    if (sLast == LOW) { // rising
      sRiseTime = micros();
      sLast = HIGH;
    }
  }
}

void pciSetup(byte pin) { // enables interrupts somehow, don't know it how works, I borrowed this code
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}
