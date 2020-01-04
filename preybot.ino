#include <Servo.h> // for ESC signals
// #include <FastLED.h> // for LEDs
/////CONSTANTS
#define noSigOffTime 100000 // if there is no signal on a channel for this many microseconds, disable
#define LEDPeriod 20 // milliseconds between LED refreshes
#define yPercent 80 // what fraction of full power can be used for moving forwards (leaving power to move around other axes)
#define xPercent 55 // what fraction of full power can be used for moving sideways
#define zPercent 45 // what fraction of full power can be used for spinning
#define lct 120 // number of leds

/////PINS
//RC input pins (must be within D8-D13)
#define xPin 10 //left-right channel
#define yPin 8 //forwards-backwards channel
#define zPin 9 //spin channel

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

// variables to be used in the interrupt routine to record pulse times
volatile uint32_t xRiseTime = 0;
volatile uint32_t yRiseTime = 0;
volatile uint32_t zRiseTime = 0;

volatile uint32_t xFallTime = 0;
volatile uint32_t yFallTime = 0;
volatile uint32_t zFallTime = 0;

volatile bool xLast = false;
volatile bool yLast = false;
volatile bool zLast = false;

// pulse lengths (microseconds)
uint32_t xTime = 0;
uint32_t yTime = 0;
uint32_t zTime = 0;

volatile byte newPulseDataFlag = 255;

unsigned long LEDTimerLastMillis = 0;

//CRGB leds[lct];

Servo lf;
Servo rf;
Servo lb;
Servo rb;

void setup() {
  Serial.begin(115200);
  //  FastLED.addLeds<DOTSTAR, ledDataPin, ledClockPin, BGR>(leds, lct);
  //  LEDsAllOff();
  //  FastLED.show();

  lf.attach(lfPin);
  rf.attach(rfPin);
  lb.attach(lbPin);
  rb.attach(rbPin);

  lf.writeMicroseconds(1500);
  rf.writeMicroseconds(1500);
  lb.writeMicroseconds(1500);
  rb.writeMicroseconds(1500);

  delayMicroseconds(noSigOffTime);

  // setup interrupt pins
  pciSetup(yPin);
  pciSetup(xPin);
  pciSetup(zPin);
}

void loop() {
  if (newPulseDataFlag != 255) {
    noInterrupts(); // prevent an interrupt from changing a value while it's being used
    xTime = xFallTime - xRiseTime;
    yTime = yFallTime - yRiseTime;
    zTime = zFallTime - zRiseTime;
    interrupts();

    x = constrain(map(xTime, 1100, 1900, 0, 200), 0, 200);
    y = constrain(map(yTime, 1100, 1900, 0, 200), 0, 200);
    z = constrain(map(zTime, 1100, 1900, 0, 200), 0, 200);
  }

  if (micros() - xFallTime > noSigOffTime or micros() - yFallTime > noSigOffTime or micros() - zFallTime > noSigOffTime) {//signal loss timeout disable
    lf.writeMicroseconds(1500); // stop
    rf.writeMicroseconds(1500);
    lb.writeMicroseconds(1500);
    rb.writeMicroseconds(1500);
    delay(1);
  } else { // enabled
    if (newPulseDataFlag != 255) { // there's been a change to the controls
      //      Serial.print(y);
      //      Serial.print(",");
      //      Serial.print(x);
      //      Serial.print(",");
      //      Serial.print(z);
      //      Serial.println(",");
      //forward:+ right:- CCW:-
      lf.writeMicroseconds(constrain(1500 + (y - 100) * 5 * yPercent / 100 - (x - 100) * 5 * xPercent / 100 + (z - 100) * 5 * zPercent / 100, 1000, 2000));

      //forward:+ right:+ CCW:+
      rf.writeMicroseconds(constrain(1500 - (y - 100) * 5 * yPercent / 100 - (x - 100) * 5 * xPercent / 100 + (z - 100) * 5 * zPercent / 100, 1000, 2000));

      //forward:+ right:+ CCW:-
      lb.writeMicroseconds(constrain(1500 + (y - 100) * 5 * yPercent / 100 + (x - 100) * 5 * xPercent / 100 + (z - 100) * 5 * zPercent / 100, 1000, 2000));

      //forward:+ right:- CCW:+
      rb.writeMicroseconds(constrain(1500 - (y - 100) * 5 * yPercent / 100 + (x - 100) * 5 * xPercent / 100 + (z - 100) * 5 * zPercent / 100, 1000, 2000));
    }
    //    Serial.print(lf.readMicroseconds());
    //    Serial.print(",");
    //    Serial.print(rf.readMicroseconds());
    //    Serial.print(",");
    //    Serial.print(lb.readMicroseconds());
    //    Serial.print(",");
    //    Serial.print(rb.readMicroseconds());
    //    Serial.println(",");
  }//end of if enabled

  //  if (millis() - LEDTimerLastMillis > LEDPeriod) { // timer code to periodically run led code
  //    LEDsPulseHSvVT(200, 200, 30, 150, 2000);
  //    FastLED.show();
  //    LEDTimerLastMillis = millis();
  //  }
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
}

void pciSetup(byte pin) { // enables interrupts somehow, don't know it how works, I borrowed this code
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


//////////////////////////////////////////////////pattern functions
////makes moving waves between two colors. larger W=shorter wave period, larger S=faster movement
//void LEDscHSVcHSVwavesWS(CHSV A, CHSV B, int W, int S) {
//  CRGB a;
//  CRGB b;
//  hsv2rgb_rainbow(A, a);
//  hsv2rgb_rainbow(B, b);
//  unsigned long milli = millis();
//  for (int i = 0; i < lct; i++) {
//    byte scaler = sin8(int(milli / S + (i * W * 255 / lct)));
//    LEDsSetLightRGB(i, map(scaler, 0, 255, 0, a.red) + map(255 - scaler, 0, 255, 0, b.red), map(scaler, 0, 255, 0, a.green) + map(255 - scaler, 0, 255, 0, b.green), map(scaler, 0, 255, 0, a.blue) + map(255 - scaler, 0, 255, 0, b.blue));
//  }
//}
////pulses (period T) a color with hue H and saturation S from value v to V with time period T
//void LEDsPulseHSvVT(int H, int S, int v, int V, int T) {
//  unsigned long milli = millis();
//  if (milli % T < T / 2) {
//    LEDsSetAllHSV(H, S, constrain(map(milli % T, 0, T / 2, v, V), v, V));
//  }
//  if (milli % T >= T / 2) {
//    LEDsSetAllHSV(H, S, constrain(map(milli % T, T / 2, T, V, v), v, V));
//  }
//}
////makes moving rainbow waves. larger wvs=shorter waves, more repeated waves. larger spd=slower waves
//void LEDsWavesOfRainbowWSV(int wvs, int spd, int v) {
//  for (int i = 0; i < lct; i++) {
//    LEDsSetLightHSV(i, byte(int(i * wvs / lct + millis() * 10 / spd)), 255, v);
//  }
//}
//
//
//
///////////////////////////////////////////light setting functions (lower level)
//void LEDsAllOff() {
//  LEDsSetAllRGB(0, 0, 0);
//}
//void LEDsSetAllHSV(int HUE, int SAT, int VAL) {
//  for (int i = 0; i < lct; i++) {
//    LEDsSetLightHSV(i, HUE, SAT, VAL);
//  }
//}
//void LEDsSetLightHSV(int J, int HUE, int SAT, int VAL) {
//  if (J >= 0 && J < lct) {
//    leds[J] = CHSV(HUE, SAT, VAL);
//  }
//}
//void LEDsSetAllRGB(int RED, int GREEN, int BLUE) {
//  for (int i = 0; i < lct; i++) {
//    LEDsSetLightRGB(i, RED, GREEN, BLUE);
//  }
//}
//void LEDsSetLightRGB(int J, int RED, int GREEN, int BLUE) {
//  if (J >= 0 && J < lct) {
//    leds[J] = CRGB(RED, GREEN, BLUE);
//  }
//}
