//===========================================================================//
//                                                                           //
//  Desc:    Arduino Code to implement a fencing scoring apparatus           //
//  Dev:     Wnew                                                            //
//  Date:    Nov  2012                                                       //
//  Updated: Sept 2015                                                       //
//  Notes:   1. Basis of algorithm from digitalwestie on github. Thanks Mate //
//           2. Used uint8_t instead of int where possible to optimise       //
//           3. Set ADC prescaler to 16 faster ADC reads                     //
//                                                                           //
//  To do:   1. Could use shift reg on lights and mode LEDs to save pins     //
//           2. Implement short circuit LEDs (already provision for it)      //
//           3. Set up debug levels correctly                                //
//                                                                           //
//===========================================================================//

//============
// #defines
//============
//TODO: set up debug levels correctly
#define DEBUG 0
#define DEBUG_VERBOSE 0
//#define TEST_LIGHTS       // turns on lights for a second on start up
//#define TEST_ADC_SPEED    // used to test sample rate of ADCs
//#define REPORT_TIMING     // prints timings over serial interface
#define BUZZERTIME 1000  // length of time the buzzer is kept on after a hit (ms)
#define LIGHTTIME 3000   // length of time the lights are kept on after a hit (ms)
#define BAUDRATE 57600   // baudrate of the serial debug interface

#include <FastLED.h>

//============
// Pin Setup
//============
// const uint8_t shortLEDA  =  8;    // Short Circuit A Light
const uint8_t offTargetB = 11;  // Off Target A Light White_Green
const uint8_t onTargetB = 12;   // On Target A Light Green
const uint8_t onTargetA = 9;    // On Target B Light Red
const uint8_t offTargetA = 10;  // Off Target B Light White_Red
// const uint8_t shortLEDB  = 13;    // Short Circuit A Light

const uint8_t lamePinB = A0;    // Lame   B pin - Analog (Epee return path) A-Green
const uint8_t weaponPinB = A1;  // Weapon B pin - Analog Analog B-Green
const uint8_t groundPinB = A2;  // Ground B pin - Analog Analog C-Green
const uint8_t groundPinA = A3;  // Ground A pin - Analog C-Red
const uint8_t weaponPinA = A4;  // Weapon A pin - Analog B-Red
const uint8_t lamePinA = A5;    // Lame   A pin - Analog (Epee return path) A-Red

const uint8_t modePin1 = 2;   // Mode change button interrupt pin 0 (digital pin 2)
const uint8_t modePin2 = 3;   // Mode change button interrupt pin 0 (digital pin 3)
const uint8_t buzzerPin = 7;  // buzzer pin
const uint8_t RGB_LED_R = 4;  // Red LED from the RGB LED
const uint8_t RGB_LED_B = 5;  // Blue LED from the RGB LED
const uint8_t RGB_LED_G = 6;  // Green LED from the RGB LED

// const uint8_t modeLeds[] = { 4, 5, 6 };  // LED pins to indicate weapon mode selected {f e s}

//=========================
// values of analog reads
//=========================
int weaponA = 0;
int weaponB = 0;
int lameA = 0;
int lameB = 0;
int groundA = 0;
int groundB = 0;

//=======================
// depress and timeouts
//=======================
long depressAtime = 0;
long depressBtime = 0;
bool lockedOut = false;

//==========================
// Lockout & Depress Times
//==========================
// the lockout time between hits for foil is 300ms +/-25ms
// the minimum amount of time the tip needs to be depressed for foil 14ms +/-1ms
// the lockout time between hits for epee is 45ms +/-5ms (40ms -> 50ms)
// the minimum amount of time the tip needs to be depressed for epee 2ms
// the lockout time between hits for sabre is 170ms +/-10ms
// the minimum amount of time the tip needs to be depressed (in contact) for sabre 0.1ms -> 1ms
// These values are stored as micro seconds for more accuracy
//                       foil     epee   sabre   foil_classic
const long lockout[] = { 300000, 45000, 170000, 300000 };  // the lockout time between hits
const long depress[] = { 14000, 2000, 1000, 2000 };        // the minimum amount of time the tip needs to be depressed



//=================
// mode constants
//=================
const uint8_t FOIL_MODE = 0;
const uint8_t EPEE_MODE = 1;
const uint8_t SABRE_MODE = 2;
const uint8_t FOIL_CLASSIC_MODE = 3;

uint8_t currentMode = EPEE_MODE;

volatile bool modeJustChangedFlag = true;
// bool modeJustChangedFlag = false;

//=========
// states
//=========
boolean depressedA = false;
boolean depressedB = false;
boolean hitOnTargA = false;
boolean hitOffTargA = false;
boolean hitOnTargB = false;
boolean hitOffTargB = false;

//=========
// Previous states
//=========

boolean depressedA_prev = false;
boolean depressedB_prev = false;
boolean hitOnTargA_prev = false;
boolean hitOffTargA_prev = false;
boolean hitOnTargB_prev = false;
boolean hitOffTargB_prev = false;
boolean buzzerPin_prev = false;


#ifdef TEST_ADC_SPEED
long now;
long loopCount = 0;
bool done = false;
#endif


//================
// Configuration
//================
void setup() {
  // set the internal pullup resistor on modePin
  pinMode(modePin1, INPUT_PULLUP);
  pinMode(modePin2, INPUT_PULLUP);

  // add the interrupt to the mode pin (interrupt is pin 0)
  //  attachInterrupt(modePin1-2, changeMode, change);
  //  attachInterrupt(modePin2-3, changeMode, change);
  attachInterrupt(digitalPinToInterrupt(2), changeMode, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), changeMode, CHANGE);

  // pinMode(modeLeds[0], OUTPUT);
  // pinMode(modeLeds[1], OUTPUT);
  // pinMode(modeLeds[2], OUTPUT);

  pinMode(RGB_LED_R, OUTPUT);
  pinMode(RGB_LED_G, OUTPUT);
  pinMode(RGB_LED_B, OUTPUT);

  // set the light pins to outputs
  pinMode(offTargetA, OUTPUT);
  pinMode(offTargetB, OUTPUT);
  pinMode(onTargetA, OUTPUT);
  pinMode(onTargetB, OUTPUT);
  // pinMode(shortLEDA, OUTPUT);
  // pinMode(shortLEDB, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // digitalWrite(modeLeds[currentMode], HIGH);

#ifdef TEST_LIGHTS
  testLights();
#endif

  // this optimises the ADC to make the sampling rate quicker
  //adcOpt();

  Serial.begin(BAUDRATE);
  Serial.println("3 Weapon Scoring Box");
  Serial.println("====================");
  Serial.print("Mode : ");
  Serial.println(currentMode);

  resetValues();

  // Sets Mode Based on Push Buttons
  checkIfModeChanged();
}


//=============
// ADC config
//=============
void adcOpt() {

  // the ADC only needs a couple of bits, the atmega is an 8 bit micro
  // so sampling only 8 bits makes the values easy/quicker to process
  // unfortunately this method only works on the Due.
  //analogReadResolution(8);

  // Data Input Disable Register
  // disconnects the digital inputs from which ever ADC channels you are using
  // an analog input will be float and cause the digital input to constantly
  // toggle high and low, this creates noise near the ADC, and uses extra
  // power Secondly, the digital input and associated DIDR switch have a
  // capacitance associated with them which will slow down your input signal
  // if you’re sampling a highly resistive load
  DIDR0 = 0x7F;

  // set the prescaler for the ADCs to 16 this allows the fastest sampling
  bitClear(ADCSRA, ADPS0);
  bitClear(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS2);
}


//============
// Main Loop
//============
void loop() {
  // use a while as a main loop as the loop() has too much overhead for fast analogReads
  // we get a 3-4% speed up on the loop this way
  while (1) {
    checkIfModeChanged();
    // read analog pins
    weaponA = analogRead(weaponPinA);
    weaponB = analogRead(weaponPinB);
    lameA = analogRead(lamePinA);
    lameB = analogRead(lamePinB);
    signalHits();
    if (currentMode == FOIL_MODE)
      foil();
    else if (currentMode == EPEE_MODE)
      epee();
    else if (currentMode == SABRE_MODE)
      sabre();
    else if (currentMode == FOIL_CLASSIC_MODE)
      foil_classic();

#ifdef TEST_ADC_SPEED
    if (loopCount == 0) {
      now = micros();
    }
    loopCount++;
    if ((micros() - now >= 1000000) && done == false) {
      Serial.print(loopCount);
      Serial.println(" readings in 1 sec");
      done = true;
    }
#endif

    if (DEBUG_VERBOSE) {
      EVERY_N_MILLIS(6000) {

        String serData = String("lamePinA is : ") + analogRead(lamePinA) + "\n"  // A - Red
                         + "weaponPinA is : " + analogRead(weaponPinA) + "\n"    //B - Red
                         + "groundPinA is: " + analogRead(groundPinA) + "\n"     // C - Red
                         + "groundPinB is : " + analogRead(groundPinB) + "\n"    // C - Green
                         + "weaponPinB is  : " + analogRead(weaponPinB) + "\n"   // B - Green
                         + "lamePinB is  : " + analogRead(lamePinB) + "\n"       // A - Green
                         + "modeJustChangedFlag is : " + modeJustChangedFlag + "\n"
                         + "ModePin1 is :" + digitalRead(modePin1) + "\n"
                         + "ModePin2 is :" + digitalRead(modePin2) + "\n"
                         + "Current Mode is : " + currentMode + "\n"
                         + "depressedA is : " + depressedA + "\n"
                         + "depressedB is : " + depressedB + "\n";

        Serial.println(serData);

        Serial.println("\n\n\n\n\n\n\n");
      }
    }
  }
}

//=====================
// Mode pin interrupt
//=====================
void changeMode() {
  // set a flag to keep the time in the ISR to a min
  modeJustChangedFlag = true;
}


//============================
// Sets the correct mode led
//============================
void setModeLeds() {
  // Uses an RGB LED to display mode
  if (currentMode == FOIL_MODE) {
    digitalWrite(RGB_LED_R, HIGH);
    digitalWrite(RGB_LED_G, LOW);
    digitalWrite(RGB_LED_B, LOW);
  } else if (currentMode == EPEE_MODE) {
    digitalWrite(RGB_LED_R, LOW);
    digitalWrite(RGB_LED_G, HIGH);
    digitalWrite(RGB_LED_B, LOW);
  } else if (currentMode == SABRE_MODE) {
    digitalWrite(RGB_LED_R, LOW);
    digitalWrite(RGB_LED_G, LOW);
    digitalWrite(RGB_LED_B, HIGH);
  } else if (currentMode == FOIL_CLASSIC_MODE) {
    digitalWrite(RGB_LED_R, HIGH);
    digitalWrite(RGB_LED_G, HIGH);
    digitalWrite(RGB_LED_B, HIGH);
  } else {
    digitalWrite(RGB_LED_R, LOW);
    digitalWrite(RGB_LED_G, LOW);
    digitalWrite(RGB_LED_B, LOW);
  }
}


//========================
// Run when mode changed
//========================
void checkIfModeChanged() {
  if (modeJustChangedFlag) {
    // Brief Delay to allow voltages to stabilize
    // This is not a time critical portion
    delay(100);
#ifdef DEBUG
    Serial.print("ModePin1 is : ");
    Serial.println(digitalRead(modePin1));
    Serial.print("ModePin2 is : ");
    Serial.println(digitalRead(modePin2));
#endif
    if (digitalRead(modePin1)) {
      if (digitalRead(modePin2)) {
        currentMode = 3;  // Classic Foil corresponding to 11
      } else {
        currentMode = 0;  // Epee corresponding to 10
      }
    } else {
      if (digitalRead(modePin2)) {
        currentMode = 2;  // Saber corresponding to 01
      } else {
        currentMode = 1;  // Foil corresponding to 00
      }
    }
    // if (digitalRead(modePin)) {
    //   if (currentMode == 2)
    //     currentMode = 0;
    //   else
    //     currentMode++;
    // }
    setModeLeds();
#ifdef DEBUG
    Serial.print("Mode changed to: ");
    Serial.println(currentMode);
#endif
    modeJustChangedFlag = false;
  }
}


//===================
// Main foil method
//===================
void foil() {

  long now = micros();
  if (((hitOnTargA || hitOffTargA) && (depressAtime + lockout[0] < now)) || ((hitOnTargB || hitOffTargB) && (depressBtime + lockout[0] < now))) {
    lockedOut = true;
  }

  // weapon A
  if (hitOnTargA == false && hitOffTargA == false) {  // ignore if A has already hit
    // off target
    if (900 < weaponA && lameB < 100) {

      if (!depressedA) {
        depressAtime = micros();
        depressedA = true;
      } else {
        if (depressAtime + depress[0] <= micros()) {
          hitOffTargA = true;
        }
      }
    } else {
      // on target
      if (400 < weaponA && weaponA < 600 && 400 < lameB && lameB < 600) {
        if (!depressedA) {
          depressAtime = micros();
          depressedA = true;
        } else {
          if (depressAtime + depress[0] <= micros()) {
            hitOnTargA = true;
          }
        }
      } else {
        // reset these values if the depress time is short.
        depressAtime = 0;
        depressedA = 0;
      }
    }
  }

  // weapon B
  if (hitOnTargB == false && hitOffTargB == false) {  // ignore if B has already hit
    // off target
    if (900 < weaponB && lameA < 100) {
      if (!depressedB) {
        depressBtime = micros();
        depressedB = true;
      } else {
        if (depressBtime + depress[0] <= micros()) {
          hitOffTargB = true;
        }
      }
    } else {
      // on target
      if (400 < weaponB && weaponB < 600 && 400 < lameA && lameA < 600) {
        if (!depressedB) {
          depressBtime = micros();
          depressedB = true;
        } else {
          if (depressBtime + depress[0] <= micros()) {
            hitOnTargB = true;
          }
        }
      } else {
        // reset these values if the depress time is short.
        depressBtime = 0;
        depressedB = 0;
      }
    }
  }
}


//===================
// Main foil classic method
//===================
void foil_classic() {
  // Uses the Pre-2005 depress timing

  long now = micros();
  if (((hitOnTargA || hitOffTargA) && (depressAtime + lockout[3] < now)) || ((hitOnTargB || hitOffTargB) && (depressBtime + lockout[3] < now))) {
    lockedOut = true;
  }

  // weapon A
  if (hitOnTargA == false && hitOffTargA == false) {  // ignore if A has already hit
    // off target
    if (900 < weaponA && lameB < 100) {

      if (!depressedA) {
        depressAtime = micros();
        depressedA = true;
      } else {
        if (depressAtime + depress[3] <= micros()) {
          hitOffTargA = true;
        }
      }
    } else {
      // on target
      if (400 < weaponA && weaponA < 600 && 400 < lameB && lameB < 600) {
        if (!depressedA) {
          depressAtime = micros();
          depressedA = true;
        } else {
          if (depressAtime + depress[3] <= micros()) {
            hitOnTargA = true;
          }
        }
      } else {
        // reset these values if the depress time is short.
        depressAtime = 0;
        depressedA = 0;
      }
    }
  }

  // weapon B
  if (hitOnTargB == false && hitOffTargB == false) {  // ignore if B has already hit
    // off target
    if (900 < weaponB && lameA < 100) {
      if (!depressedB) {
        depressBtime = micros();
        depressedB = true;
      } else {
        if (depressBtime + depress[3] <= micros()) {
          hitOffTargB = true;
        }
      }
    } else {
      // on target
      if (400 < weaponB && weaponB < 600 && 400 < lameA && lameA < 600) {
        if (!depressedB) {
          depressBtime = micros();
          depressedB = true;
        } else {
          if (depressBtime + depress[3] <= micros()) {
            hitOnTargB = true;
          }
        }
      } else {
        // reset these values if the depress time is short.
        depressBtime = 0;
        depressedB = 0;
      }
    }
  }
}

//===================
// Main epee method
//===================
void epee() {
  long now = micros();
  if ((hitOnTargA && (depressAtime + lockout[1] < now)) || (hitOnTargB && (depressBtime + lockout[1] < now))) {
    lockedOut = true;
  }

  // weapon A
  //  no hit for A yet    && weapon depress    && opponent lame touched
  if (hitOnTargA == false) {
    if (400 < weaponA && weaponA < 600 && 400 < lameA && lameA < 600) {
      if (!depressedA) {
        depressAtime = micros();
        depressedA = true;
      } else {
        if (depressAtime + depress[1] <= micros()) {
          hitOnTargA = true;
        }
      }
    } else {
      // reset these values if the depress time is short.
      if (depressedA == true) {
        depressAtime = 0;
        depressedA = 0;
      }
    }
  }

  // weapon B
  //  no hit for B yet    && weapon depress    && opponent lame touched
  if (hitOnTargB == false) {
    if (400 < weaponB && weaponB < 600 && 400 < lameB && lameB < 600) {
      if (!depressedB) {
        depressBtime = micros();
        depressedB = true;
      } else {
        if (depressBtime + depress[1] <= micros()) {
          hitOnTargB = true;
        }
      }
    } else {
      // reset these values if the depress time is short.
      if (depressedB == true) {
        depressBtime = 0;
        depressedB = 0;
      }
    }
  }
}


//===================
// Main sabre method
//===================
void sabre() {

  long now = micros();
  if (((hitOnTargA || hitOffTargA) && (depressAtime + lockout[2] < now)) || ((hitOnTargB || hitOffTargB) && (depressBtime + lockout[2] < now))) {
    lockedOut = true;
  }

  // weapon A
  if (hitOnTargA == false && hitOffTargA == false) {  // ignore if A has already hit
    // on target
    if (400 < weaponA && weaponA < 600 && 400 < lameB && lameB < 600) {
      if (!depressedA) {
        depressAtime = micros();
        depressedA = true;
      } else {
        if (depressAtime + depress[2] <= micros()) {
          hitOnTargA = true;
          if (DEBUG) {
            Serial.println("Target A Hit in Sabre.");
          }
        }
      }
    } else {
      // reset these values if the depress time is short.
      depressAtime = 0;
      depressedA = 0;
    }
  }

  // weapon B
  if (hitOnTargB == false && hitOffTargB == false) {  // ignore if B has already hit
    // on target
    if (400 < weaponB && weaponB < 600 && 400 < lameA && lameA < 600) {
      if (!depressedB) {
        depressBtime = micros();
        depressedB = true;
      } else {
        if (depressBtime + depress[2] <= micros()) {
          hitOnTargB = true;
          if (DEBUG) {
            Serial.println("Target B Hit in Sabre.");
          }
        }
      }
    } else {
      // reset these values if the depress time is short.
      depressBtime = 0;
      depressedB = 0;
    }
  }
}


//==============
// Signal Hits
//==============
void signalHits() {
  //   // non time critical, this is run after a hit has been detected
  //   if (lockedOut) {
  //     digitalWrite(onTargetA, hitOnTargA);
  //     digitalWrite(offTargetA, hitOffTargA);
  //     digitalWrite(offTargetB, hitOffTargB);
  //     digitalWrite(onTargetB, hitOnTargB);
  //     digitalWrite(buzzerPin, HIGH);

  // #ifdef DEBUG
  //     String serData = String("hitOnTargA  : ") + hitOnTargA + "\n"
  //                      + "hitOffTargA : " + hitOffTargA + "\n"
  //                      + "hitOffTargB : " + hitOffTargB + "\n"
  //                      + "hitOnTargB  : " + hitOnTargB + "\n"
  //                      + "Locked Out  : " + lockedOut + "\n";
  //     Serial.println(serData);
  // #endif
  //     resetValues();
  //   }

  // Only Updates the Lights when there is a change, but will also do so before the timing is LockOut
  if (hitOnTargA_prev == 0 && hitOnTargA == 1) { digitalWrite(onTargetA, hitOnTargA); }      // Checks when hitOnTargA turns on
  hitOnTargA_prev = hitOnTargA;                                                              // Updates Previous hitonTargA
  if (hitOffTargA_prev == 0 && hitOffTargA == 1) { digitalWrite(offTargetA, hitOffTargA); }  // Checks when hitOffTargA turns on
  hitOffTargA_prev = hitOffTargA;                                                            // Updates Previous hitOffTargA

  if (hitOffTargB_prev == 0 && hitOffTargB == 1) { digitalWrite(offTargetB, hitOffTargB); }  // Checks when hitOffTargB turns on
  hitOffTargB_prev = hitOffTargB;                                                            // Updates Previous hitOffTargB
  if (hitOnTargB_prev == 0 && hitOnTargB == 1) { digitalWrite(onTargetB, hitOnTargB); }      // Checks when hitOnTargB turns on
  hitOnTargB_prev = hitOnTargB;                                                              // Updates Previous hitOnTargB
  if (buzzerPin_prev == 0 && buzzerPin == 1) { digitalWrite(buzzerPin, HIGH); }              // Checks when buzzerPin turns on
  buzzerPin_prev = buzzerPin;                                                                // Updates Previous buzzerPin
}


//======================
// Reset all variables
//======================
void resetValues() {
  delay(BUZZERTIME);  // wait before turning off the buzzer
  digitalWrite(buzzerPin, LOW);
  delay(LIGHTTIME - BUZZERTIME);  // wait before turning off the lights
  digitalWrite(onTargetA, LOW);
  digitalWrite(offTargetA, LOW);
  digitalWrite(offTargetB, LOW);
  digitalWrite(onTargetB, LOW);
  // digitalWrite(shortLEDA, LOW);
  // digitalWrite(shortLEDB, LOW);

  lockedOut = false;
  depressAtime = 0;
  depressedA = false;
  depressBtime = 0;
  depressedB = false;

  hitOnTargA = false;
  hitOffTargA = false;
  hitOnTargB = false;
  hitOffTargB = false;

  delay(100);
}

//==============
// Test lights
//==============
void testLights() {
  digitalWrite(offTargetA, HIGH);
  digitalWrite(onTargetA, HIGH);
  digitalWrite(offTargetB, HIGH);
  digitalWrite(onTargetB, HIGH);
  // digitalWrite(shortLEDA, HIGH);
  // digitalWrite(shortLEDB, HIGH);
  delay(1000);
  resetValues();
}
