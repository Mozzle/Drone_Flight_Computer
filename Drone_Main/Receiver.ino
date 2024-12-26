/*---------------------------------------------------------
    
    Receiver.ino
    
      This file uses a non-blocking approach to measuring
      the incoming PWM signals from the RC receiver. The
      measured pulse-widths are mapped to a value range on
      a per-channel basis. The main output of this file is
      the receiverData struct. The Mode_Manager.ino
      file is the main consumer of the receiverData data. 

---------------------------------------------------------*/

/*----------------------------------------------------------------------

    Variables for interrupt functions for timing PWM signals. The 
    Arduino built-in pulseIn() function to measure PWM signals is a
    blocking function, and using that to measure 10 channels takes an
    impermissibly long amount of time.

----------------------------------------------------------------------*/

volatile unsigned long    CH1PulseBegin = 0;
volatile unsigned long    CH1PulseEnd = 0;
volatile bool             CH1NewPulseDurAvailable = false;

volatile unsigned long    CH2PulseBegin = 0;
volatile unsigned long    CH2PulseEnd = 0;
volatile bool             CH2NewPulseDurAvailable = false;

volatile unsigned long    CH3PulseBegin = 0;
volatile unsigned long    CH3PulseEnd = 0;
volatile bool             CH3NewPulseDurAvailable = false;

volatile unsigned long    CH4PulseBegin = 0;
volatile unsigned long    CH4PulseEnd = 0;
volatile bool             CH4NewPulseDurAvailable = false;

/* volatile unsigned long    CH5PulseBegin = 0;
volatile unsigned long    CH5PulseEnd = 0;
volatile bool             CH5NewPulseDurAvailable = false;

volatile unsigned long    CH6PulseBegin = 0;
volatile unsigned long    CH6PulseEnd = 0;
volatile bool             CH6NewPulseDurAvailable = false; */

volatile unsigned long    CH7PulseBegin = 0;
volatile unsigned long    CH7PulseEnd = 0;
volatile bool             CH7NewPulseDurAvailable = false;

volatile unsigned long    CH8PulseBegin = 0;
volatile unsigned long    CH8PulseEnd = 0;
volatile bool             CH8NewPulseDurAvailable = false;

volatile unsigned long    CH9PulseBegin = 0;
volatile unsigned long    CH9PulseEnd = 0;
volatile bool             CH9NewPulseDurAvailable = false;

/* volatile unsigned long    CH10PulseBegin = 0;
volatile unsigned long    CH10PulseEnd = 0;
volatile bool             CH10NewPulseDurAvailable = false; */

unsigned long curTime = 0;                      /* Current time, in ms                        */
// Ints to represent controller's stick positions, 3 position switch, and potentiometer values

/*----------------------------------------------------------------------

    Interrupt functions to measure PWM signals

----------------------------------------------------------------------*/

void IRAM_ATTR CH1Interrupt() {
  if (digitalRead(CH1) == HIGH) {
    // start measuring
    CH1PulseBegin = micros();
  }
  else {
    // stop measuring
    CH1PulseEnd = micros();
    CH1NewPulseDurAvailable = true;
  }
}
void IRAM_ATTR CH2Interrupt() {
  if (digitalRead(CH2) == HIGH) {
    // start measuring
    CH2PulseBegin = micros();
  }
  else {
    // stop measuring
    CH2PulseEnd = micros();
    CH2NewPulseDurAvailable = true;
  }
}
void IRAM_ATTR CH3Interrupt() {
  if (digitalRead(CH3) == HIGH) {
    // start measuring
    CH3PulseBegin = micros();
  }
  else {
    // stop measuring
    CH3PulseEnd = micros();
    CH3NewPulseDurAvailable = true;
  }
}
void IRAM_ATTR CH4Interrupt() {
  if (digitalRead(CH4) == HIGH) {
    // start measuring
    CH4PulseBegin = micros();
  }
  else {
    // stop measuring
    CH4PulseEnd = micros();
    CH4NewPulseDurAvailable = true;
  }
}
/* void IRAM_ATTR CH5Interrupt() {
  if (digitalRead(CH5) == HIGH) {
    // start measuring
    CH5PulseBegin = micros();
  }
  else {
    // stop measuring
    CH5PulseEnd = micros();
    CH5NewPulseDurAvailable = true;
  }
}
void IRAM_ATTR CH6Interrupt() {
  if (digitalRead(CH6) == HIGH) {
    // start measuring
    CH6PulseBegin = micros();
  }
  else {
    // stop measuring
    CH6PulseEnd = micros();
    CH6NewPulseDurAvailable = true;
  }
} */
void IRAM_ATTR CH7Interrupt() {
  if (digitalRead(CH7) == HIGH) {
    // start measuring
    CH7PulseBegin = micros();
  }
  else {
    // stop measuring
    CH7PulseEnd = micros();
    CH7NewPulseDurAvailable = true;
  }
}
void IRAM_ATTR CH8Interrupt() {
  if (digitalRead(CH8) == HIGH) {
    // start measuring
    CH8PulseBegin = micros();
  }
  else {
    // stop measuring
    CH8PulseEnd = micros();
    CH8NewPulseDurAvailable = true;
  }
}
void IRAM_ATTR CH9Interrupt() {
  if (digitalRead(CH9) == HIGH) {
    // start measuring
    CH9PulseBegin = micros();
  }
  else {
    // stop measuring
    CH9PulseEnd = micros();
    CH9NewPulseDurAvailable = true;
  }
}
/* void IRAM_ATTR CH10Interrupt() {
  if (digitalRead(CH10) == HIGH) {
    // start measuring
    CH10PulseBegin = micros();
  }
  else {
    // stop measuring
    CH10PulseEnd = micros();
    CH10NewPulseDurAvailable = true;
  }
} */

/*----------------------------------------------------------------------

    Receiver_Begin()

      Initialize pins and atttach interrupt functions for Receiver.ino

----------------------------------------------------------------------*/

void Receiver_Begin() {

// Set all pins as inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  //pinMode(CH5, INPUT);
  //pinMode(CH6, INPUT);
  pinMode(CH7, INPUT);
  pinMode(CH8, INPUT);
  pinMode(CH9, INPUT);
  //pinMode(CH10, INPUT);

// Attach interrupt callback functions to be called on high-low transitions.
  attachInterrupt(digitalPinToInterrupt(CH1), CH1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2), CH2Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH3), CH3Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH4), CH4Interrupt, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(CH5), CH5Interrupt, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(CH6), CH6Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH7), CH7Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH8), CH8Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH9), CH9Interrupt, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(CH10), CH10Interrupt, CHANGE);

}

/*----------------------------------------------------------------------

    updateReceiverData()

      If there has been a high-to-low transition on a PWM channel, then
      we have a new pulse width available and can map that width to a
      datatype representing the value of each channel. This function 
      MUST be called at least once every 20 milliseconds.

----------------------------------------------------------------------*/

void updateReceiverData() {
  if (CH1NewPulseDurAvailable) {                        // If a high->low transition has happened on the channel
    CH1NewPulseDurAvailable = false;                      // Reset flag
    if ((CH1PulseEnd - CH1PulseBegin) > 0) {                // If we have properly measured a pulse/don't have a stale PulseEnd value for some reason
      receiverData.ch1Value = map((CH1PulseEnd - CH1PulseBegin), 993, 1992, -200, 200);   // Map the pulse width to a value.
    }
    else {
      //Serial.println((CH1PulseEnd - CH1PulseBegin));
    }
    //Serial.println(CH1PulseEnd - CH1PulseBegin);
  }
  if (CH2NewPulseDurAvailable) {
    CH2NewPulseDurAvailable = false;
    if ((CH2PulseEnd - CH2PulseBegin) < 2000 && (CH2PulseEnd - CH2PulseBegin) > 0) {
      receiverData.ch2Value = map((CH2PulseEnd - CH2PulseBegin), 990, 1990, -200, 200);
    }
    //Serial.println(CH2PulseEnd - CH2PulseBegin);
  }
  if (CH3NewPulseDurAvailable) {
    CH3NewPulseDurAvailable = false;
    if ((CH3PulseEnd - CH3PulseBegin) > 0) {
      receiverData.ch3Value = map((CH3PulseEnd - CH3PulseBegin), 994, 1991, 0, PWM_MAX_VALUE);
    }
    //Serial.println(CH3PulseEnd - CH3PulseBegin);
  }
  if (CH4NewPulseDurAvailable) {
    CH4NewPulseDurAvailable = false;
    if ((CH4PulseEnd - CH4PulseBegin) > 0) {
      receiverData.ch4Value = map((CH4PulseEnd - CH4PulseBegin), 993, 1993, -200, 200);
    }
    //Serial.println(CH4PulseEnd - CH4PulseBegin);
  }
 /* if (CH5NewPulseDurAvailable) {
    CH5NewPulseDurAvailable = false;
    if ((CH5PulseEnd - CH5PulseBegin) > 0) {
      receiverData.ch5Value = map((CH5PulseEnd - CH5PulseBegin), 1000, 2000, 0, 100);
    }
    //Serial.println(CH5PulseEnd - CH5PulseBegin);
  }
  if (CH6NewPulseDurAvailable) {
    CH6NewPulseDurAvailable = false;
    if ((CH6PulseEnd - CH6PulseBegin) > 0) {
      receiverData.ch6Value = map((CH6PulseEnd - CH6PulseBegin), 995, 1995, 0, 100);
    }
    //Serial.println(CH6PulseEnd - CH6PulseBegin);
  } */
  if (CH7NewPulseDurAvailable) {
    CH7NewPulseDurAvailable = false;
    if ((CH7PulseEnd - CH7PulseBegin) > 0) {
      if ((CH7PulseEnd - CH7PulseBegin) < 1500) {
        receiverData.ch7Value = 0; //Switch flipped up (off)
      }
      else {
        receiverData.ch7Value = 1; //Switch flipped down (on)
      }
    }
    //Serial.println(CH7PulseEnd - CH7PulseBegin);
  }
  if (CH8NewPulseDurAvailable) {
    CH8NewPulseDurAvailable = false;
    if ((CH8PulseEnd - CH8PulseBegin) > 0) {
      if ((CH8PulseEnd - CH8PulseBegin) < 1500) {
        receiverData.ch8Value = 0; //Switch flipped up (off)
      }
      else {
        receiverData.ch8Value = 1; //Switch flipped down (on)
      }
    }
    //Serial.println(CH8PulseEnd - CH8PulseBegin);
  }
  if (CH9NewPulseDurAvailable) {
    CH9NewPulseDurAvailable = false;
    if ((CH9PulseEnd - CH9PulseBegin) > 0) {
      if ((CH9PulseEnd - CH9PulseBegin) < 1250) {
        receiverData.ch9Value = 0; //Switch flipped up (off)
      }
      else if ((CH9PulseEnd - CH9PulseBegin) < 1750) {
        receiverData.ch9Value = 1; //Switch flipped to middle state
      }
      else {
        receiverData.ch9Value = 2; //Switch flipped down (on)
      }
    }
    //Serial.println(CH9PulseEnd - CH9PulseBegin);
  }
 /* if (CH10NewPulseDurAvailable) {
    CH10NewPulseDurAvailable = false;
    if ((CH10PulseEnd - CH10PulseBegin) > 0) {
      if ((CH10PulseEnd - CH10PulseBegin) < 1500) {
        receiverData.ch10Value = 0; //Switch flipped up (off)
      }
      else {
        receiverData.ch10Value = 1; //Switch flipped down (on)
      }
    }
    //Serial.println(CH10PulseEnd - CH10PulseBegin);
  } */
  //Serial.print("CH1: "); Serial.print(receiverData.ch1Value); Serial.print(" CH2: "); Serial.print(receiverData.ch2Value); Serial.print(" CH3: "); Serial.print(receiverData.ch3Value); Serial.print(" CH4: "); Serial.print(receiverData.ch4Value); /* Serial.print(" CH5: "); Serial.print(receiverData.ch5Value); 
  //Serial.print(" CH6: "); Serial.print(receiverData.ch6Value); */ Serial.print(" CH7: "); Serial.print(receiverData.ch7Value); Serial.print(" CH8: "); Serial.print(receiverData.ch8Value); Serial.print(" CH9: "); Serial.print(receiverData.ch9Value); /* Serial.print(" CH10: "); Serial.println(receiverData.ch10Value); */ 

}