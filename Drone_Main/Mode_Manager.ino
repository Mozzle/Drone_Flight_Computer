
const float PITCH_THROTTLE_COEFF = 0.2;
const float ROLL_THROTTLE_COEFF = 0.2;
const float YAW_THROTTLE_COEFF = 0.1;

void Mode_Manager_Begin() {
  pinMode(PIN_MOTOR_FL, OUTPUT);
  pinMode(PIN_MOTOR_FR, OUTPUT);
  pinMode(PIN_MOTOR_BL, OUTPUT);
  pinMode(PIN_MOTOR_BR, OUTPUT);

  ledcAttachPin(PIN_MOTOR_FL, CHANNEL_MOTOR_FL); 
  ledcAttachPin(PIN_MOTOR_FR, CHANNEL_MOTOR_FR); 
  ledcAttachPin(PIN_MOTOR_BL, CHANNEL_MOTOR_BL); 
  ledcAttachPin(PIN_MOTOR_BR, CHANNEL_MOTOR_BR); 

  ledcSetup(CHANNEL_MOTOR_FL, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcSetup(CHANNEL_MOTOR_FR, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcSetup(CHANNEL_MOTOR_BL, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcSetup(CHANNEL_MOTOR_BR, PWM_FREQUENCY, PWM_RESOLUTION_BITS);

}

void Mode_Manager() {
  
  // SWITCH CONDITIONS FOR NO FLIGHT MODE
  if (receiverData.ch7Value == 0 && receiverData.ch8Value == 0 && receiverData.ch9Value == 0) {
    // Set throttle = 0, perhaps after checking that Z acceleration is ~1.0
  }

  // SWITCH CONDITIONS FOR FPV FLIGHT MODE
  if (receiverData.ch7Value == 1 && receiverData.ch8Value == 0 && receiverData.ch9Value == 0) {

    throttleOutput.motor_FL = receiverData.ch3Value - (int)(receiverData.ch2Value * PITCH_THROTTLE_COEFF) + (int)(receiverData.ch1Value * ROLL_THROTTLE_COEFF) - (int)(receiverData.ch4Value * YAW_THROTTLE_COEFF);
    throttleOutput.motor_FR = receiverData.ch3Value - (int)(receiverData.ch2Value * PITCH_THROTTLE_COEFF) - (int)(receiverData.ch1Value * ROLL_THROTTLE_COEFF) + (int)(receiverData.ch4Value * YAW_THROTTLE_COEFF);
    throttleOutput.motor_BL = receiverData.ch3Value + (int)(receiverData.ch2Value * PITCH_THROTTLE_COEFF) + (int)(receiverData.ch1Value * ROLL_THROTTLE_COEFF) + (int)(receiverData.ch4Value * YAW_THROTTLE_COEFF);
    throttleOutput.motor_BR = receiverData.ch3Value + (int)(receiverData.ch2Value * PITCH_THROTTLE_COEFF) - (int)(receiverData.ch1Value * ROLL_THROTTLE_COEFF) - (int)(receiverData.ch4Value * YAW_THROTTLE_COEFF);

    // If the throttle stick is pressed down, kill throttle
    if (receiverData.ch3Value <= 15) {
      killAllThrottle();
    }
  }

  // SWITCH CONDITIONS FOR HOVER FLIGHT MODE
  if (receiverData.ch7Value == 0 && receiverData.ch8Value == 1 && receiverData.ch9Value == 0) {
    
  }

  clampThrottle();
  ledcWrite(CHANNEL_MOTOR_FL, throttleOutput.motor_FL);
  ledcWrite(CHANNEL_MOTOR_FR, throttleOutput.motor_FR);
  ledcWrite(CHANNEL_MOTOR_BL, throttleOutput.motor_BL);
  ledcWrite(CHANNEL_MOTOR_BR, throttleOutput.motor_BR);

  Serial.print("FL: "); Serial.print(throttleOutput.motor_FL); Serial.print(" FR: "); Serial.print(throttleOutput.motor_FR); Serial.print(" BL: "); Serial.print(throttleOutput.motor_BL); Serial.print(" BR: "); Serial.println(throttleOutput.motor_BR); 

}

void killAllThrottle() {
  throttleOutput.motor_FL = 0;
  throttleOutput.motor_FR = 0;
  throttleOutput.motor_BL = 0;
  throttleOutput.motor_BR = 0;
}

void clampThrottle() {
  
  if (throttleOutput.motor_FL < 0) {
    throttleOutput.motor_FL = 0;
  }
  else if (throttleOutput.motor_FL > PWM_MAX_VALUE) {
    throttleOutput.motor_FL = PWM_MAX_VALUE;
  }

  if (throttleOutput.motor_FR < 0) {
    throttleOutput.motor_FR = 0;
  }
  else if (throttleOutput.motor_FR > PWM_MAX_VALUE) {
    throttleOutput.motor_FR = PWM_MAX_VALUE;
  }

  if (throttleOutput.motor_BL < 0) {
    throttleOutput.motor_BL = 0;
  }
  else if (throttleOutput.motor_BL > PWM_MAX_VALUE) {
    throttleOutput.motor_BL = PWM_MAX_VALUE;
  }

  if (throttleOutput.motor_BR < 0) {
    throttleOutput.motor_BR = 0;
  }
  else if (throttleOutput.motor_BR > PWM_MAX_VALUE) {
    throttleOutput.motor_BR = PWM_MAX_VALUE;
  }

  throttleOutput.motor_FL += 75;
  throttleOutput.motor_FR += 75;
  throttleOutput.motor_BL += 75;
  throttleOutput.motor_BR += 75;

}