
const float PITCH_THROTTLE_COEFF = 0.2;
const float ROLL_THROTTLE_COEFF = 0.2;
const float YAW_THROTTLE_COEFF = 0.1;

// PID gains for Hover Flight Mode. VALUES HERE ARE RANDOM AND HAVE NOT BEEN EXPERIMENTALLY TUNED
const float hover_pitch_kp = 0.8;
const float hover_pitch_ki = 0.20;
const float hover_pitch_kd = 0.01;

const float hover_roll_kp = 0.8;
const float hover_roll_ki = 0.20;
const float hover_roll_kd = 0.01;

float pitchSetpoint = 0.0;
float rollSetpoint = 0.0;
float lastPitchError = 0.0;
float lastRollError = 0.0;

unsigned long PID_now = 0;
unsigned long PID_last_time = 0;
float dt;
float PID_PitchIntegral, PID_RollIntegral;

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
  /*----------------------------------------------------------------------
  // SWITCH CONDITIONS FOR NO FLIGHT MODE
  ----------------------------------------------------------------------*/
  if (receiverData.ch7Value == 0 && receiverData.ch8Value == 0 && receiverData.ch9Value == 0) {
    // Set throttle = 0, perhaps after checking that Z acceleration is ~1.0
  }

  /*----------------------------------------------------------------------
  // 
  // SWITCH CONDITIONS FOR FPV FLIGHT MODE
  // 
  ----------------------------------------------------------------------*/
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

  /*----------------------------------------------------------------------
  // 
  // SWITCH CONDITIONS FOR HOVER FLIGHT MODE
  // 
  ----------------------------------------------------------------------*/
  if (receiverData.ch7Value == 0 && receiverData.ch8Value == 1 && receiverData.ch9Value == 0) {

    if (abs(receiverData.ch2Value) > 30) {
      pitchSetpoint = map(receiverData.ch2Value, -200, 200, -25, 25);
    }
    else {
      pitchSetpoint = 0;
    }

    if (abs(receiverData.ch1Value) > 30) {
      rollSetpoint = map(receiverData.ch1Value, -200, 200, -25, 25);
    }
    else {
      rollSetpoint = 0;
    }
    

    // PID Loop for targeting a pitch and roll value.
    float pitchOutput, rollOutput;
    float pitchError, rollError;
    float PitchProportional, PitchDerivative;
    float RollProportional, RollDerivative;
    
    PID_now = millis();
    dt = (PID_now - PID_last_time) * 0.001; //dt in seconds
    PID_last_time = PID_now;

    if (abs(FlightData.pitch) > 0.4) {
      pitchError = pitchSetpoint - FlightData.pitch;
    }
    else {
      pitchError = 0;
    }

    if (abs(FlightData.roll) > 0.4) {
      rollError = rollSetpoint - FlightData.roll;
    }
    else {
      rollError = 0;
    }


    PitchProportional = pitchError;
    RollProportional  = rollError;

    PID_PitchIntegral += pitchError * dt;
    PID_RollIntegral  += rollError * dt;

    PitchDerivative = (pitchError - lastPitchError) / dt;
    RollDerivative = (rollError - lastRollError) / dt;
    lastPitchError = pitchError;
    lastRollError = rollError;

    rollOutput  = (hover_roll_kp * RollProportional) + (hover_roll_ki * PID_RollIntegral) + (hover_roll_kd * RollDerivative);
    pitchOutput = (hover_pitch_kp * PitchProportional) + (hover_pitch_ki * PID_PitchIntegral) + (hover_pitch_kd * PitchDerivative);

    Serial.print(" pitch: "); Serial.print(FlightData.pitch); Serial.print(" roll: "); Serial.print(FlightData.roll); Serial.print("  roll Set: "); Serial.print(rollSetpoint); Serial.print(" pitch Set: "); Serial.print(pitchSetpoint); Serial.print("  roll Out: "); Serial.print(rollOutput); Serial.print(" pitch Out: "); Serial.println(pitchOutput); 

  }

  // 
  clampThrottle();
  ledcWrite(CHANNEL_MOTOR_FL, throttleOutput.motor_FL);
  ledcWrite(CHANNEL_MOTOR_FR, throttleOutput.motor_FR);
  ledcWrite(CHANNEL_MOTOR_BL, throttleOutput.motor_BL);
  ledcWrite(CHANNEL_MOTOR_BR, throttleOutput.motor_BR);

  //Serial.print("FL: "); Serial.print(throttleOutput.motor_FL); Serial.print(" FR: "); Serial.print(throttleOutput.motor_FR); Serial.print(" BL: "); Serial.print(throttleOutput.motor_BL); Serial.print(" BR: "); Serial.println(throttleOutput.motor_BR); 

}

void killAllThrottle() {
  throttleOutput.motor_FL = 0;
  throttleOutput.motor_FR = 0;
  throttleOutput.motor_BL = 0;
  throttleOutput.motor_BR = 0;
}

void clampThrottle() {
  
  if (throttleOutput.motor_FL < PWM_MIN_VALUE) {
    throttleOutput.motor_FL = PWM_MIN_VALUE;
  }
  else if (throttleOutput.motor_FL > PWM_MAX_VALUE) {
    throttleOutput.motor_FL = PWM_MAX_VALUE;
  }

  if (throttleOutput.motor_FR < PWM_MIN_VALUE) {
    throttleOutput.motor_FR = PWM_MIN_VALUE;
  }
  else if (throttleOutput.motor_FR > PWM_MAX_VALUE) {
    throttleOutput.motor_FR = PWM_MAX_VALUE;
  }

  if (throttleOutput.motor_BL < PWM_MIN_VALUE) {
    throttleOutput.motor_BL = PWM_MIN_VALUE;
  }
  else if (throttleOutput.motor_BL > PWM_MAX_VALUE) {
    throttleOutput.motor_BL = PWM_MAX_VALUE;
  }

  if (throttleOutput.motor_BR < PWM_MIN_VALUE) {
    throttleOutput.motor_BR = PWM_MIN_VALUE;
  }
  else if (throttleOutput.motor_BR > PWM_MAX_VALUE) {
    throttleOutput.motor_BR = PWM_MAX_VALUE;
  }


}