float X_A_CalibrationVal = 0;
float Y_A_CalibrationVal = 0;
float Z_A_CalibrationVal = 0;

float X_G_CalibrationVal = 0;
float Y_G_CalibrationVal = 0;
float Z_G_CalibrationVal = 0;

unsigned long last_PHR_Measurement = 0;

float angleX = 0;
float angleY = 0;
float angleZ = 0;

void LSM9DS0_Begin() {

  Wire.beginTransmission(LSM9DS0_GYRO_I2C_ADDR);
  Wire.write(LSM9DS0_CTRL_REG1_G);
  Wire.write(0b01111111);
  Wire.endTransmission();

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_CTRL_REG1_XM);
  Wire.write(0b01100111);
  Wire.endTransmission();

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_CTRL_REG2_XM);
  Wire.write(0b00001000);
  Wire.endTransmission();

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_CTRL_REG5_XM);
  Wire.write(0b11110000);
  Wire.endTransmission();

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_CTRL_REG7_XM);
  Wire.write(0b00000000);
  Wire.endTransmission();

 /* Wire.beginTransmission(LSM9DS0_GYRO_I2C_ADDR);
  Wire.write(LSM9DS0_CTRL_REG2_G);
  Wire.write(0b00100011);
  Wire.endTransmission(); */

  // High pass filter is not engaged yet. Needs to be enabled in CTRL_REG5_G:

  /*Wire.beginTransmission(LSM9DS0_GYRO_I2C_ADDR);
  Wire.write(LSM9DS0_CTRL_REG5_G);
  Wire.write(0b000);
  Wire.endTransmission(); */

  // Do XM setup as well
  
  // Load Calibration values from Non-Volatile Memory
  X_A_CalibrationVal = EEPROM.readFloat(MEM_ADDR_X_A_CALIBRATION_VAL);
  Y_A_CalibrationVal = EEPROM.readFloat(MEM_ADDR_Y_A_CALIBRATION_VAL);
  Z_A_CalibrationVal = EEPROM.readFloat(MEM_ADDR_Z_A_CALIBRATION_VAL);

  X_G_CalibrationVal = EEPROM.readFloat(MEM_ADDR_X_G_CALIBRATION_VAL);
  Y_G_CalibrationVal = EEPROM.readFloat(MEM_ADDR_Y_G_CALIBRATION_VAL);
  Z_G_CalibrationVal = EEPROM.readFloat(MEM_ADDR_Z_G_CALIBRATION_VAL);
  Serial.print("Gyroscope Calibration: X: "); Serial.print(X_G_CalibrationVal); Serial.print(" Y: "); Serial.print(Y_G_CalibrationVal); Serial.print(" Z: "); Serial.println(Z_G_CalibrationVal);

}

/*---------------------------------------------------------
    
    LSM9DS0_ReadGyroscopeData()
    
      Reads out gyroscope X, Y, and Z data into a
      LSM9DS0_GyroData struct. Units are Degrees per second.
      A calibration value set by LSM9DS0_CalibrateGyroscope()
      is added to output data.

---------------------------------------------------------*/
struct LSM9DS0_GyroData LSM9DS0_ReadGyroscopeData() {
  struct LSM9DS0_GyroData G_Data;

  byte Out_X_G_Reg[2];
  byte Out_Y_G_Reg[2];
  byte Out_Z_G_Reg[2];
  int16_t X_G_Data;
  int16_t Y_G_Data;
  int16_t Z_G_Data;

  Wire.beginTransmission(LSM9DS0_GYRO_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_X_L_G);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_GYRO_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_X_G_Reg[0] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_GYRO_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_X_H_G);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_GYRO_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_X_G_Reg[1] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_GYRO_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_Y_L_G);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_GYRO_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_Y_G_Reg[0] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_GYRO_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_Y_H_G);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_GYRO_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_Y_G_Reg[1] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_GYRO_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_Z_L_G);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_GYRO_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_Z_G_Reg[0] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_GYRO_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_Z_H_G);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_GYRO_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_Z_G_Reg[1] = Wire.read();
  }

  X_G_Data = Out_X_G_Reg[1] << 8;
  X_G_Data += Out_X_G_Reg[0];

  Y_G_Data = Out_Y_G_Reg[1] << 8;
  Y_G_Data += Out_Y_G_Reg[0];

  Z_G_Data = Out_Z_G_Reg[1] << 8;
  Z_G_Data += Out_Z_G_Reg[0];

  G_Data.X_DegPerSec = (((float) X_G_Data / 32767.0) * GYROSCOPE_RANGE) + X_G_CalibrationVal;
  G_Data.Y_DegPerSec = (((float) Y_G_Data / 32767.0) * GYROSCOPE_RANGE) + Y_G_CalibrationVal;
  G_Data.Z_DegPerSec = (((float) Z_G_Data / 32767.0) * GYROSCOPE_RANGE) + Z_G_CalibrationVal;

  //Serial.print("X: "); Serial.print(X_G_Data); Serial.print("  Y: "); Serial.print(Y_G_Data); Serial.print("  Z: "); Serial.print(Z_G_Data); Serial.print("  X dps: "); Serial.print(G_Data.X_DegPerSec); Serial.print("  Y dps: "); Serial.print(G_Data.Y_DegPerSec); Serial.print("  Z dps: "); Serial.println(G_Data.Z_DegPerSec); 

  return G_Data;
}

/*---------------------------------------------------------
    
    LSM9DS0_ReadAccelerometerData()
    
      Reads out accelerometer X, Y, and Z data into a
      LSM9DS0_AccelData struct. Units are in G's.
      A calibration value set by 
      LSM9DS0_CalibrateAccelerometer() is added to output 
      data.

---------------------------------------------------------*/

struct LSM9DS0_AccelData LSM9DS0_ReadAccelerometerData() {
  struct LSM9DS0_AccelData A_Data;

  byte Out_X_A_Reg[2];
  byte Out_Y_A_Reg[2];
  byte Out_Z_A_Reg[2];
  int16_t X_A_Data;
  int16_t Y_A_Data;
  int16_t Z_A_Data;

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_X_L_A);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_X_A_Reg[0] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_X_H_A);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_X_A_Reg[1] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_Y_L_A);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_Y_A_Reg[0] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_Y_H_A);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_Y_A_Reg[1] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_Z_L_A);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_Z_A_Reg[0] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_Z_H_A);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_Z_A_Reg[1] = Wire.read();
  }

  
  X_A_Data = Out_X_A_Reg[1] << 8;
  X_A_Data += Out_X_A_Reg[0];

  Y_A_Data = Out_Y_A_Reg[1] << 8;
  Y_A_Data += Out_Y_A_Reg[0];

  Z_A_Data = Out_Z_A_Reg[1] << 8;
  Z_A_Data += Out_Z_A_Reg[0];

  // Get Acceleration for each axis by taking the int16_t value, dividing by the half size of 
  // the int_16t datatype, multiplying by the range of the accelerometer (4Gs for this drone),
  // then lastly adding the calibration value.
  A_Data.X_AccelInG = (((float)X_A_Data / 32767.0) * ACCELEROMETER_RANGE) + X_A_CalibrationVal;
  A_Data.Y_AccelInG = (((float)Y_A_Data / 32767.0) * ACCELEROMETER_RANGE) + Y_A_CalibrationVal;
  A_Data.Z_AccelInG = (((float)Z_A_Data / 32767.0) * ACCELEROMETER_RANGE) + Z_A_CalibrationVal;

  //Serial.print("X: "); Serial.print(X_A_Data); Serial.print("  Y: "); Serial.print(Y_A_Data); Serial.print("  Z: "); Serial.print(Z_A_Data); Serial.print("  X Gs: "); Serial.print(A_Data.X_AccelInG); Serial.print("  Y Gs: "); Serial.print(A_Data.Y_AccelInG); Serial.print("  Z Gs: "); Serial.println(A_Data.Z_AccelInG); 

  return A_Data;
}

/*---------------------------------------------------------
    
    LSM9DS0_CalibrateGyroscope()
    
      Called when the quadcopter is not moving/on a test
      stand, calibration takes the mean of 10 gyroscope
      measurements and sets a calibration value. Calibration
      values are stored in EEPROM to be fetched at startup.

---------------------------------------------------------*/

void LSM9DS0_CalibrateGyroscope() {

  struct LSM9DS0_GyroData sampleData[10];
  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;

  X_G_CalibrationVal = 0;
  Y_G_CalibrationVal = 0;
  Z_G_CalibrationVal = 0;

  Serial.println("Calibrating Gyroscope!");

  for (int i = 0; i < 10; i++) {
    sampleData[i] = LSM9DS0_ReadGyroscopeData();
    delay(2);
  }

  for (int i = 0; i < 10; i++) {
    sumX += sampleData[i].X_DegPerSec;
    sumY += sampleData[i].Y_DegPerSec;
    sumZ += sampleData[i].Z_DegPerSec;
  }

  X_G_CalibrationVal = 0 - (sumX / 10.0);
  Y_G_CalibrationVal = 0 - (sumY / 10.0);
  Z_G_CalibrationVal = 0 - (sumZ / 10.0);

  EEPROM.writeFloat(MEM_ADDR_X_G_CALIBRATION_VAL, X_G_CalibrationVal);
  EEPROM.writeFloat(MEM_ADDR_Y_G_CALIBRATION_VAL, Y_G_CalibrationVal);
  EEPROM.writeFloat(MEM_ADDR_Z_G_CALIBRATION_VAL, Z_G_CalibrationVal);
  EEPROM.commit();

  Serial.print("Gyroscope Calibration: X: "); Serial.print(X_G_CalibrationVal); Serial.print(" Y: "); Serial.print(Y_G_CalibrationVal); Serial.print(" Z: "); Serial.println(Z_G_CalibrationVal);
}

/*---------------------------------------------------------
    
    LSM9DS0_CalibrateAccelerometer()
    
      Called when the quadcopter is not moving/on a test
      stand, calibration takes the mean of 10 accelerometer
      measurements and sets a calibration value. Calibration
      values are stored in EEPROM to be fetched at startup.

---------------------------------------------------------*/

void LSM9DS0_CalibrateAccelerometer() {

  struct LSM9DS0_AccelData sampleData[10];
  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;

  X_A_CalibrationVal = 0;
  Y_A_CalibrationVal = 0;
  Z_A_CalibrationVal = 0;

  //Serial.println("Calibrating Accelerometer!");

  for (int i = 0; i < 10; i++) {
    sampleData[i] = LSM9DS0_ReadAccelerometerData();
    delay(2);
  }

  for (int i = 0; i < 10; i++) {
    sumX += sampleData[i].X_AccelInG;
    sumY += sampleData[i].Y_AccelInG;
    sumZ += sampleData[i].Z_AccelInG;
  }

  X_A_CalibrationVal = 0 - (sumX / 10.0);
  Y_A_CalibrationVal = 0 - (sumY / 10.0);
  Z_A_CalibrationVal = 1 - (sumZ / 10.0);

  EEPROM.writeFloat(MEM_ADDR_X_A_CALIBRATION_VAL, X_A_CalibrationVal);
  EEPROM.writeFloat(MEM_ADDR_Y_A_CALIBRATION_VAL, Y_A_CalibrationVal);
  EEPROM.writeFloat(MEM_ADDR_Z_A_CALIBRATION_VAL, Z_A_CalibrationVal);
  EEPROM.commit();

  //Serial.print("Accelerometer Calibration: X: "); Serial.print(X_A_CalibrationVal); Serial.print(" Y: "); Serial.print(Y_A_CalibrationVal); Serial.print(" Z: "); Serial.println(Z_A_CalibrationVal);
}

/*---------------------------------------------------------
    
    LSM9DS0_ReadMagnetometerData()
    
      Reads out magnetometer X, Y, and Z data into a
      LSM9DS0_MagData struct. Units are in Gauss.

---------------------------------------------------------*/

struct LSM9DS0_MagData LSM9DS0_ReadMagnetometerData() {
  struct LSM9DS0_MagData M_Data;

  byte Out_X_M_Reg[2];
  byte Out_Y_M_Reg[2];
  byte Out_Z_M_Reg[2];
  int16_t X_M_Data;
  int16_t Y_M_Data;
  int16_t Z_M_Data;

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_X_L_M);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_X_M_Reg[0] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_X_H_M);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_X_M_Reg[1] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_Y_L_M);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_Y_M_Reg[0] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_Y_H_M);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_Y_M_Reg[1] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_Z_L_M);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_Z_M_Reg[0] = Wire.read();
  }

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_Z_H_M);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 1);
  if (Wire.available()) {
    Out_Z_M_Reg[1] = Wire.read();
  }

  X_M_Data = Out_X_M_Reg[1] << 8;
  X_M_Data += Out_X_M_Reg[0];

  Y_M_Data = Out_Y_M_Reg[1] << 8;
  Y_M_Data += Out_Y_M_Reg[0];

  Z_M_Data = Out_Z_M_Reg[1] << 8;
  Z_M_Data += Out_Z_M_Reg[0];

  M_Data.X_FieldInGauss = (((float) X_M_Data / 32767.0) * MAGNETOMETER_RANGE);
  M_Data.Y_FieldInGauss = (((float) Y_M_Data / 32767.0) * MAGNETOMETER_RANGE);
  M_Data.Z_FieldInGauss = (((float) Z_M_Data / 32767.0) * MAGNETOMETER_RANGE);

  //Serial.print("X: "); Serial.print(X_M_Data); Serial.print("  Y: "); Serial.print(Y_M_Data); Serial.print("  Z: "); Serial.print(Z_M_Data); Serial.print("  X gauss: "); Serial.print(M_Data.X_FieldInGauss); Serial.print("  Y gauss: "); Serial.print(M_Data.Y_FieldInGauss); Serial.print("  Z gauss: "); Serial.println(M_Data.Z_FieldInGauss); 

  return M_Data;
}

/*---------------------------------------------------------
    
    LSM9DS0_ReadMagnetometerData()
    
      Calculates heading based off of magnetometer data.
      0-360 degree heading scale.

---------------------------------------------------------*/

float LSM9DS0_CalculateHeading() {
  float headingInDegrees;
  struct LSM9DS0_MagData M_Data = LSM9DS0_ReadMagnetometerData();

  headingInDegrees = atan((double)(M_Data.X_FieldInGauss/M_Data.Y_FieldInGauss)) * RADIANS_TO_DEGREE_CONV;

  // If in heading range of 0-90
  if (M_Data.X_FieldInGauss < 0 && M_Data.Y_FieldInGauss > 0) {
    headingInDegrees = 0.0 - headingInDegrees;
  }
  // If in heading range of 90-180
  else if (M_Data.X_FieldInGauss < 0 && M_Data.Y_FieldInGauss < 0) {
    headingInDegrees = 180.0 - headingInDegrees;
  }
  // If in heading range of 180-270
  else if (M_Data.X_FieldInGauss > 0 && M_Data.Y_FieldInGauss < 0) {
    headingInDegrees = 180 - headingInDegrees ;
  }
  // If in heading range of 270-360
  else {
    headingInDegrees = 360.0 - headingInDegrees;
  }

  //Serial.print("  X gauss: "); Serial.print(M_Data.X_FieldInGauss); Serial.print("  Y gauss: "); Serial.print(M_Data.Y_FieldInGauss); Serial.print("   "); Serial.println(headingInDegrees);

  return headingInDegrees;
}

/*---------------------------------------------------------
    
    LSM9DS0_CalculateFlightData()
    
      Calculates pitch, heading, and roll based off of 
      Accelerometer,Gyroscope, and Magnetometer data.
      **WIP** function not working properly right now

---------------------------------------------------------*/

void LSM9DS0_CalculateFlightData() {

  float newHeading = LSM9DS0_CalculateHeading();
  struct LSM9DS0_AccelData A_Data = LSM9DS0_ReadAccelerometerData();
  struct LSM9DS0_GyroData G_Data = LSM9DS0_ReadGyroscopeData();

  float angleAccX, angleAccY;
  float sgZ = A_Data.Z_AccelInG<0 ? -1 : 1;

  angleAccX = atan2(A_Data.Y_AccelInG, sgZ*sqrt(A_Data.Z_AccelInG*A_Data.Z_AccelInG + A_Data.X_AccelInG*A_Data.X_AccelInG)) * RADIANS_TO_DEGREE_CONV;
  angleAccY = - atan2(A_Data.X_AccelInG,   sqrt(A_Data.Z_AccelInG*A_Data.Z_AccelInG + A_Data.Y_AccelInG*A_Data.Y_AccelInG)) * RADIANS_TO_DEGREE_CONV;

  unsigned long Tnew = millis();
  float dt = (Tnew - last_PHR_Measurement) * 1e-3;
  last_PHR_Measurement = Tnew;

  angleX = angleAccX + (angleX + G_Data.X_DegPerSec*dt - angleAccX);
  angleY = angleAccY + (angleY + G_Data.Y_DegPerSec*dt - angleAccY);
  angleZ = newHeading;

  Serial.print("angleAccX: "); Serial.print(angleAccX); Serial.print(" angleAccY: "); Serial.print(angleAccY); Serial.print(" angleX: "); Serial.print(angleX); Serial.print(" angleY: "); Serial.print(angleY); Serial.print(" angleZ: "); Serial.println(angleZ); 
}