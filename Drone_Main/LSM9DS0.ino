/*---------------------------------------------------------
    
    LSM9DS0.ino
    
      This file is a driver for a LSM9DS0 daughter-board 
      module, communicating via I2C. This driver reads 
      gyroscope, accelerometer, and magnetometer data from
      the module and does many of the calculations to 
      determine vehicle attitude and position.

---------------------------------------------------------*/

float X_A_CalibrationVal = 0;
float Y_A_CalibrationVal = 0;
float Z_A_CalibrationVal = 0;

float X_G_CalibrationVal = 0;
float Y_G_CalibrationVal = 0;
float Z_G_CalibrationVal = 0;

unsigned long last_PHR_Measurement = 0;

// Mahony Filter used to calculate pitch, roll, and heading in a dynamic 
// acceleration environment.
Adafruit_Mahony filter;

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
  
  // Load Calibration values from Non-Volatile Memory
  X_A_CalibrationVal = EEPROM.readFloat(MEM_ADDR_X_A_CALIBRATION_VAL);
  Y_A_CalibrationVal = EEPROM.readFloat(MEM_ADDR_Y_A_CALIBRATION_VAL);
  Z_A_CalibrationVal = EEPROM.readFloat(MEM_ADDR_Z_A_CALIBRATION_VAL);

  X_G_CalibrationVal = EEPROM.readFloat(MEM_ADDR_X_G_CALIBRATION_VAL);
  Y_G_CalibrationVal = EEPROM.readFloat(MEM_ADDR_Y_G_CALIBRATION_VAL);
  Z_G_CalibrationVal = EEPROM.readFloat(MEM_ADDR_Z_G_CALIBRATION_VAL);
  //Serial.print("Gyroscope Calibration: X: "); Serial.print(X_G_CalibrationVal); Serial.print(" Y: "); Serial.print(Y_G_CalibrationVal); Serial.print(" Z: "); Serial.println(Z_G_CalibrationVal);

  // Set PI gains for Mahony filter. Values experimentally acquired. (NOTE: WILL ALMOST SURELY NEED TO UPDATE THESE VALUES)
  filter.setKp(12);
  filter.setKi(5);

  // Initialize filter with data update rate in Hz.
  filter.begin(100);

}

/*---------------------------------------------------------
    
    LSM9DS0_ReadGyroscopeData()
    
      Reads out gyroscope X, Y, and Z data into a
      LSM9DS0_GyroData struct. Units are Degrees per second.
      A calibration value set by LSM9DS0_CalibrateGyroscope()
      is added to output data.

---------------------------------------------------------*/
void LSM9DS0_ReadGyroscopeData() {

  byte Out_X_G_Reg[2];
  byte Out_Y_G_Reg[2];
  byte Out_Z_G_Reg[2];
  int16_t X_G_Data;
  int16_t Y_G_Data;
  int16_t Z_G_Data;

  Wire.beginTransmission(LSM9DS0_GYRO_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_GYRO_BURST);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_GYRO_I2C_ADDR, 6);
  if (Wire.available()) {
    Out_X_G_Reg[0] = Wire.read();
    Out_X_G_Reg[1] = Wire.read();
    Out_Y_G_Reg[0] = Wire.read();
    Out_Y_G_Reg[1] = Wire.read();
    Out_Z_G_Reg[0] = Wire.read();
    Out_Z_G_Reg[1] = Wire.read();
  }
  Wire.endTransmission(true);

  X_G_Data = Out_X_G_Reg[1] << 8;
  X_G_Data += Out_X_G_Reg[0];

  Y_G_Data = Out_Y_G_Reg[1] << 8;
  Y_G_Data += Out_Y_G_Reg[0];

  Z_G_Data = Out_Z_G_Reg[1] << 8;
  Z_G_Data += Out_Z_G_Reg[0];

  GyroData.X_DegPerSec = (((float) X_G_Data / 32767.0) * GYROSCOPE_RANGE) + X_G_CalibrationVal;
  GyroData.Y_DegPerSec = (((float) Y_G_Data / 32767.0) * GYROSCOPE_RANGE) + Y_G_CalibrationVal;
  GyroData.Z_DegPerSec = (((float) Z_G_Data / 32767.0) * GYROSCOPE_RANGE) + Z_G_CalibrationVal;

  //Serial.print("X: "); Serial.print(X_G_Data); Serial.print("  Y: "); Serial.print(Y_G_Data); Serial.print("  Z: "); Serial.print(Z_G_Data); Serial.print("  X dps: "); Serial.print(GyroData.X_DegPerSec); Serial.print("  Y dps: "); Serial.print(GyroData.Y_DegPerSec); Serial.print("  Z dps: "); Serial.println(GyroData.Z_DegPerSec); 
}

/*---------------------------------------------------------
    
    LSM9DS0_ReadAccelerometerData()
    
      Reads out accelerometer X, Y, and Z data into a
      LSM9DS0_AccelData struct. Units are in G's.
      A calibration value set by 
      LSM9DS0_CalibrateAccelerometer() is added to output 
      data.

---------------------------------------------------------*/

void LSM9DS0_ReadAccelerometerData() {

  byte Out_X_A_Reg[2];
  byte Out_Y_A_Reg[2];
  byte Out_Z_A_Reg[2];
  int16_t X_A_Data;
  int16_t Y_A_Data;
  int16_t Z_A_Data;
  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_ACCEL_BURST);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 6);
  if (Wire.available()) {
    Out_X_A_Reg[0] = Wire.read();
    Out_X_A_Reg[1] = Wire.read();
    Out_Y_A_Reg[0] = Wire.read();
    Out_Y_A_Reg[1] = Wire.read();
    Out_Z_A_Reg[0] = Wire.read();
    Out_Z_A_Reg[1] = Wire.read();
  }
  Wire.endTransmission(true);
  
  X_A_Data = Out_X_A_Reg[1] << 8;
  X_A_Data += Out_X_A_Reg[0];

  Y_A_Data = Out_Y_A_Reg[1] << 8;
  Y_A_Data += Out_Y_A_Reg[0];

  Z_A_Data = Out_Z_A_Reg[1] << 8;
  Z_A_Data += Out_Z_A_Reg[0];

  // Get Acceleration for each axis by taking the int16_t value, dividing by the half size of 
  // the int_16t datatype, multiplying by the range of the accelerometer (4Gs for this drone),
  // then lastly adding the calibration value.
  AccelData.X_AccelInG = (((float)X_A_Data / 32767.0) * ACCELEROMETER_RANGE) + X_A_CalibrationVal;
  AccelData.Y_AccelInG = (((float)Y_A_Data / 32767.0) * ACCELEROMETER_RANGE) + Y_A_CalibrationVal;
  AccelData.Z_AccelInG = (((float)Z_A_Data / 32767.0) * ACCELEROMETER_RANGE) + Z_A_CalibrationVal;

  //Serial.print("X: "); Serial.print(X_A_Data); Serial.print("  Y: "); Serial.print(Y_A_Data); Serial.print("  Z: "); Serial.print(Z_A_Data); Serial.print("  X Gs: "); Serial.print(AccelData.X_AccelInG); Serial.print("  Y Gs: "); Serial.print(AccelData.Y_AccelInG); Serial.print("  Z Gs: "); Serial.println(AccelData.Z_AccelInG); 
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
    LSM9DS0_ReadGyroscopeData();
    sampleData[i] = GyroData;
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
    LSM9DS0_ReadAccelerometerData();
    sampleData[i] = AccelData;
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
      LSM9DS0_MagData struct. Units are in microTesla.

---------------------------------------------------------*/

void LSM9DS0_ReadMagnetometerData() {

  byte Out_X_M_Reg[2];
  byte Out_Y_M_Reg[2];
  byte Out_Z_M_Reg[2];
  int16_t X_M_Data;
  int16_t Y_M_Data;
  int16_t Z_M_Data;

  Wire.beginTransmission(LSM9DS0_ACCEL_MAG_I2C_ADDR);
  Wire.write(LSM9DS0_OUT_MAG_BURST);
  Wire.endTransmission();
  Wire.requestFrom(LSM9DS0_ACCEL_MAG_I2C_ADDR, 6);
  if (Wire.available()) {
    Out_X_M_Reg[0] = Wire.read();
    Out_X_M_Reg[1] = Wire.read();
    Out_Y_M_Reg[0] = Wire.read();
    Out_Y_M_Reg[1] = Wire.read();
    Out_Z_M_Reg[0] = Wire.read();
    Out_Z_M_Reg[1] = Wire.read();
  }
  Wire.endTransmission(true);

  X_M_Data = Out_X_M_Reg[1] << 8;
  X_M_Data += Out_X_M_Reg[0];

  Y_M_Data = Out_Y_M_Reg[1] << 8;
  Y_M_Data += Out_Y_M_Reg[0];

  Z_M_Data = Out_Z_M_Reg[1] << 8;
  Z_M_Data += Out_Z_M_Reg[0];

  MagData.X_FieldIn_uT = (((float) X_M_Data / 32767.0) * MAGNETOMETER_RANGE * 100);
  MagData.Y_FieldIn_uT = (((float) Y_M_Data / 32767.0) * MAGNETOMETER_RANGE * 100);
  MagData.Z_FieldIn_uT = (((float) Z_M_Data / 32767.0) * MAGNETOMETER_RANGE * 100);

  //Serial.print("X: "); Serial.print(X_M_Data); Serial.print("  Y: "); Serial.print(Y_M_Data); Serial.print("  Z: "); Serial.print(Z_M_Data); Serial.print("  X uT: "); Serial.print(MagData.X_FieldIn_uT); Serial.print("  Y uT: "); Serial.print(MagData.Y_FieldIn_uT); Serial.print("  Z uT: "); Serial.println(MagData.Z_FieldIn_uT); 

}

/*---------------------------------------------------------
    
    LSM9DS0_CalculateHeading()
    
      Calculates heading based off of magnetometer data.
      0-360 degree heading scale.

---------------------------------------------------------*/

float LSM9DS0_CalculateHeading() {
  float headingInDegrees;
  LSM9DS0_ReadMagnetometerData();

  headingInDegrees = atan((double)(MagData.X_FieldIn_uT/MagData.Y_FieldIn_uT)) * RADIANS_TO_DEGREE_CONV;

  // If in heading range of 0-90
  if (MagData.X_FieldIn_uT < 0 && MagData.Y_FieldIn_uT > 0) {
    headingInDegrees = 0.0 - headingInDegrees;
  }
  // If in heading range of 90-180
  else if (MagData.X_FieldIn_uT < 0 && MagData.Y_FieldIn_uT < 0) {
    headingInDegrees = 180.0 - headingInDegrees;
  }
  // If in heading range of 180-270
  else if (MagData.X_FieldIn_uT > 0 && MagData.Y_FieldIn_uT < 0) {
    headingInDegrees = 180 - headingInDegrees ;
  }
  // If in heading range of 270-360
  else {
    headingInDegrees = 360.0 - headingInDegrees;
  }

  //Serial.print("  X uT: "); Serial.print(MagData.X_FieldIn_uT); Serial.print("  Y uT: "); Serial.print(MagData.Y_FieldIn_uT); Serial.print("   "); Serial.println(headingInDegrees);

  return headingInDegrees;
}

/*---------------------------------------------------------
    
    LSM9DS0_CalculateFlightData()
    
      Calculates pitch, heading, and roll based off of 
      Accelerometer,Gyroscope, and Magnetometer data.

---------------------------------------------------------*/

void LSM9DS0_CalculateFlightData() {

  //float angleAccX, angleAccY;
  /* float sgZ = AccelData.Z_AccelInG<0 ? -1 : 1;

  angleAccX = atan2(AccelData.Y_AccelInG, sgZ*sqrt(AccelData.Z_AccelInG*AccelData.Z_AccelInG + AccelData.X_AccelInG*AccelData.X_AccelInG)) * RADIANS_TO_DEGREE_CONV;
  angleAccY = - atan2(AccelData.X_AccelInG,   sqrt(AccelData.Z_AccelInG*AccelData.Z_AccelInG + AccelData.Y_AccelInG*AccelData.Y_AccelInG)) * RADIANS_TO_DEGREE_CONV;

  unsigned long Tnew = millis();
  float dt = (Tnew - last_PHR_Measurement) * 1e-3;
  last_PHR_Measurement = Tnew;

  angleX = angleAccX + (angleX + GyroData.X_DegPerSec*dt - angleAccX);
  angleY = angleAccY + (angleY + GyroData.Y_DegPerSec*dt - angleAccY);
  angleZ = newHeading; */



/*
  FlightData.pitch = atan2(AccelData.Y_AccelInG, AccelData.Z_AccelInG)  * RADIANS_TO_DEGREE_CONV;
  FlightData.roll = -asin(AccelData.X_AccelInG)  * RADIANS_TO_DEGREE_CONV;
  
  FlightData.heading = LSM9DS0_CalculateHeading(); */

  filter.update(GyroData.X_DegPerSec, GyroData.Y_DegPerSec, GyroData.Z_DegPerSec, AccelData.X_AccelInG, AccelData.Y_AccelInG, AccelData.Z_AccelInG, MagData.X_FieldIn_uT, MagData.Y_FieldIn_uT, MagData.Z_FieldIn_uT);
  FlightData.pitch = filter.getRoll();
  FlightData.roll = filter.getPitch();
  FlightData.heading = filter.getYaw();



  //Serial.print(" pitch: "); Serial.print(FlightData.pitch); Serial.print(" roll: "); Serial.print(FlightData.roll); Serial.print(" heading: "); Serial.println(FlightData.heading); 
}