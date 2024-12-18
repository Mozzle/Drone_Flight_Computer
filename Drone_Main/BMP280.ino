/*---------------------------------------------------------
    
    BMP280.ino
    
      This file is a driver for a BMP280 daughter-board 
      module, communicating via I2C. This driver reads 
      temperature and pressure data from the BMP280,
      converts the data into usable units, and calculates
      barometric altitude.

---------------------------------------------------------*/

int32_t temp_fine;

// Calibration Values
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;

uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;

unsigned long timeOfLastTempReading = 0;
unsigned long timeOfLastPressureReading = 0;

void BMP280_Begin() {
  byte i = 0;
  byte calibrationData[24];       // Calibration data is 24 bytes long

  // Write config data
  Wire.beginTransmission(BMP280_I2C_ADDR);
  Wire.write(BMP280_CTRL_MEAS_REG);
  Wire.write(0b10101011);         // 16x pressure oversampling, 2x temp oversampling, normal power mode

  Wire.write(BMP280_CONFIG_REG);
  Wire.write(0b00010000);         // Set standby time to 0.5ms, filter coefficient to 16x
  Wire.endTransmission();

  // Read the Calibration data
  Wire.beginTransmission(BMP280_I2C_ADDR);
  Wire.write(BMP280_CALIBRATION_REG);
  Wire.endTransmission();
  Wire.requestFrom(BMP280_I2C_ADDR, 24);

  while (Wire.available()) {
    calibrationData[i] = Wire.read();
    i++;
  }

  // Fit calibration data into signed and unsigned int16_t's as per the BMP280 datasheet
  dig_T1 = calibrationData[1] << 8;
  dig_T1 += calibrationData[0];

  dig_T2 = calibrationData[3] << 8;
  dig_T2 += calibrationData[2];

  dig_T3 = calibrationData[5] << 8;
  dig_T3 += calibrationData[4];

  dig_P1 = calibrationData[7] << 8;
  dig_P1 += calibrationData[6];

  dig_P2 = calibrationData[9] << 8;
  dig_P2 += calibrationData[8];

  dig_P3 = calibrationData[11] << 8;
  dig_P3 += calibrationData[10];

  dig_P4 = calibrationData[13] << 8;
  dig_P4 += calibrationData[12];

  dig_P5 = calibrationData[15] << 8;
  dig_P5 += calibrationData[14];

  dig_P6 = calibrationData[17] << 8;
  dig_P6 += calibrationData[16];

  dig_P7 = calibrationData[19] << 8;
  dig_P7 += calibrationData[18];

  dig_P8 = calibrationData[21] << 8;
  dig_P8 += calibrationData[20];

  dig_P9 = calibrationData[23] << 8;
  dig_P9 += calibrationData[22];


 /* Serial.print(dig_T1); Serial.print(" ");
  Serial.print(dig_T2); Serial.print(" ");
  Serial.print(dig_T3); Serial.print(" ");
  Serial.print(dig_P1); Serial.print(" ");
  Serial.print(dig_P2); Serial.print(" ");
  Serial.print(dig_P3); Serial.print(" ");
  Serial.print(dig_P4); Serial.print(" ");
  Serial.print(dig_P5); Serial.print(" ");
  Serial.print(dig_P6); Serial.print(" ");
  Serial.print(dig_P7); Serial.print(" ");
  Serial.print(dig_P8); Serial.print(" ");
  Serial.println(dig_P9); */

}


/*---------------------------------------------------------
    
    BMP280_ReadTemp()
    
      Reads temperature data from the BMP280 and returns
      converted temperature in Celsius. Returned value is
      of the form "2661" = 26.61 C.

---------------------------------------------------------*/

int32_t BMP280_ReadTemp() {
  byte tempReg[3];
  byte i = 0;
  BMP280_S32_t rawTemp, var1, var2, T;

  // Get 20-bit temp data from registers
  Wire.beginTransmission(BMP280_I2C_ADDR);
  Wire.write(BMP280_TEMP_2_REG);
  Wire.endTransmission();
  Wire.requestFrom(BMP280_I2C_ADDR, 3);

  while (Wire.available()) {
    tempReg[i] = Wire.read();
    i++;
  }

  // Pack temp data into a sint32.
  rawTemp = tempReg[0] << 12;
  rawTemp += tempReg[1] << 4;
  rawTemp += tempReg[2] >> 4;

  // Conversion algorithm from BMP280 Datasheet.
  var1 = ((((rawTemp>>3) - ((BMP280_S32_t)dig_T1<<1))) * ((BMP280_S32_t)dig_T2)) >> 11;
  var2 = (((((rawTemp>>4) - ((BMP280_S32_t)dig_T1)) * ((rawTemp>>4) - ((BMP280_S32_t)dig_T1))) >> 12) *
  ((BMP280_S32_t)dig_T3)) >> 14;
  temp_fine = var1 + var2;
  T = (temp_fine * 5 + 128) >> 8;

  timeOfLastTempReading = millis();

  return T;
}

/*---------------------------------------------------------
    
    BMP280_ReadPressure()
    
      Reads pressure data from the BMP280 and returns
      converted pressure in Pascals.

---------------------------------------------------------*/

float BMP280_ReadPressure() {
  byte presReg[3];
  byte i = 0;
  BMP280_S64_t var1, var2, p;
  BMP280_S32_t rawPres;

  // If it's been longer than 500ms, get an updated
  // temperature reading.
  if (millis() - timeOfLastTempReading > 500) {
    BMP280_ReadTemp();
  }

  // Get 20-bit pressure data from registers
  Wire.beginTransmission(BMP280_I2C_ADDR);
  Wire.write(BMP280_PRESSURE_2_REG);
  Wire.endTransmission();
  Wire.requestFrom(BMP280_I2C_ADDR, 3);

  while (Wire.available()) {
    presReg[i] = Wire.read();
    i++;
  }

  // Pack temp data into a sint32.
  rawPres = presReg[0] << 12;
  rawPres += presReg[1] << 4;
  rawPres += presReg[2] >> 4;

  // Algorithm from BMP280 Datasheet
  var1 = ((BMP280_S64_t)temp_fine) - 128000;
  var2 = var1 * var1 * (BMP280_S64_t)dig_P6;
  var2 = var2 + ((var1*(BMP280_S64_t)dig_P5)<<17);
  var2 = var2 + (((BMP280_S64_t)dig_P4)<<35);
  var1 = ((var1 * var1 * (BMP280_S64_t)dig_P3)>>8) + ((var1 * (BMP280_S64_t)dig_P2)<<12);
  var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)dig_P1)>>33;
  if (var1 == 0) {
  return 0; // avoid exception caused by division by zero
  }
  p = 1048576-rawPres;
  p = (((p<<31)-var2)*3125)/var1;
  var1 = (((BMP280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((BMP280_S64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig_P7)<<4);

  return (float)(p/256);

}

/*---------------------------------------------------------
    
    BMP280_ReadAltitude()
    
      Uses Pressure data and provided sea level pressure
      to calculate approximate altitude in meters.

---------------------------------------------------------*/

float BMP280_ReadAltitude(float seaLevelhPa) {
  float altitude;
  // Formula taken from Adafruit BMP280 Driver
  float pressure = BMP280_ReadPressure();
  if (pressure == 0) {
    return -1;
  }
  pressure /= 100;
  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  //Serial.print("Altitude: "); Serial.println(altitude);
  return altitude;

}


