#include "Wire.h"
#include "EEPROM.h"
#include "Drone_BMP280.h"
#include "Drone_LSM9DS0.h"
#include "Drone_EEPROM.h"

struct LSM9DS0_GyroData GyroData;
struct LSM9DS0_AccelData AccelData;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  EEPROM_Begin();
  BMP280_Begin();
  LSM9DS0_Begin();

}

void loop() {
  //BMP280_ReadAltitude(1018.3); // Will likely need some way to load hPa for each flight

  LSM9DS0_CalculateFlightData();
  
  delay(25);
}