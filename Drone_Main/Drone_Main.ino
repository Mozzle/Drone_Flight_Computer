#include "Wire.h"
#include "EEPROM.h"
#include "Drone_BMP280.h"
#include "Drone_LSM9DS0.h"
#include "Drone_EEPROM.h"
#include "Drone_Receiver.h"

TaskHandle_t CORE0;
TaskHandle_t CORE1;

struct LSM9DS0_GyroData   GyroData;
struct LSM9DS0_AccelData  AccelData;
struct LSM9DS0_MagData    MagData;
struct Pitch_Heading_Roll FlightData;
struct ReceiverData       receiverData;
float                     Baro_Altitude;


void setup() {
  Serial.begin(115200);
  EEPROM_Begin();
  

  xTaskCreatePinnedToCore(
                    CORE0_PROCEDURE,        /* Task function.                             */
                    "CORE0_PROCEDURE",      /* name of task.                              */
                    32768,                  /* Stack size of task: 32kb                   */
                    NULL,                   /* parameter of the task                      */
                    1,                      /* priority of the task                       */
                    &CORE0,                 /* Task handle to keep track of created task  */
                    0);                     /* pin task to core 0                         */   


  xTaskCreatePinnedToCore(
                    CORE1_PROCEDURE,        /* Task function.                             */
                    "CORE1_PROCEDURE",      /* name of task.                              */
                    32768,                  /* Stack size of task: 32kb                   */
                    NULL,                   /* parameter of the task                      */
                    1,                      /* priority of the task                       */
                    &CORE1,                 /* Task handle to keep track of created task  */
                    1);                     /* pin task to core 1                         */   

}

void CORE0_PROCEDURE(void * pvParameters) {
  // Core 0 Setup
  setCpuFrequencyMhz(240);
  Receiver_Begin();

  // Core 0 Superloop
  for (;;) {
    updateReceiverData();
    delay(10);
  }
}

void CORE1_PROCEDURE(void * pvParameters) {
  // Core 1 Setup
  setCpuFrequencyMhz(240);

  Wire.begin();
  Wire.setClock(400000);    //4KHz (I2C fast mode) 
  BMP280_Begin();
  LSM9DS0_Begin();
  unsigned long usLoopTime = 0;

  // Core 1 Superloop
  for(;;){

    usLoopTime = micros();

    Baro_Altitude = BMP280_ReadAltitude(1018.3); // Will likely need some way to load hPa for each flight
    LSM9DS0_ReadMagnetometerData();
    Serial.println((micros() - usLoopTime));
    LSM9DS0_ReadAccelerometerData();
    LSM9DS0_ReadGyroscopeData();
    LSM9DS0_CalculateFlightData();

  }
} 

void loop() {

}

