/*--------------------------------------------------------------

  MICROCONTROLLER PINOUT

  MICROCONTROLLER: ESP32 (38 Pin)

  ESP32 To RC Receiver
    PIN 4  --> Channel 9
    PIN 13 --> Channel 8
    PIN 27 --> Channel 7
    PIN 32 --> Channel 3
    PIN 33 --> Channel 4
    PIN 34 --> Channel 1
    Pin 35 --> Channel 2

  ESP32 To I2C Bus (LSM9DS0, BMP280)
    PIN 21 --> SDA 
    PIN 22 --> SCL 

  ESP32 To GT-U7 GPS Module
    PIN 18 (TX) --> RX
    PIN 17 (RX) --> TX

  ESP32 To Motor ESCs
    PIN 25 --> Front Left Motor
    PIN 26 --> Front Right Motor
    PIN 16 --> Back Left Motor
    PIN 19 --> Back Right Motor


--------------------------------------------------------------*/


/*--------------------------------------------------------------
        EXTERNAL LIBRARY INCLUDES
--------------------------------------------------------------*/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include "EEPROM.h"
#include <Adafruit_AHRS_Mahony.h>


/*--------------------------------------------------------------
        INTERNAL HEADER INCLUDES
--------------------------------------------------------------*/

#include "Drone_BMP280.h"
#include "Drone_LSM9DS0.h"
#include "Drone_EEPROM.h"
#include "Drone_Receiver.h"
#include "Drone_Mode_Manager.h"


/*--------------------------------------------------------------
        GLOBAL VARIABLES
--------------------------------------------------------------*/

TaskHandle_t CORE0;
TaskHandle_t CORE1;

// Core 0 Data
struct ReceiverData       receiverData;
struct ThrottleOutput     throttleOutput;

// Core 1 Data
struct LSM9DS0_GyroData   GyroData;
struct LSM9DS0_AccelData  AccelData;
struct LSM9DS0_MagData    MagData;
struct Pitch_Heading_Roll FlightData;
float                     Baro_Altitude;
double                    GPS_Altitude;

/*----------------------------------------------------------------------

    setup()

      Main setup function for the drone. Entry point of entire program.

----------------------------------------------------------------------*/

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

/*--------------------------------------------------------------
    
    CORE0_PROCEDURE()
    
      Core 0 Interfaces with the receiver, sends PWM signals to
      the Electronic Speed Controllers, handles vehicle mode,
      and consumes/processes the data produced by Core 1 to 
      calculate motor commands.

      FILES IN THE SCOPE OF CORE 0:

      Receiver.ino

--------------------------------------------------------------*/

void CORE0_PROCEDURE(void * pvParameters) {
  //********Core 0 Setup******** 
  setCpuFrequencyMhz(240);

  // Setup Motor Output
  Mode_Manager_Begin();
  // Setup Receiver
  Receiver_Begin();

  //********Core 0 Superloop********
  for (;;) {

    updateReceiverData();
    Mode_Manager();
    delay(1); // DELETE ME LATER
  }
}


/*--------------------------------------------------------------
    
    CORE1_PROCEDURE()
    
      Core 1 Interfaces with the Barometer, Thermometer,
      Gyroscope, Accelerometer, Magnetometer, and GPS sensors
      over I2C. Core 1 does much of the data processing to
      convert sensor data into usable units and extrapolate
      vehicle attitude and position.

      FILES IN THE SCOPE OF CORE 1:

      BMP280.ino
      LSM9DS0.ino
      EEPROM.ino

--------------------------------------------------------------*/

void CORE1_PROCEDURE(void * pvParameters) {
  //********Core 1 Setup********
  setCpuFrequencyMhz(240);

  // Setup I2C
  Wire.begin();
  Wire.setClock(400000);    //4KHz (I2C fast mode) 

  // Setup GPS
  TinyGPSPlus gps;
  SoftwareSerial ss(17, 18);
  ss.begin(9600);

  // Setup BMP280 (Thermometer/Barometer)
  BMP280_Begin();

  // Setup LSM9DS0 (Gyroscope/Accelerometer/Magnetometer)
  LSM9DS0_Begin();

  double lat, lng;
  unsigned long us = 0;

  //********Core 1 Superloop********
  for(;;){
    //Serial.println(micros() - us);
    //us = micros();
    Baro_Altitude = BMP280_ReadAltitude(1018.3); // Will likely need some way to load hPa for each flight
    LSM9DS0_ReadAccelerometerData();
    LSM9DS0_ReadGyroscopeData();
    LSM9DS0_ReadMagnetometerData();
    LSM9DS0_CalculateFlightData();

    lat = gps.location.lat(); 
    lng = gps.location.lng();
    

  }
} 

void loop() {

}

