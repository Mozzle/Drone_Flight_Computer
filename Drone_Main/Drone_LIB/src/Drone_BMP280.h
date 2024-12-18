
#define BMP280_I2C_ADDR         0x76 // or 0x77

#define BMP280_CTRL_MEAS_REG    0xF4
#define BMP280_CONFIG_REG       0xF5

#define BMP280_PRESSURE_2_REG   0xF7  // Most significant 8 bits
#define BMP280_PRESSURE_1_REG   0xF8  // Next least significant 8 bits
#define BMP280_PRESSURE_0_REG   0xF9  // Least significant 4 bits of the pressure data

#define BMP280_TEMP_2_REG       0xFA  // Most significant 8 bits
#define BMP280_TEMP_1_REG       0xFB  // Next least significant 8 bits
#define BMP280_TEMP_0_REG       0xFC  // Least significant 4 bits of the temp data

#define BMP280_CALIBRATION_REG  0x88  // Beginning of the calibration data in memory

typedef int32_t   BMP280_S32_t;
typedef uint32_t  BMP280_U32_t;
typedef int64_t   BMP280_S64_t;

void BMP280_Begin();
int32_t BMP280_ReadTemp();
float BMP280_ReadPressure();
float BMP280_ReadAltitude(float seaLevelhPa);