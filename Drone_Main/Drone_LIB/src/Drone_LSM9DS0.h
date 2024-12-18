
// I2C Addresses
#define LSM9DS0_GYRO_I2C_ADDR       0x6B
#define LSM9DS0_ACCEL_MAG_I2C_ADDR  0x1D

// Register Addresses
#define LSM9DS0_CTRL_REG1_G         0x20
#define LSM9DS0_CTRL_REG2_G         0x21
#define LSM9DS0_CTRL_REG3_G         0x22
#define LSM9DS0_CTRL_REG4_G         0x23
#define LSM9DS0_CTRL_REG5_G         0x24

#define LSM9DS0_CTRL_REG1_XM        0x20
#define LSM9DS0_CTRL_REG2_XM        0x21
#define LSM9DS0_CTRL_REG5_XM        0x24
#define LSM9DS0_CTRL_REG6_XM        0x25
#define LSM9DS0_CTRL_REG7_XM        0x26

#define LSM9DS0_FIFO_CTRL_REG_G     0x2E
#define LSM9DS0_INT1_CFG_G          0x30

// X-Axis Angular Rate Data
#define LSM9DS0_OUT_X_L_G           0x28
#define LSM9DS0_OUT_X_H_G           0x29

// Y-Axis Angular Rate Data
#define LSM9DS0_OUT_Y_L_G           0x2A
#define LSM9DS0_OUT_Y_H_G           0x2B

// Z-Axis Angular Rate Data
#define LSM9DS0_OUT_Z_L_G           0x2C
#define LSM9DS0_OUT_Z_H_G           0x2D

// X-Axis Acceleration Data
#define LSM9DS0_OUT_ACCEL_BURST     0xA8
#define LSM9DS0_OUT_X_L_A           0x28
#define LSM9DS0_OUT_X_H_A           0x29

// Y-Axis Acceleration Data
#define LSM9DS0_OUT_Y_L_A           0x2A
#define LSM9DS0_OUT_Y_H_A           0x2B

// Z-Axis Acceleration Data
#define LSM9DS0_OUT_Z_L_A           0x2C
#define LSM9DS0_OUT_Z_H_A           0x2D

// X-Axis Magnetic data
#define LSM9DS0_OUT_MAG_BURST       0x88
#define LSM9DS0_OUT_X_L_M           0x08
#define LSM9DS0_OUT_X_H_M           0x09

// Y-Axis Magnetic data
#define LSM9DS0_OUT_Y_L_M           0x0A
#define LSM9DS0_OUT_Y_H_M           0x0B

// Z-Axis Magnetic data
#define LSM9DS0_OUT_Z_L_M           0x0C
#define LSM9DS0_OUT_Z_H_M           0x0D

// Accelerometer Range in Gs
#define ACCELEROMETER_RANGE         4

// Gyroscope Range in Degrees Per Second
#define GYROSCOPE_RANGE             245

// Magnetometer Range in Gauss 
#define MAGNETOMETER_RANGE          4

#define RADIANS_TO_DEGREE_CONV      57.29578

struct LSM9DS0_GyroData {
  float X_DegPerSec;
  float Y_DegPerSec;
  float Z_DegPerSec;
};

struct LSM9DS0_AccelData {
  float X_AccelInG;
  float Y_AccelInG;
  float Z_AccelInG;
};

struct LSM9DS0_MagData {
  float X_FieldInGauss;
  float Y_FieldInGauss;
  float Z_FieldInGauss;
};

struct Pitch_Heading_Roll {
  float pitch;
  float heading;
  float roll;
};

void LSM9DS0_Begin();
void LSM9DS0_ReadGyroscopeData();
void LSM9DS0_ReadAccelerometerData();

