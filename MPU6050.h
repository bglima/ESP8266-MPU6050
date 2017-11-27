#ifndef MPU5060_HEADER
#define MPU5060_HEADER

#include "stdint.h"
#include "i2c.h"

/* General definitions */
#define ADDR 0x68
#define SCL_PIN (5) // D1
#define SDA_PIN (4) // D2
#define WHO_I_AM     0x75 // decimal 117
#define PWR_MGMT_1   0x6B // decimal 107. SLEEP
#define GYRO_CONFIG  0x1B // decimal 27
#define ACCEL_CONFIG 0x1C // decimal 28
#define ACCEL_XOUT_H 0x3B // decimal 59
#define FIFO_R_W 0x74     // decimal 116
#define FIFO_COUNTH   0x72
#define ACCELEROMETER_SENSITIVITY 16384.0
#define GYROSCOPE_SENSITIVITY 131.0
#define dt 0.01     // 10 ms sample rate!

struct MPU6050 {
   uint8_t dataBuffer[14];
   int16_t accelData[3];
   int16_t gyroData[3];
};

struct MPU6050* create_dev(void); /* Create a new instance of accelerometer as struct */
bool init_mpu(); /* Wake MPU and setup GYRO and ACCEL */


#endif
