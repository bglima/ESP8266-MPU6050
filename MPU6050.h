#ifndef MPU5060_HEADER
#define MPU5060_HEADER

#include "stdint.h"
#include "stdio.h"
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

typedef union
{
    uint8_t buffer[14];         // Accessing the whole buffer

    struct                      // Accessing each value individually
    {
        int16_t accel_x : 16;
        int16_t accel_y : 16;
        int16_t accel_z : 16;
        int16_t temp : 16;
        int16_t gyro_x : 16;
        int16_t gyro_y : 16;
        int16_t gyro_z : 16;
    } value;

    struct                      // Acessing them as vectors
    {
        int16_t accel[3];
        int16_t temp;
        int16_t gyro[3];
    } vec;

} mpu_data_t;

static mpu_data_t mpu_data;

bool init_mpu(); /* Wake MPU and setup GYRO and ACCEL */
uint8_t check_mpu(); /* Reads WHO_I_AM register from MPU and checks whether it is in SLEEP mode */
bool read_values(); /* Reads dev data and fill mpu_data buffer. Returns success of operation */
void print_values(); /* Print values in screen */


#endif
