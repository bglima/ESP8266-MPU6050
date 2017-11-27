#include "MPU6050.h"

/*
 * Init MPU5060 and configure ACCELL and GYRO data.
 * Return true if sucessfuly configured
 *
 */
bool init_mpu(){
    int regs[] = {PWR_MGMT_1, GYRO_CONFIG, ACCEL_CONFIG};
    bool success = false;
    i2c_init(SCL_PIN, SDA_PIN);
    for (int i=0;i<3;i++) {
        uint8_t data[] = {regs[i], 0};  // Set each reg to zero
        success = i2c_slave_write(ADDR, data, sizeof(data));
        if (!success) return success;     // If any of registers does not respond, cancel request and return false
    }
    return success;
}

/*
 * Check if MPU responds its address and then check if it's asleep.
 * Returns MPU state: 0 = active
 *                    1 = sleeping
 *                    2 = not found
 */
uint8_t check_mpu() {
    uint8_t reg_data = -1;

    // Read WHO_I_AM register
    i2c_slave_read(ADDR, WHO_I_AM, &reg_data, 1);
    if ( reg_data != 0x68 ) // Wrong address, not found
        return 2;

    // Read PWR_MGMT_1 register
    i2c_slave_read(ADDR, PWR_MGMT_1, &reg_data, 1);
    if ( reg_data == 0x64 ) // in SLEEP mode
        return 1;

    // If it reaches here, MPU is active
    return 0;
}

/*
 * Read values and store them in mpu_data buffer.
 * Returns true if data was read successfully
 *
 */
bool read_values() {
      uint8_t buffer[14];

      if( check_mpu() ) // If return 0, MPU is active
          return false;

      bool ok = i2c_slave_read(ADDR, ACCEL_XOUT_H, buffer, 14);
      if( !ok )
          return false;

      printf("Accel X from inside: %d\n", (((int16_t)buffer[0]) << 8) | buffer[1]);
      mpu_data.value.accel_x = (((int16_t)buffer[0]) << 8) | buffer[1];
      mpu_data.value.accel_y = (((int16_t)buffer[2]) << 8) | buffer[3];
      mpu_data.value.accel_z = (((int16_t)buffer[4]) << 8) | buffer[5];
      mpu_data.value.temp = (((int16_t)buffer[6]) << 8) | buffer[7];
      mpu_data.value.gyro_x = (((int16_t)buffer[8]) << 8) | buffer[9];
      mpu_data.value.gyro_y = (((int16_t)buffer[10]) << 8) | buffer[11];
      mpu_data.value.gyro_z = (((int16_t)buffer[12]) << 8) | buffer[13];

      return true;
}

/*
 * Prints out all the data in mpu_data buffer.
 *
 */
void print_values() {
    printf("--> Values read from MPU5060 and stored in mpu_data: <--\n");
    printf("Accel X.....: %d \n", mpu_data.value.accel_x);
    printf("Accel Y.....: %d \n", mpu_data.value.accel_y);
    printf("Accel Z.....: %d \n", mpu_data.value.accel_z);
    printf("Gyro X......: %d \n", mpu_data.value.gyro_x);
    printf("Gyro Y......: %d \n", mpu_data.value.gyro_y);
    printf("Gyro Z......: %d \n", mpu_data.value.gyro_z);
    printf("Temprature..: %d \n", mpu_data.value.temp);
    printf("\n");
}




//    float pitch=0, roll=0;

//    ComplementaryFilter(accData, gyrData, &pitch, &roll);

//    printf("pitch: %f, roll: %f \n", pitch, roll);
//    // printf("Acceleration..: x: %.2f, y: %.2f, z: %.2f\n", ax * 0.061 * 9.80665 / 1000.0, ay * 0.061 * 9.80665 / 1000.0, az * 0.061 * 9.80665 / 1000.0);
//   // printf("Gyro..........: x: %d, y: %d, z: %d\n\n", gx, gy, gz);

//}

//float arctan2(float y, float x)
//{
//   float coeff_1 = 3.14/4;
//   float coeff_2 = 3*coeff_1;
//   float abs_y = fabs(y)+1e-10;      // kludge to prevent 0/0 condition
//   float angle=0, r=0;
//   if (x>=0)
//   {
//      r = (x - abs_y) / (x + abs_y);
//      angle = coeff_1 - coeff_1 * r;
//   }
//   else
//   {
//      r = (x + abs_y) / (abs_y - x);
//      angle = coeff_2 - coeff_1 * r;
//   }
//   if (y < 0)
//   return(-angle);     // negate if in quad III or IV
//   else
//   return(angle);
//}

//void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
//{
//    float pitchAcc, rollAcc;

//    // Integrate the gyroscope data -> int(angularSpeed) = angle
//    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
//    *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis

//    // Compensate for drift with accelerometer data if !bullshit
//    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
//    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
//    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
//    {
//    // Turning around the X axis results in a vector on the Y-axis
//        pitchAcc = arctan2((float)accData[1], (float)accData[2]) * 180 / M_PI;
//        *pitch = *pitch * 0.98 + pitchAcc * 0.02;

//    // Turning around the Y axis results in a vector on the X-axis
//        rollAcc = arctan2((float)accData[0], (float)accData[2]) * 180 / M_PI;
//        *roll = *roll * 0.98 + rollAcc * 0.02;
//    }

//}

