#include "MPU6050.h"

bool init_mpu(){
    int regs[] = {PWR_MGMT_1, GYRO_CONFIG, ACCEL_CONFIG, FIFO_R_W};
    bool success = false;
    for (int i=0;i<3;i++) {
        uint8_t data[] = {regs[i], 0};  // Set each reg to zero
        success = i2c_slave_write(ADDR, data, sizeof(data));
        if ( !success ) return false;
    }
    return true;
}

//void check_MPU() {
//    uint8_t reg_data = -1;
//    i2c_slave_read(ADDR, WHO_I_AM, &reg_data, 1);
//    if ( reg_data == 0x68 ) {
//        printf("MPU6050 is allright! Everything is fine!\n");
//    } else {
//        printf("MPU not found!\n");
//        return;
//    }

//    i2c_slave_read(ADDR, PWR_MGMT_1, &reg_data, 1);
//    if ( reg_data == 0x64 ) {
//        printf("MPU6050 is in SLEEP mode!\n");
//    } else {
//        printf("MPU6050 is ACTIVE.\n");
//    }

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

//void getMotion() {
//    uint8_t buffer[14];

//    i2c_slave_read(ADDR, ACCEL_XOUT_H, buffer, 14);
//    accData[0] = (((int16_t)buffer[0]) << 8) | buffer[1];
//    accData[1] = (((int16_t)buffer[2]) << 8) | buffer[3];
//    accData[2] = (((int16_t)buffer[4]) << 8) | buffer[5];
//    gyrData[0] = (((int16_t)buffer[8]) << 8) | buffer[9];
//    gyrData[1] = (((int16_t)buffer[10]) << 8) | buffer[11];
//    gyrData[2] = (((int16_t)buffer[12]) << 8) | buffer[13];

//    float pitch=0, roll=0;

//    ComplementaryFilter(accData, gyrData, &pitch, &roll);

//    printf("pitch: %f, roll: %f \n", pitch, roll);
//    // printf("Acceleration..: x: %.2f, y: %.2f, z: %.2f\n", ax * 0.061 * 9.80665 / 1000.0, ay * 0.061 * 9.80665 / 1000.0, az * 0.061 * 9.80665 / 1000.0);
//   // printf("Gyro..........: x: %d, y: %d, z: %d\n\n", gx, gy, gz);

//}
