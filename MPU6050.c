#include "MPU6050.h"

/*
 * Init MPU5060 and configure ACCELL and GYRO data.
 * Return true if sucessfuly configured
 *
 */
void init_mpu(){
    int regs[] = {PWR_MGMT_1, GYRO_CONFIG, ACCEL_CONFIG};
    i2c_init(SCL_PIN, SDA_PIN);
    for (int i=0;i<3;i++) {
        uint8_t data[] = {regs[i], 0};  // Set each reg to zero
        bool success = i2c_slave_write(ADDR, data, sizeof(data));
        if (!success) return;     // If any of registers does not respond, cancel request
    }

    read_values();

    for(int i=0; i < 3; ++i) {
        filter_accel[i] = accel[i];
        last_accel[i] = accel[i];
        last_gyro[i] = accel[i];
    }
    dt = 0.1;
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

      mpu_raw_data.value.accel_x = (((int16_t)buffer[0]) << 8) | buffer[1];
      mpu_raw_data.value.accel_y = (((int16_t)buffer[2]) << 8) | buffer[3];
      mpu_raw_data.value.accel_z = (((int16_t)buffer[4]) << 8) | buffer[5];
      mpu_raw_data.value.temp = (((int16_t)buffer[6]) << 8) | buffer[7];
      mpu_raw_data.value.gyro_x = (((int16_t)buffer[8]) << 8) | buffer[9];
      mpu_raw_data.value.gyro_y = (((int16_t)buffer[10]) << 8) | buffer[11];
      mpu_raw_data.value.gyro_z = (((int16_t)buffer[12]) << 8) | buffer[13];

      float accel_factor = GRAVITY / ACCELEROMETER_SENSITIVITY;
      float gyro_factor = 1 / GYROSCOPE_SENSITIVITY;
      float temp_factor = 1 / TEMPERATURE_SENSIVITY;
      accel[0] = mpu_raw_data.value.accel_x * accel_factor;
      accel[1] = mpu_raw_data.value.accel_y * accel_factor;
      accel[2] = mpu_raw_data.value.accel_z * accel_factor;
      temp = mpu_raw_data.value.temp * temp_factor + 36.53;
      gyro[0] = mpu_raw_data.value.gyro_x * gyro_factor;
      gyro[1] = mpu_raw_data.value.gyro_y * gyro_factor;
      gyro[2] = mpu_raw_data.value.gyro_z * gyro_factor;
      return true;
}

/*
 * Prints out all the data in mpu_data buffer.
 *
 */
void print_values() {
    printf("\n--> Printing MPU5060 read values: <--\n");
    printf("Accel (m/s^2) X: %.2f; Y:%.2f; Z:%.2f\n", accel[0], accel[1], accel[2]);
    printf("FAccel(m/s^2) X: %.2f; Y:%.2f; Z:%.2f\n", filter_accel[0], filter_accel[1], filter_accel[2]);
    printf("Gyro  (m/s^2) X: %.2f; Y:%.2f; Z:%.2f\n", gyro[0], gyro[1], gyro[2]);
    printf("Temp  (Celsius): %f \n", temp);
    printf("Vel   (m/s^2) X: %.2f; Y:%.2f; Z:%.2f\n", vel[0], vel[1], vel[2]);
    printf("Dis   (m/s^2) X: %.2f; Y:%.2f; Z:%.2f\n", dis[0], dis[1], dis[2]);
    printf("\n");
}

/*
 * Prints out filtered values
 *
 */
void print_filtered_values() {
    printf("Accel (m/s^2) X: %.2f; Y:%.2f; Z:%.2f\n", accel[0], accel[1], accel[2]);
    printf("FAccel(m/s^2) X: %.2f; Y:%.2f; Z:%.2f\n\n", filter_accel[0], filter_accel[1], filter_accel[2]);
}


/*
 * Prints out temperature
 *
 */
void print_temperature() {
    printf("Currently, the temperature is %.2f deg Celsius\n", temp);
}




/*
 * Programmer related function. Debug values read
 */
void debug_values() {
      printf("AX, AY e AZ:  %.2f;   %.2f;  %.2f. \n", accel[0], accel[1], accel[2]);
      printf("FX, FY e FZ:  %.2f;   %.2f;  %.2f. \n", filter_accel[0], filter_accel[1], filter_accel[2]);

}



/*
 * Write values to an extern buffer, so data user can use it
 */
void get_data_buffer(uint8_t *buffer)
{
    for(int i = 0; i < 14; ++i)
        buffer[i] = mpu_raw_data.buffer[i];
}

/*
 * Faster implementation of atan2
 */
float arctan2(float y, float x)
{
   float coeff_1 = 3.14/4;
   float coeff_2 = 3*coeff_1;
   float abs_y = fabs(y)+1e-10;      // kludge to prevent 0/0 condition
   float angle=0, r=0;
   if (x>=0)
   {
      r = (x - abs_y) / (x + abs_y);
      angle = coeff_1 - coeff_1 * r;
   }
   else
   {
      r = (x + abs_y) / (abs_y - x);
      angle = coeff_2 - coeff_1 * r;
   }
   if (y < 0)
   return(-angle);     // negate if in quad III or IV
   else
   return(angle);
}

/*
 * Reset values of acceleration, velocity and position.
 * Nedded to reset references
 */
void reset_ref()
{
   read_values();
   for( int i = 0; i < 3; ++i ) {
       vel[i] = 0;
       dis[i] = 0;
       filter_accel[i] = accel[i];
       offset_accel[i] = accel[i];
   }
}

/*
 * Executes the integration on the values of acceleration.
 * For now, we are integrating only in X axis
 */
void step()
{
    // Save current values
    memcpy(last_accel, accel, sizeof(accel));         // Use memcpy for faster processing
    memcpy(last_gyro, gyro, sizeof(gyro));

    // Update values of accel and gyro
    read_values();

    // Integrating acceleration to get velocity. Using trapezoidal ruile
    // Using f(x) is aprox equal (b - a) * [ f(a) + f(b) ] / 2
    float new_vel[3], new_dis[3];
    for(int i = 0; i < 3; ++i) {
        new_vel[i] = vel[i] + dt * ( accel[i] + last_accel[i] ) / 2.0;

        // Integrating velocity to get position. Also trapezoidal rule.
        new_dis[i] = dis[i] + dt * ( vel[i]   + new_vel[i] ) / 2.0;

        // Updating values of vel and distance
        vel[i] = new_vel[i];
        dis[i] = new_dis[i];
    }
}

/*
 * Set dt for integration.
 */
bool set_dt(float new_dt) {
    if ( new_dt >= 0.0001 && new_dt <= 1 ) {
        dt = new_dt;
        return true;
    }
    return false;
}

/* Get dt */
float get_dt() {
    return dt;
}

/*
 * Returns true if accel was tilted
 */
bool tapped(float thresh)
{
    float diff_accel_z = fabs(last_accel[2] - accel[2]);

    if (  diff_accel_z > thresh )
        return true;
    return false;
}


void low_filter(float alpha)
{
    filter_accel[0] = accel[0] * alpha + ( filter_accel[0] * (1 - alpha) );
    filter_accel[1] = accel[1] * alpha + ( filter_accel[1] * (1 - alpha) );
    filter_accel[2] = accel[2] * alpha + ( filter_accel[2] * (1 - alpha) );
}

float get_filt_accel( int i ) {
    return filter_accel[i];
}
