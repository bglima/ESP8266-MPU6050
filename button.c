/* Respond to a button press.
 *
 * This code combines two ways of checking for a button press -
 * busy polling (the bad way) and button interrupt (the good way).
 *
 * This sample code is in the public domain.
 */
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "i2c.h"
#include "task.h"
#include "queue.h"
#include "esp8266.h"
#include <math.h>

/* pin config */
const int gpio = 0;   /* gpio 0 usually has "PROGRAM" button attached */
const int active = 0; /* active == 0 for active low */
const gpio_inttype_t int_type = GPIO_INTTYPE_EDGE_NEG;


/* This task polls for the button and prints the tick
   count when it's seen.

   Debounced to 200ms with a simple vTaskDelay.

   This is not a good example of how to wait for button input!
*/
void buttonPollTask(void *pvParameters)
{
    printf("Polling for button press on gpio %d...\r\n", gpio);
    while(1) {
        while(gpio_read(gpio) != active)
        {
            taskYIELD();
        }
        printf("Polled for button press at %dms\r\n", xTaskGetTickCount()*portTICK_PERIOD_MS);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void gpio_intr_handler(uint8_t gpio_num);

/* This task configures the GPIO interrupt and uses it to tell
   when the button is pressed.

   The interrupt handler communicates the exact button press time to
   the task via a queue.

   This is a better example of how to wait for button input!
*/
void buttonIntTask(void *pvParameters)
{
    printf("Waiting for button press interrupt on gpio %d...\r\n", gpio);
    QueueHandle_t *tsqueue = (QueueHandle_t *)pvParameters;
    gpio_set_interrupt(gpio, int_type, gpio_intr_handler);

    uint32_t last = 0;
    while(1) {
        uint32_t button_ts;
        xQueueReceive(*tsqueue, &button_ts, portMAX_DELAY);
        button_ts *= portTICK_PERIOD_MS;
        if(last < button_ts-200) {
            printf("Button interrupt fired at %dms\r\n", button_ts);
            last = button_ts;
        }
    }
}

static QueueHandle_t tsqueue;

void gpio_intr_handler(uint8_t gpio_num)
{
    uint32_t now = xTaskGetTickCountFromISR();
    xQueueSendToBackFromISR(tsqueue, &now, NULL);
}


/* I2C definitions  */
// 5 e 4
// 0 e 2
#define ADDR 0x68
#define SCL_PIN (5)
#define SDA_PIN (4)
#define WHO_I_AM     0x75 // decimal 117
#define PWR_MGMT_1   0x6B // decimal 107. SLEEP
#define GYRO_CONFIG  0x1B // decimal 27
#define ACCEL_CONFIG 0x1C // decimal 28
#define ACCEL_XOUT_H 0x3B // decimal 59
#define FIFO_R_W 0x74 // decimal 116
#define FIFO_COUNTH          0x72
#define ACCELEROMETER_SENSITIVITY 16384.0
#define GYROSCOPE_SENSITIVITY 131.0
#define M_PI 3.14159265359  
#define dt 0.01     // 10 ms sample rate!


int16_t accData[3];
int16_t gyrData[3];

uint8_t init_MPU(){
    int regs[] = {PWR_MGMT_1, GYRO_CONFIG, ACCEL_CONFIG, FIFO_R_W};
    for (int i=0;i<3;i++) {
        uint8_t data[] = {regs[i], 0};
        i2c_slave_write(ADDR, data, sizeof(data));

    }
}

void check_MPU() {
    uint8_t reg_data = -1;
    i2c_slave_read(ADDR, WHO_I_AM, &reg_data, 1);
    if ( reg_data == 0x68 ) {
        printf("MPU6050 is allright! Everything is fine!\n");
    } else {
        printf("MPU not found!\n");
        return;
    }

    i2c_slave_read(ADDR, PWR_MGMT_1, &reg_data, 1);
    if ( reg_data == 0x64 ) {
        printf("MPU6050 is in SLEEP mode!\n");
    } else {
        printf("MPU6050 is ACTIVE.\n");
    }

}

// void getQuaternion(const uint8_t* packet) {
//     // TODO: accommodate different arrangements of sent data (ONLY default supported now)
//     // if (packet == 0) packet = dmpPacketBuffer;
//     int16_t ax = (((int16_t)packet[28]) << 8) | packet[29];
//     printf("q[0]: %d\n", ax);

//     // data[1] = ((packet[4] << 24) + (packet[5] << 16) + (packet[6] << 8) + packet[7]);
//     // data[2] = ((packet[8] << 24) + (packet[9] << 16) + (packet[10] << 8) + packet[11]);
//     // data[3] = ((packet[12] << 24) + (packet[13] << 16) + (packet[14] << 8) + packet[15]);
//     // return 0;
// }

// void getAccelAndGravity(){
//     uint8_t packetSize = 42;
//     uint8_t fifoBuffer[64];
//     int32_t quaternion[4];

//     i2c_slave_read(ADDR, FIFO_R_W, fifoBuffer, packetSize);

//     int16_t ax = (((int16_t)fifoBuffer[8]) << 8) | fifoBuffer[9];
//     printf("q[0]: %.2f\n", ax);
//     // getQuaternion(fifoBuffer);
// }

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

void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc;               
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
 
    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
    // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = arctan2((float)accData[1], (float)accData[2]) * 180 / M_PI;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
 
    // Turning around the Y axis results in a vector on the X-axis
        rollAcc = arctan2((float)accData[0], (float)accData[2]) * 180 / M_PI;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }

} 

void getMotion() {
    uint8_t buffer[14];

    i2c_slave_read(ADDR, ACCEL_XOUT_H, buffer, 14);
    accData[0] = (((int16_t)buffer[0]) << 8) | buffer[1];
    accData[1] = (((int16_t)buffer[2]) << 8) | buffer[3];
    accData[2] = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyrData[0] = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyrData[1] = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyrData[2] = (((int16_t)buffer[12]) << 8) | buffer[13];

    float pitch=0, roll=0;

    ComplementaryFilter(accData, gyrData, &pitch, &roll);

    printf("pitch: %f, roll: %f \n", pitch, roll);    
    // printf("Acceleration..: x: %.2f, y: %.2f, z: %.2f\n", ax * 0.061 * 9.80665 / 1000.0, ay * 0.061 * 9.80665 / 1000.0, az * 0.061 * 9.80665 / 1000.0);
   // printf("Gyro..........: x: %d, y: %d, z: %d\n\n", gx, gy, gz);

}

void getMotionTask(void *pvParameters)
{
    while(1) {
        getMotion();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void user_init(void)
{
     
    uart_set_baud(0, 115200);
    gpio_enable(gpio, GPIO_INPUT);

    i2c_init(SCL_PIN, SDA_PIN);
    init_MPU();
    check_MPU();
    // getAccelAndGravity();


    tsqueue = xQueueCreate(2, sizeof(uint32_t));
    xTaskCreate(buttonIntTask, "buttonIntTask", 256, &tsqueue, 2, NULL);
    xTaskCreate(buttonPollTask, "buttonPollTask", 256, NULL, 1, NULL);
    xTaskCreate(getMotionTask, "getMotionTask", 256, NULL, 1, NULL);
}
