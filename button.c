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
#include "bma2x2.h"

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

uint8_t init_MPU(){
    int regs[] = {PWR_MGMT_1, GYRO_CONFIG, ACCEL_CONFIG};
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

void getMotion() {
    uint8_t buffer[14];
    i2c_slave_read(ADDR, ACCEL_XOUT_H, buffer, 14);
    int16_t ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    int16_t ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    int16_t az = (((int16_t)buffer[4]) << 8) | buffer[5];
    int16_t gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    int16_t gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    int16_t gz = (((int16_t)buffer[12]) << 8) | buffer[13];
    printf("Acceleration..: x: %d, y: %d, z: %d\n", ax / ((int16_t)2048.0), ay / ((int16_t)2048.0), az / ((int16_t)2048.0));
    printf("Gyro..........: x: %d, y: %d, z: %d\n\n", gx, gy, gz);

}

void getMotionTask(void *pvParameters)
{
    while(1) {
        getMotion();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void user_init(void)
{
     
    uart_set_baud(0, 115200);
    gpio_enable(gpio, GPIO_INPUT);

    i2c_init(SCL_PIN, SDA_PIN);
    init_MPU();
    check_MPU();


    tsqueue = xQueueCreate(2, sizeof(uint32_t));
    xTaskCreate(buttonIntTask, "buttonIntTask", 256, &tsqueue, 2, NULL);
    xTaskCreate(buttonPollTask, "buttonPollTask", 256, NULL, 1, NULL);
    xTaskCreate(getMotionTask, "getMotionTask", 256, NULL, 1, NULL);
}
