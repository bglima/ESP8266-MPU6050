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
#define I2C_BUS 0
#define ADDR 0x68
#define WHO_I_AM     0x75 // decimal 117
#define AM_I_SLEEP   0x6B // decimal 107
#define SCL_PIN (5)
#define SDA_PIN (4)

uint8_t init_MPU(){

    int regs[] = {107,0x1B,0x1C};
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
        printf("MPU6050 is allright! Everything is fine!\n");
    }

    i2c_slave_read(ADDR, AM_I_SLEEP, &reg_data, 1);
    if ( reg_data == 0x64 ) {
        printf("MPU6050 is in SLEEP mode!\n");
    } else {
        printf("MPU6050 is ACTIVE.\n");
    }

}

void user_init(void)
{
     
    uart_set_baud(0, 115200);
    gpio_enable(gpio, GPIO_INPUT);

    uint8_t slave_addr = 0x68;  // MPU address
    uint8_t reg_addr = 0x107;   //
    uint8_t data[] = {reg_addr, 0};

    i2c_init(SCL_PIN, SDA_PIN);
    bool success = i2c_slave_write(slave_addr, data, sizeof(data));
    if (success)
    {
        printf("YAY!\n");
    } else {
        printf("AFF :(\n");
    }

    init_MPU();
    check_MPU();

 
//	// Requisitando dados
//	i2c_start(I2C_BUS);
//	i2c_write(I2C_BUS, ADDR);
//	i2c_write(I2C_BUS, 0x3B);
//	// Lendo dados
//	i2c_start(I2C_BUS);
//	i2c_write(I2C_BUS, ADDR);
//	uint8_t data = i2c_read(I2C_BUS, true);
//	printf("Data read: %x\n", data);
//	i2c_stop(I2C_BUS);

//	// WHO I AM
//	i2c_start(I2C_BUS);
//	i2c_write(I2C_BUS, ADDR);
//	i2c_write(I2C_BUS, WHO_I_AM);
//	i2c_start(I2C_BUS);
//	uint8_t who = i2c_read(I2C_BUS, true);
//	i2c_stop(I2C_BUS);
//	printf("Data WHO: %x\n", who);
	


    tsqueue = xQueueCreate(2, sizeof(uint32_t));
    xTaskCreate(buttonIntTask, "buttonIntTask", 256, &tsqueue, 2, NULL);
    xTaskCreate(buttonPollTask, "buttonPollTask", 256, NULL, 1, NULL);
}
