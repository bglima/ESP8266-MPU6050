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
#include <i2c/i2c.h>
#include "task.h"
#include "queue.h"
#include "esp8266.h"
#include "bma2x2.h"

/* I2C definitions  */
#define I2C_BUS 0
#define SCL_PIN 5
#define SDA_PIN 4
#define ADDR 0x68

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

uint8_t read_reg_mpu(uint8_t reg){
	i2c_start(I2C_BUS);
	bool acked = i2c_write(I2C_BUS, ADDR);
	if( acked ) {
		printf("Byte was acked.\n");
	} else {
		printf("Byte was not acked.\n");
	}
	i2c_stop(I2C_BUS);		
}

void user_init(void)
{
     
    uart_set_baud(0, 115200);
    gpio_enable(gpio, GPIO_INPUT);

    int conn = i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);
    if( !conn ) {
	printf("Starting I2C connection.\n");
    } else {
	printf("Connection not started. Check the pins.\n");
    }

    read_reg_mpu(0);
    

 

    tsqueue = xQueueCreate(2, sizeof(uint32_t));
    xTaskCreate(buttonIntTask, "buttonIntTask", 256, &tsqueue, 2, NULL);
    xTaskCreate(buttonPollTask, "buttonPollTask", 256, NULL, 1, NULL);
}
