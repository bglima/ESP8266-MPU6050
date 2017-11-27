#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "i2c.h"
#include "task.h"
#include "queue.h"
#include "esp8266.h"
#include "math.h"
#include "MPU6050.h"

/* pin config */
const int gpio = 0;   /* gpio 0 usually has "PROGRAM" button attached */
const int active = 0; /* active == 0 for active low */
const gpio_inttype_t int_type = GPIO_INTTYPE_EDGE_NEG;


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



//void getMotionTask(void *pvParameters)
//{
//    while(1) {
//        getMotion();
//        vTaskDelay(10 / portTICK_PERIOD_MS);
//    }
//}


void user_init(void)
{
     
    uart_set_baud(0, 115200);
    gpio_enable(gpio, GPIO_INPUT);

    // Init MPU
    bool connected = init_mpu();
    if ( connected ) {
        printf("MPU is ok!\n");
    } else {
        printf("Connection failure!\n");
        return;
    }

    // Check MPU status
    uint8_t status = check_mpu();
    switch ( status ) {
    case 0:
        printf("Lock and loaded! MPU is active!\n");
        break;
    case 1:
        printf("MPU connected, but it's taking a nap! Zzzz....\n");
        break;
    case 2:
        printf("MPU was not found or not initiated! Check connections.\n");
        break;
    }

    // Get data values
    bool ok = read_values();
    if ( !ok )
        printf("Data was not read!\n");
    else {
        printf("Data was read successfully.\n");
        print_values();
    }

//    tsqueue = xQueueCreate(2, sizeof(uint32_t));
//    xTaskCreate(buttonIntTask, "buttonIntTask", 256, &tsqueue, 2, NULL);
//    xTaskCreate(buttonPollTask, "buttonPollTask", 256, NULL, 1, NULL);
//    xTaskCreate(getMotionTask, "getMotionTask", 256, NULL, 1, NULL);

}
