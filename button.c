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
void gpio_intr_handler(uint8_t gpio_num);
int taskIndex = 0;  /*
                     * Index 1 = tap
                     *       2 = printValues
                     *       3 = printFiltered
                     *       4 = tap temperature
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
            printf("Reseting references of MPU at %dms\r\n", button_ts);
            reset_ref();    // Reset velocity and displacement

            switch (taskIndex) {
            case 1:
                taskIndex = 2;
                set_dt(0.001);
                break;
            case 2:
                set_dt(0.1);
                taskIndex = 3;
                break;
            case 3:
                set_dt(0.001);
                taskIndex = 4;
                break;
            case 4:
                set_dt(0.001);
                taskIndex = 1;
                break;
            default:
                taskIndex = 1;
                set_dt(0.001);
                break;
            }
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



void getMotionTask(void *pvParameters)
{
    while(1) {
        step();
        low_filter(0.05);
        vTaskDelay( get_dt() * 1000 / portTICK_PERIOD_MS);  // Use the get_dt() time to wait for next reading
    }
}

void getTapTask(void *pvParameters)
{
    while( 1 ) {
        vTaskDelay( get_dt() * 1000 / portTICK_PERIOD_MS ) ;
        // Only execute if it's its turn!
        if ( taskIndex == 1 || taskIndex == 4 ) {

            // Execute task!
            if ( tapped(10.0)) {

                if ( taskIndex == 1 )
                    printf("Was taped!\n");
                else if ( taskIndex == 4 )
                    print_temperature();
                vTaskDelay( 300 / portTICK_PERIOD_MS );
            }
        }
    }
}

void printValuesTask(void *pvParameters)
{
    while( 1 ) {
        vTaskDelay( 1000 / portTICK_PERIOD_MS );

        // Only execute if it's its turn!
        if ( taskIndex != 2 )
            continue;

        print_values();
    }
}

void printFiltered(void *pvParameters)
{
    while( 1 ) {
        vTaskDelay( 100 / portTICK_PERIOD_MS );

        // Only execute if it's its turn!
        if ( taskIndex != 3 )
            continue;

        print_filtered_values();
    }
}


void user_init(void)
{
     
    uart_set_baud(0, 115200);
    gpio_enable(gpio, GPIO_INPUT);

    uint8_t status = 2;
    while ( status != 0 ) {
        // Init MPU
        init_mpu();

        // Check MPU status
        status = check_mpu();
        switch ( status ) {
        case 0:
            printf("Lock and loaded! MPU is active!\n");
            break;
        case 1:
            printf("MPU connected, but it's taking a nap! Zzzz....\n");
            break;
        case 2:
            printf("MPU was not found or not initiated! Trying again in 5 secs...\n");
            break;
        }

         vTaskDelay( 5000 / portTICK_PERIOD_MS );
    }

    // Get data values
    bool ok = read_values();
    if ( !ok )
        printf("Data was not read!\n");
    else {
        printf("Data was read successfully.\n");
        print_values();
    }

    // Testing integration parameters
    set_dt(0.001);  // BEST IS 0.001
    printf("Value from get_dt: %f\n", get_dt());

    // Writing data values to an extern buffer (as we are encapsulating it)
    uint8_t dataBuffer[14];
    get_data_buffer(dataBuffer);

    // Init task!
    taskIndex = 1;
    tsqueue = xQueueCreate(5, sizeof(uint32_t));
    xTaskCreate(buttonIntTask, "buttonIntTask", 256, &tsqueue, 2, NULL);
    xTaskCreate(getMotionTask, "getMotionTask", 256, NULL, 1, NULL);
    xTaskCreate(getTapTask, "getTapTask", 256, NULL, 1, NULL);
    xTaskCreate(printValuesTask, "printValuesTask", 256, NULL, 1, NULL);
    xTaskCreate(printFiltered, "printFiltered", 256, NULL, 1, NULL);

}
