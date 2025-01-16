#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <esp_err.h>
#include "esp_timer.h"
#include "driver/gpio.h"

extern void slave_task(void *arg);
extern void master_task(void *arg);

void app_main(void)
{
    xTaskCreate(slave_task, "slave_task", 2 * 4096, NULL, 10, NULL);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // xTaskCreate(master_task, "master_task", 2 * 4096, NULL, 10, NULL);
}
