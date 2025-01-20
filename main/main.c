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

#include "protocol_ELL_defs.h"

extern void slave_main(void *arg);
extern void master_task(void *arg);

void app_main(void)
{
    gpio_config_t gpio_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pin_bit_mask = ((1ULL << TOGLE_PIN1) | (1ULL << TOGLE_PIN2)),
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    xTaskCreate(slave_main, "slave_main", 2 * 4096, NULL, 4, NULL);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    xTaskCreate(master_task, "master_task", 2 * 4096, NULL, 10, NULL);
}