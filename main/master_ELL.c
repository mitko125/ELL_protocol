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

static const char *TAG = "ELL_old_master";

// #define LEFT_UART               1

#ifdef LEFT_UART
#define MASTER_UART_PORT  UART_NUM_1
#define MASTER_TXD   CONFIG_RS485_UART1_TXD
#define MASTER_RXD   CONFIG_RS485_UART1_RXD      
#define MASTER_RTS   CONFIG_RS485_UART1_RTS
#else // LEFT_UART
#define MASTER_UART_PORT  UART_NUM_2
#define MASTER_TXD   CONFIG_RS485_UART2_TXD
#define MASTER_RXD   CONFIG_RS485_UART2_RXD      
#define MASTER_RTS   CONFIG_RS485_UART2_RTS
#endif // LEFT_UART

#define BUF_SIZE        (128)

// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define ECHO_READ_TOUT          (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks

void master_task(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG, "Start RS485 and configure UART.");
    
    {   //RS4851 2
        // Install UART driver (we don't need an event queue here)
        ESP_ERROR_CHECK(uart_driver_install(MASTER_UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, ESP_INTR_FLAG_SHARED));
        
        ESP_ERROR_CHECK(uart_set_mode(MASTER_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));
        // Configure UART parameters
        ESP_ERROR_CHECK(uart_param_config(MASTER_UART_PORT, &uart_config));

        // Set UART pins as per KConfig settings
        ESP_ERROR_CHECK(uart_set_pin(MASTER_UART_PORT, MASTER_TXD, MASTER_RXD, MASTER_RTS, UART_PIN_NO_CHANGE));

        // Set read timeout of UART TOUT feature
        ESP_ERROR_CHECK(uart_set_rx_timeout(MASTER_UART_PORT, ECHO_READ_TOUT));
    }

     // Allocate buffers for UART
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);

    ESP_LOGI(TAG, "UART start recieve loop.");

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1;
    BaseType_t xWasDelayed;
    xLastWakeTime = xTaskGetTickCount ();

    while (1) {
        
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xFrequency );
        // Извършете действие тук. Стойността xWasDelayed може да се използва за определяне
        // дали е пропуснат краен срок, ако кодът тук отне твърде много време.
        if (!xWasDelayed)
            ESP_LOGE(TAG, "Time Overflow");

        gpio_set_level(TOGLE_PIN2, !gpio_get_level(TOGLE_PIN2));

        const char command = COMAND_INP;
        uart_write_bytes(MASTER_UART_PORT, &command, 1);     
        ESP_ERROR_CHECK(uart_wait_tx_done(MASTER_UART_PORT, portMAX_DELAY));

        gpio_set_level(TOGLE_PIN2, !gpio_get_level(TOGLE_PIN2));

        int len = uart_read_bytes(MASTER_UART_PORT, data, BUF_SIZE, 0);
        if (len > 0) {
            // ESP_LOGI(TAG, "Received %u bytes:", len);

            // printf("{ ");
            // for (int i = 0; i < len; i++) {
            //     printf("0x%.2X ", (uint8_t)data[i]);
            // }
            // printf("} \n");
        }
    }
    vTaskDelete(NULL);
}