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

static const char *TAG = "ELL_master";

#define MASTER_UART_PORT UART_NUM_2

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

// I (1690) uart: ESP_INTR_FLAG_IRAM flag not set while CONFIG_UART_ISR_IN_IRAM is enabled, flag updated
    int64_t time_master = esp_timer_get_time() + 1000 * 1000;
    while (1) {
        vTaskDelay(1);
        int len = uart_read_bytes(MASTER_UART_PORT, data, BUF_SIZE, 1);
        if (len > 0) {
            // ESP_LOGI(TAG, "Received %u bytes:", len);

            // printf("{ ");
            // for (int i = 0; i < len; i++) {
            //     printf("0x%.2X ", (uint8_t)data[i]);
            // }
            // printf("} \n");
        }
        

        if (time_master <= esp_timer_get_time()) {
            time_master = esp_timer_get_time() + 10 * 1000;
            char d_send[40];
            d_send[0] = COMAND_INP;
            gpio_set_level(TOGLE_PIN2, !gpio_get_level(TOGLE_PIN2));
            uart_write_bytes(MASTER_UART_PORT, d_send, 1);     
            ESP_ERROR_CHECK(uart_wait_tx_done(MASTER_UART_PORT, portMAX_DELAY));
            gpio_set_level(TOGLE_PIN2, !gpio_get_level(TOGLE_PIN2));
        } 
    }
    vTaskDelete(NULL);
}