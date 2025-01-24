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
#include "dmx_prog.h"
#include "lader.h"

// за симулиране на контролер с LCD индикатор и ладер програма
char enable_lader, bat_rs485, fl_disp, str[30];

extern void slave_main(void *arg);
extern void master_task(void *arg);

void app_main(void)
{
    gpio_config_t gpio_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pin_bit_mask = ((1ULL << Start1Ok0) | (1ULL << EventTogle) | (1ULL << FreeTogle) | (1ULL << Error1)),
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    xTaskCreate(slave_main, "slave_main", 2 * 4096, NULL, 4, NULL);
    vTaskDelay(pdMS_TO_TICKS(10));

    init_lader_mem();
    dmx_init(115200L);

    {   // за симулиране на контролер с LCD индикатор и ладер програма това е при стартиране
        bat_rs485 = 1;
        if (def_inp || def_out) {
            fl_disp = 1;
            printf("Start RS485\n");
            strcpy(str, "I:    O: ");
            while (err_prot_in_start) {
                if (flag_overlay_lader) {
                    printf("OVERLAY LADER");
                    bat_rs485 = 1;
                    while (1) {
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                }
                if (getchar() > 0) {
                    strcpy(str, "I:    O: ");
                }
                if (disp_err_inputs && (disp_err_inputs != 1)) {
                    sprintf(str + 2, "%c%.2d", disp_err_inputs, disp_err_cou_inputs);
                    str[5] = ' ';
                    disp_err_inputs = 1;
                }
                if (disp_err_outputs && (disp_err_outputs != 1)) {
                    sprintf(str + 8, "%c%.2d", disp_err_outputs, disp_err_cou_outputs);
                    disp_err_outputs = 1;
                }
                printf("%s\n", str);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            disp_err_inputs = disp_err_outputs = 0;
            printf("RS485 Started\n");
            fl_disp = 0;
        } else
            disp_err_inputs = disp_err_outputs = 0;
        bat_rs485 = 0;
        enable_lader = 1;
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        gpio_set_level(FreeTogle, !gpio_get_level(FreeTogle));
        if (xTaskGetTickCount() > (xLastWakeTime + 3)) {
            vTaskDelay(1);
            xLastWakeTime = xTaskGetTickCount();
        } else {    // за симулиране на контролер с LCD индикатор и ладер програма това е в работен цикъл
            if (flag_overlay_lader) {
                printf("OVERLAY LADER");
                bat_rs485 = 1;
                while (1) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
            if (disp_err_inputs || disp_err_outputs) {
                bat_rs485 = 1;
                printf("ERROR RS485\n");
                strcpy(str, "I:    O: ");

                while (1) {
                    if (getchar() > 0) {
                        printf("End error RS485\n");
                        break;
                    }
                    if (disp_err_inputs && (disp_err_inputs != 1)) {
                        sprintf(str + 2, "%c%.2d", disp_err_inputs, disp_err_cou_inputs);
                        str[5] = ' ';
                        disp_err_inputs = 1;
                    }
                    if (disp_err_outputs && (disp_err_outputs != 1)) {
                        sprintf(str + 8, "%c%.2d", disp_err_outputs, disp_err_cou_outputs);
                        disp_err_outputs = 1;
                    }
                    printf("%s\n", str);
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                disp_err_outputs = disp_err_inputs = 0;
                bat_rs485 = 0;
            }
        }
    }
}

// демо настройи на ладер
void my_lader_setings(void)
{
    def_inp = 16; // брой осмици входове по протокола
    def_out = 16; // брой осмици изходи по протокола

    time_out = 250; // време за нулиране на изходите след отпадене на протокола

    cou_err_485 = 1;//3;                 // колко грешки са допустими в протокола преди аларма
    def_spi = 0;                     // колоко осмици вход/изхода има по SPI
    type_spi = 1;                    //
    data_wait_to_lader_mul_10ms = 1; // на колко по 10ms да се пуска ладера minimum 1
}

// демо ладер
void my_lader(void)
{
    if (enable_lader) {
        // register uint8_t accum;
        // static uint8_t my_stack[50];
        // uint8_t *stack_p = my_stack;

        // accum = 0;
        // if (modbus_error > 10){
        //     modbus_error = 11;
        //     accum = 1;
        // }
        // WRT(ERR_MODBUS);

        // accum = 1;
        // SET(R_FIRST_CYCLE);

        static int cou = 0;
        if (++cou == 10 * 100) {    // на 10 секунди нови изходи
            cou = 0;
            output_mem[0] = input_mem[0];
        }
    }
}