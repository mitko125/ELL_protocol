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

uint8_t enable_lader;

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
    vTaskDelay(10 / portTICK_PERIOD_MS);

    init_lader_mem();
    dmx_init(115200L);

    // bat_rs485 = 1;

    // if (def_inp) {
    //     fl_disp = 1;
    //     prn_ind_P(PSTR("\1Start RS485"));
    //     strcpy_P(str, PSTR("\2I:    O: "));
    //     while (err_prot_in_start) {
    //         extern char key_buf;
    //         if (flag_overlay_lader) {
    //             prn_ind_P(PSTR("\1OVERLAY LADER"));
    //             bat_rs485 = 1;
    //             while (1) {
    //                 NutSleep(50);
    //             }
    //         }
    //         if (key_buf) {
    //             inpkey();
    //             strcpy_P(str, PSTR("\2I:    O: "));
    //         }
    //         if (disp_err_inputs && (disp_err_inputs != 1)) {
    //             sprintf_P(str + 3, PSTR("%c%.2d"), disp_err_inputs, disp_err_cou_inputs);
    //             str[6] = ' ';
    //             disp_err_inputs = 1;
    //         }
    //         if (disp_err_outputs && (disp_err_outputs != 1)) {
    //             sprintf_P(str + 9, PSTR("%c%.2d"), disp_err_outputs, disp_err_cou_outputs);
    //             disp_err_outputs = 1;
    //         }
    //         prn_ind(str);
    //         NutSleep(10);
    //     }
    //     disp_err_inputs = disp_err_outputs = 0;
    //     prn_ind_P(PSTR("\1"));
    //     prn_ind_P(PSTR("\2"));
    //     fl_disp = 0;
    // } else
    //     disp_err_inputs = disp_err_outputs = 0;
    // bat_rs485 = 0;
    enable_lader = 1;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        gpio_set_level(FreeTogle, !gpio_get_level(FreeTogle));
        if (xTaskGetTickCount() > (xLastWakeTime + 3)) {
            vTaskDelay(1);
            xLastWakeTime = xTaskGetTickCount();
        }
    }

}

void my_lader_setings(void)
{
#ifdef SIMULATOR
    def_inp = 0; // 1;//DMX_INPUTS;      //брой осмици входове по протокола
    def_out = 0; // 2;//3;      //брой осмици изходи по протокола
#else
    def_inp = 2; // брой осмици входове по протокола
    def_out = 1; // брой осмици изходи по протокола
#endif
    time_out = 250; // време за нулиране на изходите след отпадене на протокола

    cou_err_485 = 1;//3;                 // колко грешки са допустими в протокола преди аларма
    def_spi = 0;                     // колоко осмици вход/изхода има по SPI
    type_spi = 1;                    //
    data_wait_to_lader_mul_10ms = 1; // на колко по 10ms да се пуска ладера minimum 1
}

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
    }
}