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

static const char *TAG = "main";

// за симулиране на контролер с LCD индикатор и ладер програма
char enable_lader, bat_rs485, fl_disp, str[30];

extern void slave_main(void *arg);
extern void master_task(void *arg);

void app_main(void)
{
    // за тестове със сигнал анализатор
    gpio_config_t gpio_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pin_bit_mask = ((1ULL << Start1Ok0) | (1ULL << EventTogle) | (1ULL << TimerTogle) | (1ULL << Error1)),
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));
    // бутон
    gpio_config_t gpio_conf1 = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true,
        .pin_bit_mask = 1ULL << CONFIG_BUTTON_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_conf1));

    xTaskCreate(slave_main, "slave_main", 2 * 4096, NULL, 4, NULL);
    vTaskDelay(pdMS_TO_TICKS(10));

    init_lader_mem();
    dmx_init(115200L);

    {   // за симулиране на контролер с LCD индикатор и ладер програма това е при стартиране
        bat_rs485 = 1;
        if (def_inp) {
            fl_disp = 1;
            ESP_LOGI(TAG, "Start RS485");
            strcpy(str, "\2I:    O: ");
            while (err_prot_in_start) {
                if (flag_overlay_lader) {
                    ESP_LOGE(TAG, "OVERLAY LADER");
                    bat_rs485 = 1;
                    while (1)
                        vTaskDelay(pdMS_TO_TICKS(50));
                }
                if (getchar() > 0) {
                    strcpy(str, "\2I:    O: ");
                    disp_err_inputs = disp_err_outputs = 0;
                }
                bool fl_new_error = false;
                if (disp_err_inputs && (disp_err_inputs != 1)) {
                    sprintf(str + 3, "%c%.2d", disp_err_inputs, disp_err_cou_inputs);
                    str[6] = ' ';
                    disp_err_inputs = 1;
                    fl_new_error = true;
                }
                if (disp_err_outputs && (disp_err_outputs != 1)) {
                    sprintf(str + 9, "%c%.2d", disp_err_outputs, disp_err_cou_outputs);
                    disp_err_outputs = 1;
                    fl_new_error = true;
                }
                if (fl_new_error)
                    ESP_LOGW(TAG, "%s", str + 1);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            disp_err_inputs = disp_err_outputs = 0;
            ESP_LOGI(TAG, "RS485 Started");
            fl_disp = 0;
        } else
            disp_err_inputs = disp_err_outputs = 0;
        bat_rs485 = 0;
        enable_lader = 1;
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // гледаше кога е свободен процесора
        // gpio_set_level(TimerTogle, !gpio_get_level(TimerTogle));
        
        if (xTaskGetTickCount() > (xLastWakeTime + 3)) {
            vTaskDelay(1);
            xLastWakeTime = xTaskGetTickCount();
        } else {  

            if (1) {    //за грешки в протокола на ELL
                static uint32_t old_errors_in_inputs, old_errors_in_outputs;
                if ((getchar() > 0) || (old_errors_in_inputs != errors_in_inputs) || (old_errors_in_outputs != errors_in_outputs))
                    ESP_LOGW(TAG, "ELL protocol errors inp = %lu out = %lu", 
                        old_errors_in_inputs = errors_in_inputs, old_errors_in_outputs = errors_in_outputs);
                if ( !gpio_get_level(CONFIG_BUTTON_GPIO) )
                    errors_in_inputs = errors_in_outputs = 0;
            }

            // за симулиране на контролер с LCD индикатор и ладер програма това е в работен цикъл
            // подобно на uint8_t KEY(void); в EDITOR.C
            if (flag_overlay_lader) {
                ESP_LOGE(TAG,"OVERLAY LADER");
                bat_rs485 = 1;
                while (1) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
            if (disp_err_inputs || disp_err_outputs) {
                char str[30];
                bat_rs485 = 1;
                ESP_LOGE(TAG, "ERR RS485");
                strcpy(str, "\2I:    O: ");

                while (1) {
                    if (getchar() > 0)
                        break;
                    bool fl_new_error = false;
                    if (disp_err_inputs && (disp_err_inputs != 1)) {
                        sprintf(str + 3, "%c%.2d", disp_err_inputs, disp_err_cou_inputs);
                        str[6] = ' ';
                        disp_err_inputs = 1;
                        fl_new_error = true;
                    }
                    if (disp_err_outputs && (disp_err_outputs != 1)) {
                        sprintf(str + 9, "%c%.2d", disp_err_outputs, disp_err_cou_outputs);
                        disp_err_outputs = 1;
                        fl_new_error = true;
                    }
                    if (fl_new_error)
                        ESP_LOGE(TAG, "%s", str + 1);
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                disp_err_outputs = disp_err_inputs = 0;
                bat_rs485 = 0;
            }
        }
    }
}

// демо настройки на ладер
void my_lader_setings(void)
{
    def_inp = 16; // брой осмици входове по протокола
    def_out = 16; // брой осмици изходи по протокола

    time_out = 250; // време за нулиране на изходите след отпадене на протокола

    cou_err_485 = 1;//3;                 // колко грешки са допустими в протокола преди аларма
    def_spi = 0;                     // колоко осмици вход/изхода има по SPI
    type_spi = 1;                    //
    data_wait_to_lader_mul_10ms = 2; // на колко по 10ms да се пуска ладера minimum 1
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
        if (++cou == 10 * 100) {    // за демо на 10 секунди нови изходи
            cou = 0;
            output_mem[0] = input_mem[0];
        }
    }
}