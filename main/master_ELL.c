/*************************************************************
 * 
 * ЕЛЛ - Д.Божилов 01.2025 прехвърлено на ESP32
 * 
 * сменен е файла с полубиблиотеката от dmx.c на master_ELL.c
 * 
 *************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "hal/uart_hal.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <esp_err.h>
#include "esp_timer.h"
#include "driver/gpio.h"

#include "protocol_ELL_defs.h"

#include "dmx_prog.h"
#include "lader.h"

static const char *TAG = "ELL_master";

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

#define COMAND_HIGH 0x00
#define COMAND_OUT 0x04
#define COMAND_INP 0x05
#define COMAND_TIMING 0x06
#define COMAND_AC 0x02
#define DATA_HIGH 0x30
#define CRC_HIGH 0x40
#define NAK 0x00
#define ACK_HIGH 0x50

#ifdef LONG_DMX
#define SIZE_DMX 0x20
#else
#define SIZE_DMX 0x10
#endif
uint8_t bufer_inputs[SIZE_DMX];
uint8_t bufer_outputs[SIZE_DMX];
uint8_t buf_o_spi[0x10];
uint8_t buf_i_spi[0x10];

uint8_t cou_inp;
uint8_t cou_out;
uint8_t err_prot_in_start;
uint8_t f_err_inputs;
uint8_t disp_err_inputs;
uint8_t f_err_outputs;
uint8_t disp_err_outputs;
uint8_t disp_err_cou_inputs;
uint8_t disp_err_cou_outputs;
uint8_t flag_overlay_lader;
static uint8_t c_err_inp;
static uint8_t c_err_out;
static uint8_t f_start_time_out;
static uint8_t int_err_rs485;
static uint8_t noi_clr_err_prot_in_start;

static void start_outputs(void);
static void to_start_outputs(void);
static void send_time_out(void);

typedef enum
{
    NOT = 0,
    WAIT_INPUT_DATA = 1,
    WAIT_NACK_INP = 2,
    WAIT_ACK_OUT = 3,
    WAIT_NAK_TIME_OUT = 4
} POSITION;

static volatile POSITION position;

static uint8_t char_bufer[100];
static uint8_t crc, cou, cou_wait;
static uint8_t fl_end_tr;
static uint8_t *bufer_point;

static volatile uint8_t enabled_reciv_int, enabled_transmit_int;
static void rs_d_interrupt(void)
{
    enabled_reciv_int = enabled_transmit_int = 0;
}
static void rs_e_int_reciv(void)
{
    enabled_reciv_int = 1;
}
static void rs_e_int_transmit(void)
{
    enabled_transmit_int = 1;
}

static void USART0_TX_vect(void);

static void out_bufer(void)
{
    cou_wait = 4;
    fl_end_tr = 0;
    bufer_point = char_bufer;

    ESP_ERROR_CHECK(uart_set_mode(MASTER_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));
    rs_d_interrupt();

    rs_e_int_transmit();

    uart_write_bytes(MASTER_UART_PORT, char_bufer, strlen((char *)char_bufer));
    uart_wait_tx_done(MASTER_UART_PORT, portMAX_DELAY);
    bufer_point += strlen((char *)char_bufer);
    while (enabled_transmit_int)
        USART0_TX_vect();
}

static void start_input(void)
{
    if (def_spi) {
        // остатъци от AVR в ESP32 ако има нужда ще се премине на I2C
    }
    if (position != NOT) {
        rs_d_interrupt();
        // set_reciv();
        ESP_ERROR_CHECK(uart_set_mode(MASTER_UART_PORT, UART_MODE_UART));
        cou_wait = 0;
        f_err_inputs = 'O';
        c_err_inp = cou >> 1;
        // flag_overlay_lader = 1;
        ESP_LOGE(TAG, "Lader OVERFLOW !!!");
        to_start_outputs();
    } else {
        if (cou_inp) {
            cou = crc = 0;
            char_bufer[0] = COMAND_INP;
            char_bufer[1] = 0;
            position = WAIT_INPUT_DATA;
            out_bufer();
        } else
            to_start_outputs();
    }
}

static void start_outputs(void)
{
    noi_clr_err_prot_in_start = 0;
    if (cou_out) { // OUTPUT
        register uint8_t *p, i;
        p = char_bufer;
        *p++ = COMAND_OUT;
        cou = crc = 0;
        for (i = 0; i < cou_out; i++) {
            *p = DATA_HIGH | ((bufer_outputs[i] >> 4) & 0x0f);
            crc += *p++;
            *p = DATA_HIGH | (bufer_outputs[i] & 0x0f);
            crc += *p++;
        }
        *p++ = CRC_HIGH | ((crc >> 4) & 0x0f);
        *p++ = CRC_HIGH | (crc & 0x0f);
        *p = 0;
        position = WAIT_ACK_OUT;
        out_bufer();
    }
}

static void send_time_out(void)
{
    noi_clr_err_prot_in_start = 1;
    if (time_out) { // TIME_OUT
        char_bufer[0] = COMAND_TIMING;
        crc = char_bufer[1] = DATA_HIGH | ((time_out >> 4) & 0x0f);
        crc += char_bufer[2] = DATA_HIGH | (time_out & 0x0f);
        char_bufer[3] = CRC_HIGH | ((crc >> 4) & 0x0f);
        char_bufer[4] = CRC_HIGH | (crc & 0x0f);
        char_bufer[5] = 0;
        position = WAIT_NAK_TIME_OUT;
        out_bufer();
    } else
        f_start_time_out = 0;
}

static void to_start_outputs(void)
{
    if (cou_out) {
        if (f_start_time_out) {
            send_time_out();
#ifndef MULTITASKING
            while (position != NOT)
                ;
#endif
        }
#ifdef MULTITASKING
        else
#endif
            start_outputs();
    }
}

static void start_lader(void)
{
    if (flag_overlay_lader == 0) { // if(cou_inp){
        register uint8_t err_in_prot;
        if ((err_in_prot = (f_err_inputs | f_err_outputs))) {
            if (err_prot_in_start == 0) {
                if (--int_err_rs485) {
                    f_err_inputs = 0;
                    c_err_inp = 0;
                    f_err_outputs = 0;
                    c_err_out = 0;
                    start_input();
                    return;
                }
            }
            int_err_rs485 = 1;
            if (f_err_inputs) {
                disp_err_inputs = f_err_inputs;
                disp_err_cou_inputs = c_err_inp;
            }
            if (f_err_outputs) {
                disp_err_outputs = f_err_outputs;
                disp_err_cou_outputs = c_err_out;
            }
        } else {
            int_err_rs485 = cou_err_485;
        }
        f_err_inputs = 0;
        c_err_inp = 0;
        f_err_outputs = 0;
        c_err_out = 0;

        if (err_in_prot) {
            register uint8_t i;
            for (i = 0; i < cou_out; i++)
                bufer_outputs[i] = 0;
            for (i = 0; i < def_spi; i++)
                buf_o_spi[i] = 0;
            start_input();
        } else {
            if (noi_clr_err_prot_in_start == 0)
                err_prot_in_start = 0;
            if (disp_err_inputs || disp_err_outputs) {
                register uint8_t i;
                for (i = 0; i < cou_out; i++)
                    bufer_outputs[i] = 0;
                for (i = 0; i < def_spi; i++)
                    buf_o_spi[i] = 0;
                start_input();
            } else {
                send_inp_to_lader();
                send_out_from_lader();
                start_input();
                my_lader();
            }
        }
    }
}

static void USART0_RX_vect(uint8_t c_prot)
{
    // if (UCSR0A & (_BV(FE0) | _BV(DOR0))) {
    //     c_prot = UDR0;
    // } else {
    //     c_prot = UDR0;
    {

        cou_wait = 3;
        switch (position) {
        case WAIT_INPUT_DATA:
            crc += c_prot;
            if (cou & 0x01) {
                bufer_inputs[cou >> 1] |= c_prot & 0x0f;
            } else {
                bufer_inputs[cou >> 1] = c_prot << 4;
            }
            if ((cou_inp << 1) == (++cou)) {
                position = WAIT_NACK_INP;
                char_bufer[0] = CRC_HIGH | (crc >> 4);
                char_bufer[1] = CRC_HIGH | (crc & 0x0f);
                char_bufer[2] = 0;
                out_bufer();
            }
            return;
            break;
        case WAIT_ACK_OUT:
            if (c_prot != ACK_HIGH) {
                if (c_prot == NAK) {
                    f_err_outputs = 'C';
                    c_err_out = cou;
                } else if (c_prot == (ACK_HIGH + 1)) {
                    f_err_outputs = 'R';
                    c_err_out = cou;
                    f_start_time_out = 1;
                } else if (c_prot == (ACK_HIGH + 2)) {
                    f_err_outputs = 'P';
                    c_err_out = cou;
                    f_start_time_out = 1;
                } else {
                    f_err_outputs = '?';
                    c_err_out = cou;
                }
            }
            if ((++cou) == cou_out) {
                position = NOT;
                cou_wait = 0;
                rs_d_interrupt();
            }
            return;
            break;
        case WAIT_NACK_INP:
            f_err_inputs = 'C';
            position = NOT;
            cou_wait = 0;
            rs_d_interrupt();
#ifndef MULTITASKING
            // sei();
#endif
            to_start_outputs();
            return;
            break;
        case WAIT_NAK_TIME_OUT:
            f_err_outputs = 'C';
            position = NOT;
            cou_wait = 0;
            rs_d_interrupt();
            return;
            break;
        case NOT:
            rs_d_interrupt();
            return;
            break;
        }
    }
}

static void USART0_TX_vect(void)
{
    if (fl_end_tr) {
        if (cou_wait) {
            // UDR0 = 0xff;
            uint8_t t_bute = 0xff;
            uart_write_bytes(MASTER_UART_PORT, &t_bute, 1);
            uart_wait_tx_done(MASTER_UART_PORT, portMAX_DELAY);
            cou_wait--;
            return;
        }
        rs_d_interrupt();
        fl_end_tr = 0;
        switch (position) {
        case NOT:
            return;
            break;
        case WAIT_INPUT_DATA:
            f_err_inputs = 'N';
            c_err_inp = cou >> 1;
            position = NOT;
            to_start_outputs();
            return;
            break;
        case WAIT_NACK_INP:
            position = NOT;
            to_start_outputs();
            return;
            break;
        case WAIT_ACK_OUT:
            f_err_outputs = 'N';
            c_err_out = cou;
            position = NOT;
            return;
            break;
        case WAIT_NAK_TIME_OUT:
            position = NOT;
            f_start_time_out = 0;
            return;
            break;
        }
    } else {
        if (*bufer_point) {
            // UDR0 = *bufer_point++;
            uint8_t t_bute = *bufer_point++;;
            uart_write_bytes(MASTER_UART_PORT, &t_bute, 1);
            uart_wait_tx_done(MASTER_UART_PORT, portMAX_DELAY);
        } else {
            fl_end_tr = 1;
            // set_reciv();
            ESP_ERROR_CHECK(uart_set_mode(MASTER_UART_PORT, UART_MODE_UART));
            // UDR0 = 0xff;
            uint8_t t_bute = 0xff;
            uart_write_bytes(MASTER_UART_PORT, &t_bute, 1);
            uart_wait_tx_done(MASTER_UART_PORT, portMAX_DELAY);
            cou_wait--;
            rs_e_int_reciv();
        }
    }
}

static QueueHandle_t uart0_queue;
static uint8_t dtmp[BUF_SIZE];

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;

    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {

            gpio_set_level(TOGLE_PIN2, !gpio_get_level(TOGLE_PIN2));

            switch (event.type) {
            case UART_DATA:
                uart_read_bytes(MASTER_UART_PORT, dtmp, event.size, portMAX_DELAY);
                for(int i = 0; i < event.size; i++)
                    if (dtmp[i] != 0xff)
                        USART0_RX_vect(dtmp[i]);
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGE(TAG, "hw fifo overflow");
                uart_flush_input(MASTER_UART_PORT);
                xQueueReset(uart0_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGE(TAG, "ring buffer full");
                uart_flush_input(MASTER_UART_PORT);
                xQueueReset(uart0_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGW(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGE(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGE(TAG, "uart frame error");
                break;
            //Others
            default:
                ESP_LOGW(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

static void master_task(void * arg)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = data_wait_to_lader_mul_10ms;
    BaseType_t xWasDelayed;
    xLastWakeTime = xTaskGetTickCount ();

    while (1) {
        
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xFrequency );
        // Извършете действие тук. Стойността xWasDelayed може да се използва за определяне
        // дали е пропуснат краен срок, ако кодът тук отне твърде много време.
        if (!xWasDelayed)
            ESP_LOGE(TAG, "Time Overflow");

        
        gpio_set_level(TOGLE_PIN1, !gpio_get_level(TOGLE_PIN1));
        start_lader();
        gpio_set_level(TOGLE_PIN1, !gpio_get_level(TOGLE_PIN1));

        // const char command = COMAND_INP;
        // uart_write_bytes(MASTER_UART_PORT, &command, 1);     
        // ESP_ERROR_CHECK(uart_wait_tx_done(MASTER_UART_PORT, portMAX_DELAY));
    }
    vTaskDelete(NULL);
}

void dmx_init(uint32_t boude)
{
    cou_inp = def_inp;
    cou_out = def_out;
    if (cou_inp || cou_out) {
        uart_config_t uart_config = {
            .baud_rate = boude,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        ESP_LOGI(TAG, "Start RS485 and configure UART.");
        ESP_ERROR_CHECK(uart_driver_install(MASTER_UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, ESP_INTR_FLAG_SHARED));
        ESP_ERROR_CHECK(uart_param_config(MASTER_UART_PORT, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(MASTER_UART_PORT, MASTER_TXD, MASTER_RXD, MASTER_RTS, UART_PIN_NO_CHANGE));
        ESP_ERROR_CHECK(uart_set_mode(MASTER_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

        ESP_LOGI(TAG, "UART start recieve loop.");
        uart_flush_input(MASTER_UART_PORT);
        uart_pattern_queue_reset(MASTER_UART_PORT, 20);

        //Create a task to handler UART event from ISR
        xTaskCreate(uart_event_task, "uart_event_task", 2 * 4096, NULL, 11, NULL);

        uart_intr_config_t uart_intr = {
            .intr_enable_mask = UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT | UART_INTR_TXFIFO_EMPTY,
            .rxfifo_full_thresh = 1,
            .rx_timeout_thresh = 1,
            .txfifo_empty_intr_thresh = 1,
        };
        ESP_ERROR_CHECK(uart_intr_config(MASTER_UART_PORT, &uart_intr));
    }

    position = NOT;

    err_prot_in_start = 1;
    noi_clr_err_prot_in_start = 0;
    disp_err_cou_inputs = 0;
    disp_err_cou_outputs = 0;
    c_err_inp = 0;
    c_err_out = 0;
    if (cou_inp)
        f_err_inputs = 'N';
    else
        f_err_inputs = 0;
    disp_err_inputs = ' ';
    if (cou_out)
        f_err_outputs = 'N';
    else
        f_err_outputs = 0;
    disp_err_outputs = ' ';
    f_start_time_out = 0;
    if (def_spi)
        if (type_spi != 1)
            def_spi = 0;
    int_err_rs485 = cou_err_485;

    xTaskCreate(master_task, "master_task", 2 * 4096, NULL, 4, NULL);
}