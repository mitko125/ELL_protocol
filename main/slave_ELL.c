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
#include "driver/gptimer.h"

static const char *TAG = "ELL_slave";

#define LEFT_UART 1

#ifdef LEFT_UART
#define SLAVE_UART_PORT UART_NUM_1
#define SLAVE_TXD CONFIG_RS485_UART1_TXD
#define SLAVE_RXD CONFIG_RS485_UART1_RXD
#define SLAVE_RTS CONFIG_RS485_UART1_RTS
#else // LEFT_UART
#define SLAVE_UART_PORT UART_NUM_2
#define SLAVE_TXD CONFIG_RS485_UART2_TXD
#define SLAVE_RXD CONFIG_RS485_UART2_RXD
#define SLAVE_RTS CONFIG_RS485_UART2_RTS
#endif // LEFT_UART

#define BUF_SIZE (128)

#define COMAND_HIGH 0x00
#define COMAND_OUT 0x04
#define COMAND_INP 0x05
#define COMAND_TIMING 0x06
#define DATA_HIGH 0x30
#define CRC_HIGH 0x40
#define NAK 0x00
#define ACK_HIGH 0x50

#define OUT_COU 1 // брой осмици релейни изходи
#define INP_COU 2 // брой осмици цифрови входове
#define BAUD_RATE 115200    // !!! избира се с PIS

// номер на вх/изх. настройват се с джъмпери
#define NUMBER_OUT 0    
#define NUMBER_INP 0

static uint8_t char_buf[1 + OUT_COU * 2 + INP_COU * 2];
static uint8_t bufer[OUT_COU];
static uint8_t input_bufer[INP_COU];
static uint8_t c, buf, crc, buf_crc;
static uint8_t number_out, number_inp;
static uint8_t cou, cou_bufer, cou_broj, cou_crc;
static uint8_t ack_bufer;
static uint8_t time_wait, data_time_wait;
static uint8_t flag_first, flag_set_relay;
static uint8_t time_wait_inp, data_time_wait_inp;
static uint8_t old_inp[INP_COU];
static uint8_t PIS;

// вътршни състояния по протокола;
enum
{ 
    NOT,
    OUT_COMAND,
    OUT_AKNOLICH,
    INP_COMAND,
    TIMING
} position;

// задейства релетата
static void set_relay(void) 
{
    time_wait = data_time_wait;
    // !!!
    //  PORTA=bufer[0];
    //  PORTC=bufer[1];

    static uint8_t old_relays[OUT_COU] = {0}; // за демо
    if (memcmp(old_relays, bufer, OUT_COU)) {
        memcpy(old_relays, bufer, OUT_COU);
        ESP_LOGI(TAG, "New Relays");
        ESP_LOG_BUFFER_HEX(TAG, old_relays, OUT_COU);
    }
}

static gptimer_handle_t timer_one_byte = NULL;
static uint8_t bytes_to_send_in_cb = 0;

// извежда данни по UART
static void out_buf(uint8_t bytes_to_send)
{
    uint8_t buf_send[1 + sizeof(char_buf)];
    buf_send[0] = 0xff;
    memcpy(buf_send + 1, char_buf, bytes_to_send);

    if ((PIS & 0x20) == 0) {
        if (flag_first == 0) {
            goto first;
        } else {
            flag_first = 0;
            // забавяне с един байт на данните
            bytes_to_send_in_cb = bytes_to_send;
            ESP_ERROR_CHECK(gptimer_set_raw_count(timer_one_byte, 0));
            ESP_ERROR_CHECK(gptimer_start(timer_one_byte));
        }
    } else {
first:
        flag_first = 0;
        // баз забавяне с един байт на данните
        uart_write_bytes(SLAVE_UART_PORT, buf_send + 1, bytes_to_send);
        if (flag_set_relay) {
            set_relay();
            flag_set_relay = 0;
        }
    }
}

// нулира релетата при таймоут
static void reset_relay(void)
{
    // !!!
    ;
}

// прочита (синхронно за всички платки) цифровите входове във временен буфер и ще отговаря с него
static void read_inputs(void)
{
    if (data_time_wait_inp == 0) {
        static uint8_t inp0 = 0;    // за демо
        static int opi = 0;
        if (++opi == 10) {
            opi = 0;
            inp0 ++;
        }
        for (int i = 0; i < INP_COU; i++)
            input_bufer[i] = 0;//inp0;
            // !!!
    }
}

// инициализизиране на UART
static void uart_init(void);

static bool IRAM_ATTR timer_one_byte_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    // stop timer immediately
    gptimer_stop(timer);

    uart_write_bytes(SLAVE_UART_PORT, char_buf, bytes_to_send_in_cb);
    if (flag_set_relay) {
        set_relay();
        flag_set_relay = 0;
    }
    
    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

// първоначална инициализация
static void init_fun(void)
{
    ESP_LOGI(TAG, "Create timer handle");
    
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = BAUD_RATE,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_one_byte));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_one_byte_on_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_one_byte, &cbs, NULL));

    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(timer_one_byte));

    gptimer_alarm_config_t alarm_one_byte = {
        .alarm_count = 10, // 1stat + 8data + 1stop bits
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_one_byte, &alarm_one_byte));

    uart_init();

    reset_relay();

    flag_first = flag_set_relay = 0;
    
    number_out = NUMBER_OUT;    // !!! настройват се с джъмпери
    number_inp = NUMBER_INP;    // !!! настройват се с джъмпери

    PIS = 0;        // !!! настройват се с джъмпери

    // PIS |= 0x20;    // !!! за тестове без пауза за забавяне на данните махнете закоментаряването

#if BAUD_RATE == 115200
    PIS |= 0x02;
#endif

    if (PIS & 0x80)
        number_out += 16;
    if (PIS & 0x40)
        number_inp += 16;

    position = NOT;
    if ((PIS & 0x02) == 0) { // bode	9600
        data_time_wait = 100;
    } else { // bode	115200
        data_time_wait = 10;
    }

    // за цифрово филтриране на трепещи контакти при входовете
    time_wait = data_time_wait;
    data_time_wait_inp = 0;
    read_inputs();
    for (c = 0; c < INP_COU; c++)
        old_inp[c] = input_bufer[c] = 0;
    if ((PIS & 0x08) == 0)
        if ((PIS & 0x10) == 0)
            data_time_wait_inp = 0; // 0mS
        else
            data_time_wait_inp = 5; // 5ms
    else if ((PIS & 0x10) == 0)
        data_time_wait_inp = 2; // 2mS
    else
        data_time_wait_inp = 10; // 10ms
    time_wait_inp = data_time_wait_inp;
    ack_bufer = ACK_HIGH | 0x02;
}

static void ack_relay(void) // извежда ACK,NACK за релейните изходи по протокола
{
#if OUT_COU != 0
    uint8_t c;
#endif

    if (cou == number_out) {
#if OUT_COU != 0
        for (c = 0; c < OUT_COU; c++)
            char_buf[c] = buf_crc;
#endif
        cou += OUT_COU;
        if ((cou == cou_broj) && ((crc & 0xf0) == ACK_HIGH)) {
            flag_set_relay = 1;
        }
        out_buf(OUT_COU);
    } else
        cou++;
    if (cou == cou_broj) {
        if ((crc & 0xf0) == ACK_HIGH) { // изправно приети релета
            ack_bufer = ACK_HIGH;
            if (flag_set_relay == 0)
                set_relay();
        } else { // грешка при релета
            ESP_LOGE(TAG, "Relay ERR");
        }
        position = NOT;
    };
}

// извежда  цифровите входове през UART
static void put_inp(void)
{
    if (cou == cou_broj) {
        out_buf(INP_COU * 2);
    }
}

// за четене данни от протокола,вика се от interrupt // реализация в AVR
// да не се подават грешни данни или 0xff
static void read_byte(uint8_t c)
{
    if (c == NAK) { // за NAK
        if (position == OUT_AKNOLICH) { // NAK за релаета
            crc = NAK;
            ack_relay();
        }
    } else if ((buf = (c & 0xf0)) == COMAND_HIGH) { // за КОМАНДИ
#if OUT_COU != 0
        if (c == COMAND_OUT) { // за релета
            position = OUT_COMAND;
            cou_broj = cou_bufer = 0;
            cou = number_out << 1;
            crc = 0;
        } else if (c == COMAND_TIMING) { // за тайминг
            position = TIMING;
            cou = crc = 0;
        } else
#endif
#if INP_COU != 0
            if (c == COMAND_INP) { // за входове
            read_inputs();
            position = INP_COMAND;
            cou_broj = 0;
            cou = number_inp << 1;
            crc = 0;
            for (c = 0; c < (INP_COU * 2); c += 2) {
                uint8_t buf;
                crc += char_buf[c] = DATA_HIGH | (((buf = input_bufer[c >> 1]) >> 4) & 0x0f);
                crc += char_buf[c + 1] = DATA_HIGH | (buf & 0x0f);
            }
            char_buf[c] = 0;
#ifdef RIG
            input_bufer[1] = 0;
#endif
            if (number_inp == 0)
                flag_first = 1;
            put_inp();
        } else
#endif
        { // само ако не е наша команда
            position = NOT;
        }
        cou_crc = buf_crc = 0;
    } else if (buf == DATA_HIGH) { // за ДАННИ
        cou_broj++;
        crc += c;
        if (position == OUT_COMAND) { // данни за релета
            if (cou) {
                cou--;
            } else {
#if OUT_COU != 0
                if (cou_bufer < (OUT_COU * 2)) {

                    if (cou_bufer & 1)
                        bufer[cou_bufer >> 1] |= (c & 0x0f);
                    else
                        bufer[cou_bufer >> 1] = (c << 4);
                    cou_bufer++;
                }
#endif
            }
        } else if (position == INP_COMAND) { // данни от входове
            put_inp();
        } else if (position == TIMING) { // данни за тайминг
            cou <<= 4;
            cou |= (c & 0x0f);
        }
    } else if (buf == CRC_HIGH) { // за CRC
        if (position != NOT) {
            if (cou_crc) {
                buf_crc |= (c & 0x0f);
                if (position == OUT_COMAND) { // crc за релета
                    if (number_out == 0)
                        flag_first = 1;
                    if (crc == buf_crc)
                        buf_crc = crc = ack_bufer;
                    else
                        buf_crc = crc = NAK;
                    cou_broj >>= 1;
                    cou = 0;
                    position = OUT_AKNOLICH;
                    ack_relay();
                } else if (position == INP_COMAND) { // crc за входове
                    if (crc != buf_crc) {
                        flag_first = 1;
                        char_buf[0] = NAK;
                        out_buf(1);
                        ESP_LOGE(TAG, "INP ERR crc 0x%02X != 0x%02X", crc, buf_crc);
                    } else {
                        ESP_LOGD(TAG, "INP OK");
                    }
                    position = NOT;
                } else if (position == TIMING) { // crc за тайминг
                    if (crc == buf_crc) {
                        time_wait = data_time_wait = cou;
                        ESP_LOGI(TAG, "TIMING Set = %d", cou);
                    } else {
                        flag_first = 1;
                        char_buf[0] = NAK;
                        out_buf(1);
                        ESP_LOGE(TAG, "TIMING ERR");
                    }
                    position = NOT;
                }
            } else {
                buf_crc = (c << 4);
                cou_crc++;
            }
        }
    } else if (buf == ACK_HIGH) { // за ACK
        if (position == OUT_AKNOLICH) { // за релета
            ack_relay();
        }
    } else { // за други
        position = NOT;
    }
}

// оригиналния main от AVR преправен за ESP32
void slave_main(void *arg)
{
    init_fun();

    uint32_t old_time = esp_log_timestamp();

    while (1) {
        
        // вместо реализация в AVR прекъсване на 1mS
        uint32_t new_time = esp_log_timestamp();
        for (int i = new_time - old_time; i; i--) {
            if (time_wait)
                time_wait--;
            if (data_time_wait_inp) {
                if ((--time_wait_inp) == 0) {
                    // цифрова филтрация на входовете, никога не ни е трябвала
                    // uint8_t inp, chang;
                    // time_wait_inp = data_time_wait_inp;
                    // inp = get_inp();
                    // chang = old_inp[0] ^ inp;
                    // input_bufer[0] = (input_bufer[0] & chang) | (inp & (~chang));
                    // old_inp[0] = inp;
                }
            }
        }
        old_time = new_time;

        vTaskDelay(1);

#if OUT_COU != 0
        if ((data_time_wait == 10) || (time_wait == 0)) {    // за да симулираме искане за TIMING
        // if (time_wait == 0) { // нулира релейните изходи при timewait
            reset_relay();
            if (ack_bufer == ACK_HIGH) {
                ack_bufer = ACK_HIGH | 0x01;
                ESP_LOGE(TAG, "reset_relay");
            }
            time_wait = data_time_wait;
        }
#else
        ;
#endif
#ifdef ANALOG_3
        store_dac();
#endif

#ifdef RESOLVER
        resolver_SPI();
#endif
    }

    vTaskDelete(NULL);
}

static QueueHandle_t uart0_queue;
static uint8_t dtmp[BUF_SIZE];

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;

    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
            case UART_DATA:
                uart_read_bytes(SLAVE_UART_PORT, dtmp, event.size, portMAX_DELAY);
                for(int i = 0; i < event.size; i++) {
                    if (dtmp[i] != 0xff)
                        read_byte(dtmp[i]);
                }
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGE(TAG, "hw fifo overflow");
                uart_flush_input(SLAVE_UART_PORT);
                xQueueReset(uart0_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGE(TAG, "ring buffer full");
                uart_flush_input(SLAVE_UART_PORT);
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

static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG, "Start RS485 and configure UART.");

    ESP_ERROR_CHECK(uart_driver_install(SLAVE_UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, ESP_INTR_FLAG_SHARED));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(SLAVE_UART_PORT, &uart_config));

    // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_pin(SLAVE_UART_PORT, SLAVE_TXD, SLAVE_RXD, SLAVE_RTS, UART_PIN_NO_CHANGE));

    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(SLAVE_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG, "UART start recieve loop.");

    uart_flush_input(SLAVE_UART_PORT);

    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(SLAVE_UART_PORT, 20);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2 * 4096, NULL, 11, NULL);

    ESP_ERROR_CHECK(uart_set_rx_full_threshold(SLAVE_UART_PORT, 1));
}