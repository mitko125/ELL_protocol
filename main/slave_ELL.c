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

// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define ECHO_READ_TOUT (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks














#define OUT_COU 2 // брой осмици релейни изходи
#define INP_COU 1 // брой осмици цифрови входове

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
static uint8_t *pointer_out, cou_out;
static uint8_t flag_first, flag_set_relay;
static uint8_t time_wait_inp, data_time_wait_inp;
static uint8_t old_inp[INP_COU];
static uint8_t PIS;

enum
{ // вътршни състояния по протокола;
    NOT,
    OUT_COMAND,
    OUT_AKNOLICH,
    INP_COMAND,
    TIMING
} position;

static void set_relay(void) // задейства релетата
{
    time_wait = data_time_wait;
    //  PORTA=bufer[0];
    //  PORTC=bufer[1];

    static uint8_t old_relays[OUT_COU] = {0}; // за демо
    if (memcmp(old_relays, bufer, OUT_COU)) {
        memcpy(old_relays, bufer, OUT_COU);
        ESP_LOGI(TAG, "New Relays");
        ESP_LOG_BUFFER_HEX(TAG, old_relays, OUT_COU);
    }
}

// прекъсване в AVR ако шифтрегистъра на uart е празен
// реализация в AVR засега е изпълнено коствено в ESP32
void UART_TX_vect(void)
{
    // set_transmit();
    if (cou_out) {
        // UDR = *pointer_out++;
        cou_out--;
    } else {
        // set_reciv();
        if (flag_set_relay) {
            set_relay();
            flag_set_relay = 0;
        }
        // UCR &= 0xbf;
    }
}

void out_buf(uint8_t broj)
{
    uint8_t buf_send[1 + sizeof(char_buf)];
    buf_send[0] = 0xff;
    memcpy(buf_send + 1, char_buf, broj);

    if ((PIS & 0x20) == 0) {
        if (flag_first == 0) {
            goto first;
        } else {
            // UDR = 0xff;
            flag_first = 0;
            // с един ненужен 0xff за забавяне на данните
            uart_write_bytes(SLAVE_UART_PORT, buf_send, broj + 1);
        }
    } else {
first:
        flag_first = 0;
        // set_transmit();
        // UDR = *pointer_out++;
        cou_out--;
        // баз един ненужен 0xff за забавяне на данните
        uart_write_bytes(SLAVE_UART_PORT, buf_send + 1, broj);
    }
    uart_wait_tx_done(SLAVE_UART_PORT, portMAX_DELAY);
    cou_out = 0;
    if (flag_set_relay) {
        set_relay();
        flag_set_relay = 0;
    }

// реализация в AVR
//     pointer_out = char_buf;
//     cou_out = broj;
//     if ((PIS & 0x20) == 0) {
//         if (flag_first == 0) {
//             goto first;
//         } else {
//             // UDR = 0xff;
//             flag_first = 0;
//         }
//     } else {
// first:
//         flag_first = 0;
//         // set_transmit();
//         // UDR = *pointer_out++;
//         cou_out--;
//     }
//     // UCR |= 0x40;
}

static void reset_relay(void) // нулира релетата
{
    ESP_LOGE(TAG, "reset_relay");
}

static void read_inputs(void) // прочита цифровите входове във временен буфер и отговаря с него
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
    }
}

static void init_fun(void)
{
    reset_relay();

    flag_first = flag_set_relay = 0;
    
    number_out = NUMBER_OUT;
    number_inp = NUMBER_INP;

    PIS = 0;

#if BAUD_RATE == 115200
    PIS |= 0x02;    // за демо
#endif


    PIS |= 0x20;    // !!! БЕЗ ТОВА НЕ ТРЪГВА Т.Е. НЕ ОЩЕ НЕ СЪМ ГОТОВ С ПАУЗАТА ПРИ СМЯНА НА ПОСОКАТА


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
    
    // реализация в AVR
    // TCNT0 = MS; // прекъсване на 1ms
    // PORTD &= 0xf7; /*светодиод READY	*/
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
            //      printf("\n\rRelay OK  ");
            //      for(c=0;c<OUT_COU;c++)
            //	printf("%.2X",bufer[c]);
            //      printf("\n\r");
        } else { // грешка при релета
            ESP_LOGE(TAG, "Relay ERR");
        }
        position = NOT;
    };
}

static void put_inp(void) // извежда  цифровите входове през протокола
{
    if (cou == cou_broj) {
        out_buf(INP_COU * 2);
    }
}

static void read_byte(uint8_t c) // за четене данни от протокола,вика се от interrupt // реализация в AVR
{                      // да не се подават грешни данни или 0xff
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
                        ESP_LOGE(TAG, "INP ERR 0x%02X != 0x%02X", crc, buf_crc);
                    } else {
                        ;
                        //		printf("INP OK\n\r");
                    }
                    position = NOT;
                } else if (position == TIMING) { // crc за тайминг
                    if (crc == buf_crc) {
                        time_wait = data_time_wait = cou;
                        //	      printf("’Ђ‰Њ€Ќѓ %d\n\r",cou);
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

// прекъсване на 1mS
// реализация в AVR засега не е изпълнено в ESP32
// ISR(TIMER0_OVF_vect)
// {
//     uint8_t inp, chang;

//     TCNT0 = MS;

//     if (time_wait)
//         time_wait--;
//     if (data_time_wait_inp)
//     {
//         if ((--time_wait_inp) == 0)
//         {
//             time_wait_inp = data_time_wait_inp;
//             inp = get_inp();
//             chang = old_inp[0] ^ inp;
//             input_bufer[0] = (input_bufer[0] & chang) | (inp & (~chang));
//             old_inp[0] = inp;
//         }
//     }
// }

// оригиналния main от AVR
// реализация в AVR засега не е изпълнено в ESP32
int main(void)
{
    init_fun();
    while (1) {
#if OUT_COU != 0
        if (time_wait == 0) { // нулира релейните изходи при timewait
            reset_relay();
            if (ack_bufer == ACK_HIGH)
                ack_bufer = ACK_HIGH | 0x01;
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
    return (0);
}












void slave_task(void *arg)
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

    ESP_LOGI(TAG, "Start RS485 application test and configure UART.");

    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(SLAVE_UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, ESP_INTR_FLAG_SHARED));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(SLAVE_UART_PORT, &uart_config));

    ESP_LOGI(TAG, "UART set pins, mode and install driver.");

    // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_pin(SLAVE_UART_PORT, SLAVE_TXD, SLAVE_RXD, SLAVE_RTS, UART_PIN_NO_CHANGE));

    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(SLAVE_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

    // Set read timeout of UART TOUT feature
    ESP_ERROR_CHECK(uart_set_rx_timeout(SLAVE_UART_PORT, ECHO_READ_TOUT));

    // Allocate buffers for UART
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    ESP_LOGI(TAG, "UART start recieve loop.\r");

    uart_flush_input(SLAVE_UART_PORT);





    init_fun();

    while (1)
    {
        int len = uart_read_bytes(SLAVE_UART_PORT, data, 1, 0);

        if (len > 0) {
            // ESP_LOGI(TAG, "Received %u bytes:", len);

            // printf("[ ");
            for (int i = 0; i < len; i++) {
                printf("0x%.2X", (uint8_t)data[i]);
            }
            printf("\n");

            for (int i = 0; i < len; i++)
                if (data[i] != 0xff)
                        read_byte(data[i]);
        }
    }
    vTaskDelete(NULL);
}