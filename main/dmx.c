// #pragma codeseg(RCODE)
// тук останаха само прототипи
#define DMX

#include "04100611.h"
#include <avr/interrupt.h>
/*#include <def.h>
#include <sub.h>
#include <var.h>
#include <key.h>
#include <stdio.h>
#include <string.h>*/
#include "dmx_prog.h"
#include "lader.h"

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
unsigned char bufer_inputs[SIZE_DMX]; // буфери за входове и изходи (DMX_xxxx.s07)
unsigned char bufer_outputs[SIZE_DMX];
unsigned char buf_o_spi[0x10];
unsigned char buf_i_spi[0x10];

unsigned char cou_inp;
unsigned char cou_out;           // брой входове и изходи
unsigned char err_prot_in_start; /*грешката в протокола е при стартирането му*/
unsigned char f_err_inputs;      /*флаг за грешка във входовете	*/
unsigned char disp_err_inputs;   /*вида на грешката		*/
unsigned char f_err_outputs;     /*флаг за грешка в изходите	*/
unsigned char disp_err_outputs;  /*вида на грешката		*/
unsigned char disp_err_cou_inputs;
unsigned char disp_err_cou_outputs;
static unsigned char c_err_inp;
static unsigned char c_err_out;
static unsigned char f_start_time_out;
static unsigned char int_err_rs485;
static unsigned char noi_clr_err_prot_in_start;

void start_input(void);
void start_outputs(void);
void to_start_outputs(void);
void send_time_out(void);

typedef enum
{
    NOT = 0,
    WAIT_INPUT_DATA = 1,
    WAIT_NACK_INP = 2,
    WAIT_ACK_OUT = 3,
    WAIT_NAK_TIME_OUT = 4
} POSITION;

static volatile POSITION position;

static unsigned char char_bufer[100];
static unsigned char crc, cou, cou_wait;
static unsigned char fl_end_tr;
static unsigned char *bufer_point;

#define set_transmit()                      \
    {                                       \
        register unsigned char sreg = SREG; \
        cli();                              \
        PORTE |= 0x0c;                      \
        SREG = sreg;                        \
    }
#define set_reciv()                         \
    {                                       \
        register unsigned char sreg = SREG; \
        cli();                              \
        PORTE &= ~0x0c;                     \
        SREG = sreg;                        \
    }
#define rs_e_int_transmit() (UCSR0B |= (_BV(TXCIE0)))
#define rs_e_int_reciv() (UCSR0B |= (_BV(RXCIE0)))
#define rs_d_interrupt() (UCSR0B &= ~(_BV(TXCIE0) | _BV(RXCIE0)))

void dmx_init(unsigned int boude)
{
    cou_inp = def_inp;
    cou_out = def_out;
    if (cou_inp || cou_out)
    {
        PORTE |= 0b00000010;
        DDRE |= 0b00001110;

        UCSR0A |= _BV(U2X0); // bode	115200
        UBRR0H = boude >> 8;
        UBRR0L = boude;
        UCSR0B = _BV(TXEN0) | _BV(RXEN0); // Enable RX & TX
    }
    {
        register unsigned char i;
        for (i = 0; i < SIZE_DMX; i++)
            bufer_outputs[i] = 0;
        for (i = 0; i < 0x10; i++)
            buf_o_spi[i] = 0;
        for (i = 0; i < SIZE_DMX; i++)
            bufer_inputs[i] = 0;
        for (i = 0; i < 0x10; i++)
            buf_i_spi[i] = 0;
    }
    position = NOT;

    cli();

    err_prot_in_start = 1;
    noi_clr_err_prot_in_start = 0;
    disp_err_cou_inputs = 0;
    disp_err_cou_outputs = 0;
    c_err_inp = 0;
    c_err_out = 0;
    if (cou_inp)
    {
        f_err_inputs = 'N';
    }
    else
    {
        f_err_inputs = 0;
    }
    disp_err_inputs = ' ';
    if (cou_out)
    {
        f_err_outputs = 'N';
    }
    else
    {
        f_err_outputs = 0;
    }
    disp_err_outputs = ' ';
    f_start_time_out = 0;
    if (def_spi)
    {
        if (type_spi != 1)
        {
            def_spi = 0;
        }
    }
    int_err_rs485 = cou_err_485;
    sei();
}

void out_bufer(void)
{
    cou_wait = 4;
    fl_end_tr = 0;
    bufer_point = char_bufer;
    set_transmit();
    rs_d_interrupt();
    UCSR0A |= _BV(TXC0);
    UDR0 = *bufer_point++;
    rs_e_int_transmit();
}

void start_input(void)
{
    if (def_spi)
    {
        register unsigned char i;
        register unsigned char sreg = SREG;
        cli();
        PORTE |= 0b01000000;
        PORTB &= ~0x01;
        SREG = sreg;
        for (i = 0; i < def_spi; i++)
        {
            SPDR = buf_o_spi[i];
            while ((SPSR & (_BV(SPIF))) == 0)
                ;
            buf_i_spi[i] = SPDR;
        }
        sreg = SREG;
        cli();
        PORTB |= 0x01;
        PORTE &= ~0b01000000;
        SREG = sreg;
    }
    if (position != NOT)
    {
        rs_d_interrupt();
        set_reciv();
        cou_wait = 0;
        f_err_inputs = 'O';
        c_err_inp = cou >> 1;
        to_start_outputs();
    }
    else
    {
        if (cou_inp)
        {
            cou = crc = 0;
            char_bufer[0] = COMAND_INP;
            char_bufer[1] = 0;
            position = WAIT_INPUT_DATA;
            out_bufer();
        }
        else
            to_start_outputs();
    }
}

void start_outputs(void)
{
    noi_clr_err_prot_in_start = 0;
    if (cou_out)
    { // OUTPUT
        register unsigned char *p, i;
        p = char_bufer;
        *p++ = COMAND_OUT;
        cou = crc = 0;
        for (i = 0; i < cou_out; i++)
        {
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

void send_time_out(void)
{
    noi_clr_err_prot_in_start = 1;
    if (time_out)
    { // TIME_OUT
        char_bufer[0] = COMAND_TIMING;
        crc = char_bufer[1] = DATA_HIGH | ((time_out >> 4) & 0x0f);
        crc += char_bufer[2] = DATA_HIGH | (time_out & 0x0f);
        char_bufer[3] = CRC_HIGH | ((crc >> 4) & 0x0f);
        char_bufer[4] = CRC_HIGH | (crc & 0x0f);
        char_bufer[5] = 0;
        position = WAIT_NAK_TIME_OUT;
        out_bufer();
    }
    else
        f_start_time_out = 0;
}

void to_start_outputs(void)
{
    if (cou_out)
    {
        if (f_start_time_out)
        {
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

void start_lader(void)
{
    { // if(cou_inp){
        register char err_in_prot;
        if ((err_in_prot = (f_err_inputs | f_err_outputs)))
        {
            if (err_prot_in_start == 0)
            {
                if (--int_err_rs485)
                {
                    f_err_inputs = 0;
                    c_err_inp = 0;
                    f_err_outputs = 0;
                    c_err_out = 0;
                    start_input();
                    return;
                }
            }
            int_err_rs485 = 1;
            if (f_err_inputs)
            {
                disp_err_inputs = f_err_inputs;
                disp_err_cou_inputs = c_err_inp;
            }
            if (f_err_outputs)
            {
                disp_err_outputs = f_err_outputs;
                disp_err_cou_outputs = c_err_out;
            }
        }
        else
        {
            int_err_rs485 = cou_err_485;
        }
        f_err_inputs = 0;
        c_err_inp = 0;
        f_err_outputs = 0;
        c_err_out = 0;

        if (err_in_prot)
        {
            register unsigned char i;
            for (i = 0; i < cou_out; i++)
                bufer_outputs[i] = 0;
            for (i = 0; i < def_spi; i++)
                buf_o_spi[i] = 0;
            start_input();
        }
        else
        {
            if (noi_clr_err_prot_in_start == 0)
                err_prot_in_start = 0;
            if (disp_err_inputs || disp_err_outputs)
            {
                register unsigned char i;
                for (i = 0; i < cou_out; i++)
                    bufer_outputs[i] = 0;
                for (i = 0; i < def_spi; i++)
                    buf_o_spi[i] = 0;
                start_input();
            }
            else
            {
                send_inp_to_lader();
                send_out_from_lader();
                start_input();
                my_lader();
            }
        }
    }
}

#ifndef MONITOR_RS0
// USART0_UDRE_vect
void USART0_RX_vect(void) __attribute__((signal));
void USART0_RX_vect(void)
{
    register unsigned char c_prot;
    if (UCSR0A & (_BV(FE0) | _BV(DOR0)))
    {
        c_prot = UDR0;
    }
    else
    {
        c_prot = UDR0;
        cou_wait = 3;
        switch (position)
        {
        case WAIT_INPUT_DATA:
            crc += c_prot;
            if (cou & 0x01)
            {
                bufer_inputs[cou >> 1] |= c_prot & 0x0f;
            }
            else
            {
                bufer_inputs[cou >> 1] = c_prot << 4;
            }
            if ((cou_inp << 1) == (++cou))
            {
                position = WAIT_NACK_INP;
                char_bufer[0] = CRC_HIGH | (crc >> 4);
                char_bufer[1] = CRC_HIGH | (crc & 0x0f);
                char_bufer[2] = 0;
                out_bufer();
            }
            return;
            break;
        case WAIT_ACK_OUT:
            if (c_prot != ACK_HIGH)
            {
                if (c_prot == NAK)
                {
                    f_err_outputs = 'C';
                    c_err_out = cou;
                }
                else if (c_prot == (ACK_HIGH + 1))
                {
                    f_err_outputs = 'R';
                    c_err_out = cou;
                    f_start_time_out = 1;
                }
                else if (c_prot == (ACK_HIGH + 2))
                {
                    f_err_outputs = 'P';
                    c_err_out = cou;
                    f_start_time_out = 1;
                }
                else
                {
                    f_err_outputs = '?';
                    c_err_out = cou;
                }
            }
            if ((++cou) == cou_out)
            {
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
            sei();
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

void USART0_TX_vect(void) __attribute__((signal));
void USART0_TX_vect(void)
{
    if (fl_end_tr)
    {
        if (cou_wait)
        {
            UDR0 = 0xff;
            cou_wait--;
            return;
        }
        rs_d_interrupt();
        fl_end_tr = 0;
        switch (position)
        {
        case NOT:
            return;
            break;
        case WAIT_INPUT_DATA:
            f_err_inputs = 'N';
            c_err_inp = cou >> 1;
            position = NOT;
#ifndef MULTITASKING
            sei();
#endif
            to_start_outputs();
            return;
            break;
        case WAIT_NACK_INP:
            position = NOT;
#ifndef MULTITASKING
            sei();
#endif
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
    }
    else
    {
        if (*bufer_point)
        {
            UDR0 = *bufer_point++;
        }
        else
        {
            fl_end_tr = 1;
            set_reciv();
            UDR0 = 0xff;
            cou_wait--;
            rs_e_int_reciv();
        }
    }
}
#endif
