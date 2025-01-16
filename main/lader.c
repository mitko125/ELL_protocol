#include "lader.h"
#include "dmx_prog.h"

void (*next_lader)(void);

unsigned char def_inp, def_out, time_out, cou_err_485, def_spi, type_spi, data_wait_to_lader_mul_10ms;
void my_lader_setings(void);

unsigned char input_mem[0x20] __attribute__((section(".noinit")));
unsigned char output_mem[0x20] __attribute__((section(".noinit")));
// unsigned char ;//no_bacup_mem
unsigned char relay[0x20] __attribute__((section(".noinit")));
// bacup_mem
unsigned char timer[0x10] __attribute__((section(".noinit")));
unsigned char counter[0x10] __attribute__((section(".noinit")));

void init_lader_mem(void)
{
    my_lader_setings();

    {
        register unsigned char i;
        for (i = 0; i < 0x20; i++)
            input_mem[i] = 0;
        for (i = 0; i < 0x20; i++)
            output_mem[i] = 0;
        for (i = 0; i < 0x20; i++)
            relay[i] = 0;
    }
    next_lader = 0;
}

void send_inp_to_lader(void)
{
    register unsigned char i;
    for (i = 0; i < cou_inp; i++)
        input_mem[i] = bufer_inputs[i];
    for (i = 0; i < def_spi; i++)
#ifdef LONG_DMX
        input_mem[i + cou_inp] = buf_i_spi[i];
#else
        input_mem[i + 0x10] = buf_i_spi[i];
#endif
}

void send_out_from_lader(void)
{
    register unsigned char i;
    for (i = 0; i < cou_out; i++)
        bufer_outputs[i] = output_mem[i];
    for (i = 0; i < def_spi; i++)
#ifdef LONG_DMX
        buf_o_spi[i] = output_mem[i + cou_out];
#else
        buf_o_spi[i] = output_mem[i + 0x10];
#endif
}
