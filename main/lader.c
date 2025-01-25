#include <string.h>

#include "lader.h"
#include "dmx_prog.h"

uint8_t def_inp, def_out, time_out, cou_err_485, def_spi, type_spi, data_wait_to_lader_mul_10ms;
void my_lader_setings(void);

uint8_t input_mem[0x20];
uint8_t output_mem[0x20];
uint8_t relay[0x20];
uint8_t timer[0x10];
uint8_t counter[0x10];

void init_lader_mem(void)
{
    my_lader_setings();
}

void send_inp_to_lader(void)
{
    memcpy(input_mem, bufer_inputs, cou_inp);
#ifdef LONG_DMX
    memcpy(input_mem + cou_inp, buf_i_spi, def_spi);
#else
    memcpy(input_mem + 0x10, buf_i_spi, def_spi);
#endif
}

void send_out_from_lader(void)
{
    memcpy(bufer_outputs, output_mem, cou_out);
#ifdef LONG_DMX
    memcpy(buf_o_spi, output_mem + cou_out, def_spi);
#else
    memcpy(buf_o_spi, output_mem + 0x10, def_spi);
#endif
}