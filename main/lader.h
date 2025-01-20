/********************************************************
*	      LADER.H					                                *
*  ОПИСАНИЕ НА ПОДПРОГРАМИ ЗА ЛАДЕРА			              *
*	ЕЛЛ - Д.Божилов 08.2000				                        *
********************************************************/

#include <stdlib.h>

/*	библиотеката 	L_ELL.R07	*/
/* 			LAD_ELL.R07 е за фирма ELL	*/

void send_inp_to_lader(void);       //прехвърля прочетените
//входове към ладера преди стартирането му 
void send_out_from_lader(void);     //прехвърля изработените
//от ладера изходи към протокола за изпращане към релейните платки


/*non_banked void lader(void);	ладера който се запуска с протокола
да си я направим	*/

extern void  (* next_lader)(void);

extern uint8_t def_inp,def_out,time_out,cou_err_485,def_spi,type_spi,data_wait_to_lader_mul_10ms;

extern uint8_t input_mem[];
extern uint8_t output_mem[];
//no_bacup_mem
extern uint8_t relay[];  
//bacup_mem
extern uint8_t timer[];  
extern uint8_t counter[];

void init_lader_mem(void);	//инициализира паметта за ладера lader.c
void my_lader(void);