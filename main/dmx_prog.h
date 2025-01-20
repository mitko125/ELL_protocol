/********************************************************
*	      DMX_PROG.H				*
*  ОПИСАНИЕ НА ПОДПРОГРАМИ ЗА УПРАВЛЕНИЕ ПО DMX		*
*	ЕЛЛ - Д.Божилов 08.2000				*
********************************************************/

#include <stdlib.h>

/*ТЕЗИ БИБЛИОТЕКИ СА УСАВАРШЕНСТВАНИ , РАБОТЯТ И С SPI И ИМАТ БРОЯЧ
ЗА ГРЕШКИ ПО RS485	*/

/*	библиотеката 	DMX_C_V1.R07  е за платка 04100111,DMX_CON	*/
/* 			DMX_E_V1.R07 е за платка EXTR_MPU,различава се
само по сигнала DIR_DMX	*/

/*и двете библиотеки заемат прекъсването по SCI	*/

void dmx_init(uint32_t boude);

void start_lader(void);	/*стартира ладера и протокола
вика се от птекъсването за LCD, на 10ms при 115200 или 100ms при 9600
	прототип	в DMX.C	*/

/*следните променливи и функции трябва да се удоволетворяват от други
програми:
uint8_t def_inp;	брой осмици входове
uint8_t def_out;	брой осмици изходи
uint8_t time_out;  time_out е времето за което да не се ресетват релетата,
след отпадане на RS, то е в ms и трябва да е минимум 150% по цикъла на
 протокола  time_out=20 при 115200,	=200 при 9600
uint8_t cou_err_485;	брой грешки по RS485 min=1, преди съобщение
uint8_t def_spi;	брой осмици по SPI
uint8_t type_spi;	реализиран е само тип 1,четене по 1->0,шифт 0->1, енейбала е =0


void send_inp_to_lader(void);	прехвърля прочетените
входове към ладера преди стартирането му
void send_out_from_lader(void);	прехвърля изработените
от ладера изходи към протокола за изпращане към релейните платки
void lader(void);	ладера който се запуска с протокола
*/

/*чрез следващите променливи можем да следим за грешки в протокола,
за пример виж uint8_t KEY(void) в EDITOR.C	*/
extern uint8_t err_prot_in_start;	/*грешката в протокола е при стартирането му*/
extern uint8_t f_err_inputs;	/*флаг за грешка във входовете	*/
extern uint8_t disp_err_inputs;	/*вида на грешката		*/
extern uint8_t f_err_outputs;	/*флаг за грешка в изходите	*/
extern uint8_t disp_err_outputs;	/*вида на грешката		*/
extern uint8_t disp_err_cou_inputs;
extern uint8_t disp_err_cou_outputs;
extern uint8_t cou_inp,cou_out; //брой входове и изходи 

#ifdef LONG_DMX
#define SIZE_DMX 0x20
#else
#define SIZE_DMX 0x10
#endif
extern uint8_t bufer_inputs[SIZE_DMX];    	//буфери за входове и изходи (DMX_xxxx.s07)
extern uint8_t bufer_outputs[SIZE_DMX];
extern uint8_t buf_o_spi[0x10];
extern uint8_t buf_i_spi[0x10];