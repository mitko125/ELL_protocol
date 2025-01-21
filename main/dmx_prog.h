/********************************************************
*	      DMX_PROG.H
*  ОПИСАНИЕ НА ПОДПРОГРАМИ ЗА УПРАВЛЕНИЕ ПО DMX
*	ЕЛЛ - Д.Божилов 08.2000
*
*   ЕЛЛ - Д.Божилов 01.2025 прехвърлено на ESP32
********************************************************/

#include <stdlib.h>

// инициализира протокола
void dmx_init(uint32_t boude);

/*стартира ладера и протокола вика се от птекъсването за LCD, на 10ms при 115200 или 100ms при 9600
void start_lader(void);	
но вече за нея има грижата dmx_init(..  */

/*чрез следващите променливи можем да следим за грешки в протокола,
за пример виж uint8_t KEY(void) в EDITOR.C	*/
extern uint8_t err_prot_in_start;	/*грешката в протокола е при стартирането му*/
extern uint8_t f_err_inputs;	/*флаг за грешка във входовете	*/
extern uint8_t disp_err_inputs;	/*вида на грешката		*/
extern uint8_t f_err_outputs;	/*флаг за грешка в изходите	*/
extern uint8_t disp_err_outputs;	/*вида на грешката		*/
extern uint8_t disp_err_cou_inputs; /*при коя осмица е настъпила грешката   */
extern uint8_t disp_err_cou_outputs; /*при коя осмица е настъпила грешката   */
extern uint8_t cou_inp,cou_out; //брой входове и изходи 
extern uint8_t flag_overlay_lader;  /*ладера се е препокрил	*/

// остатъци от AVR при нужда от повече вх/изх. по протокола взимахме от SPI
#ifdef LONG_DMX
#define SIZE_DMX 0x20
#else
#define SIZE_DMX 0x10
#endif
extern uint8_t bufer_inputs[SIZE_DMX], bufer_outputs[SIZE_DMX];    	//буфери за входове и изходи
extern uint8_t buf_o_spi[0x10], buf_i_spi[0x10]; //буфери за входове и изходи по SPI, остатъци от AVR в ESP32 ако има нужда ще се премине на I2C