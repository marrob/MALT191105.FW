
#ifndef VT100__H_
#define VT100__H_ 1

/* VT100 ---------------------------------------------------------------------*/
/*
 * https://www.csie.ntu.edu.tw/~r92094/c++/VT100.html
 * http://www.termsys.demon.co.uk/vtansi.htm
 */

/* Private defines -----------------------------------------------------------*/

#define VT100_CLEARSCREEN         "\033[2J"
#define VT100_CURSORHOME          "\033[H"
#define VT100_ATTR_RESET          "\033[0m"
#define VT100_ATTR_RED            "\033[31m"
#define VT100_ATTR_GREEN          "\033[32m"
#define VT100_ATTR_YELLOW         "\033[33m"
#define VT100_CUP(__v__,__h__)    ("\033["__v__";"__h__"H") /*Cursor Position*/

/* Exported types ------------------------------------------------------------*/



char* Vt100SetCursorPos(uint8_t x, uint8_t y);

#endif //VT100__H_
