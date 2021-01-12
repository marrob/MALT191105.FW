
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "vt100.h"


char* Vt100SetCursorPos(uint8_t x, uint8_t y)
{
  static char temp[16];
  sprintf(temp, "\033[%d;%dH",y , x);
  return temp;
}
