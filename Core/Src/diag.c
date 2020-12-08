
/* USER CODE BEGIN Header */
/*
 * frame_diag.c
 *
 *  Created on: 2020. márc. 6.
 *      Author: Margit Robert
 */
/* Includes ------------------------------------------------------------------*/
#include "diag.h"



/*
0123456789012345
00 RX 0x00000000 |00 RX 0x00000000 |00 RX 0x00000000 |00 RX 0x00000000 |
 */

#define _DIAG_RX  0
#define _DIAG_TX  1

DiagItem_Type Diags[] = {
    {0x01, _DIAG_RX, 0x00000000 },
    {0x03, _DIAG_RX, 0x00000000 },
    {0x04, _DIAG_RX, 0x00000000 },
};


void DiagInit(void){

}
