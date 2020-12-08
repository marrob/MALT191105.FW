/*
 * relays.h
 *
 *  Created on: 2020. márc. 5.
 *      Author: Margit Robert
 */
#ifndef SRC_DIAG_H_
#define SRC_DIAG_H_ 1

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Private defines -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct{
    uint8_t Byte1;
    uint8_t Dir;
    uint32_t Counter;
}DiagItem_Type;

void DiagInit(void);
#endif
