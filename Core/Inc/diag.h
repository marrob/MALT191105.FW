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
#include <string.h>
/* Private defines -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/



typedef struct{
    uint8_t Byte1;
    uint32_t Counter;
}DiagItemTypeDef;

typedef struct{

  struct _statusCounters
  {
    uint32_t CanRx;
    uint32_t CanRxErr;
    uint32_t UnknownFrame;
    uint32_t CanTx;
    uint32_t CanTxErr;
    uint32_t CanTxNoMailBox;
    uint32_t MemFail;
    uint32_t MemSaved;
    uint32_t MainCycleTime;
    uint32_t UpTimeSec;
  }Status;
  struct _selfTest
  {
     uint8_t MemoryState;
     uint8_t DriverLoopState;
  }SelfTest;

}DiagHandleTypeDef;

void DiagInit(DiagHandleTypeDef *h);
void DiagRxFrame(DiagHandleTypeDef *h, uint8_t *data, size_t length);
void DiagTxFrame(DiagHandleTypeDef *h, uint8_t *data, size_t length);
void DiagTask(DiagHandleTypeDef *h);

#endif
