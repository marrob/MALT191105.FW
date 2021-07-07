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


#define DIAG_FRAME_TYPE_TX   1
#define DIAG_FRAME_TYPE_RX   2


typedef struct{
    uint8_t Address;
    uint8_t Type;
    char Marker;
    uint32_t Counter;
}DiagItemTypeDef;

typedef struct{

  uint16_t ItemsCount;


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
void DiagAddFrame(DiagHandleTypeDef *h, uint8_t address, uint8_t type, char marker);
void DiagTask(DiagHandleTypeDef *h);

#endif
