/*
 * relays.h
 *
 *  Created on: 2020. márc. 5.
 *      Author: Margit Robert
 */

#ifndef SRC_RELAYS_H_
#define SRC_RELAYS_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "common.h"

/* Private defines -----------------------------------------------------------*/
#define RELAY_OK            0
#define RELAY_FAIL          1



#define RELAY_DIRVERS_CNT     DEVICE_RELAY_COUNT/8
#define RELAY_ARRAY           DEVICE_RELAY_COUNT/8  /*160db/8= 20, 32db/8 = 4 */
#define RELAY_MAX_BLOCK       DEVICE_RELAY_COUNT/8/4
#define RELAY_BLOCK_LENGTH    4


/* Exported types ------------------------------------------------------------*/
typedef struct {
  uint8_t PreState[RELAY_ARRAY];
  uint8_t PreBlockState[RELAY_ARRAY];
  uint8_t CurState[RELAY_ARRAY];
  uint32_t Counters[DEVICE_RELAY_COUNT];
  uint8_t ChangedBlocks[RELAY_MAX_BLOCK];
}RelayTypeDef;


void RelayEnable(void);

void RelayOnOne(RelayTypeDef *h, uint8_t k);
void RelayOffOne(RelayTypeDef *h, uint8_t k);

void RelayOffSeveral(RelayTypeDef *h, uint8_t *several, uint8_t block);
void RelayOnSeveral(RelayTypeDef *h, uint8_t *several, uint8_t block);
void RelayToogleSeveral(RelayTypeDef *h, uint8_t *several, uint8_t block);

uint32_t RelayCounterGet(RelayTypeDef *h, uint8_t relaynumber);
void RelayChangedBlocksUpdate(RelayTypeDef *h);

void RelaySpiTxRx(void *transmitt, void *receive, size_t size);
void RelayReset(RelayTypeDef *h);

uint8_t RelayDriverLoopTest(void);

#endif /* SRC_RELAYS_H_ */
