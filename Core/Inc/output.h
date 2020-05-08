/*
 * relays.h
 *
 *  Created on: 2020. márc. 5.
 *      Author: Margit Robert
 */

#ifndef SRC_RELAYS_H_
#define SRC_OUTPUTS_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "common.h"

/* Private defines -----------------------------------------------------------*/
#define OUTPUT_OK            0
#define OUTPUT_FAIL          1



#define OUTPUT_DIRVERS_CNT     DEVICE_OUTPUT_COUNT/8
#define OUTPUT_ARRAY           DEVICE_OUTPUT_COUNT/8  /*160db/8= 20, 32db/8 = 4 */
#define OUTPUT_MAX_BLOCK       DEVICE_OUTPUT_COUNT/8/4
#define OUTPUT_BLOCK_SIZE    4


/* Exported types ------------------------------------------------------------*/
typedef struct
{
  uint32_t Count;
  uint8_t PreState[OUTPUT_ARRAY];
  uint8_t PreBlockState[OUTPUT_ARRAY];
  uint8_t CurState[OUTPUT_ARRAY];
  uint32_t Counters[DEVICE_OUTPUT_COUNT];
  uint8_t ChangedBlocks[OUTPUT_MAX_BLOCK];
}OutputTypeDef;


void OutputEnable(void);

void OutpuOneOn(OutputTypeDef *h, uint8_t k);
void OutputOneOff(OutputTypeDef *h, uint8_t k);

void OutputOffSeveral(OutputTypeDef *h, uint8_t *several, uint8_t block);
void OutputOnSeveral(OutputTypeDef *h, uint8_t *several, uint8_t block);
void OutputSeveralToogle(OutputTypeDef *h, uint8_t *several, uint8_t block);

uint32_t OutputCounterGet(OutputTypeDef *h, uint8_t relaynumber);
void OutputChangedBlocksUpdate(OutputTypeDef *h);

void OutputSpiTxRx(void *transmitt, void *receive, size_t size);
void OutputReset(OutputTypeDef *h);

uint8_t OutputDriverLoopTest(void);

#endif /* SRC_RELAYS_H_ */
