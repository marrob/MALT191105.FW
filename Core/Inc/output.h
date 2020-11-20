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



#define SPI_IO_ARRAY_SIZE      DEVICE_OUTPUTS_COUNT/8  /*160db/8= 20, 32db/8 = 4 */




/* Exported types ------------------------------------------------------------*/
typedef struct
{
  uint32_t Count;
  uint8_t PreState[SPI_IO_ARRAY_SIZE];
  uint8_t PreBlockState[SPI_IO_ARRAY_SIZE];
  uint8_t CurState[SPI_IO_ARRAY_SIZE];
  uint32_t Counters[DEVICE_OUTPUTS_COUNT];
  uint8_t ChangedBlocks[DEVICE_BLOCKS];

  struct{
  uint8_t CurState[SPI_IO_ARRAY_SIZE];

  }Inputs;
}OutputTypeDef;


void IoInit(OutputTypeDef *context);

void OutputEnable(void);

void OutpuOneOn(OutputTypeDef *h, uint8_t k);
void OutputOneOff(OutputTypeDef *h, uint8_t k);

void OutputOffSeveral(OutputTypeDef *h, uint8_t *several, uint8_t block);
void OutputOnSeveral(OutputTypeDef *h, uint8_t *several, uint8_t block);
void OutputSeveralToogle(OutputTypeDef *h, uint8_t *several, uint8_t block);

uint32_t OutputCounterGet(OutputTypeDef *h, uint8_t relaynumber);
uint32_t OutputCounterSet(OutputTypeDef *h, uint8_t relaynumber, uint32_t value);

void OutputChangedBlocksUpdate(OutputTypeDef *h);


void OutputSpiTxRx(void *transmitt, void *receive, size_t size);
void OutputReset(OutputTypeDef *h);

uint8_t OutputDriverLoopTest(void);


void IoInputLDEnable(void);
void IoInputLDDiasable(void);


void IoTask(OutputTypeDef *context);


#endif /* SRC_RELAYS_H_ */
