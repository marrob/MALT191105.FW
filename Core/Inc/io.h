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


/*pl:
 * IO_SPI_IO_ARRAY_SIZE és IO_OUTPUT_ARRAY_SIZE közötti különbség akkor
 * van, ha SPI láncba több be van füzve az outputok mellé inputok is.
 *
 * IO_SPI_IO_ARRAY_SIZE:
 * MALT40IO: (40out + 40in)8bit=10byte
 * MALT160T: 160out/8bit = 16byte
 * MALT132: 32out/8bit=4byte
 */
#define IO_SPI_IO_ARRAY_SIZE      (DEVICE_OUTPUTS_COUNT + DEVICE_INPUTS_COUNT)/8
#define IO_OUTPUT_ARRAY_SIZE       DEVICE_OUTPUTS_COUNT / 8


/* Exported types ------------------------------------------------------------*/
typedef struct
{
  struct {
  uint32_t Count;
  uint8_t PreState[IO_OUTPUT_ARRAY_SIZE];
  uint8_t PreBlockState[IO_OUTPUT_ARRAY_SIZE];
  uint8_t CurState[IO_OUTPUT_ARRAY_SIZE];
  uint32_t Counters[IO_OUTPUT_ARRAY_SIZE];
  uint8_t ChangedBlocks[DEVICE_BLOCKS];
  uint8_t StatusAutoSendEnable;
  }Output;

  struct{
    uint8_t CurState[IO_SPI_IO_ARRAY_SIZE];
    uint8_t ChangedBlocks[DEVICE_BLOCKS];
    uint8_t StatusAutoSendEnable;
  }Input;

}IoTypeDef;


void IoInit(IoTypeDef *h);

void OutputEnable(void);

void OutpuOneOn(IoTypeDef *h, uint8_t k);
void OutputOneOff(IoTypeDef *h, uint8_t k);

void OutputOffSeveral(IoTypeDef *h, uint8_t *several, uint8_t block);
void OutputOnSeveral(IoTypeDef *h, uint8_t *several, uint8_t block);
void OutputSeveralToogle(IoTypeDef *h, uint8_t *several, uint8_t block);

uint32_t OutputCounterGet(IoTypeDef *h, uint8_t relaynumber);
uint32_t OutputCounterSet(IoTypeDef *h, uint8_t relaynumber, uint32_t value);

void OutputChangedBlocksUpdate(IoTypeDef *h);


void OutputSpiTxRx(void *transmitt, void *receive, size_t size);
void OutputReset(IoTypeDef *h);

uint8_t OutputDriverLoopTest(void);


void IoInputLDEnable(void);
void IoInputLDDiasable(void);


void IoTask(IoTypeDef *h);


#endif /* SRC_RELAYS_H_ */
