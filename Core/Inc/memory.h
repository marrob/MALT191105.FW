/*
 * memory.h
 *
 *  Created on: 2020. márc. 6.
 *      Author: Margit Robert
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_MEMORY_H_
#define INC_MEMORY_H_
/* Includes ------------------------------------------------------------------*/
#include <main.h>
/* Exported constants --------------------------------------------------------*/
#define MEM_OK    0
#define MEM_FAIL  1

#define MEM_FIRST_CHECK_CONTENT   "LIFE"
#define MEM_FLASH_PAGE_ADDR        0x0807F800
/* Exported macro ------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
typedef struct _MemoryTypeDef
{
  uint32_t RealyCounters[DEVICE_OUTPUT_COUNT];
  uint32_t RealyCounterAddr;
  uint32_t BootUpCounter;
  uint32_t BootUpCounterAddr;
  uint32_t SerialNumber;
  uint32_t SerialNumberAddr;
  uint8_t  FirstStartCheck[sizeof(MEM_FIRST_CHECK_CONTENT)];
  uint32_t FirstStartCheckNumberAddr;
  uint8_t  RealyCounterSaveRequiedFlag;

  uint32_t SaveingTimeMs;
  uint32_t LoadTimeMs;

}MemoryTypeDef;

/* Exported functions ------------------------------------------------------- */
uint8_t MemoryInit(MemoryTypeDef *mem);
uint8_t MemoryTest(void);
uint8_t MemoryLoad(MemoryTypeDef *mem);
HAL_StatusTypeDef MemoryReset(MemoryTypeDef *mem);


void MemoryResetRealyCnt(MemoryTypeDef *mem);
void MemorySave(MemoryTypeDef *mem);
void MemoryTask(MemoryTypeDef *mem);
HAL_StatusTypeDef MemoryLowWrite(uint16_t address, void *data, size_t size);
HAL_StatusTypeDef MemoryLowRead(uint16_t address, void *data, size_t size);
uint32_t MemoryChangeSerailNumber(MemoryTypeDef *mem, uint32_t serialnumber);




#endif /* INC_MEMORY_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
