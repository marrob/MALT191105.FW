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

#define MEM_FIRST_CONTENT         "I'M ALIVE"
#define MEM_TEST_CONTENT          "This is a Test"
#define MEM_FLASH_PAGE_ADDR       0x0807F800
#define MEM_START_ADDRESS         0
#define MEM_END_ADDRESS           0x3E80 //16KiloByte

#define MEM_DEV_ADDRESS           0x50
#define MEM_DEV_PAGE_SIZE         64
#define MEM_DEV_TIMEOUT_MS        100

#define MEM_DEF_NAME              "I don't know who I am..."
#define MEM_DEF_TYPE              0
#define MEM_DEF_OPTIONS           0
#define MEM_DEF_OUTPUTS           0
#define MEM_DEF_BOOTUP            0


/* Exported macro ------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/

typedef struct _MemoryTypeDef
{

  uint8_t  FirstContent[sizeof(MEM_FIRST_CONTENT)];
  int8_t   Name[DEVICE_NAME_SIZE];
  uint8_t  CardType;
  uint8_t  Options;
  uint32_t OutputsCount;
  uint32_t SerialNumber;
  uint32_t BootUpCounter;
  int8_t   Test[sizeof(MEM_TEST_CONTENT)];

  uint32_t Counters[DEVICE_OUTPUTS_COUNT];

  uint8_t  CounterSaveRequiedFlag;
  uint32_t SaveingTimeMs;
  uint32_t LoadTimeMs;
  struct _adresses
  {
    uint32_t FirstContent;
    uint32_t Name;
    uint32_t CardType;
    uint32_t Options;
    uint32_t OutputsCount;
    uint32_t SerialNumber;
    uint32_t BootUpCounter;
    uint32_t Test;

    uint32_t Counters;
  }Address;

}MemoryTypeDef;




/* Exported functions ------------------------------------------------------- */
uint8_t MemoryInit(MemoryTypeDef *mem);
uint8_t MemoryTest(MemoryTypeDef *mem);
uint8_t MemoryLoad(MemoryTypeDef *mem);
HAL_StatusTypeDef MemoryReset(MemoryTypeDef *mem);


void MemoryResetCounters(MemoryTypeDef *mem);
void MemorySave(MemoryTypeDef *mem);
void MemoryTask(MemoryTypeDef *mem);
HAL_StatusTypeDef MemoryLowWrite(uint16_t address, void *data, size_t size);
HAL_StatusTypeDef MemoryLowRead(uint16_t address, void *data, size_t size);
uint32_t MemoryChangeSerailNumber(MemoryTypeDef *mem, uint32_t serialnumber);




#endif /* INC_MEMORY_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
