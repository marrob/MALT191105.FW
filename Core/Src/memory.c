/* Includes ------------------------------------------------------------------*/
#include <output.h>
#include <stdio.h>
#include <stdlib.h>
#include "memory.h"
#include "string.h"
#include "Led.h"

/* Private define ------------------------------------------------------------*/
#define DEVICE_ADDRESS      0x50
#define TIMEOUT_MS          100
#define START_ADDRESS       0
#define END_ADDRESS         0x3E80 //16KiloByte
#define TEST_CONTENT        ("This is a Test2")
#define TEST_ADDRESS        END_ADDRESS - sizeof(TEST_CONTENT)

/* Private function prototypes -----------------------------------------------*/


uint8_t MemoryInit(MemoryTypeDef *mem)
{
  mem->RealyCounterAddr = 0;
  mem->BootUpCounterAddr = sizeof(mem->RealyCounters[0]) * DEVICE_OUTPUT_COUNT;
  mem->SerialNumberAddr = mem->BootUpCounterAddr + sizeof(mem->BootUpCounter);
  mem->FirstStartCheckNumberAddr = mem->SerialNumberAddr + sizeof(mem->SerialNumber);
  mem->RealyCounterSaveRequiedFlag = 0;
  return MEM_OK;
}

uint8_t MemoryLoad(MemoryTypeDef *mem)
{
  uint32_t timestamp = HAL_GetTick();
  HAL_StatusTypeDef status = HAL_OK;

  /*** Read First Start Check ***/
   status = MemoryLowRead(mem->FirstStartCheckNumberAddr, mem->FirstStartCheck,sizeof(MEM_FIRST_CHECK_CONTENT));
     if(status != HAL_OK)
       return MEM_FAIL;

  /*** If Frist Start => Reset ***/
  if(memcmp(mem->FirstStartCheck, MEM_FIRST_CHECK_CONTENT, sizeof(MEM_FIRST_CHECK_CONTENT))!=0 )
  {
     DeviceDbgLog("Device is running in _FIRST START_ sequence...");
     for(int i=0; i < DEVICE_OUTPUT_COUNT; i++)
        mem->RealyCounters[i]=0;
     mem->BootUpCounter = 0;

     /*** First start content write ***/
     status = MemoryLowWrite(mem->FirstStartCheckNumberAddr, MEM_FIRST_CHECK_CONTENT,sizeof(MEM_FIRST_CHECK_CONTENT));

     /*** Write a new serial number ***/
     srand(HAL_GetTick());
     mem->SerialNumber = rand();
     mem->SerialNumber &= 0x00FFFFFF;
     status = MemoryLowWrite(mem->SerialNumberAddr, &mem->SerialNumber, sizeof(mem->SerialNumber));
  }
  else
  {
    DeviceDbgLog("Device is running in _REGULAR START_ sequence...");
    /*** Read Counters ***/
    uint16_t address = 0;
    for(uint8_t rly=0; rly < DEVICE_OUTPUT_COUNT; rly++)
    {
      address = sizeof(mem->RealyCounters[0]) * rly; //K1:0 = 0x00000000
      if((status = MemoryLowRead(address, &mem->RealyCounters[rly],sizeof(mem->RealyCounters[0])))!= HAL_OK)
          break;
    }
    /*** Read Boot Up Counter ***/
    status = MemoryLowRead(mem->BootUpCounterAddr, &mem->BootUpCounter,sizeof(mem->BootUpCounter));
    mem->BootUpCounter ++;


    /*** BootUpCounterIncrase ***/
    status = MemoryLowWrite(mem->BootUpCounterAddr, &mem->BootUpCounter,sizeof(mem->BootUpCounter));

    if(status != HAL_OK)
      return MEM_FAIL;

    /*** Read Serial Number ***/
    status = MemoryLowRead(mem->SerialNumberAddr, &mem->SerialNumber,sizeof(mem->SerialNumber));
    mem->SerialNumber &= 0x00FFFFFF;
  }

  mem->LoadTimeMs = HAL_GetTick() - timestamp;

  if(status != HAL_OK)
    return MEM_FAIL;
  else
    return MEM_OK;
}

uint32_t MemoryChangeSerailNumber(MemoryTypeDef *mem, uint32_t serialnumber)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = MemoryLowRead(mem->SerialNumberAddr, &serialnumber, sizeof(mem->SerialNumber));
  mem->SerialNumber = serialnumber;
  if(status != HAL_OK)
    return MEM_FAIL;
  else
    return MEM_OK;
}

void MemoryResetRealyCnt(MemoryTypeDef *mem)
{
  for(uint8_t i=0; i<OUTPUT_ARRAY; i++)
    mem->RealyCounters[i]=0;
}

void MemoryTask(MemoryTypeDef *mem)
{
  HAL_StatusTypeDef status = HAL_OK;
  static uint32_t timestamp = 0;

  if((HAL_GetTick() - timestamp) > 1000  && mem->RealyCounterSaveRequiedFlag)
  {
    timestamp = HAL_GetTick();

    for(uint8_t rly = 0; rly < DEVICE_OUTPUT_COUNT; rly++)
    {
      uint16_t address = sizeof(mem->RealyCounters[0]) * rly;
      status = MemoryLowWrite(address, &mem->RealyCounters[rly], sizeof(mem->RealyCounters[0]));
      if (status != HAL_OK)
        break;
    }

    if(status != HAL_OK)
    {
      DeviceErrLog("MemoryTask.RealyCounterWrite");
      LedShowCode(&hLed, DEVICE_FAIL_LED, FAIL_LED_MEM_WRITE);
    }
    else
    {
      mem->SaveingTimeMs = HAL_GetTick() - timestamp;
    }
    mem->RealyCounterSaveRequiedFlag = 0;
  }
}

void MemorySave(MemoryTypeDef *mem)
{
  mem->RealyCounterSaveRequiedFlag = 1;
}


uint8_t MemoryTest(void)
{
  uint8_t writeBuf[]= {TEST_CONTENT};
  uint8_t readBuf[sizeof(TEST_CONTENT)];

  MemoryLowWrite(TEST_ADDRESS, writeBuf, sizeof(writeBuf));
  memset(readBuf, '0', sizeof(readBuf));
  MemoryLowRead(TEST_ADDRESS,readBuf, sizeof(readBuf));

   if(memcmp(writeBuf,readBuf, sizeof(writeBuf))==0)
    return MEM_OK;
  else
    return MEM_FAIL;
}

HAL_StatusTypeDef MemoryReset(MemoryTypeDef *mem)
{
  HAL_StatusTypeDef status;
  uint8_t content[sizeof(MEM_FIRST_CHECK_CONTENT)];
  memset(content,'0',sizeof(MEM_FIRST_CHECK_CONTENT));
  /*** First start content write ***/
  status = MemoryLowWrite(mem->FirstStartCheckNumberAddr, content,sizeof(MEM_FIRST_CHECK_CONTENT));
  return status;
}

HAL_StatusTypeDef MemoryLowRead(uint16_t address, void *data, size_t size)
{
  HAL_StatusTypeDef status;
  uint32_t timestamp = HAL_GetTick();

  if(size > 64)
    return HAL_ERROR;

  while (HAL_I2C_IsDeviceReady(&hi2c2, DEVICE_ADDRESS << 1, 3, TIMEOUT_MS) != HAL_OK)
  {
    if(HAL_GetTick() - timestamp > TIMEOUT_MS)
      return HAL_TIMEOUT;
  }

  status = HAL_I2C_Mem_Read(&hi2c2,
                DEVICE_ADDRESS << 1,
                address,
                I2C_MEMADD_SIZE_16BIT,
                data,
                size,
                TIMEOUT_MS);

  return status;
}

HAL_StatusTypeDef MemoryLowWrite(uint16_t address, void *data, size_t size)
{
  uint32_t timestamp = HAL_GetTick();
  HAL_StatusTypeDef status;

  if(size > 64)
    return HAL_ERROR;

  while (HAL_I2C_IsDeviceReady(&hi2c2, DEVICE_ADDRESS << 1, 3, TIMEOUT_MS) != HAL_OK)
  {
    if(HAL_GetTick() - timestamp > TIMEOUT_MS)
      return HAL_TIMEOUT;
  }

  status = HAL_I2C_Mem_Write(&hi2c2,
                DEVICE_ADDRESS << 1,
                address,
                I2C_MEMADD_SIZE_16BIT,
                data,
                size,
                TIMEOUT_MS);

  return status;
}
