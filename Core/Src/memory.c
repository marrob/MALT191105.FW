/* Includes ------------------------------------------------------------------*/
#include <output.h>
#include <stdio.h>
#include <stdlib.h>
#include "memory.h"
#include "string.h"
#include "Led.h"

/* Private define ------------------------------------------------------------*/


inline static HAL_StatusTypeDef Write(uint16_t address, void *pData, size_t size);
inline static HAL_StatusTypeDef Read(uint16_t address, void *pData, size_t size);


/* Private function prototypes -----------------------------------------------*/
uint8_t MemoryInit(MemoryTypeDef *mem)
{

  mem->Address.FirstContent = MEM_START_ADDRESS;
  mem->Address.Name = mem->Address.FirstContent + sizeof(MEM_FIRST_CONTENT);
  mem->Address.CardType = mem->Address.Name + DEVICE_NAME_SIZE;
  mem->Address.Options = mem->Address.CardType + sizeof(mem->CardType);
  mem->Address.OutputsCount = mem->Address.Options + sizeof(mem->Options);
  mem->Address.SerialNumber = mem->Address.OutputsCount + sizeof(mem->OutputsCount);
  mem->Address.BootUpCounter = mem->Address.SerialNumber + sizeof(mem->SerialNumber);

  mem->Address.BootUpCounter += 8; /*Next Page*/
  mem->Address.Test =  mem->Address.BootUpCounter + sizeof(mem->BootUpCounter);

  mem->Address.Counters = MEM_END_ADDRESS - sizeof(mem->Counters);

  mem->CounterSaveRequiedFlag = 0;
  return MEM_OK;
}

uint8_t MemoryLoad(MemoryTypeDef *mem)
{
  uint32_t timestamp = HAL_GetTick();
  HAL_StatusTypeDef status = HAL_OK;

  /*** Read First Start Check ***/
   status = MemoryLowRead(mem->Address.FirstContent, mem->FirstContent, sizeof(MEM_FIRST_CONTENT));
     if(status != HAL_OK)
       return MEM_FAIL;


  memcpy(mem->Name, MEM_DEF_NAME, sizeof(DEVICE_NAME_SIZE));

  if(memcmp(mem->FirstContent, MEM_FIRST_CONTENT, sizeof(MEM_FIRST_CONTENT))!=0 )
  {
     DeviceDbgLog("Device is running in _FIRST START_ sequence...");

     memcpy(mem->FirstContent, MEM_FIRST_CONTENT, sizeof(MEM_FIRST_CONTENT));
     status = MemoryLowWrite(mem->Address.FirstContent, mem->FirstContent, sizeof(MEM_FIRST_CONTENT));
      if(status != HAL_OK)
        return MEM_FAIL;

     memcpy(mem->Name, MEM_DEF_NAME, sizeof(mem->Name));
     status = MemoryLowWrite(mem->Address.Name, mem->Name, sizeof(mem->Name));
     if(status != HAL_OK)
       return MEM_FAIL;

     mem->CardType = MEM_DEF_TYPE;
     status = MemoryLowWrite(mem->Address.CardType, &mem->CardType, sizeof(mem->CardType));
     if(status != HAL_OK)
       return MEM_FAIL;

     mem->Options = MEM_DEF_OPTIONS;
     status = MemoryLowWrite(mem->Address.Options, &mem->Options, sizeof(mem->Options));
     if(status != HAL_OK)
       return MEM_FAIL;

     mem->OutputsCount = MEM_DEF_OUTPUTS;
     status = MemoryLowWrite(mem->Address.OutputsCount, &mem->OutputsCount, sizeof(mem->OutputsCount));
     if(status != HAL_OK)
       return MEM_FAIL;

     mem->SerialNumber = *((uint32_t *)UID_BASE);
     mem->SerialNumber &= 0x00FFFFFF;
     status = MemoryLowWrite(mem->Address.SerialNumber, &mem->SerialNumber, sizeof(mem->SerialNumber));

     mem->BootUpCounter = MEM_DEF_BOOTUP;
     status = MemoryLowWrite(mem->Address.BootUpCounter, &mem->BootUpCounter, sizeof(mem->BootUpCounter));
     if(status != HAL_OK)
       return MEM_FAIL;


     for(int i=0; i < DEVICE_OUTPUT_COUNT; i++)
     {
       mem->Counters[i]=0;
       status = MemoryLowWrite(mem->Address.Counters + i * (sizeof(mem->Counters)/DEVICE_OUTPUT_COUNT),
                             &mem->Counters[i],
                             sizeof(mem->Counters)/DEVICE_OUTPUT_COUNT);
       if (status != HAL_OK)
         break;
     }
  }
  else
  {
    DeviceDbgLog("Device is running in _REGULAR START_ sequence...");

    status = MemoryLowRead(mem->Address.Name, mem->Name, sizeof(mem->Name));
    if(status != HAL_OK)
      return MEM_FAIL;

    status = MemoryLowRead(mem->Address.CardType, &mem->CardType, sizeof(mem->CardType));
    if(status != HAL_OK)
      return MEM_FAIL;

    status = MemoryLowRead(mem->Address.Options, &mem->Options,sizeof(mem->Options));
    if(status != HAL_OK)
      return MEM_FAIL;

    status = MemoryLowRead(mem->Address.OutputsCount, &mem->Options,sizeof(mem->OutputsCount));
    if(status != HAL_OK)
      return MEM_FAIL;

    status = MemoryLowRead(mem->Address.SerialNumber, &mem->SerialNumber,sizeof(mem->SerialNumber));
    if(status != HAL_OK)
      return MEM_FAIL;

    status = MemoryLowRead(mem->Address.BootUpCounter, &mem->BootUpCounter, sizeof(mem->BootUpCounter));
    mem->BootUpCounter ++;
    if(status != HAL_OK)
      return MEM_FAIL;

    status = MemoryLowWrite(mem->Address.BootUpCounter, &mem->BootUpCounter, sizeof(mem->BootUpCounter));
    if(status != HAL_OK)
      return MEM_FAIL;

    status = MemoryLowRead(mem->Address.SerialNumber, &mem->SerialNumber,sizeof(mem->SerialNumber));
    mem->SerialNumber &= 0x00FFFFFF;

    for(uint8_t i = 0; i < DEVICE_OUTPUT_COUNT; i++)
    {
        status = MemoryLowRead(mem->Address.Counters + i * (sizeof(mem->Counters)/DEVICE_OUTPUT_COUNT),
                              &mem->Counters[i],
                              sizeof(mem->Counters)/DEVICE_OUTPUT_COUNT);
        if(status != HAL_OK)
          return MEM_FAIL;
    }
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
  status = MemoryLowRead(mem->Address.SerialNumber, &serialnumber, sizeof(mem->SerialNumber));
  mem->SerialNumber = serialnumber;
  if(status != HAL_OK)
    return MEM_FAIL;
  else
    return MEM_OK;
}

void MemoryResetCounters(MemoryTypeDef *mem)
{
  for(uint8_t i=0; i < DEVICE_OUTPUT_COUNT; i++)
    mem->Counters[i]=0;

  mem->CounterSaveRequiedFlag = 1;
}

void MemoryTask(MemoryTypeDef *mem)
{
  HAL_StatusTypeDef status = HAL_OK;
  static uint32_t timestamp = 0;

  if((HAL_GetTick() - timestamp) > 1000  && mem->CounterSaveRequiedFlag)
  {
    timestamp = HAL_GetTick();

    for(uint8_t i = 0; i < DEVICE_OUTPUT_COUNT; i++)
    {
      status = MemoryLowWrite(mem->Address.Counters + i * (sizeof(mem->Counters)/DEVICE_OUTPUT_COUNT),
                              &mem->Counters[i],
                              (sizeof(mem->Counters)/DEVICE_OUTPUT_COUNT));
      if (status != HAL_OK)
        break;
    }

    if(status != HAL_OK)
    {
      DeviceErrLog("MemoryTask.CounterWrite");
      LedShowCode(&hLed, DEVICE_FAIL_LED, FAIL_LED_MEM_WRITE);
    }
    else
    {
      mem->SaveingTimeMs = HAL_GetTick() - timestamp;
    }
    mem->CounterSaveRequiedFlag = 0;
  }
}

void MemorySave(MemoryTypeDef *mem)
{
  mem->CounterSaveRequiedFlag = 1;
}


uint8_t MemoryTest(MemoryTypeDef *mem)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = MemoryLowWrite(mem->Address.Test, MEM_TEST_CONTENT, sizeof(mem->Test));
  if(status != HAL_OK)
    return status;

  status = MemoryLowRead(mem->Address.Test, mem->Test, sizeof(mem->Test));
  if(status != HAL_OK)
    return status;

  if(memcmp(MEM_TEST_CONTENT, mem->Test, sizeof(mem->Test)) == 0)
  {
    memset(mem->Test, 0x00, sizeof(mem->Test));
    status = MemoryLowWrite(mem->Address.Test, mem->Test, sizeof(mem->Test));
    if(status != HAL_OK)
      return status;
    else
      return MEM_OK;
  }
  else
    return MEM_FAIL;
}

HAL_StatusTypeDef MemoryReset(MemoryTypeDef *mem)
{
  HAL_StatusTypeDef status;
  uint8_t data[sizeof(MEM_FIRST_CONTENT)];
  memset(data, 0x00, sizeof(MEM_FIRST_CONTENT));
  status = MemoryLowWrite(mem->Address.FirstContent, data, sizeof(MEM_FIRST_CONTENT));
  return status;
}

HAL_StatusTypeDef MemoryLowRead(uint16_t address, void *pData, size_t size)
{
  uint32_t timestamp = HAL_GetTick();
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t count = 0, toWrite = 0, numOfPage = 0, numOfSingle = 0;

  uint8_t *ptr = (uint8_t*)pData;
  /*Adott page-en ennyi bájtot írhat még (hogy ne lépjen át a következöre)*/
  count = MEM_DEV_PAGE_SIZE - address;     /*35 = 64 - 10*/
  /*Ennyi _egész_ page-et kell íri*/
  numOfPage = size / MEM_DEV_PAGE_SIZE;    /*2 = 129 / 64 */
  /*Ennyi bájtot kell írni egy új nem egész page-re*/
  numOfSingle = size % MEM_DEV_PAGE_SIZE; /*1 =  129 % 64 */


  while (HAL_I2C_IsDeviceReady(&hi2c2, MEM_DEV_ADDRESS << 1, 3, MEM_DEV_TIMEOUT_MS) != HAL_OK)
  {
    if(HAL_GetTick() - timestamp > MEM_DEV_TIMEOUT_MS)
      return HAL_TIMEOUT;
  }

  if((address % MEM_DEV_PAGE_SIZE) == 0)
  {/*A cim új page elején kezdődik*/

    if(numOfPage == 0)
    {
      /*Csak egy Page-et kell írni*/
      status = Read(address, ptr, size);
    }
    else
    {
      while(numOfPage--) //size > AT45DBX_PAGE_SIZE
      {
        status = Read(address, ptr, MEM_DEV_PAGE_SIZE);
        address +=  MEM_DEV_PAGE_SIZE;
        ptr += MEM_DEV_PAGE_SIZE;
      }
      if(numOfSingle!= 0)
        status = Read(address, ptr, numOfSingle);
    }
  }
  else
  {
    if(numOfPage == 0)
    {/*Csak egy Page-et kell írni*/

       /*Megnézzük, hogy az adott pagen mennyi bájtot írhatunk*/
       /*ha több numOfSingle bájt van mint amennyit az akualis page elvisel(count), akkor két lépésben írjuk*/
       if(numOfSingle > count)
       {
          /*az elsö page-re ennyit irhatunk*/
          toWrite = numOfSingle - count;

          status = Read(address, ptr, count);
          address += count;
          ptr += count;
          status = Read(address, ptr, toWrite);
       }
       else
       {
         /*ra fer az adott page-re*/
         status = Read(address, ptr, size);
       }
    }
    else
    { /*több page-en kell dolgozni*/

      size -= count;
      numOfPage =  size / MEM_DEV_PAGE_SIZE;
      numOfSingle = size % MEM_DEV_PAGE_SIZE;
      /*
       *ezután csak egész pagek-en kell dolgozni
       */
      status = Read(address, ptr, count);
      address += count;
      ptr += count;

      while(numOfPage--)
      {
          Read(address, ptr, MEM_DEV_PAGE_SIZE);
          address +=  MEM_DEV_PAGE_SIZE;
          ptr += MEM_DEV_PAGE_SIZE;
      }
      if(numOfSingle != 0)
      {
        status = Read(address, ptr, numOfSingle);
      }
    }
  }
  return status;
}

inline static HAL_StatusTypeDef Read(uint16_t address, void *pData, size_t size)
{
  return HAL_I2C_Mem_Read( &hi2c2,
                            MEM_DEV_ADDRESS << 1,
                            address,
                            I2C_MEMADD_SIZE_16BIT,
                            pData,
                            size,
                            MEM_DEV_TIMEOUT_MS);
}




HAL_StatusTypeDef MemoryLowWrite(volatile uint16_t address, void *pData, size_t size)
{
  uint32_t timestamp = HAL_GetTick();
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t count = 0, toWrite = 0, numOfPage = 0, numOfSingle = 0;

  uint8_t *ptr = (uint8_t*)pData;
  /*Adott page-en ennyi bájtot írhat még (hogy ne lépjen át a következöre)*/
  count = MEM_DEV_PAGE_SIZE - address;     /*35 = 64 - 10*/
  /*Ennyi _egész_ page-et kell íri*/
  numOfPage = size / MEM_DEV_PAGE_SIZE;    /*2 = 129 / 64 */
  /*Ennyi bájtot kell írni egy új nem egész page-re*/
  numOfSingle = size % MEM_DEV_PAGE_SIZE; /*1 =  129 % 64 */


  while (HAL_I2C_IsDeviceReady(&hi2c2, MEM_DEV_ADDRESS << 1, 3, MEM_DEV_TIMEOUT_MS) != HAL_OK)
  {
    if(HAL_GetTick() - timestamp > MEM_DEV_TIMEOUT_MS)
      return HAL_TIMEOUT;
  }

  if((address % MEM_DEV_PAGE_SIZE) == 0)
  {/*A cim új page elején kezdődik*/

    if(numOfPage == 0)
    {
      /*Csak egy Page-et kell írni*/
      status = Write(address, ptr, size);
    }
    else
    {
      while(numOfPage--) //size > AT45DBX_PAGE_SIZE
      {
        status = Write(address, ptr, MEM_DEV_PAGE_SIZE);
        address +=  MEM_DEV_PAGE_SIZE;
        ptr += MEM_DEV_PAGE_SIZE;
      }
      if(numOfSingle!= 0)
        status = Write(address, ptr, numOfSingle);
    }
  }
  else
  {
    if(numOfPage == 0)
    {/*Csak egy Page-et kell írni*/

       /*Megnézzük, hogy az adott pagen mennyi bájtot írhatunk*/
       /*ha több numOfSingle bájt van mint amennyit az akualis page elvisel(count), akkor két lépésben írjuk*/
       if(numOfSingle > count)
       {
          /*az elsö page-re ennyit irhatunk*/
          toWrite = numOfSingle - count;

          status = Write(address, ptr, count);
          address += count;
          ptr += count;
          status = Write(address, ptr, toWrite);
       }
       else
       { /*ra fer az adott page-re*/
         status = Write(address, ptr, size);
       }
    }
    else
    { /*több page-en kell dolgozni*/

      size -= count;
      numOfPage =  size / MEM_DEV_PAGE_SIZE;
      numOfSingle = size % MEM_DEV_PAGE_SIZE;
      /*
       * ezután csak egész pagek-en kell dolgozni
       */
      status = Write(address, ptr, count);
      address += count;
      ptr += count;

      while(numOfPage--)
      {
          Write(address, ptr, MEM_DEV_PAGE_SIZE);
          address +=  MEM_DEV_PAGE_SIZE;
          ptr += MEM_DEV_PAGE_SIZE;
      }
      if(numOfSingle != 0)
      {
        status = Write(address, ptr, numOfSingle);
      }
    }
  }
  return status;
}

inline static HAL_StatusTypeDef Write(uint16_t address, void *pData, size_t size)
{
  return HAL_I2C_Mem_Write( &hi2c2,
                            MEM_DEV_ADDRESS << 1,
                            address,
                            I2C_MEMADD_SIZE_16BIT,
                            pData,
                            size,
                            MEM_DEV_TIMEOUT_MS);
}
