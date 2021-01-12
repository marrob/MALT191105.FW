/* USER CODE BEGIN Header */
/*
 * main.c
 *
 *  Created on: 2020. márc. 6.
 *      Author: Margit Robert
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <io.h>
#include "common.h"
#include "LiveLed.h"
#include "memory.h"
#include "StringPlus.h"
#include "diag.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct _CanBusSpeedType
{
  uint32_t Baud;
  uint8_t Pre;
  uint32_t BS1;
  uint32_t BS2;
  uint32_t SJW;
  uint8_t  StatusTxDelayMs;
}CanBusSpeedTypeDef;

typedef struct _AppTypeDef
{
  uint8_t Address;
  int8_t Name[DEVICE_NAME_SIZE];
  uint16_t Version;
  uint8_t CardType;
  uint8_t Options;
  uint32_t SerialNumber;
  MemoryTypeDef Memory;
  IoTypeDef Io;
  CanBusSpeedTypeDef *CanSpeed;

}DeviceTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
DeviceTypeDef Device;
LiveLED_HnadleTypeDef hLiveLed;
LedHandle_Type        hLed;
DiagHandleTypeDef hDiag;


CanBusSpeedTypeDef CanSpeeds[] =
/*  Baud,       Div,  BS1,            BS2,            JSW          StatusTxDelayMs */
{
   {  50000,    60,   CAN_BS1_8TQ,    CAN_BS2_3TQ,    CAN_SJW_4TQ, 38 },
   { 100000,    30,   CAN_BS1_8TQ,    CAN_BS2_3TQ,    CAN_SJW_4TQ, 27 },
   { 125000,    24,   CAN_BS1_8TQ,    CAN_BS2_3TQ,    CAN_SJW_4TQ, 16 },
   { 250000,    12,   CAN_BS1_8TQ,    CAN_BS2_3TQ,    CAN_SJW_4TQ, 5 },
};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void LiveLedOff(void);
void LiveLedOn(void);
void FailLedOn(void);
void FailLedOff(void);

void DebugTask(void);
uint8_t GetAddress(void);
uint8_t GetSpeed(void);

static void CanInit(CanBusSpeedTypeDef *speed);
void RespInfo();
void CanRespRlyCnt(uint8_t relaynumber);
static HAL_StatusTypeDef CanRespSend(uint8_t address, uint8_t *frame, size_t size);
void OutputStatusTask(void);
void UpTimeIncrementTask(void);

void EepromOn(void);
void EepromOff(void);


LedItem_Type LedList[1] = {
  { DEVICE_FAIL_LED,  &FailLedOn,   &FailLedOff, },
};


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Rx Fifo 0 message pending callback
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef   rxHeader;
  uint8_t               frame[8];
  hDiag.Status.CanRx++;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, frame) != HAL_OK)
  {
    hDiag.Status.CanRxErr++;
  }

  #ifdef DEBUG
    DiagRxFrame(&hDiag,frame, rxHeader.DLC);
  #endif

  if (rxHeader.ExtId == HOST_ADDRESS )
  {

    /*** ResetModule ***/
    if(frame[0] == 0xAA)
    {
      if(frame[1] == Device.Address || frame[1] == 0xFF )
      {
        HAL_NVIC_SystemReset();
      }
      else if(frame[1] == Device.Address)
          HAL_NVIC_SystemReset();
    }
    /*** GetInfoByAddress ***/
    else if(frame[0] == 0xAB)
    {
      if(frame[1] == Device.Address || frame[1] == 0xFF)
      {
        RespInfo();

        for(uint8_t i=0; i < DEVICE_OUTPUTS_COUNT; i++)
            Device.Memory.Counters[i] = Device.Io.Output.Counters[i];
        if(hDiag.Status.MemFail == 0){
          hDiag.Status.MemSaved++;
          MemorySave(&Device.Memory);
        }
        else{
          hDiag.Status.MemFail++;
        }
      }
    }
    else
    {
      hDiag.Status.UnknownFrame++;
    }
  }
  else if(rxHeader.ExtId == (CARD_RX_ADDRESS | DEVICE_FAMILY_CODE << 8 | Device.Address))
  {
    /***  ClrOneOutput ***/
    if(frame[0]== DEVICE_FAMILY_CODE && frame[1] == 0x01 && frame[3] == 0)
    {
      OutputOneOff(&Device.Io, frame[2]);
    }
    /***  SetOneOutput ***/
    else if(frame[0]== DEVICE_FAMILY_CODE && frame[1] == 0x01 && frame[3] == 1)
    {
      OutpuOneOn(&Device.Io,frame[2]);
    }
    /*** ResetIo ***/
    else if(frame[0] == DEVICE_FAMILY_CODE && frame [1] == 0x03 && frame[6]== 0x06)
    {
      OutputReset(&Device.Io);
      memset(Device.Io.Output.ChangedBlocks,0x01,DEVICE_BLOCKS);
    }
    /*** ClrOutputst ***/
    else if(frame[0]== DEVICE_FAMILY_CODE && frame[1] == 0x03 && frame[6] == 0x00)
    {
      uint8_t temp []= {frame[2], frame[3], frame[4], frame[5] };
      OutputOffSeveral(&Device.Io,temp, frame[7]);
    }
    /***  SetOutputs ***/
    else if(frame[0]== DEVICE_FAMILY_CODE && frame[1] == 0x03 && frame[6] == 0x01)
    {
      uint8_t temp []= {frame[2], frame[3], frame[4], frame[5] };
      OutputOnSeveral(&Device.Io, temp, frame[7]);
    }
    /*** SetToogleOutputs***/
    else if(frame[0]== DEVICE_FAMILY_CODE && frame[1] == 0x03 && frame[6] == 0x02)
    {
      uint8_t temp[] = {frame[2],frame[3], frame[4], frame[5]};
      OutputSeveralToogle(&Device.Io, temp, frame[7]);
    }
    /*** GetOutputsStatus ***/
    else if(frame[0] == DEVICE_FAMILY_CODE && frame[1] == 0x04)
    {
      Device.Io.Output.StatusAutoSendEnable = frame[2];
      memset(Device.Io.Output.ChangedBlocks, 0x01, DEVICE_BLOCKS);
    }
    /*** GetOneInputStatus ***/
    else if(frame[0] == DEVICE_FAMILY_CODE && frame[1] == 0x06)
    {
      /*Todo:GetOneInputStatus meg kell valósitani
       */
    }
    /*** GetInputsStatus ***/
    else if(frame[0] == DEVICE_FAMILY_CODE && frame[1] == 0x07)
    {
      Device.Io.Input.StatusAutoSendEnable = frame[2];
      memset(Device.Io.Input.ChangedBlocks, 0x01, DEVICE_BLOCKS);
    }
    /*** GetSerialNumber  ***/
    else if(frame[0] == DEVICE_FAMILY_CODE && frame[1] == 0xDE && frame[2] == 0xF5)
    {
      uint8_t data[] = { DEVICE_FAMILY_CODE, 0xDE, 0xF0, 0xC3, 0xD8, 0x12, 0x00 };
      memcpy(data + 3, (uint8_t*)&Device.Memory.SerialNumber, DEVICE_SN_SIZE);
      CanRespSend(Device.Address, data, sizeof(data));
    }
    /*** HostStart ***/
    else if(frame[0] == DEVICE_FAMILY_CODE && frame [1] == 0xEE && frame[2] == 0x11)
    {
      uint8_t data[] = { DEVICE_FAMILY_CODE, 0xEE, 0x12, 0x01 };
      CanRespSend(Device.Address, data, sizeof(data));
    }
    /*** CountersReset ***/
    else if(frame[0] == DEVICE_FAMILY_CODE && frame [1] == 0x0EE && frame[2]== 0x00)
    {
      MemoryResetCounters(&Device.Memory);
      memset(Device.Io.Output.ChangedBlocks,0x01,DEVICE_BLOCKS);
    }
    /*** GetCounter ***/
    else if(frame[0] == DEVICE_FAMILY_CODE && frame[1] == 0xEE && frame[2] == 0x01)
    {
      CanRespRlyCnt(frame[3]);
    }
    /*** SetPortCounter ***/
    else if (frame[0] == DEVICE_FAMILY_CODE && frame[1] == 0xEE && frame[2] == 0x02)
    {
       uint8_t port = frame[3];
       uint8_t value[]={frame[4],frame[5],frame[6], frame[7] };
       OutputCounterSet(&Device.Io, port,*((uint32_t*)value) );
       memset(Device.Io.Output.ChangedBlocks,0x01,DEVICE_BLOCKS);
    }
    /*** SaveCounters ***/
    else if (frame[0] == DEVICE_FAMILY_CODE && frame[1] == 0xEE && frame[2] == 0x03)
    {
      for(uint8_t i=0; i < DEVICE_OUTPUTS_COUNT; i++)
          Device.Memory.Counters[i] = Device.Io.Output.Counters[i];
      if(hDiag.Status.MemFail == 0){
        hDiag.Status.MemSaved++;
        MemorySave(&Device.Memory);
      }
      else{
        hDiag.Status.MemFail++;
      }
      memset(Device.Io.Output.ChangedBlocks,0x01,DEVICE_BLOCKS);
    }

    /*** ReadEEPROM -> RespEEPROM ***/
    else if(frame[0] == DEVICE_FAMILY_CODE && frame[1] == 0xFA && frame[2] == 0xF1)
    {
      uint16_t addr = 0;
      addr |= frame[4] << 8;  /* address high */
      addr |= frame[3];       /* address low  */
      uint8_t data[2];
      memset(data, 0x00, sizeof(data));
      if(MemoryLowRead(addr, data, sizeof(data)) != HAL_OK)
      {
        hDiag.Status.MemFail++;
      }
      else
      {
        uint8_t temp[] = {DEVICE_FAMILY_CODE, 0xFA, 0xF1, frame[3], frame[4], data[0], data[1] };
        CanRespSend(Device.Address,temp, sizeof(temp));
      }
    }
    /*** WriteEEPROM ***/
    else if(frame[0] == DEVICE_FAMILY_CODE && frame[1] == 0xFA && frame[2] == 0xF2)
    {
      uint16_t addr = 0;
      addr |= frame[4] << 8;  /* address high */
      addr |= frame[3];       /* address low  */
      uint8_t data[] = { frame[5], frame[6] };
      if(MemoryLowWrite(addr, data, sizeof(data)) != HAL_OK)
      {
        hDiag.Status.MemFail++;
      }
      else
      {
        memset(Device.Io.Output.ChangedBlocks,0x01,DEVICE_BLOCKS);
      }
    }
    else
    {
      hDiag.Status.UnknownFrame++;
    }
  }
}

void CanRespRlyCnt(uint8_t relaynumber)
{
  uint32_t value = OutputCounterGet(&Device.Io, relaynumber);
  uint8_t data[] = { DEVICE_FAMILY_CODE, 0xEE, 0x01, relaynumber, 0xFF, 0xFF, 0xFF, 0xFF };
  memcpy(data + sizeof(value), &value, sizeof(value));
  CanRespSend(Device.Address, data, sizeof(data));
}

/*** Ask All Info Response ***/
void RespInfo(void)
{
  uint8_t data[] = {0xF0, 0x01, DEVICE_FAMILY_CODE, Device.Address, DEVICE_OPTION_CODE, 0x00, 0x00};
  memcpy(data + sizeof (data) - sizeof(uint16_t), (uint8_t*)&Device.Version, sizeof(uint16_t));
  CanRespSend(Device.Address, data, sizeof(data));
}

 inline static HAL_StatusTypeDef CanRespSend(uint8_t address, uint8_t *frame, size_t size)
{
  HAL_StatusTypeDef status = HAL_OK;
  CAN_TxHeaderTypeDef   txHeader;
  uint32_t              txMailbox;
  txHeader.ExtId = CARD_TX_ADDRESS | DEVICE_FAMILY_CODE << 8 | address;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.IDE = CAN_ID_EXT;
  txHeader.DLC = size;
  uint8_t buffer[8];
  memset(buffer,0xAA,sizeof(buffer));
  memcpy(buffer,frame, size);
  status = HAL_CAN_AddTxMessage(&hcan, &txHeader, buffer, &txMailbox);
#ifdef DEBUG
  DiagTxFrame(&hDiag, frame, size);
#endif
  hDiag.Status.CanTx++;
  if(status != HAL_OK)
    hDiag.Status.CanTxErr++;
  return status;
}

/*
 * Akimenetek statuszat kuldi, ha tobb block van, akkor csak azt a blokkot kuldi
 * amiben a változás történt(1 block max 4bájt)
 * Ha észreveszi hogy nincs szabad MailBox, akkor X ideig vár hogy felszabaduljon.
 * */

//#define DEBUG_STATUS

#ifdef DEBUG_STATUS
 uint8_t LastStatusFrame[8];
#endif
void OutputStatusTask(void)
{
  static uint8_t block = 0;
  static uint32_t lastSentTimestamp = 0;
#ifdef DEBUG_STATUS
  char buffer[80];
#endif

  /**
   * Blokonként nézzük meg, hogy történt-e változás, ha igen akkor azt az egy blokkot küljdük vissza amiben a változás történt.
   */
  if(block < DEVICE_BLOCKS)
  {
    if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 0)
    {
      if(HAL_GetTick() - lastSentTimestamp >= 10)
      {
        if(Device.Io.Output.ChangedBlocks[block])
        {
#ifdef DEBUG_STATUS
          /*** RespOutputsStatus ***/
          uint8_t data[] = { DEVICE_FAMILY_CODE, 0x04, 0x00, 0x00, 0x00, 0x00,  Device.Resp.StatusOutput};
#else
          uint8_t data[] = { DEVICE_FAMILY_CODE, 0x04, 0x00, 0x00, 0x00, 0x00,  block};
#endif
          /* block -> byte index
           * 0 -> 0..3,
           * 1 -> 4..7
           * 2 -> 8..11
           * 3 -> 12..15
           * 4 -> 16..19
           */
          memcpy(data + 2, Device.Io.Output.CurState + block * DEVICE_BLOCK_SIZE, DEVICE_BLOCK_SIZE);

          if(CanRespSend(Device.Address, data, sizeof(data))== HAL_OK)
          {
#ifdef DEBUG_STATUS
            memcpy(LastStatusFrame, data, sizeof(data));
            StringPlusDataToHexaString(data, buffer, sizeof(data));
            DeviceDbgLog(buffer);
#endif
            Device.Io.Output.ChangedBlocks[block] = 0;
          }
        }
        block++;
      }
    }
    else{
      hDiag.Status.CanTxNoMailBox++;
      lastSentTimestamp = HAL_GetTick();
    }
  }
  else{
    block = 0;
    if(Device.Io.Output.StatusAutoSendEnable)
        OutputChangedBlocksUpdate(&Device.Io);
  }
}

void InputStatusTask(){
  static uint8_t block = 0;
  static uint32_t lastSentTimestamp = 0;

  if(block < DEVICE_BLOCKS)
    {
      if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 0)
      {
        if(HAL_GetTick() - lastSentTimestamp >= 10)
        {
          if(Device.Io.Input.ChangedBlocks[block])
          {
  #ifdef DEBUG_STATUS
            uint8_t data[] = { DEVICE_FAMILY_CODE, 0x07, 0x00, 0x00, 0x00, 0x00,  Device.Resp.StatusOutput};
  #else
            /** RespInputsStatus ***/
            uint8_t data[] = { DEVICE_FAMILY_CODE, 0x07, 0x00, 0x00, 0x00, 0x00,  block};
  #endif
            /* block -> byte index
             * 0 -> 0..3,
             * 1 -> 4..7
             * 2 -> 8..11
             * 3 -> 12..15
             * 4 -> 16..19
             */
            memcpy(data + 2, Device.Io.Input.CurState + block * DEVICE_BLOCK_SIZE, DEVICE_BLOCK_SIZE);

            if(CanRespSend(Device.Address, data, sizeof(data))== HAL_OK)
            {
  #ifdef DEBUG_STATUS
              memcpy(LastStatusFrame, data, sizeof(data));
              StringPlusDataToHexaString(data, buffer, sizeof(data));
              DeviceDbgLog(buffer);
  #endif
              Device.Io.Input.ChangedBlocks[block] = 0;
            }
          }
          block++;
        }
      }
      else
      {
        hDiag.Status.CanTxNoMailBox++;
        lastSentTimestamp = HAL_GetTick();
      }
    }
    else
    {
      block = 0;
      if(Device.Io.Input.StatusAutoSendEnable)
        IoChangedBlocksUpdate(&Device.Io);
    }
}


void UpTimeIncrementTask(void)
{
  static int32_t timestamp = 0;

  if((HAL_GetTick() - timestamp) >= 1000)
  {
    timestamp = HAL_GetTick();
    hDiag.Status.UpTimeSec++;
  }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  printf(VT100_CLEARSCREEN);
  printf(VT100_CURSORHOME);
  printf(VT100_ATTR_RESET);

#ifdef DEBUG
  printf(VT100_ATTR_RED);
    DeviceUsrLog("This is a DEBUG version.");
  printf(VT100_ATTR_RESET);
#endif

  DeviceUsrLog("Manufacturer:%s, Name:%s, Version:%04X",DEVICE_MNF, DEVICE_FIRST_NAME, DEVICE_FW);
  /*** Leds ***/
  hLed.pLedTable = LedList;
  hLed.Records = sizeof(LedList)/sizeof(LedItem_Type);
  LedInit(&hLed);
  LedOff(&hLed, DEVICE_FAIL_LED);

  /*** LiveLed ***/
  hLiveLed.LedOffFnPtr = &LiveLedOff;
  hLiveLed.LedOnFnPtr = &LiveLedOn;
  hLiveLed.HalfPeriodTimeMs = 500;
  LiveLedInit(&hLiveLed);

  /*** Memory ***/
  HAL_Delay(100);
  EepromOn();
  uint8_t dummy[] = {0xFF};
  HAL_I2C_Master_Transmit(&hi2c2, 0xFF, dummy, sizeof(dummy), 100  );

  MemoryInit(&Device.Memory);
  if(MemoryTest(&Device.Memory) != MEM_OK)
  {
    hDiag.SelfTest.MemoryState = 1;
    hDiag.Status.MemFail++;
    DeviceErrLog("MemoryTest Failed...");
    LedShowCode(&hLed, DEVICE_FAIL_LED, FAIL_LED_MEM_TEST);
  }

//  MemoryReset(&Device.Memory);

  if(MemoryLoad(&Device.Memory)!= MEM_OK)
  {
    hDiag.SelfTest.MemoryState = 1;
    hDiag.Status.MemFail++;
    LedShowCode(&hLed, DEVICE_FAIL_LED, FAIL_LED_MEM_LOAD);
  }
  else
  {
    /*** EEPROM LOAD ***/
#if DEVICE_UNI_FW
    memcpy(Device.Name, Device.Memory.Name, DEVICE_NAME_SIZE);
    Device.CardType = Device.Memory.CardType;
    Device.Options = Device.Memory.Options;
    Device.Io.Count = Device.Memory.OutputsCount;
    Device.SerialNumber = Device.Memory.SerialNumber;

    for(uint8_t i=0; i < Device.Io.Count; i++)
      Device.Io.Counters[i]=Device.Memory.Counters[i];
#else
    for(uint8_t i=0; i < DEVICE_OUTPUTS_COUNT; i++)
      Device.Io.Output.Counters[i]=Device.Memory.Counters[i];
#endif
  }
  DeviceUsrLog("SerialNumber:%lu, BootUpCounter:%lu", Device.Memory.SerialNumber, Device.Memory.BootUpCounter);

#if defined(CONFIG_MALT40IO)
  /*Push-Pull*/
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin : RLY_G_Pin */
  GPIO_InitStruct.Pin = DO_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_G_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RLY_WR_Pin */
  GPIO_InitStruct.Pin = RLY_WR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RLY_WR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DI_LD_GPIO_Port Default Hight*/
  GPIO_InitStruct.Pin = DI_LD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DI_LD_GPIO_Port, &GPIO_InitStruct);

  IoInputLDEnable();

#elif defined(CONFIG_MALT160T)
  /*Push-Pull*/
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin : RLY_G_Pin */
  GPIO_InitStruct.Pin = DO_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_G_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RLY_WR_Pin */
  GPIO_InitStruct.Pin = RLY_WR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RLY_WR_GPIO_Port, &GPIO_InitStruct);

#elif defined(CONFIG_MALT132)
  /*default - Open Drain*/
#elif defined(CONFIG_MALT23THV)
  /*default - Open Drain*/
#elif defined(CONFIG_MALT16PIN)
  /*Push-Pull*/
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin : RLY_G_Pin */
  GPIO_InitStruct.Pin = DO_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_G_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RLY_WR_Pin */
  GPIO_InitStruct.Pin = RLY_WR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RLY_WR_GPIO_Port, &GPIO_InitStruct);
#endif

  /*** OutputsCount Driver Test ***/
  if(OutputDriverLoopTest()!= IO_OK)
  {
    LedShowCode(&hLed, DEVICE_FAIL_LED, FAIL_LED_RLY_DRV);
    hDiag.SelfTest.DriverLoopState = 1;
    DeviceErrLog("OutputDriverLoopTest: FAIL");
  }

  /*** Defaults ***/
  OutputReset(&Device.Io);
  OutputEnable();
  memset(Device.Io.Output.ChangedBlocks, 0x00, DEVICE_BLOCKS);
  Device.Address = GetAddress();
  Device.Version = DEVICE_FW;
  Device.CanSpeed = &CanSpeeds[GetSpeed()];
  DeviceUsrLog("CAN: Address: %d, Baudrate: %lu Baud",Device.Address, CanSpeeds[GetSpeed()].Baud);

  CanInit(Device.CanSpeed);
  RespInfo();

  Device.Io.Output.StatusAutoSendEnable = 0;
  Device.Io.Input.StatusAutoSendEnable = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t timestamp = HAL_GetTick();
    LiveLedTask(&hLiveLed);
    LedTask(&hLed);

    MemoryTask(&Device.Memory);
    InputStatusTask();
    OutputStatusTask();

    UpTimeIncrementTask();

    hDiag.Status.MainCycleTime = HAL_GetTick() - timestamp;

    static uint32_t timestamp2;
    if((HAL_GetTick() - timestamp2) > 5)
    {
        timestamp2 = HAL_GetTick();
        IoTask(&Device.Io);
    }
#ifdef DEBUG
    DiagTask(&hDiag);
#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 40;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_4TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  HAL_CAN_MspDeInit(&hcan);
  /* FIGYELEM NEM EZT AZ INITET HASZNALOD, a CUBE ezt mindig letrehozza!!! */
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 1000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RLY_CLK_Pin|RLY_MOSI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FAIL_LED_GPIO_Port, FAIL_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LIVE_LED_Pin|RLY_WR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DO_G_Pin|EEP_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DI_CE_GPIO_Port, DI_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RLY_CLK_Pin RLY_MOSI_Pin */
  GPIO_InitStruct.Pin = RLY_CLK_Pin|RLY_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RLY_MISO_Pin */
  GPIO_InitStruct.Pin = RLY_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RLY_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FAIL_LED_Pin */
  GPIO_InitStruct.Pin = FAIL_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FAIL_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIVE_LED_Pin */
  GPIO_InitStruct.Pin = LIVE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIVE_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP1_Pin DIP2_Pin DIP3_Pin DIP4_Pin
                           DIP5_Pin */
  GPIO_InitStruct.Pin = DIP1_Pin|DIP2_Pin|DIP3_Pin|DIP4_Pin
                          |DIP5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP6_Pin DIP7_Pin DIP8_Pin */
  GPIO_InitStruct.Pin = DIP6_Pin|DIP7_Pin|DIP8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DO_G_Pin */
  GPIO_InitStruct.Pin = DO_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_G_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RLY_WR_Pin */
  GPIO_InitStruct.Pin = RLY_WR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RLY_WR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EEP_ON_Pin DI_CE_Pin */
  GPIO_InitStruct.Pin = EEP_ON_Pin|DI_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* CanInit -------------------------------------------------------------------*/

static void CanInit(CanBusSpeedTypeDef *speed)
{
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = speed->Pre;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = speed->SJW;
  hcan.Init.TimeSeg1 = speed->BS1;
  hcan.Init.TimeSeg2 = speed->BS2;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
   Error_Handler();
  }

  /*** Filter Init***/
  CAN_FilterTypeDef  sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
   DeviceErrLog("HAL_CAN_ConfigFilter[%s:%d]",__FILE__,__LINE__);
  }

  /*** Start the CAN peripheral ***/
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
   DeviceErrLog("HAL_CAN_Start[%s:%d]",__FILE__,__LINE__);
  }

  /*** Activate CAN RX notification ***/
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
   DeviceErrLog("HAL_CAN_ActivateNotification[%s:%d]",__FILE__,__LINE__);
  }
}

/* printf -------------------------------------------------------------------*/
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
  return len;
}

void ConsoleWrite(char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
}

/* LEDs ---------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);
}

void FailLedOn(void)
{
  HAL_GPIO_WritePin(FAIL_LED_GPIO_Port, FAIL_LED_Pin, GPIO_PIN_RESET);
}
void FailLedOff(void)
{
  HAL_GPIO_WritePin(FAIL_LED_GPIO_Port, FAIL_LED_Pin, GPIO_PIN_SET);
}

/* Address Switch -----------------------------------------------------------*/
uint8_t GetAddress(void)
{
  uint8_t val = 0;
  (HAL_GPIO_ReadPin(DIP3_GPIO_Port,DIP3_Pin) == GPIO_PIN_SET)? (val&=~0x01): (val|=0x01);
  (HAL_GPIO_ReadPin(DIP4_GPIO_Port,DIP4_Pin) == GPIO_PIN_SET)? (val &=~0x02):(val|=0x02);
  (HAL_GPIO_ReadPin(DIP5_GPIO_Port,DIP5_Pin) == GPIO_PIN_SET)? (val &=~0x04):(val|=0x04);
  (HAL_GPIO_ReadPin(DIP6_GPIO_Port,DIP6_Pin) == GPIO_PIN_SET)? (val &=~0x08):(val|=0x08);
  (HAL_GPIO_ReadPin(DIP7_GPIO_Port,DIP7_Pin) == GPIO_PIN_SET)? (val &=~0x10):(val|=0x10);
  (HAL_GPIO_ReadPin(DIP8_GPIO_Port,DIP8_Pin) == GPIO_PIN_SET)? (val &=~0x20):(val|=0x20);

  return val;
}

uint8_t GetSpeed(void)
{
  uint8_t val = 0;
  (HAL_GPIO_ReadPin(DIP3_GPIO_Port,DIP1_Pin) == GPIO_PIN_SET)? (val&=~0x01): (val|=0x01);
  (HAL_GPIO_ReadPin(DIP4_GPIO_Port,DIP2_Pin) == GPIO_PIN_SET)? (val &=~0x02):(val|=0x02);
  return val;
}

/* EEPROM -----------------------------------------------------------*/
void EepromOn(void)
{
  HAL_GPIO_WritePin(EEP_ON_GPIO_Port, EEP_ON_Pin, GPIO_PIN_RESET);
}
void EepromOff(void)
{
  HAL_GPIO_WritePin(EEP_ON_GPIO_Port, EEP_ON_Pin, GPIO_PIN_SET);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
