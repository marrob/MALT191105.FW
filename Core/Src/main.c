/* USER CODE BEGIN Header */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "LiveLed.h"
#include "relays.h"
#include "memory.h"
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
}CanBusSpeedTypeDef;

typedef struct _AppTypeDef
{
  uint8_t Address;
  uint16_t Version;
  MemoryTypeDef Memory;
  RelayTypeDef Relay;
  CanBusSpeedTypeDef *CanSpeed;
  uint8_t StatusAutoSendEnable;
  struct _statusCounters
  {
    uint32_t CanRx;
    uint32_t CanRxErr;
    uint32_t UnknownFrame;
    uint32_t CanTx;
    uint32_t CanTxErr;
    uint32_t MemFail;
    uint32_t MemSaved;
    uint32_t MainCycleTime;
  }Status;

  struct _requestCounters
  {
    uint32_t AskAllInfo;
    uint32_t GlobalReset;
    uint32_t Reset;
    uint32_t HostStart;
    uint32_t Status;
    uint32_t SetOneRealy;
    uint32_t OffOneRelay;
    uint32_t SeveralOn;
    uint32_t SeveralOff;
    uint32_t SeveralToogle;
    uint32_t RelayCounter;
    uint32_t SerialNumber;
    uint32_t ResetRlyCnt;
  }Req;

  struct _responseCounters
  {
    uint32_t AskAllInfo;
    uint32_t Status;
    uint32_t RelayCounter;

  }Resp;

  struct _selfTest
  {
     uint8_t MemoryState;
     uint8_t DriverLoopState;
  }SelfTest;

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
uint8_t SerialSendEnable = 0;

CanBusSpeedTypeDef CanSpeeds[] =
/*  Baud,       Div,  BS1,            BS2,            JSW         */
{
   {  50000,    60,   CAN_BS1_8TQ,    CAN_BS2_3TQ,    CAN_SJW_4TQ },
   { 100000,    30,   CAN_BS1_8TQ,    CAN_BS2_3TQ,    CAN_SJW_4TQ },
   { 125000,    24,   CAN_BS1_8TQ,    CAN_BS2_3TQ,    CAN_SJW_4TQ },
   { 250000,    12,   CAN_BS1_8TQ,    CAN_BS2_3TQ,    CAN_SJW_4TQ },
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void MX_CAN_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void LiveLedOff(void);
void LiveLedOn(void);
void FeilLedOn(void);
void FailLedOff(void);

void DebugTask(void);
uint8_t GetAddress(void);
uint8_t GetSpeed(void);

static void CanInit(CanBusSpeedTypeDef *speed);
void CanAskAllInfoResponse();
void CanRespRlyCnt(uint8_t relaynumber);
HAL_StatusTypeDef CanRespSend(uint8_t address, uint8_t *frame, size_t size);
void StatusTask(void);
void TestMsgSenderTask(void);


LedItem_Type LedList[1] = {
  { DEVICE_FAIL_LED,  &FeilLedOn,   &FailLedOff, },
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
  Device.Status.CanRx++;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, frame) != HAL_OK)
  {
    Device.Status.CanRxErr++;
  }
  else if (rxHeader.ExtId == HOST_ADDRESS )
  {
    /*** Ask All Info Request ***/
    if(memcmp(frame, (uint8_t[]){0xAB, 0xFF}, 2)==0)
    {
      Device.Req.AskAllInfo ++;
      CanAskAllInfoResponse();
      for(uint8_t i=0; i < DEVICE_RELAY_COUNT; i++)
      {
        Device.Status.MemSaved++;
        Device.Memory.RealyCounters[i] = Device.Relay.Counters[i];
      }
      if(Device.Status.MemFail == 0)
      {
        MemorySave(&Device.Memory);
      }
      else
      {
        Device.Status.MemFail++;
      }
    }

    /*** Global Reset Request ***/
    else if(memcmp(frame, (uint8_t[]){0xAA, 0xFF}, 2)==0)
    {
      Device.Req.GlobalReset++;
      RelayReset(&Device.Relay);
      memset(Device.Relay.ChangedBlocks,0x01,RELAY_MAX_BLOCK);
      CanAskAllInfoResponse();
    }
    else
    {
      Device.Status.UnknownFrame++;
    }
  }
  if(rxHeader.ExtId == (CARD_RX_ADDRESS | CARD_TYPE << 8 | Device.Address))
  {
    /*** Set Off One Relay Request ***/
    if(frame[0]== CARD_TYPE && frame[1] == 0x01 && frame[3] == 0)
    {
      Device.Req.OffOneRelay++;
      RelayOffOne(&Device.Relay, frame[2]);
    }
    /*** Set On One Relay Request ***/
    else if(frame[0]== CARD_TYPE && frame[1] == 0x01 && frame[3] == 1)
    {
      Device.Req.SetOneRealy++;
      RelayOnOne(&Device.Relay,frame[2]);
    }
    /*** Set OFF Several Relay Request ***/
    else if(frame[0]== CARD_TYPE && frame[1] == 0x03 && frame[6] == 0x00)
    {
      Device.Req.SeveralOff++;
      uint8_t temp []= {frame[2], frame[3], frame[4], frame[5] };
      RelayOffSeveral(&Device.Relay,temp, frame[7]);
    }
    /*** Set ON Several Relay Request ***/
    else if(frame[0]== CARD_TYPE && frame[1] == 0x03 && frame[6] == 0x01)
    {
      Device.Req.SeveralOn++;
      uint8_t temp []= {frame[2], frame[3], frame[4], frame[5] };
      RelayOnSeveral(&Device.Relay, temp, frame[7]);
    }
    /*** Toogle Several Relays Request ***/
    else if(frame[0]== CARD_TYPE && frame[1] == 0x03 && frame[6] == 0x02)
    {
      Device.Req.SeveralToogle++;
      uint8_t temp[] = {frame[2],frame[3], frame[4], frame[5]};
      RelayToogleSeveral(&Device.Relay, temp, frame[7]);
    }
    /*** Status Request  ***/
    else if(frame[0] == CARD_TYPE && frame[1] == 0x04 && frame[2] == 0x01)
    {
      Device.Req.Status++;
      Device.StatusAutoSendEnable = frame[2];
      memset(Device.Relay.ChangedBlocks,0x01,RELAY_MAX_BLOCK);
    }
    /*** Host Start Request ***/
    else if(frame[0] == CARD_TYPE && frame [1] == 0xEE && frame[2] == 0x11)
    {
      Device.Req.HostStart++;
      uint8_t data[] = { CARD_TYPE, 0xEE, 0x12, 0x01 };
      CanRespSend(Device.Address, data, sizeof(data));
    }
    /*** Reset Request ***/
    else if(frame[0] == CARD_TYPE && frame [1] == 0x03 && frame[6]== 0x06)
    {
      Device.Req.Reset++;
      RelayReset(&Device.Relay);
      memset(Device.Relay.ChangedBlocks,0x01,RELAY_MAX_BLOCK);
    }
    /*Relay Counter Reset Request*/
    else if(frame[0] == CARD_TYPE && frame [1] == 0x03 && frame[6]== 0x07)
    {
      Device.Req.ResetRlyCnt++;
      MemoryResetRealyCnt(&Device.Memory);
      memset(Device.Relay.ChangedBlocks,0x01,RELAY_MAX_BLOCK);
    }
    /*** Get Realy Counter ***/
    else if(frame[0] == CARD_TYPE && frame[1] == 0xEE && frame[2] == 0x01)
    {
      Device.Req.RelayCounter++;
      CanRespRlyCnt(frame[3]);
    }
    /*** Serial Number Request ***/
    else if(frame[0] == CARD_TYPE && frame[1] == 0xDE && frame[2] == 0xF5)
    {
      Device.Req.SerialNumber++;
      uint8_t data[] = { CARD_TYPE, 0xDE, 0xF0, 0xC3, 0xD8, 0x12, 0x00 };
      memcpy(data + 3, (uint8_t*)&Device.Memory.SerialNumber, DEVICE_SN_SIZE);
      CanRespSend(Device.Address, data, sizeof(data));
    }
    else
    {
      Device.Status.UnknownFrame++;
    }
  }
}

void CanRespRlyCnt(uint8_t relaynumber)
{
  Device.Resp.RelayCounter++;
  uint32_t value = RelayCounterGet(&Device.Relay, relaynumber);
  uint8_t data[] = { CARD_TYPE, 0xEE, 0x01, relaynumber, 0xFF, 0xFF, 0xFF, 0xFF };
  memcpy(data + sizeof(value), &value, sizeof(value));
  CanRespSend(Device.Address, data, sizeof(data));
}

/*** Ask All Info Response ***/
void CanAskAllInfoResponse(void)
{
  Device.Resp.AskAllInfo ++;
  uint8_t data[] = {0xF0, 0x01, CARD_TYPE, Device.Address, CARD_OPTIONS, 0x00, 0x00};
  memcpy(data + sizeof (data) - sizeof(uint16_t), (uint8_t*)&Device.Version, sizeof(uint16_t));
  CanRespSend(Device.Address, data, sizeof(data));
}

HAL_StatusTypeDef CanRespSend(uint8_t address, uint8_t *frame, size_t size)
{
  HAL_StatusTypeDef status = HAL_OK;
  CAN_TxHeaderTypeDef   txHeader;
  uint32_t              txMailbox;
  txHeader.ExtId = CARD_TX_ADDRESS | CARD_TYPE << 8 | address;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.IDE = CAN_ID_EXT;
  txHeader.DLC = size;
  status = HAL_CAN_AddTxMessage(&hcan, &txHeader, frame, &txMailbox);
  Device.Status.CanTx++;
  if(status != HAL_OK)
    Device.Status.CanTxErr++;
  return status;
}

void StatusTask(void)
{
  static uint8_t block = 0;

  if(block < RELAY_MAX_BLOCK)
  {
    if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 0)
    {
      if(Device.Relay.ChangedBlocks[block])
      {
        uint8_t data[] = { CARD_TYPE, 0x04, 0x00, 0x00, 0x00, 0x00, block};
        /* block -> byte index
         * 0 -> 0..3,
         * 1 -> 4..7
         * 2 -> 8..11
         * 3 -> 12..15
         * 4 -> 16..19
         */
        memcpy(data + 2, Device.Relay.CurState + block * RELAY_BLOCK_LENGTH, RELAY_BLOCK_LENGTH);

        if(CanRespSend(Device.Address, data, sizeof(data))== HAL_OK)
        {
          Device.Relay.ChangedBlocks[block] = 0;
          Device.Resp.Status++;
        }
      }
      block++;
    }
    else
    {
      Device.Status.CanTxErr++;
    }
  }
  else
  {
    block = 0;
    if(Device.StatusAutoSendEnable)
      RelayChangedBlocksUpdate(&Device.Relay);
  }
}

void DebugTask(void)
{
  static uint32_t timestamp;
  static uint8_t status = 0;
  char buff[DEVICE_STR_SIZE];
  memset(buff, 0x00, DEVICE_STR_SIZE);

  if(HAL_GetTick() - timestamp > 250)
  {
    timestamp = HAL_GetTick();

    status++;
    DeviceConsoleWrite(VT100_CUP("7","1"));
    if(status == 1)
      sprintf(buff, "-");
    else if(status == 2)
      sprintf(buff, "\\");
    else if(status == 3)
      sprintf(buff, "|");
    else if (status == 4)
      sprintf(buff, "/");
    else
      status = 0;
    DeviceConsoleWrite(buff);

    DeviceConsoleWrite(VT100_CUP("7","2"));
    sprintf(buff, "---STATUS-------------------------------------------------------------");
    DeviceConsoleWrite(buff);

    DeviceConsoleWrite(VT100_CUP("8","0"));
    sprintf(buff,"CanRx          %06lu | CanRxErr:     %06lu | UnknownFrame: %06lu", Device.Status.CanRx, Device.Status.CanRxErr, Device.Status.UnknownFrame);
    DeviceConsoleWrite(buff);
    DeviceConsoleWrite(VT100_CUP("9","0"));
    sprintf(buff,"CanTx:         %06lu | CanTxErr:     %06lu", Device.Status.CanTx, Device.Status.CanTxErr);
    DeviceConsoleWrite(buff);
    DeviceConsoleWrite(VT100_CUP("10","0"));
    sprintf(buff,"MemSaved:      %06lu | MemFail:      %06lu | MainCycleRate:%06lu",Device.Status.MemSaved, Device.Status.MemFail, Device.Status.MainCycleTime);
    DeviceConsoleWrite(buff);
    DeviceConsoleWrite(VT100_CUP("11","0"));
    sprintf(buff,"MemLoadTime:   %04lums | SaveingTime:  %04lums",Device.Memory.LoadTimeMs, Device.Memory.SaveingTimeMs);
    DeviceConsoleWrite(buff);
    DeviceConsoleWrite(VT100_CUP("12","0"));
    sprintf(buff,"StatusAutoSend:%d      |", Device.StatusAutoSendEnable);
    DeviceConsoleWrite(buff);

    DeviceConsoleWrite(VT100_CUP("15","0"));
    sprintf(buff, "---REQUESTS-----------------------------------------------------------");
    DeviceConsoleWrite(buff);
    DeviceConsoleWrite(VT100_CUP("16","0"));
    sprintf(buff,"AskAllInfo:    %06lu | GlobalReset:  %06lu | Reset:        %06lu", Device.Req.AskAllInfo, Device.Req.GlobalReset, Device.Req.Reset);
    DeviceConsoleWrite(buff);
    DeviceConsoleWrite(VT100_CUP("17","0"));
    sprintf(buff,"HostStart:     %06lu | Status:       %06lu | SetOneRealyl: %06lu", Device.Req.HostStart, Device.Req.Status, Device.Req.SetOneRealy);
    DeviceConsoleWrite(buff);
    DeviceConsoleWrite(VT100_CUP("18","0"));
    sprintf(buff,"OffOneRelay:   %06lu | SeveralOn:    %06lu | SeveralOff:   %06lu", Device.Req.OffOneRelay, Device.Req.SeveralOn, Device.Req.SeveralOff);
    DeviceConsoleWrite(buff);
    DeviceConsoleWrite(VT100_CUP("19","0"));
    sprintf(buff,"SeveralToogle: %06lu | RelayCounter: %06lu | SerialNumber: %06lu", Device.Req.SeveralToogle, Device.Req.RelayCounter, Device.Req.SerialNumber);
    DeviceConsoleWrite(buff);
    DeviceConsoleWrite(VT100_CUP("20","0"));
    sprintf(buff,"ResetRlyCnt:   %06lu |", Device.Req.ResetRlyCnt);
    DeviceConsoleWrite(buff);

    DeviceConsoleWrite(VT100_CUP("21","0"));
    sprintf(buff, "---RESPONSE-----------------------------------------------------------");
    DeviceConsoleWrite(buff);

    DeviceConsoleWrite(VT100_CUP("22","0"));
    sprintf(buff,"AskAllInfoReq: %06lu | AskAllInfo:   %06lu | RelayCounter: %06lu", Device.Resp.Status, Device.Resp.AskAllInfo, Device.Resp.RelayCounter);
    DeviceConsoleWrite(buff);
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

  DeviceUsrLog("Manufacturer:%s, Name:%s, Version:%04X",DEVICE_MNF, DEVICE_NAME, DEVICE_FW);
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
  uint8_t dummy[] = {0xFF};
  HAL_I2C_Master_Transmit(&hi2c2, 0xFF, dummy, sizeof(dummy), 100  );

  MemoryInit(&Device.Memory);
  if(MemoryTest() != MEM_OK)
  {
    Device.SelfTest.MemoryState = 1;
    Device.Status.MemFail++;
    DeviceErrLog("MemoryTest Failed...");
    LedShowCode(&hLed, DEVICE_FAIL_LED, FAIL_LED_MEM_TEST);
  }

  //MemoryReset(&Device.Memory);

  if(MemoryLoad(&Device.Memory)!= MEM_OK)
  {
    Device.SelfTest.MemoryState = 1;
    Device.Status.MemFail++;
    LedShowCode(&hLed, DEVICE_FAIL_LED, FAIL_LED_MEM_LOAD);
  }
  else
  {
    for(uint8_t i=0; i < RELAY_ARRAY; i++)
    {
      Device.Relay.Counters[i]=Device.Memory.RealyCounters[i];
    }
  }
  DeviceUsrLog("SerialNumber:%lu, BootUpCounter:%lu", Device.Memory.SerialNumber, Device.Memory.BootUpCounter);

  /*** Relay Driver Test ***/
  if(RelayDriverLoopTest()!= RELAY_OK)
  {
    LedShowCode(&hLed, DEVICE_FAIL_LED, FAIL_LED_RLY_DRV);
    Device.SelfTest.DriverLoopState = 1;
    DeviceErrLog("RelayDriverLoopTest: FAIL");
  }

  /*** Defaults ***/
  RelayReset(&Device.Relay);
  RelayEnable();
  memset(Device.Relay.ChangedBlocks,0x00, RELAY_MAX_BLOCK);
  Device.Address = GetAddress();
  Device.Version = DEVICE_FW;
  Device.CanSpeed = &CanSpeeds[GetSpeed()];
  DeviceUsrLog("CAN: Address: %d, Baudrate: %lu Baud",Device.Address, CanSpeeds[GetSpeed()].Baud);

  CanInit(Device.CanSpeed);
  CanAskAllInfoResponse();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t timestamp = HAL_GetTick();
    LiveLedTask(&hLiveLed);
    LedTask(&hLed);
    DebugTask();
    MemoryTask(&Device.Memory);
    StatusTask();

    Device.Status.MainCycleTime = HAL_GetTick() - timestamp;


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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  HAL_GPIO_WritePin(RLY_G_GPIO_Port, RLY_G_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : RLY_CLK_Pin RLY_MOSI_Pin */
  GPIO_InitStruct.Pin = RLY_CLK_Pin|RLY_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
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

  /*Configure GPIO pins : LIVE_LED_Pin RLY_WR_Pin */
  GPIO_InitStruct.Pin = LIVE_LED_Pin|RLY_WR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  /*Configure GPIO pin : RLY_G_Pin */
  GPIO_InitStruct.Pin = RLY_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RLY_G_GPIO_Port, &GPIO_InitStruct);

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

void DeviceConsoleWrite(char *str)
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

void FeilLedOn(void)
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
