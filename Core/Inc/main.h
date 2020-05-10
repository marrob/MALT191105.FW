/* USER CODE BEGIN Header */
/*
 * main.h
 *
 *  Created on: 2020. márc. 6.
 *      Author: Margit Robert
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "LED.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi2;
extern LedHandle_Type    hLed;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#ifdef DEBUG
  #define DEVICE_DEBUG_LEVEL    3
#else
  #define DEVICE_DEBUG_LEVEL    2
#endif

#if (DEVICE_DEBUG_LEVEL > 0)
#define  DeviceUsrLog(...)  {printf(__VA_ARGS__);\
                             printf("\r\n");}
#else
#define DeviceUsrLog(...)
#endif

#if (DEVICE_DEBUG_LEVEL > 1)

#define  DeviceErrLog(...)  {printf(VT100_ATTR_RED);\
                             printf("ERROR.DEVICE:") ;\
                             printf(__VA_ARGS__);\
                             printf(VT100_ATTR_RESET);\
                             printf("\r\n");}
#else
#define DeviceErrLog(...)
#endif

#if (DEVICE_DEBUG_LEVEL > 2)
#define  DeviceDbgLog(...)  {printf(VT100_ATTR_YELLOW);\
                             printf("DEBUG.DEVICE:") ;\
                             printf(__VA_ARGS__);\
                             printf(VT100_ATTR_RESET);\
                             printf("\r\n");}
#else
#define DeviceDbgLog(...)
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void ConsoleWrite(char *str);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RLY_CLK_Pin GPIO_PIN_13
#define RLY_CLK_GPIO_Port GPIOC
#define RLY_MISO_Pin GPIO_PIN_14
#define RLY_MISO_GPIO_Port GPIOC
#define RLY_MOSI_Pin GPIO_PIN_15
#define RLY_MOSI_GPIO_Port GPIOC
#define FAIL_LED_Pin GPIO_PIN_1
#define FAIL_LED_GPIO_Port GPIOA
#define LIVE_LED_Pin GPIO_PIN_2
#define LIVE_LED_GPIO_Port GPIOA
#define DIP1_Pin GPIO_PIN_3
#define DIP1_GPIO_Port GPIOA
#define DIP2_Pin GPIO_PIN_4
#define DIP2_GPIO_Port GPIOA
#define DIP3_Pin GPIO_PIN_5
#define DIP3_GPIO_Port GPIOA
#define DIP4_Pin GPIO_PIN_6
#define DIP4_GPIO_Port GPIOA
#define DIP5_Pin GPIO_PIN_7
#define DIP5_GPIO_Port GPIOA
#define DIP6_Pin GPIO_PIN_0
#define DIP6_GPIO_Port GPIOB
#define DIP7_Pin GPIO_PIN_1
#define DIP7_GPIO_Port GPIOB
#define DIP8_Pin GPIO_PIN_2
#define DIP8_GPIO_Port GPIOB
#define RLY_G_Pin GPIO_PIN_12
#define RLY_G_GPIO_Port GPIOB
#define RLY_WR_Pin GPIO_PIN_8
#define RLY_WR_GPIO_Port GPIOA
#define EEP_ON_Pin GPIO_PIN_4
#define EEP_ON_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/***CARD_NAME   CARD_TYPE   6TL-OPTIONS-ALTON***
 * MALT160T     0x15        0x05
 * MALT132      0x03        0x00
 * MALT23HV     0x03        0x10
 *
 *
 *
 */
#define CARD_TYPE     0x03
#define CARD_OPTIONS  0x10

/* Generic  -------------------------------------------------------------------*/
#if (CARD_TYPE == 0x15)
  #define DEVICE_NAME                 "MALT160T"
  #define DEVICE_PCB                  "00"
  #define DEVICE_OUTPUT_COUNT         160
  #define DEVICE_BLOCK_SIZE           4
  #define DEVICE_BLOCKS               DEVICE_OUTPUT_COUNT/8/4
#elif(CARD_TYPE == 0x03 && CARD_OPTIONS == 0)
  #define DEVICE_NAME                 "MALT132"
  #define DEVICE_PCB                  "00"
  #define DEVICE_OUTPUT_COUNT         32
  #define DEVICE_BLOCK_SIZE           4
  #define DEVICE_BLOCKS               DEVICE_OUTPUT_COUNT/8/4
#elif(CARD_TYPE == 0x03 && CARD_OPTIONS == 0x10)
  #define DEVICE_NAME                 "MALT23TV00"
  #define DEVICE_PCB                  "00"
  #define DEVICE_OUTPUT_COUNT         24
  #define DEVICE_BLOCK_SIZE           3 /*3 bajt van egy blokban, max 4 bájt lehet*/
  #define DEVICE_BLOCKS               1
#endif

#define CARD_RX_ADDRESS             0x15510000
#define CARD_TX_ADDRESS             0x15520000
#define HOST_ADDRESS                0x1558FFFF
#define DEVICE_SN_SIZE              3
#define DEVICE_DELAY_STATUS_TX_MS   5

#ifndef DEBUG
  #define DEVICE_FW           0x006A
#else
  #define DEVICE_FW           0x006D
#endif

#define DEVICE_MNF          "AltonTech"
#define DEVICE_NAME_SIZE    32
#define DEVICE_FW_SIZE      sizeof(DEVICE_FW)
#define DEVICE_PCB_SIZE     sizeof(DEVICE_PCB)
#define DEVICE_MNF_SIZE     sizeof(DEVICE_MNF)

#define DEVICE_OK           0
#define DEVICE_FAIL         1

#define DEVICE_FAIL_LED     0
#define DEVICE_STR_SIZE     80

#define FAIL_LED_MEM_LOAD   1
#define FAIL_LED_MEM_TEST   2
#define FAIL_LED_RLY_DRV    3
#define FAIL_LED_MEM_WRITE  4


/* VT100 ---------------------------------------------------------------------*/
/*
 * https://www.csie.ntu.edu.tw/~r92094/c++/VT100.html
 * http://www.termsys.demon.co.uk/vtansi.htm
 */
#define VT100_CLEARSCREEN         "\033[2J"
#define VT100_CURSORHOME          "\033[H"
#define VT100_ATTR_RESET          "\033[0m"
#define VT100_ATTR_RED            "\033[31m"
#define VT100_ATTR_GREEN          "\033[32m"
#define VT100_ATTR_YELLOW         "\033[33m"
#define VT100_CUP(__v__,__h__)    ("\033["__v__";"__h__"H") /*Cursor Position*/



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
