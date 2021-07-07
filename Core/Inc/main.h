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
#include "vt100.h"
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

/***CARD_NAME   CARD_TYPE   6TL-OPTIONS-ALTON***
 * MALT160T     0x15        0x05
 * MALT132      0x03        0x00
 * MALT23HV     0x03        0x10
 */


//#define CONFIG_MALT160T
//#define CONFIG_MALT132
//#define CONFIG_MALT23THV
#define CONFIG_MALT24VI
//#define CONFIG_MALT16PIN
//#define CONFIG_MALT40IO


#if  defined(CONFIG_MALT160T)
  #define DEVICE_FIRST_NAME           "MALT160T"
  #define DEVICE_PCB                  "V00"
  #define DEVICE_OUTPUTS_COUNT        160
  #define DEVICE_INPUTS_COUNT         0
  #define DEVICE_BLOCK_SIZE           4
  #define DEVICE_BLOCKS               DEVICE_OUTPUTS_COUNT/8/4
  #define DEVICE_FAMILY_CODE          0x15
  #define DEVICE_OPTION_CODE          0x05
#elif defined(CONFIG_MALT132)
  #define DEVICE_FIRST_NAME           "MALT132"
  #define DEVICE_PCB                  "V00"
  #define DEVICE_OUTPUTS_COUNT        32
  #define DEVICE_INPUTS_COUNT         0
  #define DEVICE_BLOCK_SIZE           4
  #define DEVICE_BLOCKS               DEVICE_OUTPUTS_COUNT/8/4
  #define DEVICE_FAMILY_CODE          0x03
  #define DEVICE_OPTION_CODE          0x00
#elif defined(CONFIG_MALT23THV)
  #define DEVICE_FIRST_NAME           "MALT23THV"
  #define DEVICE_PCB                  "V00"
  #define DEVICE_OUTPUTS_COUNT        24
 #define DEVICE_INPUTS_COUNT          0
  #define DEVICE_BLOCK_SIZE           3 /*3 bajt van egy blokban, max 4 bájt lehet*/
  #define DEVICE_BLOCKS               1
  #define DEVICE_FAMILY_CODE          0x03
  #define DEVICE_OPTION_CODE          0x10
#elif defined(CONFIG_MALT24VI)
  #define DEVICE_FIRST_NAME           "MALT24VI"
  #define DEVICE_PCB                  "V00"
  #define DEVICE_OUTPUTS_COUNT        24
  #define DEVICE_INPUTS_COUNT         0
  #define DEVICE_BLOCK_SIZE           3
  #define DEVICE_BLOCKS               1
  #define DEVICE_FAMILY_CODE          0x03
  #define DEVICE_OPTION_CODE          0x11
#elif defined(CONFIG_MALT16PIN)
  #define DEVICE_FIRST_NAME           "MALT16PIN"
  #define DEVICE_PCB                  "V00"
  #define DEVICE_OUTPUTS_COUNT        16
  #define DEVICE_INPUTS_COUNT         0
  #define DEVICE_BLOCK_SIZE           3
  #define DEVICE_BLOCKS               1
  #define DEVICE_FAMILY_CODE          0x05
  #define DEVICE_OPTION_CODE          0x00
#elif defined(CONFIG_MALT40IO)
  #define DEVICE_FIRST_NAME           "MALT40IO"
  #define DEVICE_PCB                  "V00"
  #define DEVICE_OUTPUTS_COUNT        40
  #define DEVICE_INPUTS_COUNT         40
  #define DEVICE_BLOCK_SIZE           4
  #define DEVICE_BLOCKS               2
  #define DEVICE_FAMILY_CODE          0x01
  #define DEVICE_OPTION_CODE          0x00
#else
#error "Imserelten konfiguracio"
#endif

#if (DEVICE_INPUTS_COUNT !=0 && DEVICE_OUTPUTS_COUNT !=0)
  #if DEVICE_INPUTS_COUNT != DEVICE_OUTPUTS_COUNT
    #error "Kimenetek és a bemenetek szamanak egyeznie kell"
  #endif
#endif

#define CARD_RX_ADDRESS             0x15510000
#define CARD_TX_ADDRESS             0x15520000
#define HOST_ADDRESS                0x1558FFFF
#define DEVICE_SN_SIZE              3


#ifndef DEBUG
  #define DEVICE_FW           0x011A
#else
  #define DEVICE_FW           0x011D
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
#define DO_G_Pin GPIO_PIN_12
#define DO_G_GPIO_Port GPIOB
#define RLY_WR_Pin GPIO_PIN_8
#define RLY_WR_GPIO_Port GPIOA
#define EEP_ON_Pin GPIO_PIN_4
#define EEP_ON_GPIO_Port GPIOB
#define DI_CE_Pin GPIO_PIN_5
#define DI_CE_GPIO_Port GPIOB
void   MX_CAN_Init(void);
/* USER CODE BEGIN Private defines */

#if defined(CONFIG_MALT40IO)

  /*1*/
  #undef DO_G_Pin
  #undef DO_G_GPIO_Port

  #define DO_G_Pin GPIO_PIN_7
  #define DO_G_GPIO_Port GPIOB

  /*2*/
  #undef RLY_WR_Pin
  #undef RLY_WR_GPIO_Port

  #define RLY_WR_Pin GPIO_PIN_6
  #define RLY_WR_GPIO_Port GPIOB

  #define DI_LD_Pin  GPIO_PIN_8
  #define DI_LD_GPIO_Port GPIOA

#endif

/* Generic  -------------------------------------------------------------------*/


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






/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
