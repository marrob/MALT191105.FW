
/* USER CODE BEGIN Header */
/*
 * frame_diag.c
 *
 *  Created on: 2020. márc. 6.
 *      Author: Margit Robert
 */
/* Includes ------------------------------------------------------------------*/
#include "diag.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "vt100.h"


DiagItemTypeDef DiagItems[40];


void DiagInit(DiagHandleTypeDef *h){


  DiagAddFrame(h,0xAA,DIAG_FRAME_TYPE_TX,'-');
  DiagAddFrame(h,0x55,DIAG_FRAME_TYPE_TX,'-');

}


void DiagAddFrame(DiagHandleTypeDef *h, uint8_t address, uint8_t type, char marker)
{

  uint8_t foundIt = 0;
  for(uint8_t i = 1; i < h->ItemsCount; i++ )
  {
    if(DiagItems[i].Address == address && DiagItems[i].Type == type)
    {
       foundIt = 1;
       DiagItems[i].Counter++;
       DiagItems[h->ItemsCount].Address = address;
       DiagItems[h->ItemsCount].Marker= marker;
       DiagItems[i].Marker= marker;
    }
  }

  if(!foundIt)
  {
    DiagItems[h->ItemsCount].Counter++;
    DiagItems[h->ItemsCount].Address = address;
    DiagItems[h->ItemsCount].Type = type;
    DiagItems[h->ItemsCount].Marker= marker;
    h->ItemsCount++;
  }
}

void DiagTask(DiagHandleTypeDef *h )
{
  static uint8_t status = 0;
  static uint32_t timestamp;
  static uint8_t consoleLine;


  char buff[DEVICE_STR_SIZE];
  memset(buff, 0x00, DEVICE_STR_SIZE);

  consoleLine = 8;

  if(HAL_GetTick() - timestamp > 250)
  {
    timestamp = HAL_GetTick();

    sprintf(buff, "---STATUS-------------------------------------------------------------");
    status++;

    ConsoleWrite(Vt100SetCursorPos(1, consoleLine++));
    if(status == 1)
      buff[0]='-';
    else if(status == 2)
      buff[0]='\\';
    else if(status == 3)
      buff[0]='|';
    else if (status == 4)
      buff[0]='/';
    else
      status = 0;
    ConsoleWrite(buff);

    buff[0] = 0;

    uint8_t index = 0;
    char temp[80];

    for(uint8_t rows = 0; rows < 20; rows++)
    {
      for(uint8_t coulmns = 0; coulmns < 6; coulmns++){
        if(index > h->ItemsCount - 1)
          break;
        if(DiagItems[index].Type == DIAG_FRAME_TYPE_RX)
          sprintf(temp, "R%c:0x%02X:%05lu ", DiagItems[index].Marker, DiagItems[index].Address, DiagItems[index].Counter);
        else
          sprintf(temp, "T%c:0x%02X:%05lu ", DiagItems[index].Marker, DiagItems[index].Address, DiagItems[index].Counter);
        strcat(buff, temp);
        index++;
      }
      strcat(buff, "\r\n");
      ConsoleWrite(Vt100SetCursorPos(1, consoleLine++));
      ConsoleWrite(buff);
      buff[0] = 0;
      if(index > h->ItemsCount - 1)
        break;
    }
  }
}


