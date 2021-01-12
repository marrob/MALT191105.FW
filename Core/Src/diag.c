
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



DiagItemTypeDef DiagRx[] = {
  {0x00, 0},
  {0x01, 0},
  {0x03, 0},
  {0x04, 0},
  {0x06, 0},
  {0x07, 0},
  {0xDE, 0},
  {0xEE, 0},
  {0xFA, 0},
};

DiagItemTypeDef DiagTx[] = {
  {0x00, 0},
  {0x04, 0},
  {0x07, 0},
  {0xF0, 0},
  {0xFA, 0},
  {0xEE, 0}
};

#define DiagRxCount() (sizeof(DiagRx)/sizeof(DiagItemTypeDef))
#define DiagTxCount() (sizeof(DiagTx)/sizeof(DiagItemTypeDef))
char* SetConsolePos(uint8_t x, uint8_t y);

void DiagInit(DiagHandleTypeDef *h){

}


void DiagRxFrame(DiagHandleTypeDef *h, uint8_t *data, size_t length)
{

  uint8_t foundIt = 0;
  for(uint8_t i = 0; i < DiagRxCount(); i++ )
  {
    uint8_t temp = data[1];
    if(DiagRx[i].Byte1 == temp)
    {
      foundIt = 1;
       DiagRx[i].Counter++;
    }
  }

  if(!foundIt)
  {
    DiagRx[0].Counter++;
  }
}

void DiagTxFrame(DiagHandleTypeDef *h, uint8_t *data, size_t length){

  uint8_t foundIt = 0;
  for(uint8_t i=0; i<DiagTxCount(); i++ )
  {

    if(DiagTx[i].Byte1 ==  data[1])
    {
      foundIt = 1;
      DiagTx[i].Counter++;
    }
  }

  if(!foundIt)
  {
    DiagTx[0].Counter++;
  }

}


void DiagTask(DiagHandleTypeDef *h )
{
  static uint8_t status = 0;
  static uint32_t timestamp;
  static uint8_t consoleLine;
  char buff[DEVICE_STR_SIZE];
  memset(buff, 0x00, DEVICE_STR_SIZE);


  consoleLine = 7;
  uint8_t itemPerColumn = 5;

  if(HAL_GetTick() - timestamp > 250)
  {
    timestamp = HAL_GetTick();
    sprintf(buff, "---STATUS-------------------------------------------------------------");
    status++;

    ConsoleWrite(Vt100SetCursorPos(1, consoleLine++));
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
    ConsoleWrite(buff);


    /*
    0123456789012345
    0x00:65353 0x00:65353 0x00:65353 0x00:65353
    0x00:65353 0x00:65353 0x00:65353 0x00:65353
    */

    sprintf(buff, "---RX---\r\n");
    ConsoleWrite(Vt100SetCursorPos(1, consoleLine++));
    ConsoleWrite(buff);
    buff[0] = 0;

    uint8_t item = 0;
    char temp[12];
    uint8_t i = 0;

    do
    {
      do
      {
        if (i >= itemPerColumn || item > DiagRxCount() - 1)
        {
          strcat(buff, "\r\n");
          ConsoleWrite(Vt100SetCursorPos(1, consoleLine++));
          ConsoleWrite(buff);
          buff[0] = 0;
          i = 0;
          break;
        }
        else
        {
          sprintf(temp, "0x%02X:%05lu ", DiagRx[item].Byte1, DiagRx[item].Counter);
          strcat(buff, temp);
          item++;
          i++;
        }
      } while (1);
    } while (item < DiagRxCount());

    sprintf(buff, "---TX---\r\n");
    ConsoleWrite(buff);
    ConsoleWrite(Vt100SetCursorPos(1, consoleLine++));
    i = 0;
    item = 0;
    buff[0] = 0;
    do
    {
      do
      {
        if (i >= itemPerColumn || item > DiagTxCount() - 1)
        {
          strcat(buff, "\r\n");
          ConsoleWrite(Vt100SetCursorPos(1, consoleLine++));
          ConsoleWrite(buff);
          buff[0] = 0;
          i = 0;
          break;
        }
        else
        {
          sprintf(temp, "0x%02X:%05lu ", DiagTx[item].Byte1, DiagTx[item].Counter);
          strcat(buff, temp);
          item++;
          i++;
        }
      } while (1);
    } while (item < DiagTxCount());


  }
}





//#undef DEBUG
/*
void DebugTask(void)
{
#ifdef DEBUG
DiagTxCount
  static uint8_t status = 0;
  char buff[DEVICE_STR_SIZE];
  memset(buff, 0x00, DEVICE_STR_SIZE);

  if(HAL_GetTick() - timestamp > 250)
  {
    timestamp = HAL_GetTick();

    status++;
    ConsoleWrite(VT100_CUP("7","1"));
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
    ConsoleWrite(buff);

    ConsoleWrite(VT100_CUP("7","2"));
    sprintf(buff, "---STATUS-------------------------------------------------------------");
    ConsoleWrite(buff);

    ConsoleWrite(VT100_CUP("8","0"));
    sprintf(buff,"CanRx          %06lu | CanRxErr:     %06lu | UnknownFrame: %06lu", Device.Status.CanRx, Device.Status.CanRxErr, Device.Status.UnknownFrame);
    ConsoleWrite(buff);
    ConsoleWrite(VT100_CUP("9","0"));
    sprintf(buff,"CanTx:         %06lu | CanTxErr:     %06lu | UpTimeSec:    %06lu", Device.Status.CanTx, Device.Status.CanTxErr,Device.Status.UpTimeSec);
    ConsoleWrite(buff);
    ConsoleWrite(VT100_CUP("10","0"));
    sprintf(buff,"MemSaved:      %06lu | MemFail:      %06lu | MainCycleRate:%06lu",Device.Status.MemSaved, Device.Status.MemFail, Device.Status.MainCycleTime);
    ConsoleWrite(buff);
    ConsoleWrite(VT100_CUP("11","0"));
    sprintf(buff,"MemLoadTime:   %04lums | SaveingTime:  %04lums",Device.Memory.LoadTimeMs, Device.Memory.SaveingTimeMs);
    ConsoleWrite(buff);
    ConsoleWrite(VT100_CUP("12","0"));
    sprintf(buff,"StatusAutoSend:%d      |", Device.Io.Output.StatusAutoSendEnable);
    ConsoleWrite(buff);

    ConsoleWrite(VT100_CUP("15","0"));
    sprintf(buff, "---REQUESTS-----------------------------------------------------------");
    ConsoleWrite(buff);
    ConsoleWrite(VT100_CUP("16","0"));
    sprintf(buff,"AskAllInfo:    %06lu | Reset:        %06lu", Device.Req.AskAllInfo, Device.Req.Reset);
    ConsoleWrite(buff);
    ConsoleWrite(VT100_CUP("17","0"));
    sprintf(buff,"HostStart:     %06lu | StatusOutput:       %06lu | OneOn:        %06lu", Device.Req.HostStart, Device.Req.StatusOutput, Device.Req.OneOn);
    ConsoleWrite(buff);
    ConsoleWrite(VT100_CUP("18","0"));
    sprintf(buff,"OneOff:        %06lu | SeveralOn:    %06lu | SeveralOff:   %06lu", Device.Req.OneOff, Device.Req.SeveralOn, Device.Req.SeveralOff);
    ConsoleWrite(buff);
    ConsoleWrite(VT100_CUP("19","0"));
    sprintf(buff,"SeveralToogle: %06lu | RelayCounter: %06lu | SerialNumber: %06lu", Device.Req.SeveralToogle, Device.Req.RelayCounter, Device.Req.SerialNumber);
    ConsoleWrite(buff);
    ConsoleWrite(VT100_CUP("20","0"));
    sprintf(buff,"ResetRlyCnt:   %06lu |", Device.Req.ResetRlyCnt);
    ConsoleWrite(buff);

    ConsoleWrite(VT100_CUP("21","0"));
    sprintf(buff, "---RESPONSE-----------------------------------------------------------");
    ConsoleWrite(buff);

    ConsoleWrite(VT100_CUP("22","0"));
    sprintf(buff,"AskAllInfoReq: %06lu | AskAllInfo:   %06lu | RelayCounter: %06lu", Device.Resp.Status, Device.Resp.AskAllInfo, Device.Resp.RelayCounter);
    ConsoleWrite(buff);
  }
#endif
}
*/
