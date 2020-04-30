/******************** (C) COPYRIGHT 2015 marrob Design *************************
* File Name          : LedDrv.c
* Author             : Margit R�bert
* Date First Issued  : 2014.05.10
* Description        : Generic LED Drive: On,Off,Blink 
********************************************************************************/
/********************************************************************************
Update: 
- 2015.03.25:  Blink bekapcsol�sakor vizsg�lja az eloz� allapotot, ha
               blink volt el�tte is, akkor nem bantja az idoziteseket
- 2016.11.15: Jav�tva a Timestamp t�pusa uint_16-r�l int32_t-re v�ltozott.
- 2018.12.20: Comments update
- 2020.02.24: STM32CubeIDE Support...
- 2020.04.20: egyszerüsitesek, - LedCode fv hozzadva
********************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#include "Led.h"


/**
  * @brief  Led Init
  * @param hnd: Handle
  */
void LedInit(LedHandle_Type *hnd)
{

}

/**
  * @brief  Led Blink
  * @param hnd: Handle
  */
void LedBlink(LedHandle_Type *hnd, uint8_t id, uint16_t periodTime)
{
  if(hnd->pLedTable[id].State.Pre != LED_STATE_BLINK)
  {
      hnd->pLedTable[id].Timestamp = HAL_GetTick();
      hnd->pLedTable[id].State.Curr = LED_STATE_BLINK;
  }
  hnd->pLedTable[id].PeriodTime = periodTime;
  hnd->pLedTable[id].Pwm = 0;
}

/**
  * @brief  LedBlinkPwm
  * @param hnd: Handle
  */
void LedBlinkPwm(LedHandle_Type *hnd, uint8_t id, uint16_t periodTime, uint8_t pwm)
{
  if(hnd->pLedTable[id].State.Pre != LED_STATE_BLINK)
  {
      hnd->pLedTable[id].Timestamp = HAL_GetTick();
      hnd->pLedTable[id].State.Curr = LED_STATE_BLINK;
  }
  hnd->pLedTable[id].PeriodTime = periodTime;
  hnd->pLedTable[id].Pwm = pwm;
}


/**
 * 1: | ON:200ms | OFF:2sec
 * 2: | ON:100ms | OFF:100ms| ON:100ms | OFF:2sec
 * 3: | ON:100ms | OFF:100ms| ON:100ms | OFF:100ms| ON:100ms| OFF:2sec
 */
void LedShowCode(LedHandle_Type *hnd, uint8_t id, uint8_t code)
{
  if(hnd->pLedTable[id].State.Pre != LED_STATE_CODE)
  {
      hnd->pLedTable[id].Timestamp = HAL_GetTick();
      hnd->pLedTable[id].State.Curr = LED_STATE_CODE;
      hnd->pLedTable[id].State.Next = LED_STATE_CODE;
  }
  hnd->pLedTable[id].Code = code;
  hnd->pLedTable[id].CodeCnt = 0;
}

/**
  * @brief  Led On
  * @param hnd: Handle
  */
void LedOn(LedHandle_Type *hnd, uint8_t id)
{
  hnd->pLedTable[id].State.Curr = LED_STATE_ON;
}

/**
  * @brief  Led Off
  * @param hnd: Handle
  */
void LedOff(LedHandle_Type *hnd, uint8_t id)
{
  hnd->pLedTable[id].State.Curr = LED_STATE_OFF;
}

/**
  * @brief  LED Task
  * @param hnd: Handle
  */
void LedTask(LedHandle_Type *hnd)
{
  for(uint8_t i = 0; i< hnd->Records; i++)
  {
    switch (hnd->pLedTable[i].State.Curr)
    {
      case LED_STATE_IDLE:
      {
        break;
      }
      case LED_STATE_ON:
      {
        if(hnd->pLedTable[i].State.Pre != LED_STATE_ON)
        {
            hnd->pLedTable[i].LEDOn();
            hnd->pLedTable[i].State.Next = LED_STATE_ON;
        }
        break;
      }
      case LED_STATE_OFF:
      {
        if(hnd->pLedTable[i].State.Pre != LED_STATE_OFF)
        {
            hnd->pLedTable[i].LEDOff();
            hnd->pLedTable[i].State.Next = LED_STATE_OFF;
        }
        break;
      }
      case LED_STATE_BLINK:
      {
        if(hnd->pLedTable[i].State.Pre != LED_STATE_BLINK)
        {
            hnd->pLedTable[i].Timestamp = HAL_GetTick();
            hnd->pLedTable[i].State.Next = LED_STATE_BLINK;
        }
        if((HAL_GetTick()- hnd->pLedTable[i].Timestamp) > hnd->pLedTable[i].PeriodTime)
        {
          hnd->pLedTable[i].Timestamp = HAL_GetTick();
          if(hnd->pLedTable[i].BlinkFlag)
          {
            if(hnd->pLedTable[i].Pwm != 0)
            {
              hnd->pLedTable[i].PwmValue++;
              if( hnd->pLedTable[i].PwmValue > hnd->pLedTable[i].Pwm)
              {
                  hnd->pLedTable[i].PwmValue = 0;
                  hnd->pLedTable[i].LEDOn();
                  hnd->pLedTable[i].BlinkFlag = 0;
              }
            }
            else
            {
                hnd->pLedTable[i].LEDOn();
                hnd->pLedTable[i].BlinkFlag = 0;
            }
          }
          else
          {
              hnd->pLedTable[i].LEDOff();
              hnd->pLedTable[i].BlinkFlag = 1;
          }
        }
      break;
      }
      case LED_STATE_CODE:
      {
        if(hnd->pLedTable[i].State.Pre != LED_STATE_CODE)
        {
          hnd->pLedTable[i].Timestamp = HAL_GetTick();
          hnd->pLedTable[i].BlinkFlag = 1;
          hnd->pLedTable[i].CodeCnt = 0;

        }

        if((HAL_GetTick()- hnd->pLedTable[i].Timestamp) > 400)
        {
          hnd->pLedTable[i].Timestamp = HAL_GetTick();

          if(hnd->pLedTable[i].BlinkFlag)
          {
            hnd->pLedTable[i].LEDOn();
            hnd->pLedTable[i].BlinkFlag = 0;
          }
          else
          {
            hnd->pLedTable[i].LEDOff();
            hnd->pLedTable[i].BlinkFlag = 1;
          }

          if(hnd->pLedTable[i].CodeCnt < (2 * hnd->pLedTable[i].Code))
          {
            hnd->pLedTable[i].CodeCnt++;
            hnd->pLedTable[i].State.Next = LED_STATE_CODE;
          }
          else
          {
            hnd->pLedTable[i].LEDOff();
            hnd->pLedTable[i].State.Next = LED_STATE_CODE_STOP;
          }
        }
        break;
      }
      case LED_STATE_CODE_STOP:
      {
        if(hnd->pLedTable[i].State.Pre != LED_STATE_CODE_STOP)
        {
          hnd->pLedTable[i].Timestamp = HAL_GetTick();
          hnd->pLedTable[i].BlinkFlag = 0;
        }

        if((HAL_GetTick()- hnd->pLedTable[i].Timestamp) > 3000)
        {
          hnd->pLedTable[i].State.Next = LED_STATE_CODE;
        }
        break;
      }
    }
      hnd->pLedTable[i].State.Pre = hnd->pLedTable[i].State.Curr;
    hnd->pLedTable[i].State.Curr = hnd->pLedTable[i].State.Next;
  }
}
/******************* (C) COPYRIGHT 2015 marrob Design *****END OF FILE******/
