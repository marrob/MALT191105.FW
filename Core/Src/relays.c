/*
 * relays.c
 *
 *  Created on: 2020. márc. 5.
 *      Author: Margit Robert
 */

/* Includes ------------------------------------------------------------------*/
#include "relays.h"

/* Private function prototypes -----------------------------------------------*/
void RelayUpdate(uint8_t *state);
void ArrayToolsU8SetBit(const uint16_t index, void* array);
static void RelayCounterUpdate(uint8_t *pre, uint8_t *cur, uint32_t *relaycounter);

/* Relays ---------------------------------------------------------------------*/
/*
 * k: 1..RELAY_COUNT
 */
void RelayOnOne(RelayTypeDef *h, uint8_t k)
{
  memcpy(h->PreState, h->CurState, RELAY_ARRAY);
  uint8_t temp[RELAY_ARRAY];
  memset(temp, 0x00, RELAY_ARRAY);
  ArrayToolsU8SetBit(k, temp);
  for(uint8_t i=0; i < RELAY_ARRAY; i++ )
    h->CurState[i]|=temp[i];
  RelayCounterUpdate(h->PreState, h->CurState, h->Counters);
  RelayUpdate(h->CurState);
}

/*
 * k: 1..RELAY_COUNT
 */
void RelayOffOne(RelayTypeDef *h, uint8_t k)
{
  memcpy(h->PreState, h->CurState, RELAY_ARRAY);
  uint8_t temp[RELAY_ARRAY];
  memset(temp, 0x00, RELAY_ARRAY);
  ArrayToolsU8SetBit(k, temp);
  for(uint8_t i=0; i<RELAY_ARRAY; i++)
    temp[i] = ~temp[i];
  for(uint8_t i=0; i < RELAY_ARRAY; i++ )
    h->CurState[i] &=temp[i];
  RelayCounterUpdate(h->PreState, h->CurState, h->Counters);
  RelayUpdate(h->CurState);
}

void RelayToogleSeveral(RelayTypeDef *h, uint8_t *several, uint8_t block)
{
  memcpy(h->PreState, h->CurState, RELAY_ARRAY);
  for(uint8_t i=0; i<RELAY_BLOCK_LENGTH; i++)
  {
    if(several[i])
    { /*csak tooglezunk amelyik bájtvan volt 1-es*/
      if(several[i] & h->CurState[block * RELAY_BLOCK_LENGTH + i])
      {
        h->CurState[block * RELAY_BLOCK_LENGTH + i] &= ~several[i];
      }
      else
      {
        h->CurState[block * RELAY_BLOCK_LENGTH + i] |= several[i];
      }
      break;
    }
  }
  RelayCounterUpdate(h->PreState, h->CurState, h->Counters);
  RelayUpdate(h->CurState);
}

/*
 * Egy blokkban 4-bájt van.
 * Ez egyszerre csak 4-bájtot tud modositani
 * A 160 relénél 5 blokk van (5x4bájt =20bájt * 8 bit =160).
 */
void RelayOffSeveral(RelayTypeDef *h, uint8_t *several, uint8_t block)
{
  memcpy(h->PreState, h->CurState, RELAY_ARRAY);
  for(uint8_t i=0; i < RELAY_BLOCK_LENGTH; i++)
    h->CurState[block * RELAY_BLOCK_LENGTH + i] &= ~several[i];
  RelayCounterUpdate(h->PreState, h->CurState, h->Counters);
  RelayUpdate(h->CurState);
}

void RelayOnSeveral(RelayTypeDef *h, uint8_t *several, uint8_t block)
{
  memcpy(h->PreState, h->CurState, RELAY_ARRAY);
  for(uint8_t i=0; i < RELAY_BLOCK_LENGTH; i++)
    h->CurState[block * RELAY_BLOCK_LENGTH + i] |= several[i];
  RelayCounterUpdate(h->PreState, h->CurState, h->Counters);
  RelayUpdate(h->CurState);
}

void RelayEnable(void)
{
  HAL_GPIO_WritePin(RLY_G_GPIO_Port, RLY_G_Pin, GPIO_PIN_RESET);
}

void ArrayToolsU8SetBit(const uint16_t index, void* array)
{
  uint8_t *ptr = array;
  uint16_t bitIndex;
  uint8_t  mask;
  uint8_t  byteIndex;

  mask = 0x80;
  byteIndex = 0x00;
  bitIndex = 1;

  byteIndex = index/8;
  if(index % 8)
  {
     bitIndex = index - (byteIndex*8);
     mask = 1;
     mask <<= bitIndex;
  }
  else
     mask = 0x01;

  ptr[byteIndex]|= mask;
}


void RelayReset(RelayTypeDef *h)
{
  memset(h->PreState, 0x00, RELAY_ARRAY);
  memset(h->CurState, 0x00, RELAY_ARRAY);
  RelayUpdate(h->CurState);
}

void RelayUpdate(uint8_t *state)
{
  uint8_t i,j;
  uint8_t dummy[RELAY_ARRAY];
  uint8_t reverse[RELAY_ARRAY];

  memset(dummy, 0x00, sizeof(dummy));

  /*Ezt egyszer hardveresen kell megcsinalni*/
  j = RELAY_ARRAY-1;
  for(i=0; i< RELAY_ARRAY; i++)
    reverse[j--] = state[i];

  HAL_SPI_TransmitReceive(&hspi2, reverse, dummy, sizeof(state), 100);

  /*ADS_LD# -> Load/Write */
  HAL_GPIO_WritePin(RLY_WR_GPIO_Port, RLY_WR_Pin, GPIO_PIN_SET);
  DelayUs(1);
  HAL_GPIO_WritePin(RLY_WR_GPIO_Port, RLY_WR_Pin, GPIO_PIN_RESET);
}

void RelayChangedBlocksUpdate(RelayTypeDef *h)
{
  uint8_t dif[RELAY_ARRAY];
 //
  for(uint8_t i = 0; i < RELAY_ARRAY; i++)
  {
    dif[i] = h->PreBlockState[i] ^ h->CurState[i];
    if(dif[i])
    {
      /*Az i/4-blockban volt változás*/
      h->ChangedBlocks[i/4] = 1;
      /*Tudomasul vettük a változást*/
      h->PreBlockState[i] = h->CurState[i];
    }
  }
}

static void RelayCounterUpdate(uint8_t *pre, uint8_t *cur, uint32_t *relaycounter)
{
  uint8_t dif[RELAY_ARRAY];
  uint8_t relay_index = 0;
  for(uint8_t i = 0; i < RELAY_ARRAY; i++)
  {
    dif[i] = pre[i] ^ cur[i];
    /*Ahol változás volt, azt keresseük ami 0-ról 1-re váltott és a hozzá tartozó indexet*/
    uint8_t mask = 0x01;
    for(uint8_t j = 0; j < 8; j++)
    {
        if(dif[i] & mask & cur[i])
        {
          relaycounter[relay_index]++;
        }
        mask<<=1;
        relay_index++;
    }
  }
}
/*
 * relaynumber: 0..RELAY_COUNT-1
 */
uint32_t RelayCounterGet(RelayTypeDef *h, uint8_t relaynumber)
{
  if(relaynumber > DEVICE_RELAY_COUNT)
  {
    return UINT32_MAX;
  }
  return h->Counters[relaynumber];
}




uint8_t RelayDriverLoopTest(void)
{
  /*4x8=32bit*/

  uint8_t testvector[RELAY_DIRVERS_CNT*2];
  uint8_t result[RELAY_DIRVERS_CNT*2];
  memset(testvector,0x55, sizeof(testvector)/2);
  memset(result,0x00,  sizeof(testvector));

  HAL_SPI_TransmitReceive(&hspi2, testvector, result, RELAY_DIRVERS_CNT * 2, 100);
  if(memcmp(testvector, result + RELAY_DIRVERS_CNT, RELAY_DIRVERS_CNT) == 0)
    return RELAY_OK;
  else
    return RELAY_FAIL;
}

