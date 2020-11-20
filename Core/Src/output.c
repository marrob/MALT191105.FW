/*
 * relays.c
 *
 *  Created on: 2020. márc. 5.
 *      Author: Margit Robert
 */

/* Includes ------------------------------------------------------------------*/
#include <output.h>

/* Private function prototypes -----------------------------------------------*/
static void Update(const uint8_t *state);
static void ArrayToolsU8SetBit(const uint16_t index, void* array);
static void CounterUpdate(uint8_t *pre, uint8_t *cur, uint32_t *relaycounter);
inline static void InputLoad(void);
inline static void OutputWrite(void);

/* ---------------------------------------------------------------------------*/

void IoInit(OutputTypeDef *context)
{

}

/*
 * k: 0..DEVICE_OUTPUT_COUNT-1
 */
void OutpuOneOn(OutputTypeDef *h, uint8_t k)
{
  memcpy(h->PreState, h->CurState, SPI_IO_ARRAY_SIZE);
  uint8_t temp[SPI_IO_ARRAY_SIZE];
  memset(temp, 0x00, SPI_IO_ARRAY_SIZE);
  ArrayToolsU8SetBit(k, temp);
  for(uint8_t i=0; i < SPI_IO_ARRAY_SIZE; i++ )
    h->CurState[i]|=temp[i];
  CounterUpdate(h->PreState, h->CurState, h->Counters);
//!  Update(h->CurState);
}

/*
 * k: 0..DEVICE_OUTPUT_COUNT-1
 */
void OutputOneOff(OutputTypeDef *h, uint8_t k)
{
  memcpy(h->PreState, h->CurState, SPI_IO_ARRAY_SIZE);
  uint8_t temp[SPI_IO_ARRAY_SIZE];
  memset(temp, 0x00, SPI_IO_ARRAY_SIZE);
  ArrayToolsU8SetBit(k, temp);
  for(uint8_t i=0; i<SPI_IO_ARRAY_SIZE; i++)
    temp[i] = ~temp[i];
  for(uint8_t i=0; i < SPI_IO_ARRAY_SIZE; i++ )
    h->CurState[i] &=temp[i];
  CounterUpdate(h->PreState, h->CurState, h->Counters);
//!  Update(h->CurState);
}

void OutputSeveralToogle(OutputTypeDef *h, uint8_t *several, uint8_t block)
{
  memcpy(h->PreState, h->CurState, SPI_IO_ARRAY_SIZE);
  for(uint8_t i = 0; i < DEVICE_BLOCK_SIZE; i++)
  {
    if(several[i])
    { /*csak tooglezunk amelyik bájtvan volt 1-es*/
      if(several[i] & h->CurState[block * DEVICE_BLOCK_SIZE + i])
        h->CurState[block * DEVICE_BLOCK_SIZE + i] &= ~several[i];
      else
        h->CurState[block * DEVICE_BLOCK_SIZE + i] |= several[i];
    }
  }
  CounterUpdate(h->PreState, h->CurState, h->Counters);
//!  Update(h->CurState);
}

/*
 * Egy blokkban 4-bájt van.
 * Ez egyszerre csak 4-bájtot tud modositani
 * A 160 relénél 5 blokk van (5x4bájt =20bájt * 8 bit =160).
 */
void OutputOffSeveral(OutputTypeDef *h, uint8_t *several, uint8_t block)
{
  memcpy(h->PreState, h->CurState, SPI_IO_ARRAY_SIZE);
  for(uint8_t i=0; i < DEVICE_BLOCK_SIZE; i++)
    h->CurState[block * DEVICE_BLOCK_SIZE + i] &= ~several[i];
  CounterUpdate(h->PreState, h->CurState, h->Counters);
  Update(h->CurState);
}

void OutputOnSeveral(OutputTypeDef *h, uint8_t *several, uint8_t block)
{
  memcpy(h->PreState, h->CurState, SPI_IO_ARRAY_SIZE);
  for(uint8_t i=0; i < DEVICE_BLOCK_SIZE; i++)
    h->CurState[block * DEVICE_BLOCK_SIZE + i] |= several[i];
  CounterUpdate(h->PreState, h->CurState, h->Counters);
//!  Update(h->CurState);
}

void OutputEnable(void)
{
  HAL_GPIO_WritePin(DO_G_GPIO_Port, DO_G_Pin, GPIO_PIN_RESET);
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


void OutputReset(OutputTypeDef *h)
{
  memset(h->PreState, 0x00, SPI_IO_ARRAY_SIZE);
  memset(h->CurState, 0x00, SPI_IO_ARRAY_SIZE);
//!  Update(h->CurState);
}

void Update(const uint8_t *state)
{

  uint8_t i,j;
  uint8_t buffer[SPI_IO_ARRAY_SIZE];
  uint8_t dummy[SPI_IO_ARRAY_SIZE];
  uint8_t reverse[SPI_IO_ARRAY_SIZE];

#if defined(CONFIG_MALT160T)
  j=0;
  for(i=0; i < SPI_IO_ARRAY_SIZE/2; i++)
  {
    buffer[j]= state[i];
    j+=2;
  }
  j=1;
  for(i=SPI_IO_ARRAY_SIZE/2; i < SPI_IO_ARRAY_SIZE; i++)
  {
    buffer[j]= state[i];
    j+=2;
  }
#else
  memcpy(buffer, state, SPI_IO_ARRAY_SIZE);
#endif

  memset(dummy, 0x00, sizeof(dummy));

  /*Ezt egyszer hardveresen kell megcsinalni*/
  j = SPI_IO_ARRAY_SIZE-1;
  for(i=0; i< SPI_IO_ARRAY_SIZE; i++)
    reverse[j--] = buffer[i];

  HAL_SPI_TransmitReceive(&hspi2, reverse, dummy, SPI_IO_ARRAY_SIZE, 100);

  /*#RLY_WR -> Load/Write */
  HAL_GPIO_WritePin(RLY_WR_GPIO_Port, RLY_WR_Pin, GPIO_PIN_SET);
  DelayUs(1);
  HAL_GPIO_WritePin(RLY_WR_GPIO_Port, RLY_WR_Pin, GPIO_PIN_RESET);
}






void OutputChangedBlocksUpdate(OutputTypeDef *h)
{
  uint8_t dif[SPI_IO_ARRAY_SIZE];

  for(uint8_t i = 0; i < SPI_IO_ARRAY_SIZE; i++)
  {
    dif[i] = h->PreBlockState[i] ^ h->CurState[i];
    if(dif[i])
    {

      /*
       * pl 160relé setén az OUTPUT_ARRAY 160/8 = 20 bájt
       * egy OUTPUT_BLOCK_SIZE ban 4 bájt van, ezért 4db
       * üzenteben lehet kiükldeni az összes változást.
       * De csak azt a blokkot küldjük ki, ahol a változás történt
       * ChangedBlocks indexei azokat a blokkat jelölik
       * amiket ki kell küldeni
       */

      /*Az i/4-blockban volt változás mivel egy blokban 4bajt van*/
      h->ChangedBlocks[i/DEVICE_BLOCK_SIZE] = 1;
      /*Tudomasul vettük a változást*/
      h->PreBlockState[i] = h->CurState[i];
    }
  }
}

static void CounterUpdate(uint8_t *pre, uint8_t *cur, uint32_t *relaycounter)
{
  uint8_t dif[SPI_IO_ARRAY_SIZE];
  uint8_t relay_index = 0;
  for(uint8_t i = 0; i < SPI_IO_ARRAY_SIZE; i++)
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
uint32_t OutputCounterGet(OutputTypeDef *h, uint8_t relaynumber)
{
  if(relaynumber > DEVICE_OUTPUTS_COUNT)
  {
    return 0;
  }
  return h->Counters[relaynumber];
}

uint32_t OutputCounterSet(OutputTypeDef *h, uint8_t relaynumber, uint32_t value)
{
  if(relaynumber > DEVICE_OUTPUTS_COUNT)
  {
    return 0;
  }
  else
  {
    h->Counters[relaynumber] = value;
  }
}


void IoInputLDEnable(void)
{
  HAL_GPIO_WritePin(DI_LD_GPIO_Port, DI_LD_Pin, GPIO_PIN_SET);
}

void IoInputLDDiasable(void)
{
  HAL_GPIO_WritePin(DI_LD_GPIO_Port, DI_LD_Pin, GPIO_PIN_RESET);
}


void IoTask(OutputTypeDef *context)
{
    uint8_t i,j;
    uint8_t buffer[SPI_IO_ARRAY_SIZE];
    uint8_t input[SPI_IO_ARRAY_SIZE];
    uint8_t output[SPI_IO_ARRAY_SIZE];

    uint8_t *state = context->CurState;

  #if defined(CONFIG_MALT160T)
    j=0;
    for(i=0; i < SPI_IO_ARRAY_SIZE/2; i++)
    {
      buffer[j]= state[i];
      j+=2;
    }
    j=1;
    for(i=SPI_IO_ARRAY_SIZE/2; i < SPI_IO_ARRAY_SIZE; i++)
    {
      buffer[j]= state[i];
      j+=2;
    }
  #else
    memcpy(buffer, state, SPI_IO_ARRAY_SIZE);
  #endif

  #if defined(CONFIG_MALT40IO)
    InputLoad();
  #endif

    memset(input, 0x00, sizeof(input));

    /*Ezt egyszer hardveresen kell megcsinalni*/
    j = SPI_IO_ARRAY_SIZE-1;
    for(i=0; i< SPI_IO_ARRAY_SIZE; i++)
      output[j--] = buffer[i];

    HAL_SPI_TransmitReceive(&hspi2, output, input, SPI_IO_ARRAY_SIZE, 100);



    OutputWrite();

}

inline static void InputLoad(void){
  HAL_GPIO_WritePin(DI_LD_GPIO_Port, DI_LD_Pin, GPIO_PIN_RESET);
  DelayUs(1);
  HAL_GPIO_WritePin(DI_LD_GPIO_Port, DI_LD_Pin, GPIO_PIN_SET);
  DelayUs(1);
}

inline static void OutputWrite(void){
  HAL_GPIO_WritePin(RLY_WR_GPIO_Port, RLY_WR_Pin, GPIO_PIN_SET);
  DelayUs(1);
  HAL_GPIO_WritePin(RLY_WR_GPIO_Port, RLY_WR_Pin, GPIO_PIN_RESET);
}

uint8_t OutputDriverLoopTest(void)
{
  uint8_t testvector[SPI_IO_ARRAY_SIZE *2];
  uint8_t result[SPI_IO_ARRAY_SIZE * 2];
  memset(testvector,0x00, sizeof(testvector));
  memset(testvector,0x55, sizeof(testvector)/2);
  memset(result,0x00,  sizeof(testvector));

#if defined(CONFIG_MALT40IO)
  IoInputLDEnable();
#endif

//do{
/*
 * A szkop
 * 200ns/div
 * 2V/div-es állásban kell hogy legyen
 */
  HAL_SPI_TransmitReceive(&hspi2, testvector, result, SPI_IO_ARRAY_SIZE * 2, 100);
  HAL_Delay(100);
//}while(1);
  if(memcmp(testvector, result + SPI_IO_ARRAY_SIZE, SPI_IO_ARRAY_SIZE) == 0)
    return OUTPUT_OK;
  else
    return OUTPUT_FAIL;
}

