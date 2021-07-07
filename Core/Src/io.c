/*
 * relays.c
 *
 *  Created on: 2020. márc. 5.
 *      Author: Margit Robert
 */

/* Includes ------------------------------------------------------------------*/
#include <io.h>

/* Private function prototypes -----------------------------------------------*/
static void ArrayToolsU8SetBit(const uint16_t index, void* array);
static void CounterUpdate(uint8_t *pre, uint8_t *cur, uint32_t *relaycounter);


/* ---------------------------------------------------------------------------*/

void IoInit(IoTypeDef *context)
{

}

/*
 * k: 0..DEVICE_OUTPUT_COUNT-1
 */
void OutpuOneOn(IoTypeDef *h, uint8_t k)
{
  memcpy(h->Output.PreState, h->Output.CurState, IO_OUTPUT_ARRAY_SIZE);
  uint8_t temp[IO_OUTPUT_ARRAY_SIZE];
  memset(temp, 0x00, IO_OUTPUT_ARRAY_SIZE);
  ArrayToolsU8SetBit(k, temp);
  for(uint8_t i=0; i < IO_OUTPUT_ARRAY_SIZE; i++ )
    h->Output.CurState[i]|=temp[i];
  CounterUpdate(h->Output.PreState, h->Output.CurState, h->Output.Counters);
}

/*
 * k: 0..DEVICE_OUTPUT_COUNT-1
 */
void OutputOneOff(IoTypeDef *h, uint8_t k)
{
  memcpy(h->Output.PreState, h->Output.CurState, IO_OUTPUT_ARRAY_SIZE);
  uint8_t temp[IO_OUTPUT_ARRAY_SIZE];
  memset(temp, 0x00, IO_OUTPUT_ARRAY_SIZE);
  ArrayToolsU8SetBit(k, temp);
  for(uint8_t i=0; i < IO_OUTPUT_ARRAY_SIZE; i++)
    temp[i] = ~temp[i];
  for(uint8_t i=0; i < IO_OUTPUT_ARRAY_SIZE; i++ )
    h->Output.CurState[i] &=temp[i];
  CounterUpdate(h->Output.PreState, h->Output.CurState, h->Output.Counters);
}

void OutputSeveralToogle(IoTypeDef *h, uint8_t *several, uint8_t block)
{
  memcpy(h->Output.PreState, h->Output.CurState, IO_OUTPUT_ARRAY_SIZE);
  for(uint8_t i = 0; i < DEVICE_BLOCK_SIZE; i++)
  {
    if(several[i])
    { /*csak tooglezunk amelyik bájtvan volt 1-es*/
      if(several[i] & h->Output.CurState[block * DEVICE_BLOCK_SIZE + i])
        h->Output.CurState[block * DEVICE_BLOCK_SIZE + i] &= ~several[i];
      else
        h->Output.CurState[block * DEVICE_BLOCK_SIZE + i] |= several[i];
    }
  }
  CounterUpdate(h->Output.PreState, h->Output.CurState, h->Output.Counters);
}

/*
 * Egy blokkban 4-bájt van.
 * Ez egyszerre csak 4-bájtot tud modositani
 * A 160 relénél 5 blokk van (5x4bájt =20bájt * 8 bit =160).
 */
void OutputOffSeveral(IoTypeDef *h, uint8_t *several, uint8_t block)
{
  memcpy(h->Output.PreState, h->Output.CurState, IO_SPI_IO_ARRAY_SIZE);
  for(uint8_t i=0; i < DEVICE_BLOCK_SIZE; i++)
    h->Output.CurState[block * DEVICE_BLOCK_SIZE + i] &= ~several[i];
  CounterUpdate(h->Output.PreState, h->Output.CurState, h->Output.Counters);
}

void OutputOnSeveral(IoTypeDef *h, uint8_t *several, uint8_t block)
{
  memcpy(h->Output.PreState, h->Output.CurState, IO_SPI_IO_ARRAY_SIZE);
  for(uint8_t i=0; i < DEVICE_BLOCK_SIZE; i++)
    h->Output.CurState[block * DEVICE_BLOCK_SIZE + i] |= several[i];
  CounterUpdate(h->Output.PreState, h->Output.CurState, h->Output.Counters);
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


void OutputReset(IoTypeDef *h)
{
  memset(h->Output.PreState, 0x00, IO_OUTPUT_ARRAY_SIZE);
  memset(h->Output.CurState, 0x00, IO_OUTPUT_ARRAY_SIZE);

  IoTask(h);
}

void OutputChangedBlocksUpdate(IoTypeDef *h)
{
  uint8_t dif[IO_OUTPUT_ARRAY_SIZE];
  for(uint8_t i = 0; i < IO_OUTPUT_ARRAY_SIZE; i++)
  {
    dif[i] = h->Output.PreState[i] ^ h->Output.CurState[i];
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
      h->Output.ChangedBlocks[i/DEVICE_BLOCK_SIZE] = 1;
      /*Tudomasul vettük a változást*/
      h->Output.PreState[i] = h->Output.CurState[i];
    }
  }
}

void IoChangedBlocksUpdate(IoTypeDef *h)
{
  uint8_t dif[IO_INPUT_ARRAY_SIZE];
  for(uint8_t i = 0; i < IO_INPUT_ARRAY_SIZE; i++)
  {
    dif[i] = h->Input.PreState[i] ^ h->Input.CurState[i];
    if(dif[i])
    {

      /*Az i/4-blockban volt változás mivel egy blokban 4bajt van*/
      h->Input.ChangedBlocks[i/DEVICE_BLOCK_SIZE] = 1;
      /*Tudomasul vettük a változást*/
      h->Input.PreState[i] = h->Input.CurState[i];
    }
  }
}

static void CounterUpdate(uint8_t *pre, uint8_t *cur, uint32_t *relaycounter)
{
  uint8_t dif[IO_OUTPUT_ARRAY_SIZE];
  uint8_t relay_index = 0;
  for(uint8_t i = 0; i < IO_OUTPUT_ARRAY_SIZE; i++)
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
uint32_t OutputCounterGet(IoTypeDef *h, uint8_t relaynumber)
{
  if(relaynumber > DEVICE_OUTPUTS_COUNT)
    return 0;
  else
    return h->Output.Counters[relaynumber];
}

void OutputCounterSet(IoTypeDef *h, uint8_t relaynumber, uint32_t value)
{
  if(relaynumber < DEVICE_OUTPUTS_COUNT)
  {
    h->Output.Counters[relaynumber] = value;
  }
}

#if defined(CONFIG_MALT40IO)
void IoInputLDEnable(void)
{
  HAL_GPIO_WritePin(DI_LD_GPIO_Port, DI_LD_Pin, GPIO_PIN_SET);
}

void IoInputLDDiasable(void)
{
  HAL_GPIO_WritePin(DI_LD_GPIO_Port, DI_LD_Pin, GPIO_PIN_RESET);
}
#endif


void IoTask(IoTypeDef *context)
{
  uint8_t output_buffer[IO_SPI_IO_ARRAY_SIZE];
  uint8_t input_buffer[IO_SPI_IO_ARRAY_SIZE];
  memset(output_buffer, 0x00, IO_SPI_IO_ARRAY_SIZE);
  memset(input_buffer, 0x00, IO_SPI_IO_ARRAY_SIZE);

  #if defined(CONFIG_MALT160T)
    uint8_t buffer[IO_OUTPUT_ARRAY_SIZE];
    uint8_t i,j;
    j=0;
    for(i=0; i < IO_OUTPUT_ARRAY_SIZE/2; i++)
    {
      buffer[j]= state[i];
      j+=2;
    }
    j=1;
    for(i=IO_OUTPUT_ARRAY_SIZE/2; i < IO_OUTPUT_ARRAY_SIZE; i++)
    {
      buffer[j]= state[i];
      j+=2;
    }
  #elif defined(CONFIG_MALT40IO)
    //input load
    HAL_GPIO_WritePin(DI_LD_GPIO_Port, DI_LD_Pin, GPIO_PIN_RESET);
    DelayUs(1);
    HAL_GPIO_WritePin(DI_LD_GPIO_Port, DI_LD_Pin, GPIO_PIN_SET);
    DelayUs(1);

    for(uint8_t j=0, i = IO_OUTPUT_ARRAY_SIZE; i; i--){
      output_buffer[i-1] = context->Output.CurState[j++];
    }
    HAL_SPI_TransmitReceive(&hspi2, output_buffer, input_buffer, IO_SPI_IO_ARRAY_SIZE, 100);

    //outputs write
    HAL_GPIO_WritePin(RLY_WR_GPIO_Port, RLY_WR_Pin, GPIO_PIN_SET);
    DelayUs(1);
    HAL_GPIO_WritePin(RLY_WR_GPIO_Port, RLY_WR_Pin, GPIO_PIN_RESET);

    for(uint8_t j=0, i = IO_SPI_IO_ARRAY_SIZE; j < IO_OUTPUT_ARRAY_SIZE ; i--, j++){
      context->Input.CurState[j] = input_buffer[i-1];
    }

  #elif defined(CONFIG_MALT132) || defined(CONFIG_MALT23THV) || defined(CONFIG_MALT16PIN)

    for(uint8_t j=0, i = IO_OUTPUT_ARRAY_SIZE; i; i--){
      output_buffer[i-1] = context->Output.CurState[j++];
    }

    HAL_SPI_TransmitReceive(&hspi2, output_buffer, input_buffer, IO_SPI_IO_ARRAY_SIZE, 100);

    //outputs write
    HAL_GPIO_WritePin(RLY_WR_GPIO_Port, RLY_WR_Pin, GPIO_PIN_SET);
    DelayUs(1);
    HAL_GPIO_WritePin(RLY_WR_GPIO_Port, RLY_WR_Pin, GPIO_PIN_RESET);

  #elif defined(CONFIG_MALT24VI)

    /*
     *
     *  bytes           [0]                  [1]                      [2]
     *           MSB          LSB  MSB                 LSB  MSB                  LSB
     *           8,7,6,5,4,3,2,1
     *  bits    |1,2,3,4,5,6,7,8 | 9,10,11,12,13,14,15,16 | 17,18,19,20,21,22,23,24 |
     *  Y1:set  |              1                                                    | 0x000001
     *  Y24:set |                                            1                      | 0x800000
     *  korrekció után
     *  Y1:set  |  1                                                                | = 0x40
    */
    uint32_t y = *((uint32_t*)context->Output.CurState);
    uint32_t o = 0;

    y & 0x000001 ? (o |= 0x000040): (o &= ~0x000040);    //Y1 -> 2.bit
    y & 0x000002 ? (o |= 0x000080): (o &= ~0x000080);    //Y2 -> 1.bit
    y & 0x000004 ? (o |= 0x000010): (o &= ~0x000010);    //Y3 -> 4.bit
    y & 0x000008 ? (o |= 0x000020): (o &= ~0x000020);    //Y4 -> 3.bit
    y & 0x000010 ? (o |= 0x000002): (o &= ~0x000002);    //Y5 -> 2.bit
    y & 0x000020 ? (o |= 0x000001): (o &= ~0x000001);    //Y6 -> 8.bit
    y & 0x000040 ? (o |= 0x000008): (o &= ~0x000008);    //Y7 -> 5.bit
    y & 0x000080 ? (o |= 0x000004): (o &= ~0x000004);    //Y8 -> 6.bit

    y & 0x000100 ? (o |= 0x004000): (o &= ~0x004000);    //Y9  -> 10.bit
    y & 0x000200 ? (o |= 0x008000): (o &= ~0x008000);    //Y10 -> 9.bit
    y & 0x000400 ? (o |= 0x001000): (o &= ~0x001000);    //Y11 -> .bit
    y & 0x000800 ? (o |= 0x002000): (o &= ~0x002000);    //Y12 -> .bit
    y & 0x001000 ? (o |= 0x000200): (o &= ~0x000200);    //Y13 -> .bit
    y & 0x002000 ? (o |= 0x000100): (o &= ~0x000100);    //Y14 -> .bit
    y & 0x004000 ? (o |= 0x000800): (o &= ~0x000800);    //Y15 -> .bit
    y & 0x008000 ? (o |= 0x000400): (o &= ~0x000400);    //Y16 -> .bit

    y & 0x010000 ? (o |= 0x400000): (o &= ~0x400000);    //Y17  -> .bit
    y & 0x020000 ? (o |= 0x800000): (o &= ~0x800000);    //Y18 -> .bit
    y & 0x040000 ? (o |= 0x100000): (o &= ~0x100000);    //Y19 -> .bit
    y & 0x080000 ? (o |= 0x200000): (o &= ~0x200000);    //Y20 -> .bit
    y & 0x100000 ? (o |= 0x020000): (o &= ~0x020000);    //Y21 -> .bit
    y & 0x200000 ? (o |= 0x010000): (o &= ~0x010000);    //Y22 -> .bit
    y & 0x400000 ? (o |= 0x080000): (o &= ~0x080000);    //Y23 -> .bit
    y & 0x800000 ? (o |= 0x040000): (o &= ~0x040000);    //Y24 -> .bit

    output_buffer[2] |= o & 0xFF;
    output_buffer[1] |= (o >>8) & 0xFF;
    output_buffer[0] |= (o >>16) & 0xFF;

    HAL_SPI_TransmitReceive(&hspi2, output_buffer, input_buffer, IO_SPI_IO_ARRAY_SIZE, 100);

    //outputs write
    HAL_GPIO_WritePin(RLY_WR_GPIO_Port, RLY_WR_Pin, GPIO_PIN_SET);
    DelayUs(1);
    HAL_GPIO_WritePin(RLY_WR_GPIO_Port, RLY_WR_Pin, GPIO_PIN_RESET);

  #else
    #error "Imserelten konfiguracio"
  #endif

}

uint8_t OutputDriverLoopTest(void)
{
  uint8_t testvector[IO_SPI_IO_ARRAY_SIZE *2];
  uint8_t result[IO_SPI_IO_ARRAY_SIZE * 2];
  memset(testvector,0x00, sizeof(testvector));
  memset(testvector,0x55, sizeof(testvector)/2);
  memset(result,0x00,  sizeof(testvector));

//do{
/*
 * A szkop
 * 200ns/div
 * 2V/div-es állásban kell hogy legyen
 */
  HAL_SPI_TransmitReceive(&hspi2, testvector, result, IO_SPI_IO_ARRAY_SIZE * 2, 100);
  HAL_Delay(100);
//}while(1);
  if(memcmp(testvector, result + IO_SPI_IO_ARRAY_SIZE, IO_SPI_IO_ARRAY_SIZE) == 0)
    return IO_OK;
  else
    return IO_FAIL;
}

