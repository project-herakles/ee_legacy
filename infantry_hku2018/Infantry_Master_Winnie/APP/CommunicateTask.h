#ifndef _COMMUNICATE_TASK__
#define _COMMUNICATE_TASK__

#include "fifo.h"
//used to receive frame header(5 bits)
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  //UART_HandleTypeDef *huart;//?
  FIFO_S_t           *data_fifo;
  uint16_t           buff_size;
  uint8_t            *buff[2];
  uint16_t           read_index;
  uint16_t           write_index;
} uart_dma_rxdata_t;

typedef struct
{
  frame_header_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

void judgement_handler(uint8_t tmp,uint8_t sof);
//void communicate_para_init(void);
//void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof);
//void pc_data_handler(uint8_t protocol_packet[]);//not defined yet
#endif

/*all FIFO_S_t should be fifo_s_t
//! FIFO Memory Model (Single Byte Mode)
typedef struct
{
    uint8_t* pStartAddr;                   //!< FIFO Memory Pool Start Address
    uint8_t* pEndAddr;                     //!< FIFO Memory Pool End Address
    uint32_t Free;                         //!< The capacity of FIFO
    uint32_t Used;                         //!< The number of elements in FIFO
    uint8_t* pReadIndex;                   //!< FIFO Data Read Index Pointer
    uint8_t* pWriteIndex;                  //!< FIFO Data Write Index Pointer
}FIFO_S_t;

->
typedef struct
{
  uint8_t   *start_addr;                   //Start Address
  uint8_t   *end_addr;                     //End Address
  uint32_t  free;                         //The capacity of FIFO
  uint32_t  buf_size;                     //Buffer size
  uint32_t  used;                         //The number of elements in FIFO
  uint8_t   read_index;                   //Read Index Pointer
  uint8_t   write_index;                  //Write Index Pointer
  osMutexId mutex;
} fifo_s_t;
where osMutexId is a void pointer
*/

