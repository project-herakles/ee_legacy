#include "main.h"
/* Originally comm_task.c + communication.c
   mutex operation? 
 */


static unpack_data_t judge_rx_obj;

/*use DMA and FIFO
void unpack_fifo_data(uint8_t tmp,uint8_t sof)
{
  uint8_t byte = 0;
  
  while ( FIFO_S_CountUsed(judge_rx_obj.data_fifo) )
  {
    byte = FIFO_S_Get(judge_rx_obj.data_fifo);
    switch(judge_rx_obj.unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          judge_rx_obj.unpack_step = STEP_LENGTH_LOW;
          judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;
        }
        else
        {
          judge_rx_obj.index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        judge_rx_obj.data_len = byte;
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;
        judge_rx_obj.unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        judge_rx_obj.data_len |= (byte << 8);
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;

        if(judge_rx_obj.data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
        {
          judge_rx_obj.unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          judge_rx_obj.unpack_step = STEP_HEADER_SOF;
          judge_rx_obj.index = 0;
        }
      }break;
    
      case STEP_FRAME_SEQ:
      {
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;
        judge_rx_obj.unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;

        if (judge_rx_obj.index == HEADER_LEN)
        {
          if ( verify_crc8_check_sum(judge_rx_obj.protocol_packet, HEADER_LEN) )
          {
            judge_rx_obj.unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            judge_rx_obj.unpack_step = STEP_HEADER_SOF;
            judge_rx_obj.index = 0;
          }
        }
      }break;  

      case STEP_DATA_CRC16:
      {
        if (judge_rx_obj.index < (HEADER_LEN + CMD_LEN + judge_rx_obj.data_len + CRC_LEN))
        {
           judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;  
        }
        if (judge_rx_obj.index >= (HEADER_LEN + CMD_LEN + judge_rx_obj.data_len + CRC_LEN))
        {
          judge_rx_obj.unpack_step = STEP_HEADER_SOF;
          judge_rx_obj.index = 0;

          if ( verify_crc16_check_sum(judge_rx_obj.protocol_packet, HEADER_LEN + CMD_LEN + judge_rx_obj.data_len + CRC_LEN) )
          {
            if (sof == UP_REG_ID)
            {
              pc_data_handler(judge_rx_obj.protocol_packet);
            }
            else  //DN_REG_ID
            {
              judgement_data_handler(judge_rx_obj.protocol_packet);
            }
          }
        }
      }break;

      default:
      {
        judge_rx_obj.unpack_step = STEP_HEADER_SOF;
        judge_rx_obj.index = 0;
      }break;
    }
  }
}
*/

void judgement_handler(uint8_t byte,uint8_t sof)
{
  
    switch(judge_rx_obj.unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          judge_rx_obj.unpack_step = STEP_LENGTH_LOW;
          judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;
        }
        else
        {
          judge_rx_obj.index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        judge_rx_obj.data_len = byte;
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;
        judge_rx_obj.unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        judge_rx_obj.data_len |= (byte << 8);
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;

        if(judge_rx_obj.data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
        {
          judge_rx_obj.unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          judge_rx_obj.unpack_step = STEP_HEADER_SOF;
          judge_rx_obj.index = 0;
        }
      }break;
    
      case STEP_FRAME_SEQ:
      {
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;
        judge_rx_obj.unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;

        if (judge_rx_obj.index == HEADER_LEN)
        {
          if ( verify_crc8_check_sum(judge_rx_obj.protocol_packet, HEADER_LEN) )
          {
            judge_rx_obj.unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            judge_rx_obj.unpack_step = STEP_HEADER_SOF;
            judge_rx_obj.index = 0;
          }
        }
      }break;  

      case STEP_DATA_CRC16:
      {
        if (judge_rx_obj.index < (HEADER_LEN + CMD_LEN + judge_rx_obj.data_len + CRC_LEN))
        {
           judge_rx_obj.protocol_packet[judge_rx_obj.index++] = byte;  
        }
        if (judge_rx_obj.index >= (HEADER_LEN + CMD_LEN + judge_rx_obj.data_len + CRC_LEN))
        {
          judge_rx_obj.unpack_step = STEP_HEADER_SOF;
          judge_rx_obj.index = 0;

          if ( verify_crc16_check_sum(judge_rx_obj.protocol_packet, HEADER_LEN + CMD_LEN + judge_rx_obj.data_len + CRC_LEN) )
          {
            if (sof == UP_REG_ID)
            {
              //pc_data_handler(judge_rx_obj.protocol_packet);
            }
            else  //DN_REG_ID
            {
              judgement_data_handler(judge_rx_obj.protocol_packet);
            }
          }
        }
      }break;

      default:
      {
        judge_rx_obj.unpack_step = STEP_HEADER_SOF;
        judge_rx_obj.index = 0;
      }break;
    }
  }

//unpack if receive not successful
/*
void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof)
{
  uint8_t tx_buf[PROTOCAL_FRAME_MAX_SIZE];
  
  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
  
  protocol_packet_pack(cmd_id, p_data, len, sof, tx_buf);
  
  // use mutex operation 
  if (sof == UP_REG_ID)
    FIFO_S_Puts(&pc_txdata_fifo, tx_buf, frame_length);
  else if (sof == DN_REG_ID)
    FIFO_S_Puts(&judge_txdata_fifo, tx_buf, frame_length);
  else
    return ;
}
*/
/*
void pc_data_handler(uint8_t protocol_packet[])//to be finished
{
	return;
}

extern FIFO_S_t _USART3_RX_FIFO;

void communicate_para_init(void)
{
	judge_rx_obj.data_fifo = &_USART3_RX_FIFO;
	judge_rx_obj.unpack_step = STEP_HEADER_SOF;
}
*/


