#include "RLS_encoder.h"
#include "usart.h"
#include "stdio.h"
#include "main.h"

uint8_t g_encoder_recv_buf[ENCODER_RECV_LEN] = {0};                   // 编码器向主机发送


/**
 * @brief       解析编码器数据
 * @param       pData：编码器数据
 * @note        
 *              
 * @retval      -1：编码器存在错误或警告；0：数据完整；1：数据不完整
 */
int Extract_Encoder_Data(uint8_t *pData, int *eData)
{
  // 需要补充CRC校验确保接收数据的完整
  
  /* 检测错误位和警告位 */
  uint8_t error = 0;
  uint8_t warning = 0;
  uint8_t error_bit = (uint8_t)pData[10];
  error |= error_bit >> 7;
  warning |= (error_bit >> 6) & 0x01;
  /* 解析位置 */
  uint32_t position = 0;
  uint32_t multiturn = 0;
  if (error != 0 && warning != 0)
  {
    /* 1-17bits保存位置（共17bits） */
    position |= (uint8_t)pData[4];
    position |= ((uint8_t)pData[3]) << 8;
    position |= ((uint8_t)pData[6]) << 8 * 2;
    position &= 0x1FFF;
    /* 18-33bits保存圈数（共16bits） */
    multiturn |= (uint8_t)pData[6];
    multiturn |= ((uint8_t)pData[5]) << 8;
    multiturn |= ((uint8_t)pData[8]) << 8 * 2;
    multiturn >>= 1;
    multiturn &= 0xFFFF;
    *eData = multiturn * 65536 + position;
    return 0;
  }
  else
  {
      return -1;
  }
}

/**
 * @brief       主机发送读取位置指令，编码器返回关节位置
 * @param       pData读取位置指令，rData从机返回位置
 * @note        
 *              
 * @retval      
 */
HAL_StatusTypeDef Encoder_Send_recv(int *eData)
{
//  uint16_t rxlen = 0;
  uint8_t encoder_send_buf[ENCODER_SEND_LEN] = {0xFF, 0x03, 0x00, 0x02, 0x00, 0x04, 0xF0, 0x17};
  
  HAL_UART_Transmit(&huart3, encoder_send_buf, ENCODER_SEND_LEN, 10);
  
  if(HAL_UART_Receive_IT(&huart3, (uint8_t *)g_encoder_recv_buf, ENCODER_RECV_LEN) == HAL_OK)
  {
    return HAL_OK;
  }
  return HAL_ERROR;
  
//  if(rxlen == 0)
//  {
//    return HAL_TIMEOUT;
//  }
//  if(rxlen != ENCODER_RECV_LEN)
//  {
//    return HAL_ERROR;
//  }
//  
//  uint8_t *rp = (uint8_t *)&g_encoder_recv_buf;
//  if(rp[0] == 0xFF)
//  {
//    Extract_Encoder_Data(g_encoder_recv_buf, eData);
//    return HAL_OK;
//  }
//  return HAL_ERROR;
}
