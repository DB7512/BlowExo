#include "RLS_encoder.h"
#include "usart.h"
#include "stdio.h"
#include "main.h"

uint8_t encoder_send_buf[ENCODER_SEND_LEN] = {0xFF, 0x03, 0x00, 0x02, 0x00, 0x04, 0xF0, 0x17};                   // ����������������

uint8_t encoder_recv_buf[ENCODER_RECV_LEN] = {0};

/**
 * @brief       ��������������
 * @param       pData������������
 * @note        
 *              
 * @retval      -1�����������ڴ���򾯸棻0������������1�����ݲ�����
 */
int Extract_Encoder_Data(uint8_t *pData, int *eData)
{
  // ��Ҫ����CRCУ��ȷ���������ݵ�����
  
  /* ������λ�;���λ */
  uint8_t error = 0;
  uint8_t warning = 0;
  uint8_t error_bit = (uint8_t)pData[10];
  error |= error_bit >> 7;
  warning |= (error_bit >> 6) & 0x01;
  /* ����λ�� */
  uint32_t position = 0;
  uint32_t multiturn = 0;
  if (error != 0 && warning != 0)
  {
    /* 1-17bits����λ�ã���17bits�� */
    position |= (uint8_t)pData[4];
    position |= ((uint8_t)pData[3]) << 8;
    position |= ((uint8_t)pData[6]) << 8 * 2;
    position &= 0x1FFF;
    /* 18-33bits����Ȧ������16bits�� */
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
 * @brief       �������Ͷ�ȡλ��ָ����������عؽ�λ��
 * @param       rData�ӻ�����λ��
 * @note        
 *              
 * @retval      
 */
void Encoder_Send_Recv(uint8_t *rData)
{
  uint8_t encoder_send_buf[ENCODER_SEND_LEN] = {0xFF, 0x03, 0x00, 0x02, 0x00, 0x04, 0xF0, 0x17};        // FF 03 00 02 00 04 F0 17
  
  HAL_UART_Transmit(&huart3, encoder_send_buf, ENCODER_SEND_LEN, 10);
  
  HAL_UART_Receive_IT(&huart3, (uint8_t *)rData, ENCODER_RECV_LEN);
  
}
