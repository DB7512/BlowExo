#ifndef __RLS_ENCODER_H
#define __RLS_ENCODER_H

#include <stdint.h>
#include "main.h" // stm32 hal

#define ENCODER_SEND_LEN 8
#define ENCODER_RECV_LEN 13

extern uint8_t g_encoder_recv_buf[ENCODER_RECV_LEN];

extern int Extract_Encoder_Data(uint8_t *pData, int *eData);

HAL_StatusTypeDef Encoder_Send_recv(int *eData);

#endif
