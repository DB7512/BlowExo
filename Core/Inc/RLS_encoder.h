#ifndef __RLS_ENCODER_H
#define __RLS_ENCODER_H

#include <stdint.h>
#include "main.h" // stm32 hal

#define ENCODER_SEND_LEN 8
#define ENCODER_RECV_LEN 13

//extern uint8_t encoder_send_buf[ENCODER_SEND_LEN];

extern uint8_t encoder_recv_buf[ENCODER_RECV_LEN];

extern int Extract_Encoder_Data(uint8_t *pData, int *eData);

extern void Encoder_Send_Recv(uint8_t *rData);

#endif
