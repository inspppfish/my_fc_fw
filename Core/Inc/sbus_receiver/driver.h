//
// Created by insppp on 2022/8/7.
//

#ifndef MY_FC_FW_DRIVER_H
#define MY_FC_FW_DRIVER_H

#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#endif //MY_FC_FW_DRIVER_H
