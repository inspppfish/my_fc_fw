/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң��������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж�������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       ��������ͨ�������ж�����������freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "sbus_receiver/remote_control.h"

#include "main.h"
#include "stdio.h"


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//ң�������Ʊ���
RC_ctrl_t rc_ctrl;

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ң������ʼ��
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          ��ȡң��������ָ��
  * @param[in]      none
  * @retval         ң��������ָ��
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}


//�����ж�
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[1] | (sbus_buf[2] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[2] >> 3) | (sbus_buf[3] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[3] >> 6) | (sbus_buf[4] << 2) | (sbus_buf[5] << 10)) &0x07ff;//!< Channel 2
    rc_ctrl->rc.ch[3] = ((sbus_buf[5] >> 1) | (sbus_buf[6] << 7)) & 0x07ff; //!< Channel 3
	//todo: configure other channels && check

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
