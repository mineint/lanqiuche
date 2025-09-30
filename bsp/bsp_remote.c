#include "bsp_remote.h"
#include "main.h"
#include "usart.h"



int8_t Channel_optimization(int16_t value)
{
    int8_t a;
    switch (value)
    {
    case 191:
        a = 1;
        break;
    case 1792:
        a = 3;
        break;

    case 172:
        a = 1;
        break;
    case 992:
        a = 2;
        break;
    case 997:
        a = 2;
        break;
    case 1810:
        a = 3;
        break;
    }
    return a;
}
ELRS Elrs;
void ELRSDataProcess(uint8_t *rdata)
{
   
    Elrs.ch[1] = ((rdata[3] >> 0) | (rdata[4] << 8)) & 0x07FF;
    Elrs.ch[2] = ((rdata[4] >> 3) | (rdata[5] << 5)) & 0x07FF;
    Elrs.ch[3] = ((rdata[5] >> 6) | (rdata[6] << 2) | (rdata[7] << 10)) & 0x07FF;
    Elrs.ch[4] = ((rdata[7] >> 1) | (rdata[8] << 7)) & 0x07FF;
    Elrs.ch[5] = ((rdata[8] >> 4) | (rdata[9] << 4)) & 0x07FF;
    Elrs.ch[6] = ((rdata[9] >> 7) | (rdata[10] << 1) | (rdata[11] << 9)) & 0x07FF;
    Elrs.ch[7] = ((rdata[11] >> 2) | (rdata[12] << 6)) & 0x07FF;
    Elrs.ch[8] = ((rdata[12] >> 5) | (rdata[13] << 3)) & 0x07FF;
    Elrs.ch[9] = ((rdata[14] >> 0) | (rdata[15] << 8)) & 0x07FF;
    Elrs.ch[10] = ((rdata[15] >> 3) | (rdata[16] << 5)) & 0x07FF;

    Elrs.ch[1] -= 992;
    Elrs.ch[2] -= 992;
    Elrs.ch[3] -= 992;
    Elrs.ch[4] -= 992;
    Elrs.ch[10] -= 992;

    Elrs.ch[5] = Channel_optimization(Elrs.ch[5]);
    Elrs.ch[6] = Channel_optimization(Elrs.ch[6]);
    Elrs.ch[7] = Channel_optimization(Elrs.ch[7]);
    Elrs.ch[8] = Channel_optimization(Elrs.ch[8]);
    Elrs.ch[9] = Channel_optimization(Elrs.ch[9]);

    for (uint8_t i = 5; i < 10; i++)
    {
        if (Elrs.ch_last[i] != Elrs.ch[i])
            Elrs.ch_flag[i] = Elrs.ch[i];
        Elrs.ch_last[i] = Elrs.ch[i];
    }
    if (Elrs.ch[1] > -100 && Elrs.ch[1] < 100)
    {
        Elrs.ch[1] = 0;
    }
    if (Elrs.ch[2] > -100 && Elrs.ch[2] < 100)
    {
        Elrs.ch[2] = 0;
    }
    if (Elrs.ch[3] > -100 && Elrs.ch[3] < 100)
    {
        Elrs.ch[3] = 0;
    }
    if (Elrs.ch[4] > -100 && Elrs.ch[4] < 100)
    {
        Elrs.ch[4] = 0;
    }
}


extern DMA_HandleTypeDef hdma_usart1_rx;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        if (Size == 26) 
            ELRSDataProcess(Elrs.rdata);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Elrs.rdata, 27);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}

