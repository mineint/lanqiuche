#ifndef __BSP_REMOTE_H
#define __BSP_REMOTE_H
#include "stdint.h"


typedef struct
{
    uint8_t rdata[28];   
    int16_t ch[11];      
    uint8_t ch_flag[10]; 
    uint8_t ch_last[10]; 
} ELRS;


extern ELRS Elrs;

int8_t Channel_optimization(int16_t value);
void ELRSDataProcess(uint8_t *rdata);

#endif