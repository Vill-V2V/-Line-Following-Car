#ifndef HUIDU_H
#define HUIDU_H
#include "ti_msp_dl_config.h"
#include <stdint.h>

// 八路灰度结构体：误差值、踩线数量、原始ADC数组
typedef struct
{
    int16_t error;
    uint8_t line_cnt;
    uint16_t adc_val[8];
} Huidu_Status_t;

Huidu_Status_t Huidu_Read(void);

#endif