/**
 * @brief   电流采样相关定义
 * @note    本文件包含电流采样的相关定义
 *          1. 接口
 *              curr_sample_flag    枚举类型,标志电流采样的状态(采集初始偏移值/采集运行电流)
 */

#ifndef __CURR_SAMPLE_H
#define __CURR_SAMPLE_H

#include "stm32h7xx.h"
typedef enum {
    CURR_SAMPLE_GET_OFFSET, // 采集偏移值
    CURR_SAMPLE_RUNNING     // 采集运行值
} curr_sample_flag;

typedef struct {
    int adc_val_u;                // u相ADC值
    int adc_val_v;                // v相ADC值
    float adc_val_u_offset;         // u相ADC偏移
    float adc_val_v_offset;         // v相ADC偏移
    float curr_u;                 // u相电流
    float curr_v;                 // v相电流
    float curr_w;                 // w相电流
    curr_sample_flag sample_flag; // ADC采样状态
} curr_sample_t;

void adc_2_curr(curr_sample_t *_curr_sample);

#endif