/**
 * @brief   电流采样相关定义
 * @attention   1. ADC 相关
 *                 H7 ADC 是 16 位的(0-65535映射),经过电压检查验证基本上是严格的线性关系。
 *                 ADC * 3.3 / 65535 = Voltage
 *              2. M 新动力电流采样相关
 *                 该逆变器使用采集相线电流的方式进行电流采集;
 *                 上电时 ADC 平均值进行采集得到 0A 的传感器输出电压值;
 *                 然后使用 9.6 A/V 的线性关系计算相线电流。
 * @note        接口类型        curr_sample_flag    枚举类型,标志电流采样的状态(采集初始偏移值/采集运行电流)
 *              接口函数    adc_2_curr()    将 ADC 值转换为实际电流值
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