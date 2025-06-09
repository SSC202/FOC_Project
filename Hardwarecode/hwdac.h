#ifndef __HWDAC_H
#define __HWDAC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"
#include "stm32h7xx.h"

// AD5676 引脚定义
#define DAC_SYNC_H   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET)
#define DAC_SYNC_L   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET)
#define DAC_GAIN_H   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET)
#define DAC_GAIN_L   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET)
#define DAC_RESET_H  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET)
#define DAC_RESET_L  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET)
#define DAC_LDAC_H   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET)
#define DAC_LDAC_L   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET)
#define DAC_RESETL_H HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_SET)
#define DAC_RESETL_L HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET)

void AD5676_init(void);

void hw_dac_output(void);

extern uint16_t hwdac_value1; // 外部DAC1通道值
extern uint16_t hwdac_value2; // 外部DAC2通道值
extern uint16_t hwdac_value3; // 外部DAC3通道值
extern uint16_t hwdac_value4; // 外部DAC4通道值

#ifdef __cplusplus
}
#endif
#endif
