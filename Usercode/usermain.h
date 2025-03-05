#ifndef __USERMAIN_H
#define __USERMAIN_H

#include "main.h"
#include "stm32h7xx.h"
#include "tim.h"
#include "spi.h"
#include "adc.h"
#include "usart.h"

#include "stdio.h"

// FOC 相关文件
#include "coordinate_transform.h"
#include "svpwm.h"
#include "curr_sample.h"

// 硬件配置相关文件
#include "ad2s1210.h"

extern uint8_t system_enable;

void usermain(void);

#endif
