#ifndef __USERMAIN_H
#define __USERMAIN_H

#include "main.h"
#include "stm32h7xx.h"
#include "tim.h"
#include "spi.h"
#include "adc.h"
#include "dac.h"
#include "usart.h"

#include "stdio.h"

// FOC 相关文件
#include "coordinate_transform.h"
#include "svpwm.h"
#include "curr_sample.h"

// 编码器相关文件
#include "ad2s1210.h"

// 用户相关文件
#include "userpara.h"
#include "userinit.h"
#include "drivefoc.h"

void usermain(void);

#endif
