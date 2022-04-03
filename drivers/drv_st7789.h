/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-03     王浩       the first version
 */
#ifndef DRIVERS_DRV_ST7789_H_
#define DRIVERS_DRV_ST7789_H_

#include "stdlib.h"
#include "drv_common.h"
#include "uart_config.h"


void st7789_hard_init(void);

void st7789_s_command(uint8_t  command);
void st7789_s_data(uint16_t  data);

void st7789_r_data(uint8_t*  data);

void st7789_read_id1(uint8_t*  ID);



#endif /* DRIVERS_DRV_ST7789_H_ */
