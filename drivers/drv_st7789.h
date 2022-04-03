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



#define USE_HORIZONTAL 2  //


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 280

#else
#define LCD_W 280
#define LCD_H 240
#endif


void st7789_hard_init(void);
void st7789_command_init(void);

void st7789_s_command(uint8_t  command);
void st7789_s_16data(uint16_t  data);
void st7789_s_8data(uint8_t  data);

void st7789_r_data(uint8_t*  data);

void st7789_read_id1(uint8_t*  ID);
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
int st7789_connect_check(void);

void LCD_Clear(uint16_t color);





#endif /* DRIVERS_DRV_ST7789_H_ */
