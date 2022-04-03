/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-03     RT-Thread    first version
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include "stdlib.h"
#include "drv_common.h"
#include "drv_st7789.h"
#include "board.h"
#include "stm32f1xx_hal_conf.h"

int main(void)

{

     rt_thread_delay(5000);
     st7789_hard_init();

      if(st7789_connect_check())
      {
          rt_kprintf("connect_ok!\n");
      }
      else {
          rt_kprintf("st7789_check_connect_fail!\n");
     }
      st7789_command_init();
      LCD_Address_Set(0, 0, LCD_H, LCD_W);
      while(1){
          st7789_s_16data(0xfff);
          //HAL_Delay(1);
      }



    return RT_EOK;
}



