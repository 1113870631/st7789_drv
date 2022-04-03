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
     uint8_t tmp=0;
     st7789_hard_init();
     st7789_read_id1(&tmp);
    return RT_EOK;
}
