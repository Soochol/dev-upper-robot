/**
 * @file    iwdg.h
 * @brief   IWDG peripheral — independent watchdog.
 *
 * MX_IWDG_Init() is called from T_WDG task body (not main.c) so the
 * countdown never starts before the kicker task is running [C1].
 */

#ifndef IWDG_H
#define IWDG_H

#include "stm32f1xx_hal.h"

extern IWDG_HandleTypeDef hiwdg;

void MX_IWDG_Init(void);

#endif /* IWDG_H */
