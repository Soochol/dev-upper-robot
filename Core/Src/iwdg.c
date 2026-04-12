/**
 * @file    iwdg.c
 * @brief   IWDG peripheral init — independent watchdog.
 *
 * LSI ~40 kHz, prescaler 64, reload 250 → timeout ~400 ms.
 * Called from T_WDG task body so the countdown starts only when the
 * kicker is already running [C1].
 */

#include "iwdg.h"
#include "app/config.h"

IWDG_HandleTypeDef hiwdg;

void MX_IWDG_Init(void)
{
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_VAL;
    hiwdg.Init.Reload    = IWDG_RELOAD_VAL;

    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        /* IWDG init failure is unrecoverable — the watchdog peripheral
         * is always present on STM32F103. If this fails, the hardware
         * is in a bad state. Let the system run without HW watchdog;
         * the canary mechanism still logs stalls via RTT. */
    }
}
