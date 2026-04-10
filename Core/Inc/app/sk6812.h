/**
 * @file    sk6812.h
 * @brief   SK6812MINI RGB LED driver via TIM2_CH2 + DMA.
 *
 * 2 LEDs, 3-channel GRB, 800 kHz single-wire protocol.
 * Each bit is a PWM pulse: bit 1 = high duty (~0.6µs), bit 0 = low duty
 * (~0.3µs). DMA streams the entire frame (48 bits + reset) in ~110 µs
 * with zero CPU involvement after the start call.
 *
 * Hardware: TIM2 CH2 (PB3), Prescaler=0, ARR=79 → 64MHz/80 = 800kHz.
 * DMA1_Channel7, half-word, memory-to-peripheral, normal mode.
 *
 * Usage:
 *   sk6812_init();                           // once at startup
 *   sk6812_set(0, (sk6812_color_t){0,255,0}); // LED 0 = green
 *   sk6812_set(1, (sk6812_color_t){255,0,0}); // LED 1 = red
 *   sk6812_update();                         // fire DMA, non-blocking
 */

#ifndef APP_SK6812_H
#define APP_SK6812_H

#include <stdint.h>
#include <stdbool.h>

#define SK6812_NUM_LEDS  2

/* Color in GRB order (SK6812 wire format). Constructing as {r, g, b}
 * in user code is fine — the driver reorders to GRB internally. */
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} sk6812_color_t;

/**
 * @brief  Initialize the SK6812 driver. Clears all LEDs to off.
 *         Call once from T_PID startup, after actuators_init.
 */
void sk6812_init(void);

/**
 * @brief  Set the color of one LED in the local buffer.
 *         Does NOT transmit — call sk6812_update() to push changes.
 * @param  index  0 or 1
 * @param  color  RGB color (driver converts to GRB wire order)
 */
void sk6812_set(uint8_t index, sk6812_color_t color);

/**
 * @brief  Transmit the frame buffer via DMA. Non-blocking — returns
 *         immediately and DMA completes in ~110 µs.
 *         If a previous DMA transfer is still in progress, this call
 *         is silently skipped (frame dropped).
 */
void sk6812_update(void);

/**
 * @brief  Returns true while DMA is actively transmitting.
 */
bool sk6812_is_busy(void);

#endif /* APP_SK6812_H */
