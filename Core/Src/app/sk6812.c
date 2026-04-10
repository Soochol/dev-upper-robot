/**
 * @file    sk6812.c
 * @brief   SK6812MINI RGB LED driver — TIM2_CH2 + DMA1_Channel7.
 *
 * Each LED needs 24 bits (GRB, MSB first). Each bit is encoded as a
 * PWM pulse width in the DMA buffer:
 *
 *   bit 1 → CCR = BIT_1_CCR (high duty, ~0.5 µs high out of 1.25 µs)
 *   bit 0 → CCR = BIT_0_CCR (low duty, ~0.25 µs high out of 1.25 µs)
 *
 * After the data bits, a reset period of ≥50 µs (all-zero CCR values)
 * latches the colors into the LEDs.
 *
 * Timer config (from .ioc via tim.c):
 *   TIM2 prescaler = 0, ARR = 79 → 64 MHz / 80 = 800 kHz
 *   DMA1_Ch7: MEM_TO_PERIPH, HALFWORD, NORMAL, MemInc=ON
 *
 * The HAL_TIM_PWM_PulseFinishedCallback override at the bottom stops
 * the DMA and clears the busy flag. It is declared without __weak so
 * it overrides the default empty implementation in the HAL.
 */

#include "tim.h"
#include "app/sk6812.h"

/* ========================================================================
 * Timing constants derived from TIM2 ARR = 80 (period = 1.25 µs)
 * ======================================================================== */

#define TIM_ARR          80u

/* SK6812 datasheet: T0H=0.3µs(±0.15), T1H=0.6µs(±0.15)
 * At 64MHz, 1 count = 15.625 ns.
 * BIT_0: 20 counts = 0.3125 µs → within T0H spec (0.15~0.45)
 * BIT_1: 40 counts = 0.625 µs  → within T1H spec (0.45~0.75) */
#define BIT_0_CCR        20u
#define BIT_1_CCR        40u

/* Reset period: ≥50 µs of low output. Each zero-CCR pulse = 1.25 µs,
 * so 50 / 1.25 = 40 pulses minimum. Use 48 for margin. */
#define RESET_PERIODS    48u

#define BITS_PER_LED     24u
/* +1 at the front: dummy zero pulse to absorb the STM32 DMA first-pulse
 * glitch. Without it the first CCR write outputs the stale value (0) for
 * one PWM cycle, which SK6812 interprets as a data bit and shifts the
 * entire frame by 1 bit — causing LED 0 and LED 1 to show different
 * colors even when fed the same RGB values. */
#define DMA_LEAD_DUMMY   1u
#define DMA_BUF_LEN      (DMA_LEAD_DUMMY + SK6812_NUM_LEDS * BITS_PER_LED + RESET_PERIODS)
/* = 1 + 2*24 + 48 = 97 half-words */

/* ========================================================================
 * Static state
 * ======================================================================== */

/* DMA buffer — must be static (not on any task stack) because DMA
 * accesses it asynchronously after sk6812_update() returns. */
static uint16_t         g_dma_buf[DMA_BUF_LEN];
static sk6812_color_t   g_colors[SK6812_NUM_LEDS];
static volatile uint8_t g_busy = 0;

/* ========================================================================
 * Internal helpers
 * ======================================================================== */

/* Pack one byte (8 bits, MSB first) into 8 consecutive DMA slots. */
static void pack_byte(uint16_t **pp, uint8_t byte)
{
    for (int bit = 7; bit >= 0; bit--) {
        **pp = (byte & (1u << bit)) ? BIT_1_CCR : BIT_0_CCR;
        (*pp)++;
    }
}

/* Rebuild the entire DMA buffer from g_colors[]. */
static void rebuild_buf(void)
{
    uint16_t *p = g_dma_buf;

    /* Dummy zero pulse — absorbs the first-pulse glitch. SK6812 sees
     * a short low pulse (<0.3 µs high) which is below the bit-0
     * threshold and gets ignored as inter-frame noise. */
    *p++ = 0;

    for (int i = 0; i < SK6812_NUM_LEDS; i++) {
        /* SK6812 wire order: G R B (MSB first per byte). */
        pack_byte(&p, g_colors[i].g);
        pack_byte(&p, g_colors[i].r);
        pack_byte(&p, g_colors[i].b);
    }

    /* Reset period — drive output low for ≥50 µs. */
    for (uint32_t i = 0; i < RESET_PERIODS; i++) {
        *p++ = 0;
    }
}

/* ========================================================================
 * Public API
 * ======================================================================== */

void sk6812_init(void)
{
    for (int i = 0; i < SK6812_NUM_LEDS; i++) {
        g_colors[i] = (sk6812_color_t){0, 0, 0};
    }
    rebuild_buf();
    g_busy = 0;

    /* Send one all-off frame to ensure LEDs start dark. */
    sk6812_update();
}

void sk6812_set(uint8_t index, sk6812_color_t color)
{
    if (index < SK6812_NUM_LEDS) {
        g_colors[index] = color;
    }
}

void sk6812_update(void)
{
    if (g_busy) return;  /* previous frame still in flight — drop */

    rebuild_buf();
    g_busy = 1;

    /* HAL_TIM_PWM_Start_DMA expects uint32_t* but our buffer is uint16_t.
     * This is fine: DMA is configured for HALFWORD transfer, so it reads
     * 16-bit values from the source address regardless of the pointer type.
     * Length is in units of the configured data width (half-words). */
    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2,
                           (uint32_t *)g_dma_buf, DMA_BUF_LEN);
}

bool sk6812_is_busy(void)
{
    return g_busy != 0;
}

/* ========================================================================
 * DMA transfer complete callback
 * ======================================================================== */

/**
 * @brief  Called by HAL when TIM2 DMA transfer finishes (all CCR values
 *         written). We stop the PWM so the output goes idle-low, and
 *         clear the busy flag so the next sk6812_update() can proceed.
 *
 * Overrides the __weak default in stm32f1xx_hal_tim.c. If another
 * timer also uses this callback, add an Instance check (htim->Instance).
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_2);
        g_busy = 0;
    }
}
