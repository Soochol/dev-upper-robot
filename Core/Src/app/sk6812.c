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
/* Leading reset: 48 all-zero CCR pulses = 60 µs of continuous LOW before
 * any data bits. This forces the SK6812 into its reset state regardless
 * of how the previous DMA transfer ended or where TIM2's counter happened
 * to freeze on Stop_DMA. A single dummy zero was not enough because the
 * counter can resume mid-cycle on Start_DMA, producing a truncated first
 * pulse that the SK6812 interprets as data. A full 60 µs reset makes
 * the initial state deterministic. */
#define LEAD_RESET       RESET_PERIODS
#define DMA_BUF_LEN      (LEAD_RESET + SK6812_NUM_LEDS * BITS_PER_LED + RESET_PERIODS)
/* = 48 + 2*24 + 48 = 144 half-words = 288 bytes */

/* ========================================================================
 * Static state
 * ======================================================================== */

/* DMA buffer — must be static (not on any task stack) because DMA
 * accesses it asynchronously after sk6812_update() returns. */
static uint16_t         g_dma_buf[DMA_BUF_LEN];
static sk6812_color_t   g_colors[SK6812_NUM_LEDS];
static volatile uint8_t g_busy = 0;
static volatile uint8_t g_dirty = 1;  /* 1: init 시 첫 프레임 전송 필요 */

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

    /* Leading reset period — 48 × 1.25 µs = 60 µs of continuous LOW.
     * SK6812 requires ≥50 µs low to enter reset. After this, it is
     * guaranteed to be waiting for the first data bit. */
    for (uint32_t i = 0; i < LEAD_RESET; i++) {
        *p++ = 0;
    }

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
        if (g_colors[index].r != color.r ||
            g_colors[index].g != color.g ||
            g_colors[index].b != color.b) {
            g_colors[index] = color;
            g_dirty = 1;
        }
    }
}

void sk6812_update(void)
{
    if (g_busy || !g_dirty) return;

    g_dirty = 0;
    rebuild_buf();
    g_busy = 1;

    /* Reset TIM2 counter and CCR to prevent a truncated first pulse.
     * HAL_TIM_PWM_Stop_DMA freezes the counter at an arbitrary value;
     * without this reset the first PWM cycle after Start_DMA would
     * resume mid-period, producing a partial pulse that the SK6812
     * interprets as a data bit. */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

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
