/**
 * @file    trigger_ml.c
 * @brief   ML-based trigger provider — Random Forest inference via m2cgen.
 *
 * Calls the auto-generated score() function from model_rf.c (produced by
 * train.py + m2cgen). The score function takes 9 features and outputs
 * class probabilities [P(FORCE_DOWN), P(FORCE_UP)].
 *
 * Directional filtering: only fires transitions that make sense from
 * the current FSM state (same semantics as trigger_rule.c).
 */

#include "app/trigger.h"
#include "app/features.h"

/* m2cgen-generated model — lives in Core/Src/app/model_rf.c.
 * Replaced each training iteration by train.py --output. */
extern void score(double *input, double *output);

/* Probability threshold. Higher = fewer false positives but slower
 * response. 0.6 is a conservative starting point. */
#define ML_PROB_THRESHOLD  0.6

static uint8_t ml_eval(const sensor_snapshot_t *snap, fsm_state_t cur)
{
    if (!snap->ml_feat) return TRIG_EVENT_NONE;

    /* Convert float[9] → double[9] for m2cgen score(). */
    double input[ML_FEAT_COUNT];
    for (int i = 0; i < ML_FEAT_COUNT; i++) {
        input[i] = (double)snap->ml_feat->v[i];
    }

    double output[2];  /* [P(FORCE_DOWN), P(FORCE_UP)] */
    score(input, output);

    /* Directional filtering: only fire valid state transitions. */
    if (cur == FSM_FORCE_DOWN && output[1] > ML_PROB_THRESHOLD) {
        return TRIG_EVENT_FORCE_UP;
    }
    if (cur == FSM_FORCE_UP && output[0] > ML_PROB_THRESHOLD) {
        return TRIG_EVENT_FORCE_DOWN;
    }

    return TRIG_EVENT_NONE;
}

const trigger_provider_t trig_ml = {
    .name = "ml",
    .eval = ml_eval,
};
