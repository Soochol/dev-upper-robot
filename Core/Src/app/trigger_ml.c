/**
 * @file    trigger_ml.c
 * @brief   ML-based trigger provider — Decision Tree stub (Phase 4).
 *
 * v1 always returns NONE. Replace the body of ml_eval with the output
 * of sklearn → m2cgen code generation once a trained model exists.
 * See TODO.md "Decision Tree ML model" for the full pipeline.
 */

#include "app/trigger.h"

static uint8_t ml_eval(const sensor_snapshot_t *snap, fsm_state_t cur)
{
    (void)snap;
    (void)cur;
    /* Stub: always returns no event. Real model arrives Phase 6. */
    return TRIG_EVENT_NONE;
}

const trigger_provider_t trig_ml = {
    .name = "ml",
    .eval = ml_eval,
};
