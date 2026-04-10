/**
 * @file    trigger.c
 * @brief   Trigger provider selector — compile-time dispatch to rule or ML.
 */

#include "app/trigger.h"
#include "app/config.h"

/* Compile-time provider selection. The #if compiles in only one provider
 * path, so the other's code is dead-stripped by the linker (--gc-sections)
 * and does not cost Flash. To switch, change TRIGGER_SOURCE in config.h
 * and rebuild. Runtime switching (pointer reassignment) would live here
 * as a future Phase 6+ enhancement. */

#if TRIGGER_SOURCE == TRIG_SRC_ML
static const trigger_provider_t *g_provider = &trig_ml;
#else
static const trigger_provider_t *g_provider = &trig_rule;
#endif

uint8_t trigger_eval(const sensor_snapshot_t *snap, fsm_state_t current_state)
{
    return g_provider->eval(snap, current_state);
}

const char *trigger_provider_name(void)
{
    return g_provider->name;
}
