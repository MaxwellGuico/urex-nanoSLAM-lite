#pragma once

#include "nanolite_core.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t max_iterations;
    float odometry_weight;
    float loop_weight;
    float relaxation;
    float max_translation_step_m;
    float max_yaw_step_rad;
} nanolite_optimizer_config_t;

typedef struct {
    bool changed;
    uint8_t iterations;
    float initial_cost;
    float final_cost;
    float maximum_translation_change_m;
    float maximum_yaw_change_rad;
} nanolite_optimizer_report_t;

nanolite_optimizer_config_t nanolite_default_optimizer_config(void);

/* Bounded SE(2) constraint relaxation. Pose zero is always held fixed. */
bool nanolite_graph_optimize(nanolite_graph_t *graph,
                             const nanolite_optimizer_config_t *config,
                             nanolite_optimizer_report_t *report);

#ifdef __cplusplus
}
#endif
