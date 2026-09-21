#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "nanolite_core.h"
#include "nanolite_map.h"

/* Low-priority, read-only bridge from PX4's fused NED pose to NanoSLAM-Lite. */
#define NANOLITE_TASK_CORE      0
#define NANOLITE_TASK_PRIORITY  1
#define NANOLITE_TASK_STACK     6144

typedef struct {
    uint8_t pose_count;
    uint8_t odometry_count;
    uint8_t loop_count;
    bool initialized;
    bool saturated;
    uint32_t stale_samples;
    uint32_t rejected_samples;
    uint8_t keyscan_count;
    uint16_t active_scan_points;
    uint32_t loop_attempts;
    uint32_t loop_accepts;
    uint32_t loop_rejects;
    uint32_t map_rebuilds;
    uint16_t map_observed_cells;
    uint16_t map_candidate_cells;
    uint16_t map_occupied_cells;
    uint32_t map_ray_count;
    uint32_t map_dropped_rays;
} nanolite_task_status_t;

/* Create internal synchronization. Call once before spawning tasks. */
void nanolite_task_init(void);

/* FreeRTOS task entry. Observes telemetry only and never sends a setpoint. */
void nanolite_task(void *arg);

/* Thread-safe compact status for logging/telemetry. */
nanolite_task_status_t nanolite_task_get_status(void);

/* Copy the packed 40x40 map for future telemetry/visualization. */
bool nanolite_task_copy_map(nanolite_map_t *destination);
