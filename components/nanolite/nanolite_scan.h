#pragma once

#include "nanolite_core.h"
#include "nanolite_tof.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NANOLITE_MAX_SCAN_POINTS 160U
#define NANOLITE_MAX_STORED_SCANS 16U
#define NANOLITE_SENSOR_COUNT 8U

typedef struct {
    float x_m;
    float y_m;
} nanolite_point_t;

typedef struct {
    nanolite_point_t points[NANOLITE_MAX_SCAN_POINTS];
    uint32_t timestamp_us;
    uint16_t point_count;
    uint8_t pose_index;
    bool valid;
} nanolite_scan_t;

typedef struct {
    float min_point_separation_m;
    uint16_t min_points;
} nanolite_scan_config_t;

typedef struct {
    nanolite_scan_t scan;
    uint16_t frame_count;
} nanolite_scan_builder_t;

typedef struct {
    nanolite_scan_t scans[NANOLITE_MAX_STORED_SCANS];
    uint8_t count;
    uint8_t next_slot;
} nanolite_scan_store_t;

nanolite_scan_config_t nanolite_default_scan_config(void);

bool nanolite_scan_builder_init(nanolite_scan_builder_t *builder,
                                uint8_t pose_index,
                                uint32_t timestamp_us);

/* Add one ring snapshot. Points are stored in the reference pose's local
 * frame, so they remain usable after the pose graph is corrected. */
uint16_t nanolite_scan_builder_add_returns(
    nanolite_scan_builder_t *builder,
    const nanolite_pose_t *reference_pose,
    const nanolite_pose_t *observation_pose,
    const nanolite_tof_return_t *returns,
    size_t return_count,
    const float sensor_yaw_rad[NANOLITE_SENSOR_COUNT],
    float horizontal_fov_rad,
    bool column_zero_clockwise,
    const nanolite_scan_config_t *config);

bool nanolite_scan_builder_finish(nanolite_scan_builder_t *builder,
                                  const nanolite_scan_config_t *config,
                                  nanolite_scan_t *result);

void nanolite_scan_store_init(nanolite_scan_store_t *store);

/* Returns the slot used, or -1 when the scan is invalid. Old scans are
 * replaced in a deterministic ring once capacity is reached. */
int nanolite_scan_store_push(nanolite_scan_store_t *store,
                             const nanolite_scan_t *scan);

size_t nanolite_scan_storage_bytes(void);

#ifdef __cplusplus
}
#endif
