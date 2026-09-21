#pragma once

#include "nanolite_core.h"
#include "nanolite_icp.h"
#include "nanolite_map.h"
#include "nanolite_optimize.h"
#include "nanolite_scan.h"
#include "nanolite_tof.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    nanolite_graph_config_t graph;
    nanolite_scan_config_t scan;
    nanolite_icp_config_t icp;
    nanolite_optimizer_config_t optimizer;
    uint8_t minimum_loop_pose_separation;
    float maximum_loop_candidate_distance_m;
    float maximum_loop_candidate_yaw_rad;
    float horizontal_fov_rad;
    bool column_zero_clockwise;
    bool enable_loop_closure;
} nanolite_slam_config_t;

typedef struct {
    nanolite_graph_t graph;
    nanolite_map_t map;
    nanolite_scan_store_t scan_store;
    nanolite_scan_builder_t scan_builder;
    nanolite_icp_workspace_t icp_workspace;
    nanolite_slam_config_t config;
    nanolite_icp_result_t last_icp;
    nanolite_optimizer_report_t last_optimizer;
    nanolite_pose_t optimizer_pose_backup[NANOLITE_MAX_POSES];
    nanolite_pose_t map_from_odometry;
    float sensor_yaw_rad[NANOLITE_SENSOR_COUNT];
    uint32_t last_return_timestamp_us[NANOLITE_SENSOR_COUNT];
    uint32_t completed_scan_count;
    uint32_t attempted_loop_count;
    uint32_t accepted_loop_count;
    uint32_t rejected_loop_count;
    uint32_t map_rebuild_count;
    bool scan_builder_active;
    bool initialized;
} nanolite_slam_t;

typedef struct {
    uint8_t pose_count;
    uint8_t loop_count;
    uint8_t stored_scan_count;
    uint16_t active_scan_points;
    uint32_t completed_scan_count;
    uint32_t attempted_loop_count;
    uint32_t accepted_loop_count;
    uint32_t rejected_loop_count;
    uint32_t map_rebuild_count;
    bool saturated;
    bool initialized;
} nanolite_slam_status_t;

nanolite_slam_config_t nanolite_default_slam_config(void);

bool nanolite_slam_init(
    nanolite_slam_t *slam,
    const nanolite_slam_config_t *config,
    const nanolite_pose_t *odometry_anchor,
    const float sensor_yaw_rad[NANOLITE_SENSOR_COUNT]);

/* Feed a synchronized odometry pose and the newest reduced ring returns.
 * Duplicate ToF timestamps are ignored. A key pose automatically closes the
 * preceding scan and attempts one conservative loop closure. */
nanolite_pose_result_t nanolite_slam_observe(
    nanolite_slam_t *slam,
    const nanolite_pose_t *odometry_pose,
    const nanolite_tof_return_t *returns,
    size_t return_count);

/* Apply the current map-from-odometry correction to a live raw pose. */
bool nanolite_slam_correct_odometry_pose(const nanolite_slam_t *slam,
                                         const nanolite_pose_t *odometry_pose,
                                         nanolite_pose_t *corrected_pose);

/* Close the current scan without waiting for another key pose. A fresh scan
 * builder is started at the latest corrected pose. */
bool nanolite_slam_finalize_active_scan(nanolite_slam_t *slam,
                                        bool attempt_loop_closure);

/* Rebuild the occupancy grid from retained scan points and corrected poses. */
bool nanolite_slam_rebuild_map(nanolite_slam_t *slam);

nanolite_slam_status_t nanolite_slam_get_status(const nanolite_slam_t *slam);

size_t nanolite_slam_storage_bytes(void);

#ifdef __cplusplus
}
#endif
