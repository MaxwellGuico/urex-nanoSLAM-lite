#pragma once

#include "nanolite_core.h"
#include "nanolite_scan.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t max_iterations;
    uint16_t min_pairs;
    float max_correspondence_m;
    float min_overlap;
    float max_final_rmse_m;
    float max_translation_correction_m;
    float max_yaw_correction_rad;
    float min_rmse_improvement_m;
    float translation_epsilon_m;
    float yaw_epsilon_rad;
} nanolite_icp_config_t;

typedef struct {
    nanolite_point_t source_world[NANOLITE_MAX_SCAN_POINTS];
    nanolite_point_t target_world[NANOLITE_MAX_SCAN_POINTS];
    uint8_t nearest_target[NANOLITE_MAX_SCAN_POINTS];
    uint8_t matched[NANOLITE_MAX_SCAN_POINTS];
} nanolite_icp_workspace_t;

typedef struct {
    bool accepted;
    bool converged;
    uint8_t iterations;
    uint16_t pair_count;
    float overlap;
    float initial_rmse_m;
    float final_rmse_m;
    float translation_correction_m;
    float yaw_correction_rad;
    nanolite_pose_t corrected_source_pose;
    nanolite_edge_t loop_edge;
} nanolite_icp_result_t;

nanolite_icp_config_t nanolite_default_icp_config(void);

/* Match source against target. Scan points are local to their associated
 * poses. The returned edge is target_pose -> corrected_source_pose. */
bool nanolite_icp_match(const nanolite_scan_t *source,
                        const nanolite_pose_t *source_pose,
                        const nanolite_scan_t *target,
                        const nanolite_pose_t *target_pose,
                        const nanolite_icp_config_t *config,
                        nanolite_icp_workspace_t *workspace,
                        nanolite_icp_result_t *result);

#ifdef __cplusplus
}
#endif
