#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NANOLITE_MAX_POSES 64U
#define NANOLITE_MAX_LOOP_EDGES 8U

enum {
    NANOLITE_POSE_KEY = 1U << 0,
    NANOLITE_POSE_SCAN = 1U << 1,
    NANOLITE_POSE_CORRECTED = 1U << 2,
};

typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;
    uint32_t timestamp_us;
    uint16_t sequence;
    uint8_t flags;
    uint8_t reserved;
} nanolite_pose_t;

typedef struct {
    uint8_t from_index;
    uint8_t to_index;
    uint16_t reserved;
    float dx_m;
    float dy_m;
    float dyaw_rad;
} nanolite_edge_t;

typedef struct {
    float key_translation_m;
    float key_yaw_rad;
} nanolite_graph_config_t;

typedef enum {
    NANOLITE_POSE_REJECTED = 0,
    NANOLITE_POSE_SKIPPED = 1,
    NANOLITE_POSE_ADDED = 2,
    NANOLITE_POSE_FULL = 3,
} nanolite_pose_result_t;

typedef struct {
    nanolite_pose_t poses[NANOLITE_MAX_POSES];
    nanolite_edge_t odometry[NANOLITE_MAX_POSES - 1U];
    nanolite_edge_t loop_edges[NANOLITE_MAX_LOOP_EDGES];
    nanolite_graph_config_t config;
    uint8_t pose_count;
    uint8_t odometry_count;
    uint8_t loop_count;
    bool saturated;
} nanolite_graph_t;

nanolite_graph_config_t nanolite_default_graph_config(void);

bool nanolite_graph_init(nanolite_graph_t *graph,
                         const nanolite_graph_config_t *config,
                         const nanolite_pose_t *anchor);

nanolite_pose_result_t nanolite_graph_observe_pose(
    nanolite_graph_t *graph, const nanolite_pose_t *pose, uint8_t flags);

bool nanolite_graph_add_loop_edge(nanolite_graph_t *graph,
                                  uint8_t from_index,
                                  uint8_t to_index,
                                  float dx_m,
                                  float dy_m,
                                  float dyaw_rad);

const nanolite_pose_t *nanolite_graph_latest_pose(
    const nanolite_graph_t *graph);

float nanolite_wrap_angle(float angle_rad);

size_t nanolite_graph_storage_bytes(void);

#ifdef __cplusplus
}
#endif
