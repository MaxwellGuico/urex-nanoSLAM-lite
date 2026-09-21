#include "nanolite_core.h"

#include <math.h>
#include <string.h>

#define NANOLITE_PI 3.14159265358979323846f

static bool finite_pose(const nanolite_pose_t *pose)
{
    return pose != NULL && isfinite(pose->x_m) && isfinite(pose->y_m) &&
           isfinite(pose->yaw_rad);
}

float nanolite_wrap_angle(float angle_rad)
{
    while (angle_rad > NANOLITE_PI) {
        angle_rad -= 2.0f * NANOLITE_PI;
    }
    while (angle_rad < -NANOLITE_PI) {
        angle_rad += 2.0f * NANOLITE_PI;
    }
    return angle_rad;
}

static nanolite_edge_t relative_edge(uint8_t from_index,
                                     uint8_t to_index,
                                     const nanolite_pose_t *from,
                                     const nanolite_pose_t *to)
{
    const float world_dx = to->x_m - from->x_m;
    const float world_dy = to->y_m - from->y_m;
    const float cosine = cosf(from->yaw_rad);
    const float sine = sinf(from->yaw_rad);
    nanolite_edge_t edge = {
        .from_index = from_index,
        .to_index = to_index,
        .reserved = 0,
        .dx_m = cosine * world_dx + sine * world_dy,
        .dy_m = -sine * world_dx + cosine * world_dy,
        .dyaw_rad = nanolite_wrap_angle(to->yaw_rad - from->yaw_rad),
    };
    return edge;
}

nanolite_graph_config_t nanolite_default_graph_config(void)
{
    const nanolite_graph_config_t config = {
        .key_translation_m = 0.25f,
        .key_yaw_rad = 15.0f * NANOLITE_PI / 180.0f,
    };
    return config;
}

bool nanolite_graph_init(nanolite_graph_t *graph,
                         const nanolite_graph_config_t *config,
                         const nanolite_pose_t *anchor)
{
    if (graph == NULL || !finite_pose(anchor)) {
        return false;
    }

    const nanolite_graph_config_t selected =
        config == NULL ? nanolite_default_graph_config() : *config;
    if (!isfinite(selected.key_translation_m) ||
        !isfinite(selected.key_yaw_rad) ||
        selected.key_translation_m <= 0.0f || selected.key_yaw_rad <= 0.0f) {
        return false;
    }

    memset(graph, 0, sizeof(*graph));
    graph->config = selected;
    graph->poses[0] = *anchor;
    graph->poses[0].yaw_rad = nanolite_wrap_angle(anchor->yaw_rad);
    graph->poses[0].sequence = 0;
    graph->poses[0].flags = NANOLITE_POSE_KEY;
    graph->poses[0].reserved = 0;
    graph->pose_count = 1;
    return true;
}

nanolite_pose_result_t nanolite_graph_observe_pose(
    nanolite_graph_t *graph, const nanolite_pose_t *pose, uint8_t flags)
{
    if (graph == NULL || graph->pose_count == 0 || !finite_pose(pose)) {
        return NANOLITE_POSE_REJECTED;
    }

    const nanolite_pose_t *last = &graph->poses[graph->pose_count - 1U];
    if (pose->timestamp_us < last->timestamp_us) {
        return NANOLITE_POSE_REJECTED;
    }

    const float dx = pose->x_m - last->x_m;
    const float dy = pose->y_m - last->y_m;
    const float distance_sq = dx * dx + dy * dy;
    const float yaw_delta = fabsf(nanolite_wrap_angle(pose->yaw_rad - last->yaw_rad));
    const bool forced = (flags & (NANOLITE_POSE_KEY | NANOLITE_POSE_SCAN)) != 0;
    const bool moved = distance_sq >=
                       graph->config.key_translation_m *
                           graph->config.key_translation_m;
    const bool turned = yaw_delta >= graph->config.key_yaw_rad;
    if (!forced && !moved && !turned) {
        return NANOLITE_POSE_SKIPPED;
    }

    if (graph->pose_count >= NANOLITE_MAX_POSES) {
        graph->saturated = true;
        return NANOLITE_POSE_FULL;
    }

    const uint8_t next_index = graph->pose_count;
    nanolite_pose_t stored = *pose;
    stored.yaw_rad = nanolite_wrap_angle(stored.yaw_rad);
    stored.sequence = next_index;
    stored.flags = (uint8_t)(flags | NANOLITE_POSE_KEY);
    stored.reserved = 0;

    graph->odometry[graph->odometry_count] =
        relative_edge((uint8_t)(next_index - 1U), next_index, last, &stored);
    ++graph->odometry_count;
    graph->poses[next_index] = stored;
    ++graph->pose_count;
    return NANOLITE_POSE_ADDED;
}

bool nanolite_graph_add_loop_edge(nanolite_graph_t *graph,
                                  uint8_t from_index,
                                  uint8_t to_index,
                                  float dx_m,
                                  float dy_m,
                                  float dyaw_rad)
{
    if (graph == NULL || from_index >= graph->pose_count ||
        to_index >= graph->pose_count || from_index == to_index ||
        graph->loop_count >= NANOLITE_MAX_LOOP_EDGES || !isfinite(dx_m) ||
        !isfinite(dy_m) || !isfinite(dyaw_rad)) {
        return false;
    }

    nanolite_edge_t edge = {
        .from_index = from_index,
        .to_index = to_index,
        .reserved = 0,
        .dx_m = dx_m,
        .dy_m = dy_m,
        .dyaw_rad = nanolite_wrap_angle(dyaw_rad),
    };
    graph->loop_edges[graph->loop_count] = edge;
    ++graph->loop_count;
    return true;
}

const nanolite_pose_t *nanolite_graph_latest_pose(
    const nanolite_graph_t *graph)
{
    if (graph == NULL || graph->pose_count == 0) {
        return NULL;
    }
    return &graph->poses[graph->pose_count - 1U];
}

size_t nanolite_graph_storage_bytes(void)
{
    return sizeof(nanolite_graph_t);
}
