#include "nanolite_slam.h"

#include <math.h>
#include <string.h>

#define NANOLITE_PI 3.14159265358979323846f

nanolite_slam_config_t nanolite_default_slam_config(void)
{
    const nanolite_slam_config_t config = {
        .graph = {
            .key_translation_m = 0.25f,
            .key_yaw_rad = 15.0f * NANOLITE_PI / 180.0f,
        },
        .scan = {
            .min_point_separation_m = 0.03f,
            .min_points = 12U,
        },
        .icp = {
            .max_iterations = 10U,
            .min_pairs = 8U,
            .max_correspondence_m = 0.45f,
            .min_overlap = 0.45f,
            .max_final_rmse_m = 0.15f,
            .max_translation_correction_m = 0.75f,
            .max_yaw_correction_rad = 45.0f * NANOLITE_PI / 180.0f,
            .min_rmse_improvement_m = 0.005f,
            .translation_epsilon_m = 0.001f,
            .yaw_epsilon_rad = 0.001f,
        },
        .optimizer = {
            .max_iterations = 3U,
            .odometry_weight = 1.0f,
            .loop_weight = 4.0f,
            .relaxation = 0.45f,
            .max_translation_step_m = 0.20f,
            .max_yaw_step_rad = 10.0f * NANOLITE_PI / 180.0f,
        },
        .minimum_loop_pose_separation = 6U,
        .maximum_loop_candidate_distance_m = 1.0f,
        .maximum_loop_candidate_yaw_rad = 60.0f * NANOLITE_PI / 180.0f,
        .horizontal_fov_rad = 45.0f * NANOLITE_PI / 180.0f,
        .column_zero_clockwise = true,
        .enable_loop_closure = true,
    };
    return config;
}

static bool valid_pose(const nanolite_pose_t *pose)
{
    return pose != NULL && isfinite(pose->x_m) && isfinite(pose->y_m) &&
           isfinite(pose->yaw_rad);
}

static nanolite_pose_t transform_pose(const nanolite_pose_t *transform,
                                      const nanolite_pose_t *pose)
{
    const float cosine = cosf(transform->yaw_rad);
    const float sine = sinf(transform->yaw_rad);
    nanolite_pose_t result = *pose;
    result.x_m = transform->x_m + cosine * pose->x_m - sine * pose->y_m;
    result.y_m = transform->y_m + sine * pose->x_m + cosine * pose->y_m;
    result.yaw_rad = nanolite_wrap_angle(transform->yaw_rad + pose->yaw_rad);
    return result;
}

static nanolite_pose_t correction_between(const nanolite_pose_t *before,
                                          const nanolite_pose_t *after)
{
    nanolite_pose_t correction = {0};
    correction.yaw_rad = nanolite_wrap_angle(after->yaw_rad - before->yaw_rad);
    const float cosine = cosf(correction.yaw_rad);
    const float sine = sinf(correction.yaw_rad);
    correction.x_m = after->x_m - (cosine * before->x_m - sine * before->y_m);
    correction.y_m = after->y_m - (sine * before->x_m + cosine * before->y_m);
    return correction;
}

static bool loop_edge_exists(const nanolite_graph_t *graph,
                             uint8_t first,
                             uint8_t second)
{
    for (uint8_t index = 0U; index < graph->loop_count; ++index) {
        const nanolite_edge_t *edge = &graph->loop_edges[index];
        if ((edge->from_index == first && edge->to_index == second) ||
            (edge->from_index == second && edge->to_index == first)) {
            return true;
        }
    }
    return false;
}

static const nanolite_scan_t *select_loop_candidate(
    const nanolite_slam_t *slam,
    const nanolite_scan_t *source)
{
    const nanolite_pose_t *source_pose = &slam->graph.poses[source->pose_index];
    const nanolite_scan_t *best = NULL;
    float best_score = INFINITY;
    for (uint8_t slot = 0U; slot < NANOLITE_MAX_STORED_SCANS; ++slot) {
        const nanolite_scan_t *candidate = &slam->scan_store.scans[slot];
        if (!candidate->valid || candidate == source ||
            candidate->pose_index >= slam->graph.pose_count) {
            continue;
        }
        const uint8_t separation = source->pose_index > candidate->pose_index
                                       ? (uint8_t)(source->pose_index -
                                                   candidate->pose_index)
                                       : (uint8_t)(candidate->pose_index -
                                                   source->pose_index);
        if (separation < slam->config.minimum_loop_pose_separation ||
            loop_edge_exists(&slam->graph, source->pose_index,
                             candidate->pose_index)) {
            continue;
        }
        const nanolite_pose_t *candidate_pose =
            &slam->graph.poses[candidate->pose_index];
        const float distance = hypotf(source_pose->x_m - candidate_pose->x_m,
                                      source_pose->y_m - candidate_pose->y_m);
        const float yaw_difference = fabsf(nanolite_wrap_angle(
            source_pose->yaw_rad - candidate_pose->yaw_rad));
        if (distance > slam->config.maximum_loop_candidate_distance_m ||
            yaw_difference > slam->config.maximum_loop_candidate_yaw_rad) {
            continue;
        }
        const float score = distance + 0.1f * yaw_difference;
        if (score < best_score) {
            best = candidate;
            best_score = score;
        }
    }
    return best;
}

bool nanolite_slam_rebuild_map(nanolite_slam_t *slam)
{
    if (slam == NULL || !slam->initialized || slam->graph.pose_count == 0U ||
        !nanolite_map_init(&slam->map, slam->graph.poses[0].x_m,
                           slam->graph.poses[0].y_m)) {
        return false;
    }
    for (uint8_t slot = 0U; slot < NANOLITE_MAX_STORED_SCANS; ++slot) {
        const nanolite_scan_t *scan = &slam->scan_store.scans[slot];
        if (!scan->valid || scan->pose_index >= slam->graph.pose_count) {
            continue;
        }
        const nanolite_pose_t *pose = &slam->graph.poses[scan->pose_index];
        for (uint16_t point = 0U; point < scan->point_count; ++point) {
            (void)nanolite_map_integrate_scan_point(
                &slam->map, pose, scan->points[point].x_m,
                scan->points[point].y_m);
        }
    }
    if (slam->scan_builder_active &&
        slam->scan_builder.scan.pose_index < slam->graph.pose_count) {
        const nanolite_scan_t *scan = &slam->scan_builder.scan;
        const nanolite_pose_t *pose = &slam->graph.poses[scan->pose_index];
        for (uint16_t point = 0U; point < scan->point_count; ++point) {
            (void)nanolite_map_integrate_scan_point(
                &slam->map, pose, scan->points[point].x_m,
                scan->points[point].y_m);
        }
    }
    ++slam->map_rebuild_count;
    return true;
}

static bool attempt_loop_closure(nanolite_slam_t *slam,
                                 const nanolite_scan_t *source)
{
    if (!slam->config.enable_loop_closure ||
        slam->graph.loop_count >= NANOLITE_MAX_LOOP_EDGES) {
        return false;
    }
    const nanolite_scan_t *target = select_loop_candidate(slam, source);
    if (target == NULL) {
        return false;
    }
    ++slam->attempted_loop_count;
    const nanolite_pose_t *source_pose = &slam->graph.poses[source->pose_index];
    const nanolite_pose_t *target_pose = &slam->graph.poses[target->pose_index];
    if (!nanolite_icp_match(source, source_pose, target, target_pose,
                            &slam->config.icp, &slam->icp_workspace,
                            &slam->last_icp) || !slam->last_icp.accepted) {
        ++slam->rejected_loop_count;
        return false;
    }
    if (!nanolite_graph_add_loop_edge(
            &slam->graph, slam->last_icp.loop_edge.from_index,
            slam->last_icp.loop_edge.to_index, slam->last_icp.loop_edge.dx_m,
            slam->last_icp.loop_edge.dy_m,
            slam->last_icp.loop_edge.dyaw_rad)) {
        ++slam->rejected_loop_count;
        return false;
    }

    const nanolite_pose_t latest_before =
        slam->graph.poses[slam->graph.pose_count - 1U];
    memcpy(slam->optimizer_pose_backup, slam->graph.poses,
           (size_t)slam->graph.pose_count * sizeof(slam->graph.poses[0]));
    if (!nanolite_graph_optimize(&slam->graph, &slam->config.optimizer,
                                 &slam->last_optimizer) ||
        !slam->last_optimizer.changed ||
        !isfinite(slam->last_optimizer.final_cost) ||
        slam->last_optimizer.final_cost >= slam->last_optimizer.initial_cost) {
        memcpy(slam->graph.poses, slam->optimizer_pose_backup,
               (size_t)slam->graph.pose_count * sizeof(slam->graph.poses[0]));
        --slam->graph.loop_count;
        ++slam->rejected_loop_count;
        return false;
    }
    const nanolite_pose_t *latest_after =
        &slam->graph.poses[slam->graph.pose_count - 1U];
    const nanolite_pose_t correction =
        correction_between(&latest_before, latest_after);
    slam->map_from_odometry =
        transform_pose(&correction, &slam->map_from_odometry);
    ++slam->accepted_loop_count;
    (void)nanolite_slam_rebuild_map(slam);
    return true;
}

bool nanolite_slam_finalize_active_scan(nanolite_slam_t *slam,
                                        bool attempt_loop)
{
    if (slam == NULL || !slam->initialized || !slam->scan_builder_active) {
        return false;
    }
    nanolite_scan_t completed;
    const bool usable = nanolite_scan_builder_finish(
        &slam->scan_builder, &slam->config.scan, &completed);
    slam->scan_builder_active = false;
    bool accepted = false;
    if (usable) {
        slam->graph.poses[completed.pose_index].flags |= NANOLITE_POSE_SCAN;
        const int slot = nanolite_scan_store_push(&slam->scan_store, &completed);
        if (slot >= 0) {
            ++slam->completed_scan_count;
            if (attempt_loop) {
                accepted = attempt_loop_closure(
                    slam, &slam->scan_store.scans[(uint8_t)slot]);
            }
        }
    }

    const nanolite_pose_t *latest = nanolite_graph_latest_pose(&slam->graph);
    slam->scan_builder_active = latest != NULL && nanolite_scan_builder_init(
        &slam->scan_builder, (uint8_t)(slam->graph.pose_count - 1U),
        latest == NULL ? 0U : latest->timestamp_us);
    return usable || accepted;
}

bool nanolite_slam_init(
    nanolite_slam_t *slam,
    const nanolite_slam_config_t *config,
    const nanolite_pose_t *odometry_anchor,
    const float sensor_yaw_rad[NANOLITE_SENSOR_COUNT])
{
    if (slam == NULL || !valid_pose(odometry_anchor) ||
        sensor_yaw_rad == NULL) {
        return false;
    }
    const nanolite_slam_config_t selected =
        config == NULL ? nanolite_default_slam_config() : *config;
    if (selected.minimum_loop_pose_separation == 0U ||
        !isfinite(selected.maximum_loop_candidate_distance_m) ||
        selected.maximum_loop_candidate_distance_m <= 0.0f ||
        !isfinite(selected.maximum_loop_candidate_yaw_rad) ||
        selected.maximum_loop_candidate_yaw_rad <= 0.0f ||
        !isfinite(selected.horizontal_fov_rad) ||
        selected.horizontal_fov_rad <= 0.0f) {
        return false;
    }
    for (uint8_t sensor = 0U; sensor < NANOLITE_SENSOR_COUNT; ++sensor) {
        if (!isfinite(sensor_yaw_rad[sensor])) {
            return false;
        }
    }

    memset(slam, 0, sizeof(*slam));
    slam->config = selected;
    memcpy(slam->sensor_yaw_rad, sensor_yaw_rad,
           sizeof(slam->sensor_yaw_rad));
    nanolite_scan_store_init(&slam->scan_store);
    if (!nanolite_graph_init(&slam->graph, &selected.graph, odometry_anchor) ||
        !nanolite_map_init(&slam->map, odometry_anchor->x_m,
                           odometry_anchor->y_m) ||
        !nanolite_scan_builder_init(&slam->scan_builder, 0U,
                                    odometry_anchor->timestamp_us)) {
        return false;
    }
    slam->map_from_odometry.yaw_rad = 0.0f;
    slam->scan_builder_active = true;
    slam->initialized = true;
    return true;
}

nanolite_pose_result_t nanolite_slam_observe(
    nanolite_slam_t *slam,
    const nanolite_pose_t *odometry_pose,
    const nanolite_tof_return_t *returns,
    size_t return_count)
{
    return nanolite_slam_observe_timed(slam, odometry_pose, returns, NULL,
                                       return_count);
}

nanolite_pose_result_t nanolite_slam_observe_timed(
    nanolite_slam_t *slam,
    const nanolite_pose_t *odometry_pose,
    const nanolite_tof_return_t *returns,
    const nanolite_pose_t *return_odometry_poses,
    size_t return_count)
{
    if (slam == NULL || !slam->initialized || !valid_pose(odometry_pose) ||
        (return_count > 0U && returns == NULL)) {
        return NANOLITE_POSE_REJECTED;
    }
    nanolite_pose_t mapped_pose =
        transform_pose(&slam->map_from_odometry, odometry_pose);
    const nanolite_pose_result_t pose_result =
        nanolite_graph_observe_pose(&slam->graph, &mapped_pose, 0U);
    if (pose_result == NANOLITE_POSE_REJECTED) {
        return pose_result;
    }
    if (pose_result == NANOLITE_POSE_ADDED) {
        (void)nanolite_slam_finalize_active_scan(slam, true);
        mapped_pose = slam->graph.poses[slam->graph.pose_count - 1U];
    }

    nanolite_tof_return_t fresh[NANOLITE_SENSOR_COUNT];
    nanolite_pose_t fresh_poses[NANOLITE_SENSOR_COUNT];
    size_t fresh_count = 0U;
    for (size_t index = 0U; index < return_count; ++index) {
        const nanolite_tof_return_t *observation = &returns[index];
        const nanolite_pose_t *observation_odometry_pose =
            return_odometry_poses == NULL ? odometry_pose
                                          : &return_odometry_poses[index];
        if (observation->sensor_id >= NANOLITE_SENSOR_COUNT ||
            observation->timestamp_us == 0U ||
            !valid_pose(observation_odometry_pose) ||
            observation->timestamp_us ==
                slam->last_return_timestamp_us[observation->sensor_id]) {
            continue;
        }
        slam->last_return_timestamp_us[observation->sensor_id] =
            observation->timestamp_us;
        if (!observation->valid || observation->distance_mm == 0U ||
            observation->column >= 8U ||
            fresh_count >= NANOLITE_SENSOR_COUNT) {
            continue;
        }
        fresh[fresh_count++] = *observation;
        fresh_poses[fresh_count - 1U] = transform_pose(
            &slam->map_from_odometry, observation_odometry_pose);
        (void)nanolite_map_integrate_tof(
            &slam->map, &fresh_poses[fresh_count - 1U], observation,
            slam->sensor_yaw_rad[observation->sensor_id],
            slam->config.horizontal_fov_rad,
            slam->config.column_zero_clockwise);
    }
    if (fresh_count > 0U && slam->scan_builder_active) {
        const uint8_t reference_index = slam->scan_builder.scan.pose_index;
        if (reference_index < slam->graph.pose_count) {
            for (size_t index = 0U; index < fresh_count; ++index) {
                (void)nanolite_scan_builder_add_returns(
                    &slam->scan_builder, &slam->graph.poses[reference_index],
                    &fresh_poses[index], &fresh[index], 1U,
                    slam->sensor_yaw_rad, slam->config.horizontal_fov_rad,
                    slam->config.column_zero_clockwise, &slam->config.scan);
            }
        }
    }
    return pose_result;
}

bool nanolite_slam_correct_odometry_pose(const nanolite_slam_t *slam,
                                         const nanolite_pose_t *odometry_pose,
                                         nanolite_pose_t *corrected_pose)
{
    if (slam == NULL || !slam->initialized || !valid_pose(odometry_pose) ||
        corrected_pose == NULL) {
        return false;
    }
    *corrected_pose = transform_pose(&slam->map_from_odometry, odometry_pose);
    return true;
}

nanolite_slam_status_t nanolite_slam_get_status(const nanolite_slam_t *slam)
{
    nanolite_slam_status_t status = {0};
    if (slam == NULL) {
        return status;
    }
    status.pose_count = slam->graph.pose_count;
    status.loop_count = slam->graph.loop_count;
    status.stored_scan_count = slam->scan_store.count;
    status.active_scan_points = slam->scan_builder.scan.point_count;
    status.completed_scan_count = slam->completed_scan_count;
    status.attempted_loop_count = slam->attempted_loop_count;
    status.accepted_loop_count = slam->accepted_loop_count;
    status.rejected_loop_count = slam->rejected_loop_count;
    status.map_rebuild_count = slam->map_rebuild_count;
    status.saturated = slam->graph.saturated;
    status.initialized = slam->initialized;
    return status;
}

size_t nanolite_slam_storage_bytes(void)
{
    return sizeof(nanolite_slam_t);
}
