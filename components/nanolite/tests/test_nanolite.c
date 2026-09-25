#include "nanolite_core.h"
#include "nanolite_icp.h"
#include "nanolite_map.h"
#include "nanolite_optimize.h"
#include "nanolite_scan.h"
#include "nanolite_slam.h"
#include "nanolite_tof.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static nanolite_pose_t pose(float x, float y, float yaw, uint32_t timestamp_us)
{
    const nanolite_pose_t value = {
        .x_m = x,
        .y_m = y,
        .yaw_rad = yaw,
        .timestamp_us = timestamp_us,
    };
    return value;
}

static void test_pose_selection_and_edges(void)
{
    nanolite_graph_t graph;
    nanolite_pose_t anchor = pose(0.0f, 0.0f, 0.0f, 100U);
    assert(nanolite_graph_init(&graph, NULL, &anchor));
    assert(graph.pose_count == 1U);
    assert(graph.poses[0].flags == NANOLITE_POSE_KEY);

    nanolite_pose_t small_move = pose(0.10f, 0.0f, 0.0f, 200U);
    assert(nanolite_graph_observe_pose(&graph, &small_move, 0U) ==
           NANOLITE_POSE_SKIPPED);

    nanolite_pose_t key = pose(0.25f, 0.0f, 0.0f, 300U);
    assert(nanolite_graph_observe_pose(&graph, &key, 0U) ==
           NANOLITE_POSE_ADDED);
    assert(graph.pose_count == 2U);
    assert(graph.odometry_count == 1U);
    assert(fabsf(graph.odometry[0].dx_m - 0.25f) < 1.0e-6f);

    nanolite_pose_t scan = pose(0.26f, 0.0f, 0.0f, 400U);
    assert(nanolite_graph_observe_pose(&graph, &scan, NANOLITE_POSE_SCAN) ==
           NANOLITE_POSE_ADDED);
    assert((graph.poses[2].flags & NANOLITE_POSE_SCAN) != 0U);
    assert(nanolite_graph_add_loop_edge(&graph, 0U, 2U, 0.0f, 0.0f, 0.0f));
    assert(graph.loop_count == 1U);
}

static void test_capacity_is_explicit(void)
{
    nanolite_graph_t graph;
    nanolite_pose_t anchor = pose(0.0f, 0.0f, 0.0f, 0U);
    assert(nanolite_graph_init(&graph, NULL, &anchor));
    for (uint8_t index = 1U; index < NANOLITE_MAX_POSES; ++index) {
        nanolite_pose_t next = pose(index * 0.30f, 0.0f, 0.0f, index);
        assert(nanolite_graph_observe_pose(&graph, &next, 0U) ==
               NANOLITE_POSE_ADDED);
    }
    nanolite_pose_t overflow = pose(20.0f, 0.0f, 0.0f, 1000U);
    assert(nanolite_graph_observe_pose(&graph, &overflow, 0U) ==
           NANOLITE_POSE_FULL);
    assert(graph.pose_count == NANOLITE_MAX_POSES);
    assert(graph.saturated);
}

static void test_middle_row_tof_reduction(void)
{
    int16_t ranges[NANOLITE_TOF_ZONE_COUNT];
    uint8_t statuses[NANOLITE_TOF_ZONE_COUNT];
    for (size_t index = 0; index < NANOLITE_TOF_ZONE_COUNT; ++index) {
        ranges[index] = 1000;
        statuses[index] = 0;
    }

    statuses[0] = 5U;
    ranges[0] = 60;
    statuses[3U * 8U + 6U] = 5U;
    ranges[3U * 8U + 6U] = 800;
    statuses[4U * 8U + 2U] = 9U;
    ranges[4U * 8U + 2U] = 450;
    statuses[4U * 8U + 1U] = 5U;
    ranges[4U * 8U + 1U] = 2501;

    nanolite_tof_return_t result;
    assert(nanolite_reduce_tof_frame(ranges, statuses, 3U, 12345U, NULL,
                                     &result));
    assert(result.valid);
    assert(result.distance_mm == 450U);
    assert(result.column == 2U);
    assert(result.sensor_id == 3U);
    assert(result.timestamp_us == 12345U);

    memset(statuses, 0, sizeof(statuses));
    assert(nanolite_reduce_tof_frame(ranges, statuses, 3U, 12346U, NULL,
                                     &result));
    assert(!result.valid);
}

static void test_map_projection_and_hit_confirmation(void)
{
    nanolite_map_t map;
    assert(nanolite_map_init(&map, 10.0f, -2.0f));
    assert(nanolite_map_storage_bytes() < 512U);

    int16_t anchor_x;
    int16_t anchor_y;
    assert(nanolite_map_world_to_cell(&map, 10.0f, -2.0f,
                                      &anchor_x, &anchor_y));
    assert(anchor_x == 20);
    assert(anchor_y == 20);

    nanolite_pose_t origin = pose(10.0f, -2.0f, 0.0f, 100U);
    nanolite_tof_return_t hit = {
        .distance_mm = 1000U,
        .column = 3U,
        .sensor_id = 0U,
        .timestamp_us = 100U,
        .valid = true,
    };

    /* A zero-width test FoV keeps the ray on +x while still exercising the
     * ToF projection path. */
    assert(nanolite_map_integrate_tof(&map, &origin, &hit,
                                      0.0f, 0.0001f, true));
    assert(map.ray_count == 1U);
    assert(map.candidate_cells == 1U);
    assert(map.occupied_cells == 0U);
    assert(nanolite_map_get_cell(&map, 20U, 20U) == NANOLITE_MAP_FREE);
    assert(nanolite_map_get_cell(&map, 22U, 20U) ==
           NANOLITE_MAP_HIT_CANDIDATE);

    assert(nanolite_map_integrate_tof(&map, &origin, &hit,
                                      0.0f, 0.0001f, true));
    assert(map.candidate_cells == 0U);
    assert(map.occupied_cells == 1U);
    assert(nanolite_map_get_cell(&map, 22U, 20U) == NANOLITE_MAP_OCCUPIED);

    nanolite_tof_return_t invalid = hit;
    invalid.valid = false;
    assert(!nanolite_map_integrate_tof(&map, &origin, &invalid,
                                       0.0f, 0.0001f, true));
    assert(map.ray_count == 2U);
}

static void test_scan_aggregation(void)
{
    nanolite_scan_builder_t builder;
    nanolite_pose_t reference = pose(1.0f, 2.0f, 0.2f, 100U);
    assert(nanolite_scan_builder_init(&builder, 0U, 100U));

    float sensor_yaws[NANOLITE_SENSOR_COUNT];
    nanolite_tof_return_t returns[NANOLITE_SENSOR_COUNT];
    for (uint8_t index = 0U; index < NANOLITE_SENSOR_COUNT; ++index) {
        sensor_yaws[index] = (float)index * 0.78539816339f;
        returns[index].distance_mm = (uint16_t)(800U + 25U * index);
        returns[index].column = (uint8_t)(index % 8U);
        returns[index].sensor_id = index;
        returns[index].timestamp_us = 200U;
        returns[index].valid = true;
    }
    nanolite_scan_config_t config = nanolite_default_scan_config();
    config.min_point_separation_m = 0.005f;
    assert(nanolite_scan_builder_add_returns(
               &builder, &reference, &reference, returns,
               NANOLITE_SENSOR_COUNT, sensor_yaws, 0.78539816339f, true,
               &config) == NANOLITE_SENSOR_COUNT);

    nanolite_pose_t moved = pose(1.05f, 2.0f, 0.2f, 300U);
    for (uint8_t index = 0U; index < NANOLITE_SENSOR_COUNT; ++index) {
        returns[index].timestamp_us = 300U;
    }
    assert(nanolite_scan_builder_add_returns(
               &builder, &reference, &moved, returns, NANOLITE_SENSOR_COUNT,
               sensor_yaws, 0.78539816339f, true, &config) > 0U);
    nanolite_scan_t scan;
    assert(nanolite_scan_builder_finish(&builder, &config, &scan));
    assert(scan.point_count >= config.min_points);
    assert(scan.pose_index == 0U);
}

static void make_l_shape_scan(nanolite_scan_t *scan,
                              uint8_t pose_index,
                              const nanolite_pose_t *actual_pose)
{
    memset(scan, 0, sizeof(*scan));
    scan->pose_index = pose_index;
    scan->valid = true;
    const float cosine = cosf(actual_pose->yaw_rad);
    const float sine = sinf(actual_pose->yaw_rad);
    for (uint16_t index = 0U; index < 15U; ++index) {
        const float world_x = 0.1f * (float)index;
        const float world_y = 0.0f;
        const float dx = world_x - actual_pose->x_m;
        const float dy = world_y - actual_pose->y_m;
        scan->points[scan->point_count].x_m = cosine * dx + sine * dy;
        scan->points[scan->point_count].y_m = -sine * dx + cosine * dy;
        ++scan->point_count;
    }
    for (uint16_t index = 1U; index <= 15U; ++index) {
        const float world_x = 0.0f;
        const float world_y = 0.1f * (float)index;
        const float dx = world_x - actual_pose->x_m;
        const float dy = world_y - actual_pose->y_m;
        scan->points[scan->point_count].x_m = cosine * dx + sine * dy;
        scan->points[scan->point_count].y_m = -sine * dx + cosine * dy;
        ++scan->point_count;
    }
}

static void test_icp_recovers_pose_correction(void)
{
    nanolite_pose_t target_pose = pose(0.0f, 0.0f, 0.0f, 100U);
    target_pose.sequence = 0U;
    nanolite_pose_t actual_source_pose = pose(0.20f, -0.10f, 0.08f, 200U);
    actual_source_pose.sequence = 8U;
    nanolite_pose_t estimated_source_pose =
        pose(0.32f, -0.02f, 0.14f, 200U);
    estimated_source_pose.sequence = 8U;

    nanolite_scan_t target;
    nanolite_scan_t source;
    make_l_shape_scan(&target, 0U, &target_pose);
    make_l_shape_scan(&source, 8U, &actual_source_pose);

    nanolite_icp_workspace_t workspace;
    nanolite_icp_result_t result;
    assert(nanolite_icp_match(&source, &estimated_source_pose, &target,
                              &target_pose, NULL, &workspace, &result));
    assert(result.accepted);
    assert(result.final_rmse_m < result.initial_rmse_m);
    assert(fabsf(result.corrected_source_pose.x_m - actual_source_pose.x_m) <
           0.04f);
    assert(fabsf(result.corrected_source_pose.y_m - actual_source_pose.y_m) <
           0.04f);
    assert(fabsf(nanolite_wrap_angle(result.corrected_source_pose.yaw_rad -
                                     actual_source_pose.yaw_rad)) < 0.04f);
}

static void test_pose_graph_optimizer_reduces_loop_error(void)
{
    nanolite_graph_t graph;
    nanolite_graph_config_t graph_config = nanolite_default_graph_config();
    graph_config.key_translation_m = 0.1f;
    nanolite_pose_t anchor = pose(0.0f, 0.0f, 0.0f, 0U);
    assert(nanolite_graph_init(&graph, &graph_config, &anchor));
    for (uint8_t index = 1U; index <= 4U; ++index) {
        nanolite_pose_t next = pose(1.1f * (float)index, 0.0f, 0.0f,
                                    (uint32_t)index);
        assert(nanolite_graph_observe_pose(&graph, &next, 0U) ==
               NANOLITE_POSE_ADDED);
    }
    assert(nanolite_graph_add_loop_edge(&graph, 0U, 4U, 4.0f, 0.0f, 0.0f));
    nanolite_optimizer_config_t config = nanolite_default_optimizer_config();
    config.max_iterations = 12U;
    nanolite_optimizer_report_t report;
    assert(nanolite_graph_optimize(&graph, &config, &report));
    assert(report.changed);
    assert(report.final_cost < report.initial_cost);
    assert(fabsf(graph.poses[4].x_m - 4.0f) < 0.20f);
    assert(graph.poses[0].x_m == 0.0f);
}

static void test_high_level_package_ingests_data(void)
{
    nanolite_slam_t slam;
    float sensor_yaws[NANOLITE_SENSOR_COUNT];
    for (uint8_t index = 0U; index < NANOLITE_SENSOR_COUNT; ++index) {
        sensor_yaws[index] = (float)index * 0.78539816339f;
    }
    nanolite_pose_t anchor = pose(0.0f, 0.0f, 0.0f, 100U);
    assert(nanolite_slam_init(&slam, NULL, &anchor, sensor_yaws));

    nanolite_tof_return_t returns[NANOLITE_SENSOR_COUNT];
    for (uint8_t index = 0U; index < NANOLITE_SENSOR_COUNT; ++index) {
        returns[index].distance_mm = 1000U;
        returns[index].column = 3U;
        returns[index].sensor_id = index;
        returns[index].timestamp_us = 200U;
        returns[index].valid = true;
    }
    nanolite_pose_t first = pose(0.0f, 0.0f, 0.0f, 200U);
    assert(nanolite_slam_observe(&slam, &first, returns,
                                 NANOLITE_SENSOR_COUNT) ==
           NANOLITE_POSE_SKIPPED);
    assert(slam.map.ray_count == NANOLITE_SENSOR_COUNT);
    assert(slam.scan_builder.scan.point_count == NANOLITE_SENSOR_COUNT);

    for (uint8_t index = 0U; index < NANOLITE_SENSOR_COUNT; ++index) {
        returns[index].timestamp_us = 300U;
        returns[index].distance_mm = 1100U;
    }
    nanolite_pose_t key = pose(0.30f, 0.0f, 0.0f, 300U);
    assert(nanolite_slam_observe(&slam, &key, returns,
                                 NANOLITE_SENSOR_COUNT) ==
           NANOLITE_POSE_ADDED);
    const nanolite_slam_status_t status = nanolite_slam_get_status(&slam);
    assert(status.pose_count == 2U);
    assert(status.stored_scan_count == 0U);
    assert(status.active_scan_points == NANOLITE_SENSOR_COUNT);
    nanolite_pose_t corrected;
    assert(nanolite_slam_correct_odometry_pose(&slam, &key, &corrected));
    assert(fabsf(corrected.x_m - key.x_m) < 1.0e-6f);
    assert(nanolite_slam_storage_bytes() == sizeof(nanolite_slam_t));
}

static void test_timestamp_aligned_return_pose(void)
{
    nanolite_slam_t slam;
    float sensor_yaws[NANOLITE_SENSOR_COUNT] = {0};
    nanolite_slam_config_t config = nanolite_default_slam_config();
    config.graph.key_translation_m = 1.0f;
    config.horizontal_fov_rad = 0.0001f;
    nanolite_pose_t anchor = pose(0.0f, 0.0f, 0.0f, 100U);
    assert(nanolite_slam_init(&slam, &config, &anchor, sensor_yaws));

    nanolite_tof_return_t observation = {
        .distance_mm = 1000U,
        .column = 3U,
        .sensor_id = 0U,
        .timestamp_us = 200U,
        .valid = true,
    };
    const nanolite_pose_t current = pose(0.40f, 0.0f, 0.0f, 300U);
    const nanolite_pose_t capture = pose(-0.40f, 0.0f, 0.0f, 200U);
    assert(nanolite_slam_observe_timed(&slam, &current, &observation,
                                       &capture, 1U) ==
           NANOLITE_POSE_SKIPPED);

    /* The endpoint is at x=0.60 m from the anchor when projected from the
     * capture pose. Using the current pose incorrectly would place it at
     * x=1.40 m, one grid cell farther away. */
    assert(nanolite_map_get_cell(&slam.map, 21U, 20U) ==
           NANOLITE_MAP_HIT_CANDIDATE);
    assert(nanolite_map_get_cell(&slam.map, 22U, 20U) ==
           NANOLITE_MAP_UNKNOWN);
    assert(slam.scan_builder.scan.point_count == 1U);
    assert(fabsf(slam.scan_builder.scan.points[0].x_m - 0.60f) < 0.01f);
}

static void test_end_to_end_loop_pipeline(void)
{
    nanolite_slam_t slam;
    float sensor_yaws[NANOLITE_SENSOR_COUNT] = {0};
    nanolite_slam_config_t config = nanolite_default_slam_config();
    config.graph.key_translation_m = 0.1f;
    nanolite_pose_t anchor = pose(0.0f, 0.0f, 0.0f, 100U);
    assert(nanolite_slam_init(&slam, &config, &anchor, sensor_yaws));

    const float path[8][3] = {
        {1.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f},
        {2.0f, 1.0f, 0.0f}, {2.0f, 2.0f, 0.0f},
        {1.0f, 2.0f, 0.0f}, {0.0f, 2.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}, {0.2f, 0.1f, 0.1f},
    };
    for (uint8_t index = 0U; index < 8U; ++index) {
        nanolite_pose_t next = pose(path[index][0], path[index][1],
                                    path[index][2],
                                    (uint32_t)(200U + index));
        assert(nanolite_slam_observe(&slam, &next, NULL, 0U) ==
               NANOLITE_POSE_ADDED);
    }
    assert(slam.graph.pose_count == 9U);

    nanolite_pose_t origin = pose(0.0f, 0.0f, 0.0f, 100U);
    origin.sequence = 0U;
    nanolite_scan_t target;
    make_l_shape_scan(&target, 0U, &origin);
    assert(nanolite_scan_store_push(&slam.scan_store, &target) >= 0);

    nanolite_pose_t actual_return = pose(0.0f, 0.0f, 0.0f, 300U);
    actual_return.sequence = 8U;
    make_l_shape_scan(&slam.scan_builder.scan, 8U, &actual_return);
    slam.scan_builder.frame_count = 4U;
    slam.scan_builder_active = true;

    assert(nanolite_slam_finalize_active_scan(&slam, true));
    assert(slam.attempted_loop_count == 1U);
    assert(slam.accepted_loop_count == 1U);
    assert(slam.graph.loop_count == 1U);
    assert(slam.map_rebuild_count == 1U);
    assert(slam.last_icp.accepted);
    assert(slam.last_optimizer.final_cost < slam.last_optimizer.initial_cost);
    assert(slam.map.ray_count > 0U);
}

int main(void)
{
    test_pose_selection_and_edges();
    test_capacity_is_explicit();
    test_middle_row_tof_reduction();
    test_map_projection_and_hit_confirmation();
    test_scan_aggregation();
    test_icp_recovers_pose_correction();
    test_pose_graph_optimizer_reduces_loop_error();
    test_high_level_package_ingests_data();
    test_timestamp_aligned_return_pose();
    test_end_to_end_loop_pipeline();
    printf("nanolite tests passed; graph_bytes=%zu map_bytes=%zu "
           "slam_bytes=%zu poses=%u loops=%u\n",
           nanolite_graph_storage_bytes(), nanolite_map_storage_bytes(),
           nanolite_slam_storage_bytes(),
           NANOLITE_MAX_POSES, NANOLITE_MAX_LOOP_EDGES);
    return 0;
}
