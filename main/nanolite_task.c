#include "nanolite_task.h"

#include "mavlink_task.h"
#include "tof_task.h"

#include "nanolite_slam.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>

static const char *TAG = "nanolite";

#define NANOLITE_PI 3.14159265358979323846f
#define NANOLITE_POSE_HISTORY_CAPACITY 16U
#define NANOLITE_POSE_FUTURE_TOLERANCE_US 60000U

static nanolite_slam_t s_slam;
static SemaphoreHandle_t s_graph_mutex;
static bool s_initialized;
static uint32_t s_stale_samples;
static uint32_t s_rejected_samples;
static uint32_t s_last_observe_us;
static uint32_t s_maximum_observe_us;
static uint32_t s_unaligned_tof_returns;
static nanolite_pose_t s_pose_history[NANOLITE_POSE_HISTORY_CAPACITY];
static uint8_t s_pose_history_count;

#if CONFIG_NANOLITE_MAP_SERIAL_DUMP
static char s_map_hex[NANOLITE_MAP_PACKED_BYTES * 2U + 1U];
static nanolite_map_t s_map_snapshot;
#if CONFIG_NANOLITE_GRAPH_SERIAL_DUMP
static nanolite_graph_t s_graph_snapshot;
#endif
static uint32_t s_snapshot_sequence;
#endif

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void pose_history_reset(const nanolite_pose_t *pose)
{
    s_pose_history_count = 0U;
    if (pose != NULL) {
        s_pose_history[0] = *pose;
        s_pose_history_count = 1U;
    }
}

static void pose_history_push(const nanolite_pose_t *pose)
{
    if (pose == NULL) {
        return;
    }
    if (s_pose_history_count > 0U &&
        pose->timestamp_us <=
            s_pose_history[s_pose_history_count - 1U].timestamp_us) {
        return;
    }
    if (s_pose_history_count < NANOLITE_POSE_HISTORY_CAPACITY) {
        s_pose_history[s_pose_history_count++] = *pose;
        return;
    }
    memmove(&s_pose_history[0], &s_pose_history[1],
            (NANOLITE_POSE_HISTORY_CAPACITY - 1U) *
                sizeof(s_pose_history[0]));
    s_pose_history[NANOLITE_POSE_HISTORY_CAPACITY - 1U] = *pose;
}

static bool pose_history_interpolate(uint32_t timestamp_us,
                                     nanolite_pose_t *interpolated)
{
    if (interpolated == NULL || s_pose_history_count == 0U) {
        return false;
    }
    const nanolite_pose_t *oldest = &s_pose_history[0];
    const nanolite_pose_t *newest =
        &s_pose_history[s_pose_history_count - 1U];
    if (timestamp_us < oldest->timestamp_us) {
        return false;
    }
    if (timestamp_us >= newest->timestamp_us) {
        if (timestamp_us - newest->timestamp_us >
            NANOLITE_POSE_FUTURE_TOLERANCE_US) {
            return false;
        }
        *interpolated = *newest;
        interpolated->timestamp_us = timestamp_us;
        return true;
    }

    for (uint8_t index = 1U; index < s_pose_history_count; ++index) {
        const nanolite_pose_t *after = &s_pose_history[index];
        if (timestamp_us > after->timestamp_us) {
            continue;
        }
        const nanolite_pose_t *before = &s_pose_history[index - 1U];
        const uint32_t interval_us =
            after->timestamp_us - before->timestamp_us;
        if (interval_us == 0U) {
            return false;
        }
        const float alpha =
            (float)(timestamp_us - before->timestamp_us) /
            (float)interval_us;
        *interpolated = *before;
        interpolated->x_m =
            before->x_m + alpha * (after->x_m - before->x_m);
        interpolated->y_m =
            before->y_m + alpha * (after->y_m - before->y_m);
        interpolated->yaw_rad = nanolite_wrap_angle(
            before->yaw_rad + alpha * nanolite_wrap_angle(
                after->yaw_rad - before->yaw_rad));
        interpolated->timestamp_us = timestamp_us;
        interpolated->sequence = 0U;
        interpolated->flags = 0U;
        interpolated->reserved = 0U;
        return true;
    }
    return false;
}

#if CONFIG_NANOLITE_TOF_BENCH_TELEMETRY
static uint32_t tof_age_ms(uint32_t time_ms,
                           const nanolite_tof_return_t *observation)
{
    if (observation->timestamp_us == 0U) {
        return UINT32_MAX;
    }
    return time_ms - observation->timestamp_us / 1000U;
}

static void log_tof_bench_snapshot(
    uint32_t sequence,
    uint32_t time_ms,
    const nanolite_tof_return_t returns[TOF_SENSOR_COUNT])
{
    uint32_t age_ms[TOF_SENSOR_COUNT];
    uint8_t valid_mask = 0U;
    uint8_t fresh_mask = 0U;

    for (uint8_t sensor = 0; sensor < TOF_SENSOR_COUNT; ++sensor) {
        age_ms[sensor] = tof_age_ms(time_ms, &returns[sensor]);
        if (returns[sensor].valid) {
            valid_mask |= (uint8_t)(1U << sensor);
            if (age_ms[sensor] <=
                (uint32_t)CONFIG_NANOLITE_MAP_TOF_MAX_AGE_MS) {
                fresh_mask |= (uint8_t)(1U << sensor);
            }
        }
    }

    ESP_LOGI(TAG,
             "NANOTOF v=1 seq=%lu now_ms=%lu gate_mm=%d "
             "valid=0x%02x fresh=0x%02x "
             "s0=%u,%u,%lu,%lu s1=%u,%u,%lu,%lu "
             "s2=%u,%u,%lu,%lu s3=%u,%u,%lu,%lu "
             "s4=%u,%u,%lu,%lu s5=%u,%u,%lu,%lu "
             "s6=%u,%u,%lu,%lu s7=%u,%u,%lu,%lu",
             (unsigned long)sequence,
             (unsigned long)time_ms,
             CONFIG_NANOLITE_MAP_EFFECTIVE_RANGE_MM,
             (unsigned)valid_mask,
             (unsigned)fresh_mask,
             (unsigned)returns[0].distance_mm,
             (unsigned)returns[0].column,
             (unsigned long)returns[0].timestamp_us,
             (unsigned long)age_ms[0],
             (unsigned)returns[1].distance_mm,
             (unsigned)returns[1].column,
             (unsigned long)returns[1].timestamp_us,
             (unsigned long)age_ms[1],
             (unsigned)returns[2].distance_mm,
             (unsigned)returns[2].column,
             (unsigned long)returns[2].timestamp_us,
             (unsigned long)age_ms[2],
             (unsigned)returns[3].distance_mm,
             (unsigned)returns[3].column,
             (unsigned long)returns[3].timestamp_us,
             (unsigned long)age_ms[3],
             (unsigned)returns[4].distance_mm,
             (unsigned)returns[4].column,
             (unsigned long)returns[4].timestamp_us,
             (unsigned long)age_ms[4],
             (unsigned)returns[5].distance_mm,
             (unsigned)returns[5].column,
             (unsigned long)returns[5].timestamp_us,
             (unsigned long)age_ms[5],
             (unsigned)returns[6].distance_mm,
             (unsigned)returns[6].column,
             (unsigned long)returns[6].timestamp_us,
             (unsigned long)age_ms[6],
             (unsigned)returns[7].distance_mm,
             (unsigned)returns[7].column,
             (unsigned long)returns[7].timestamp_us,
             (unsigned long)age_ms[7]);
}
#endif

static bool telemetry_is_fresh(const drone_state_t *state, uint32_t time_ms)
{
    if (state == NULL || state->last_pos_ms == 0 || state->last_att_ms == 0 ||
        !isfinite(state->x) || !isfinite(state->y) ||
        !isfinite(state->heading)) {
        return false;
    }

    return (uint32_t)(time_ms - state->last_pos_ms) <=
               (uint32_t)CONFIG_NANOLITE_POSE_MAX_AGE_MS &&
           (uint32_t)(time_ms - state->last_att_ms) <=
               (uint32_t)CONFIG_NANOLITE_POSE_MAX_AGE_MS;
}

static nanolite_pose_t pose_from_px4(const drone_state_t *state,
                                     uint32_t time_ms)
{
    nanolite_pose_t pose = {
        .x_m = state->x,
        .y_m = state->y,
        .yaw_rad = state->heading,
        /* The graph uses a compact 32-bit timestamp. At microsecond units it
         * wraps after about 71 minutes, longer than the intended mission. */
        .timestamp_us = time_ms * 1000U,
        .sequence = 0,
        .flags = 0,
        .reserved = 0,
    };
    return pose;
}

static bool reset_graph(const nanolite_pose_t *anchor)
{
    nanolite_slam_config_t config = nanolite_default_slam_config();
    config.graph.key_translation_m =
        (float)CONFIG_NANOLITE_KEY_TRANSLATION_CM / 100.0f;
    config.graph.key_yaw_rad =
        (float)CONFIG_NANOLITE_KEY_YAW_DEG * 3.14159265358979323846f / 180.0f;
    float sensor_yaw_rad[NANOLITE_SENSOR_COUNT];
    for (uint8_t sensor = 0U; sensor < NANOLITE_SENSOR_COUNT; ++sensor) {
        sensor_yaw_rad[sensor] =
            tof_sensor_angle_deg(sensor) * NANOLITE_PI / 180.0f;
    }

    s_initialized = nanolite_slam_init(&s_slam, &config, anchor,
                                       sensor_yaw_rad);
    if (s_initialized) {
        ESP_LOGI(TAG,
                 "graph/map reset: anchor=(%.2f,%.2f,%.1fdeg) "
                 "poses=%u package=%uB scans=%u points/scan=%u",
                 anchor->x_m, anchor->y_m,
                 anchor->yaw_rad * 180.0f / NANOLITE_PI,
                 (unsigned)NANOLITE_MAX_POSES,
                 (unsigned)nanolite_slam_storage_bytes(),
                 (unsigned)NANOLITE_MAX_STORED_SCANS,
                 (unsigned)NANOLITE_MAX_SCAN_POINTS);
    }
    return s_initialized;
}

#if CONFIG_NANOLITE_MAP_SERIAL_DUMP
static void log_bench_snapshot(const nanolite_pose_t *pose)
{
    static const char digits[] = "0123456789abcdef";
    int16_t robot_x = -1;
    int16_t robot_y = -1;
    uint32_t sequence;

    /* Copy the map and graph in one short critical section. Logging the
     * potentially large snapshot happens after releasing the mutex so status
     * readers never wait on UART output. */
    xSemaphoreTake(s_graph_mutex, portMAX_DELAY);
    s_map_snapshot = s_slam.map;
#if CONFIG_NANOLITE_GRAPH_SERIAL_DUMP
    s_graph_snapshot = s_slam.graph;
#endif
    ++s_snapshot_sequence;
    sequence = s_snapshot_sequence;
    (void)nanolite_map_world_to_cell(&s_map_snapshot, pose->x_m, pose->y_m,
                                     &robot_x, &robot_y);
    xSemaphoreGive(s_graph_mutex);

    for (size_t index = 0; index < NANOLITE_MAP_PACKED_BYTES; ++index) {
        const uint8_t byte = s_map_snapshot.cells[index];
        s_map_hex[index * 2U] = digits[byte >> 4U];
        s_map_hex[index * 2U + 1U] = digits[byte & 0x0fU];
    }
    s_map_hex[NANOLITE_MAP_PACKED_BYTES * 2U] = '\0';

    ESP_LOGI(TAG,
             "NANOMAP v=1 seq=%lu anchor=%.2f,%.2f robot=%d,%d "
             "size=%u res_cm=50 rays=%lu cells=%s",
             (unsigned long)sequence,
             s_map_snapshot.anchor_x_m, s_map_snapshot.anchor_y_m,
             (int)robot_x, (int)robot_y,
             (unsigned)NANOLITE_MAP_SIZE,
             (unsigned long)s_map_snapshot.ray_count,
             s_map_hex);

#if CONFIG_NANOLITE_GRAPH_SERIAL_DUMP
    ESP_LOGI(TAG,
             "NANOGRAPH_BEGIN v=1 seq=%lu poses=%u odom=%u loops=%u "
             "live=%.3f,%.3f,%.5f",
             (unsigned long)sequence,
             (unsigned)s_graph_snapshot.pose_count,
             (unsigned)s_graph_snapshot.odometry_count,
             (unsigned)s_graph_snapshot.loop_count,
             pose->x_m, pose->y_m, pose->yaw_rad);

    for (uint8_t index = 0U; index < s_graph_snapshot.pose_count; ++index) {
        const nanolite_pose_t *node = &s_graph_snapshot.poses[index];
        ESP_LOGI(TAG,
                 "NANONODE v=1 seq=%lu i=%u x=%.3f y=%.3f yaw=%.5f "
                 "t_us=%lu flags=0x%02x",
                 (unsigned long)sequence,
                 (unsigned)index,
                 node->x_m, node->y_m, node->yaw_rad,
                 (unsigned long)node->timestamp_us,
                 (unsigned)node->flags);
    }

    for (uint8_t index = 0U; index < s_graph_snapshot.odometry_count; ++index) {
        const nanolite_edge_t *edge = &s_graph_snapshot.odometry[index];
        ESP_LOGI(TAG,
                 "NANOEDGE v=1 seq=%lu kind=odom i=%u from=%u to=%u "
                 "dx=%.3f dy=%.3f dyaw=%.5f",
                 (unsigned long)sequence,
                 (unsigned)index,
                 (unsigned)edge->from_index,
                 (unsigned)edge->to_index,
                 edge->dx_m, edge->dy_m, edge->dyaw_rad);
    }

    for (uint8_t index = 0U; index < s_graph_snapshot.loop_count; ++index) {
        const nanolite_edge_t *edge = &s_graph_snapshot.loop_edges[index];
        ESP_LOGI(TAG,
                 "NANOEDGE v=1 seq=%lu kind=loop i=%u from=%u to=%u "
                 "dx=%.3f dy=%.3f dyaw=%.5f",
                 (unsigned long)sequence,
                 (unsigned)index,
                 (unsigned)edge->from_index,
                 (unsigned)edge->to_index,
                 edge->dx_m, edge->dy_m, edge->dyaw_rad);
    }

    ESP_LOGI(TAG, "NANOGRAPH_END v=1 seq=%lu",
             (unsigned long)sequence);
#endif
}
#endif

void nanolite_task_init(void)
{
    memset(&s_slam, 0, sizeof(s_slam));
    s_initialized = false;
    s_stale_samples = 0;
    s_rejected_samples = 0;
    s_last_observe_us = 0;
    s_maximum_observe_us = 0;
    s_unaligned_tof_returns = 0;
    pose_history_reset(NULL);
#if CONFIG_NANOLITE_MAP_SERIAL_DUMP
    memset(s_map_hex, 0, sizeof(s_map_hex));
    memset(&s_map_snapshot, 0, sizeof(s_map_snapshot));
#if CONFIG_NANOLITE_GRAPH_SERIAL_DUMP
    memset(&s_graph_snapshot, 0, sizeof(s_graph_snapshot));
#endif
    s_snapshot_sequence = 0;
#endif
    s_graph_mutex = xSemaphoreCreateMutex();
    configASSERT(s_graph_mutex != NULL);
}

nanolite_task_status_t nanolite_task_get_status(void)
{
    nanolite_task_status_t status;
    xSemaphoreTake(s_graph_mutex, portMAX_DELAY);
    status.pose_count = s_slam.graph.pose_count;
    status.odometry_count = s_slam.graph.odometry_count;
    status.loop_count = s_slam.graph.loop_count;
    status.initialized = s_initialized;
    status.saturated = s_slam.graph.saturated;
    status.stale_samples = s_stale_samples;
    status.rejected_samples = s_rejected_samples;
    status.keyscan_count = s_slam.scan_store.count;
    status.active_scan_points = s_slam.scan_builder.scan.point_count;
    status.loop_attempts = s_slam.attempted_loop_count;
    status.loop_accepts = s_slam.accepted_loop_count;
    status.loop_rejects = s_slam.rejected_loop_count;
    status.map_rebuilds = s_slam.map_rebuild_count;
    status.map_observed_cells = s_slam.map.observed_cells;
    status.map_candidate_cells = s_slam.map.candidate_cells;
    status.map_occupied_cells = s_slam.map.occupied_cells;
    status.map_ray_count = s_slam.map.ray_count;
    status.map_dropped_rays = s_slam.map.dropped_rays;
    status.last_observe_us = s_last_observe_us;
    status.maximum_observe_us = s_maximum_observe_us;
    status.unaligned_tof_returns = s_unaligned_tof_returns;
    xSemaphoreGive(s_graph_mutex);
    return status;
}

bool nanolite_task_copy_map(nanolite_map_t *destination)
{
    if (destination == NULL) {
        return false;
    }
    xSemaphoreTake(s_graph_mutex, portMAX_DELAY);
    const bool available = s_initialized;
    if (available) {
        *destination = s_slam.map;
    }
    xSemaphoreGive(s_graph_mutex);
    return available;
}

bool nanolite_task_copy_graph(nanolite_graph_t *destination)
{
    if (destination == NULL) {
        return false;
    }
    xSemaphoreTake(s_graph_mutex, portMAX_DELAY);
    const bool available = s_initialized;
    if (available) {
        *destination = s_slam.graph;
    }
    xSemaphoreGive(s_graph_mutex);
    return available;
}

void nanolite_task(void *arg)
{
    (void)arg;
    const TickType_t period =
        pdMS_TO_TICKS(1000U / (uint32_t)CONFIG_NANOLITE_POSE_RATE_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    bool was_armed = false;
    uint32_t last_stale_log_ms = 0;
    uint32_t last_full_log_ms = 0;
    uint32_t last_map_log_ms = 0;
#if CONFIG_NANOLITE_TOF_BENCH_TELEMETRY
    uint32_t last_tof_bench_log_ms = 0;
    uint32_t tof_bench_sequence = 0;
#endif
#if CONFIG_NANOLITE_MAP_SERIAL_DUMP
    uint32_t last_map_dump_ms = 0;
#endif

    ESP_LOGI(TAG,
             "read-only PX4 pose bridge ready: %dHz max_age=%dms key=%dcm/%ddeg",
             CONFIG_NANOLITE_POSE_RATE_HZ,
             CONFIG_NANOLITE_POSE_MAX_AGE_MS,
             CONFIG_NANOLITE_KEY_TRANSLATION_CM,
             CONFIG_NANOLITE_KEY_YAW_DEG);

#if CONFIG_NANOLITE_TOF_BENCH_TELEMETRY
    ESP_LOGI(TAG,
             "NANOTOF enabled: interval=%dms "
             "tuple=distance_mm,column,timestamp_us,age_ms "
             "yaw_deg=s0:%.0f,s1:%.0f,s2:%.0f,s3:%.0f,"
             "s4:%.0f,s5:%.0f,s6:%.0f,s7:%.0f",
             CONFIG_NANOLITE_TOF_BENCH_INTERVAL_MS,
             tof_sensor_angle_deg(0U), tof_sensor_angle_deg(1U),
             tof_sensor_angle_deg(2U), tof_sensor_angle_deg(3U),
             tof_sensor_angle_deg(4U), tof_sensor_angle_deg(5U),
             tof_sensor_angle_deg(6U), tof_sensor_angle_deg(7U));
#endif

    for (;;) {
        const drone_state_t state = mavlink_get_state();
        /* Sample time after copying telemetry so a higher-priority MAVLink
         * update cannot make the copied timestamps appear to be in the future. */
        const uint32_t time_ms = now_ms();

        if (!telemetry_is_fresh(&state, time_ms)) {
            xSemaphoreTake(s_graph_mutex, portMAX_DELAY);
            ++s_stale_samples;
            xSemaphoreGive(s_graph_mutex);
            if ((uint32_t)(time_ms - last_stale_log_ms) >= 5000U) {
                last_stale_log_ms = time_ms;
                ESP_LOGW(TAG, "waiting for fresh PX4 position and attitude");
            }
            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        const nanolite_pose_t pose = pose_from_px4(&state, time_ms);
        const bool graph_reset_requested =
            !s_initialized || (state.armed && !was_armed);
        if (graph_reset_requested) {
            pose_history_reset(&pose);
        } else {
            pose_history_push(&pose);
        }
        nanolite_tof_return_t returns[TOF_SENSOR_COUNT];
        nanolite_pose_t return_poses[TOF_SENSOR_COUNT];
        memset(return_poses, 0, sizeof(return_poses));
        (void)tof_get_mapping_returns(CONFIG_NANOLITE_MAP_EFFECTIVE_RANGE_MM,
                                      returns);
        /* The ToF task may publish a frame while this task is waiting for its
         * scan mutex, so use a post-snapshot time for ToF age checks. */
        const uint32_t mapping_time_ms = now_ms();
#if CONFIG_NANOLITE_TOF_BENCH_TELEMETRY
        if ((uint32_t)(mapping_time_ms - last_tof_bench_log_ms) >=
            (uint32_t)CONFIG_NANOLITE_TOF_BENCH_INTERVAL_MS) {
            last_tof_bench_log_ms = mapping_time_ms;
            ++tof_bench_sequence;
            log_tof_bench_snapshot(tof_bench_sequence, mapping_time_ms,
                                   returns);
        }
#endif
        uint8_t scan_mask = 0U;
        uint32_t unaligned_this_cycle = 0U;
        for (uint8_t sensor = 0U; sensor < TOF_SENSOR_COUNT; ++sensor) {
            const uint32_t frame_ms = returns[sensor].timestamp_us / 1000U;
            if (returns[sensor].timestamp_us == 0U ||
                (uint32_t)(mapping_time_ms - frame_ms) >
                    (uint32_t)CONFIG_NANOLITE_MAP_TOF_MAX_AGE_MS) {
                returns[sensor].valid = false;
            } else if (returns[sensor].valid &&
                       !pose_history_interpolate(returns[sensor].timestamp_us,
                                                 &return_poses[sensor])) {
                returns[sensor].valid = false;
                ++unaligned_this_cycle;
            } else if (returns[sensor].valid) {
                scan_mask |= (uint8_t)(1U << sensor);
            }
        }
        nanolite_pose_result_t result = NANOLITE_POSE_SKIPPED;
        uint8_t pose_count = 0;
        uint32_t map_ray_count = 0;
        uint16_t map_observed = 0;
        uint16_t map_candidates = 0;
        uint16_t map_occupied = 0;
        uint32_t map_dropped = 0;
        uint8_t keyscan_count = 0;
        uint16_t active_scan_points = 0;
        uint32_t loop_attempts = 0;
        uint32_t loop_accepts = 0;
        uint32_t loop_rejects = 0;
        uint32_t map_rebuilds = 0;
        uint32_t observe_us = 0;
        uint32_t maximum_observe_us = 0;
        uint32_t unaligned_tof_returns = 0;
        bool loop_was_attempted = false;
        bool loop_was_accepted = false;
        nanolite_icp_result_t loop_icp = {0};
        nanolite_optimizer_report_t loop_optimizer = {0};
        nanolite_pose_t corrected_pose = pose;
#if CONFIG_NANOLITE_MAP_SERIAL_DUMP
        bool map_available = false;
#endif

        xSemaphoreTake(s_graph_mutex, portMAX_DELAY);
        s_unaligned_tof_returns += unaligned_this_cycle;
        if (graph_reset_requested) {
            if (!reset_graph(&pose)) {
                ++s_rejected_samples;
            }
        }
        if (s_initialized) {
            const uint32_t attempts_before = s_slam.attempted_loop_count;
            const uint32_t accepts_before = s_slam.accepted_loop_count;
            const int64_t observe_start_us = esp_timer_get_time();
            result = nanolite_slam_observe_timed(
                &s_slam, &pose, returns, return_poses, TOF_SENSOR_COUNT);
            const int64_t observe_elapsed_us =
                esp_timer_get_time() - observe_start_us;
            s_last_observe_us = observe_elapsed_us > (int64_t)UINT32_MAX
                                    ? UINT32_MAX
                                    : (uint32_t)observe_elapsed_us;
            if (s_last_observe_us > s_maximum_observe_us) {
                s_maximum_observe_us = s_last_observe_us;
            }
            loop_was_attempted =
                s_slam.attempted_loop_count != attempts_before;
            loop_was_accepted =
                s_slam.accepted_loop_count != accepts_before;
            if (loop_was_attempted) {
                loop_icp = s_slam.last_icp;
                loop_optimizer = s_slam.last_optimizer;
            }
            if (result == NANOLITE_POSE_REJECTED) {
                ++s_rejected_samples;
            }
            (void)nanolite_slam_correct_odometry_pose(
                &s_slam, &pose, &corrected_pose);
        }
        pose_count = s_slam.graph.pose_count;
        map_ray_count = s_slam.map.ray_count;
        map_observed = s_slam.map.observed_cells;
        map_candidates = s_slam.map.candidate_cells;
        map_occupied = s_slam.map.occupied_cells;
        map_dropped = s_slam.map.dropped_rays;
        keyscan_count = s_slam.scan_store.count;
        active_scan_points = s_slam.scan_builder.scan.point_count;
        loop_attempts = s_slam.attempted_loop_count;
        loop_accepts = s_slam.accepted_loop_count;
        loop_rejects = s_slam.rejected_loop_count;
        map_rebuilds = s_slam.map_rebuild_count;
        observe_us = s_last_observe_us;
        maximum_observe_us = s_maximum_observe_us;
        unaligned_tof_returns = s_unaligned_tof_returns;
#if CONFIG_NANOLITE_MAP_SERIAL_DUMP
        map_available = s_initialized;
#endif
        xSemaphoreGive(s_graph_mutex);

        if (result == NANOLITE_POSE_ADDED) {
            ESP_LOGI(TAG,
                     "pose %u/%u: N=%.2f E=%.2f yaw=%.1fdeg scan=0x%02x",
                     (unsigned)pose_count, (unsigned)NANOLITE_MAX_POSES,
                     corrected_pose.x_m, corrected_pose.y_m,
                     corrected_pose.yaw_rad * 180.0f / NANOLITE_PI,
                     (unsigned)scan_mask);
        } else if (result == NANOLITE_POSE_FULL &&
                   (uint32_t)(time_ms - last_full_log_ms) >= 5000U) {
            last_full_log_ms = time_ms;
            ESP_LOGW(TAG, "pose graph full at %u poses; observations paused",
                     (unsigned)pose_count);
        }

        if (loop_was_attempted) {
            if (loop_was_accepted) {
                ESP_LOGI(TAG,
                         "loop accepted %u->%u pairs=%u overlap=%.2f "
                         "rmse=%.3f->%.3f corr=%.2fm/%.1fdeg "
                         "cost=%.4f->%.4f slam_us=%lu",
                         (unsigned)loop_icp.loop_edge.from_index,
                         (unsigned)loop_icp.loop_edge.to_index,
                         (unsigned)loop_icp.pair_count,
                         loop_icp.overlap,
                         loop_icp.initial_rmse_m,
                         loop_icp.final_rmse_m,
                         loop_icp.translation_correction_m,
                         loop_icp.yaw_correction_rad * 180.0f / NANOLITE_PI,
                         loop_optimizer.initial_cost,
                         loop_optimizer.final_cost,
                         (unsigned long)observe_us);
            } else {
                ESP_LOGW(TAG,
                         "loop rejected pairs=%u overlap=%.2f "
                         "rmse=%.3f->%.3f corr=%.2fm/%.1fdeg slam_us=%lu",
                         (unsigned)loop_icp.pair_count,
                         loop_icp.overlap,
                         loop_icp.initial_rmse_m,
                         loop_icp.final_rmse_m,
                         loop_icp.translation_correction_m,
                         loop_icp.yaw_correction_rad * 180.0f / NANOLITE_PI,
                         (unsigned long)observe_us);
            }
        }

        if ((uint32_t)(time_ms - last_map_log_ms) >= 5000U) {
            last_map_log_ms = time_ms;
            ESP_LOGI(TAG,
                     "map: rays=%lu observed=%u candidate=%u occupied=%u "
                     "dropped=%lu scans=%u active_pts=%u loops=%lu/%lu/%lu "
                     "rebuilds=%lu range=%dmm step_us=%lu max_step_us=%lu "
                     "unaligned=%lu",
                     (unsigned long)map_ray_count,
                     (unsigned)map_observed,
                     (unsigned)map_candidates,
                     (unsigned)map_occupied,
                     (unsigned long)map_dropped,
                     (unsigned)keyscan_count,
                     (unsigned)active_scan_points,
                     (unsigned long)loop_accepts,
                     (unsigned long)loop_attempts,
                     (unsigned long)loop_rejects,
                     (unsigned long)map_rebuilds,
                     CONFIG_NANOLITE_MAP_EFFECTIVE_RANGE_MM,
                     (unsigned long)observe_us,
                     (unsigned long)maximum_observe_us,
                     (unsigned long)unaligned_tof_returns);
        }

#if CONFIG_NANOLITE_MAP_SERIAL_DUMP
        if (map_available &&
            (uint32_t)(time_ms - last_map_dump_ms) >=
                (uint32_t)CONFIG_NANOLITE_MAP_DUMP_INTERVAL_S * 1000U) {
            last_map_dump_ms = time_ms;
            log_bench_snapshot(&corrected_pose);
        }
#endif

        was_armed = state.armed;
        vTaskDelayUntil(&last_wake, period);
    }
}
