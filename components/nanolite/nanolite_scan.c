#include "nanolite_scan.h"

#include <math.h>
#include <string.h>

nanolite_scan_config_t nanolite_default_scan_config(void)
{
    const nanolite_scan_config_t config = {
        .min_point_separation_m = 0.03f,
        .min_points = 12U,
    };
    return config;
}

bool nanolite_scan_builder_init(nanolite_scan_builder_t *builder,
                                uint8_t pose_index,
                                uint32_t timestamp_us)
{
    if (builder == NULL || pose_index >= NANOLITE_MAX_POSES) {
        return false;
    }
    memset(builder, 0, sizeof(*builder));
    builder->scan.pose_index = pose_index;
    builder->scan.timestamp_us = timestamp_us;
    return true;
}

static bool add_point(nanolite_scan_t *scan,
                      float x_m,
                      float y_m,
                      float min_separation_m)
{
    const float min_distance_sq = min_separation_m * min_separation_m;
    for (uint16_t index = 0; index < scan->point_count; ++index) {
        const float dx = scan->points[index].x_m - x_m;
        const float dy = scan->points[index].y_m - y_m;
        if (dx * dx + dy * dy < min_distance_sq) {
            return false;
        }
    }
    if (scan->point_count >= NANOLITE_MAX_SCAN_POINTS) {
        return false;
    }
    scan->points[scan->point_count].x_m = x_m;
    scan->points[scan->point_count].y_m = y_m;
    ++scan->point_count;
    return true;
}

uint16_t nanolite_scan_builder_add_returns(
    nanolite_scan_builder_t *builder,
    const nanolite_pose_t *reference_pose,
    const nanolite_pose_t *observation_pose,
    const nanolite_tof_return_t *returns,
    size_t return_count,
    const float sensor_yaw_rad[NANOLITE_SENSOR_COUNT],
    float horizontal_fov_rad,
    bool column_zero_clockwise,
    const nanolite_scan_config_t *config)
{
    if (builder == NULL || reference_pose == NULL || observation_pose == NULL ||
        returns == NULL || sensor_yaw_rad == NULL ||
        !isfinite(reference_pose->x_m) || !isfinite(reference_pose->y_m) ||
        !isfinite(reference_pose->yaw_rad) ||
        !isfinite(observation_pose->x_m) ||
        !isfinite(observation_pose->y_m) ||
        !isfinite(observation_pose->yaw_rad) ||
        !isfinite(horizontal_fov_rad) || horizontal_fov_rad <= 0.0f) {
        return 0U;
    }

    const nanolite_scan_config_t selected =
        config == NULL ? nanolite_default_scan_config() : *config;
    if (!isfinite(selected.min_point_separation_m) ||
        selected.min_point_separation_m < 0.0f || selected.min_points == 0U ||
        selected.min_points > NANOLITE_MAX_SCAN_POINTS) {
        return 0U;
    }

    const float reference_cosine = cosf(reference_pose->yaw_rad);
    const float reference_sine = sinf(reference_pose->yaw_rad);
    uint16_t added = 0U;
    bool saw_valid_return = false;

    for (size_t index = 0; index < return_count; ++index) {
        const nanolite_tof_return_t *observation = &returns[index];
        if (!observation->valid || observation->distance_mm == 0U ||
            observation->column >= 8U ||
            observation->sensor_id >= NANOLITE_SENSOR_COUNT ||
            !isfinite(sensor_yaw_rad[observation->sensor_id])) {
            continue;
        }

        float column_offset =
            (3.5f - (float)observation->column) * horizontal_fov_rad / 8.0f;
        if (!column_zero_clockwise) {
            column_offset = -column_offset;
        }
        const float bearing = observation_pose->yaw_rad +
                              sensor_yaw_rad[observation->sensor_id] +
                              column_offset;
        const float range_m = (float)observation->distance_mm / 1000.0f;
        const float world_x = observation_pose->x_m + range_m * cosf(bearing);
        const float world_y = observation_pose->y_m + range_m * sinf(bearing);
        const float world_dx = world_x - reference_pose->x_m;
        const float world_dy = world_y - reference_pose->y_m;
        const float local_x =
            reference_cosine * world_dx + reference_sine * world_dy;
        const float local_y =
            -reference_sine * world_dx + reference_cosine * world_dy;
        saw_valid_return = true;
        if (add_point(&builder->scan, local_x, local_y,
                      selected.min_point_separation_m)) {
            ++added;
        }
        if (observation->timestamp_us > builder->scan.timestamp_us) {
            builder->scan.timestamp_us = observation->timestamp_us;
        }
    }
    if (saw_valid_return) {
        ++builder->frame_count;
    }
    return added;
}

bool nanolite_scan_builder_finish(nanolite_scan_builder_t *builder,
                                  const nanolite_scan_config_t *config,
                                  nanolite_scan_t *result)
{
    if (builder == NULL || result == NULL) {
        return false;
    }
    const nanolite_scan_config_t selected =
        config == NULL ? nanolite_default_scan_config() : *config;
    if (selected.min_points == 0U ||
        selected.min_points > NANOLITE_MAX_SCAN_POINTS ||
        builder->scan.point_count < selected.min_points) {
        return false;
    }
    builder->scan.valid = true;
    *result = builder->scan;
    return true;
}

void nanolite_scan_store_init(nanolite_scan_store_t *store)
{
    if (store != NULL) {
        memset(store, 0, sizeof(*store));
    }
}

int nanolite_scan_store_push(nanolite_scan_store_t *store,
                             const nanolite_scan_t *scan)
{
    if (store == NULL || scan == NULL || !scan->valid ||
        scan->point_count == 0U ||
        scan->point_count > NANOLITE_MAX_SCAN_POINTS ||
        scan->pose_index >= NANOLITE_MAX_POSES) {
        return -1;
    }
    const uint8_t slot = store->next_slot;
    store->scans[slot] = *scan;
    store->next_slot = (uint8_t)((slot + 1U) % NANOLITE_MAX_STORED_SCANS);
    if (store->count < NANOLITE_MAX_STORED_SCANS) {
        ++store->count;
    }
    return (int)slot;
}

size_t nanolite_scan_storage_bytes(void)
{
    return sizeof(nanolite_scan_store_t) + sizeof(nanolite_scan_builder_t);
}
