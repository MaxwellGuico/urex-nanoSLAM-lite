#include "nanolite_map.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static bool inside(int16_t x, int16_t y)
{
    return x >= 0 && x < (int16_t)NANOLITE_MAP_SIZE &&
           y >= 0 && y < (int16_t)NANOLITE_MAP_SIZE;
}

static uint16_t cell_index(uint8_t x, uint8_t y)
{
    return (uint16_t)y * NANOLITE_MAP_SIZE + x;
}

static void set_cell(nanolite_map_t *map,
                     int16_t x,
                     int16_t y,
                     nanolite_map_cell_t value)
{
    if (!inside(x, y)) {
        return;
    }

    const uint16_t index = cell_index((uint8_t)x, (uint8_t)y);
    const uint8_t shift = (uint8_t)((index & 3U) * 2U);
    const uint8_t previous = (uint8_t)((map->cells[index >> 2U] >> shift) & 3U);
    if (previous == (uint8_t)value) {
        return;
    }

    if (previous == NANOLITE_MAP_UNKNOWN && value != NANOLITE_MAP_UNKNOWN) {
        ++map->observed_cells;
    }
    if (previous == NANOLITE_MAP_HIT_CANDIDATE) {
        --map->candidate_cells;
    }
    if (previous == NANOLITE_MAP_OCCUPIED) {
        --map->occupied_cells;
    }
    if (value == NANOLITE_MAP_HIT_CANDIDATE) {
        ++map->candidate_cells;
    }
    if (value == NANOLITE_MAP_OCCUPIED) {
        ++map->occupied_cells;
    }

    const uint8_t mask = (uint8_t)(3U << shift);
    map->cells[index >> 2U] =
        (uint8_t)((map->cells[index >> 2U] & (uint8_t)~mask) |
                  ((uint8_t)value << shift));
}

static void mark_free(nanolite_map_t *map, int16_t x, int16_t y)
{
    const nanolite_map_cell_t state =
        nanolite_map_get_cell(map, (uint8_t)x, (uint8_t)y);
    if (state == NANOLITE_MAP_UNKNOWN) {
        set_cell(map, x, y, NANOLITE_MAP_FREE);
    }
}

static void mark_hit(nanolite_map_t *map, int16_t x, int16_t y)
{
    const nanolite_map_cell_t state =
        nanolite_map_get_cell(map, (uint8_t)x, (uint8_t)y);
    if (state == NANOLITE_MAP_OCCUPIED) {
        return;
    }
    if (state == NANOLITE_MAP_HIT_CANDIDATE) {
        set_cell(map, x, y, NANOLITE_MAP_OCCUPIED);
    } else {
        set_cell(map, x, y, NANOLITE_MAP_HIT_CANDIDATE);
    }
}

bool nanolite_map_init(nanolite_map_t *map, float anchor_x_m, float anchor_y_m)
{
    if (map == NULL || !isfinite(anchor_x_m) || !isfinite(anchor_y_m)) {
        return false;
    }
    memset(map, 0, sizeof(*map));
    map->anchor_x_m = anchor_x_m;
    map->anchor_y_m = anchor_y_m;
    return true;
}

bool nanolite_map_world_to_cell(const nanolite_map_t *map,
                                float world_x_m,
                                float world_y_m,
                                int16_t *cell_x,
                                int16_t *cell_y)
{
    if (map == NULL || cell_x == NULL || cell_y == NULL ||
        !isfinite(world_x_m) || !isfinite(world_y_m)) {
        return false;
    }

    const float center = (float)NANOLITE_MAP_SIZE * 0.5f;
    const int16_t x = (int16_t)floorf(
        (world_x_m - map->anchor_x_m) / NANOLITE_MAP_RESOLUTION_M + center);
    const int16_t y = (int16_t)floorf(
        (world_y_m - map->anchor_y_m) / NANOLITE_MAP_RESOLUTION_M + center);
    *cell_x = x;
    *cell_y = y;
    return inside(x, y);
}

static bool integrate_ray(nanolite_map_t *map,
                          float start_x_m,
                          float start_y_m,
                          float end_x_m,
                          float end_y_m)
{
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;
    if (!nanolite_map_world_to_cell(map, start_x_m, start_y_m, &x0, &y0) ||
        !nanolite_map_world_to_cell(map, end_x_m, end_y_m, &x1, &y1)) {
        ++map->dropped_rays;
        return false;
    }

    int16_t x = x0;
    int16_t y = y0;
    const int16_t dx = (int16_t)abs(x1 - x0);
    const int16_t sx = x0 < x1 ? 1 : -1;
    const int16_t dy = (int16_t)-abs(y1 - y0);
    const int16_t sy = y0 < y1 ? 1 : -1;
    int16_t error = (int16_t)(dx + dy);

    for (;;) {
        if (x == x1 && y == y1) {
            mark_hit(map, x, y);
            break;
        }
        mark_free(map, x, y);
        const int16_t twice_error = (int16_t)(2 * error);
        if (twice_error >= dy) {
            error = (int16_t)(error + dy);
            x = (int16_t)(x + sx);
        }
        if (twice_error <= dx) {
            error = (int16_t)(error + dx);
            y = (int16_t)(y + sy);
        }
    }

    ++map->ray_count;
    return true;
}

bool nanolite_map_integrate_tof(nanolite_map_t *map,
                                const nanolite_pose_t *pose,
                                const nanolite_tof_return_t *observation,
                                float sensor_yaw_rad,
                                float horizontal_fov_rad,
                                bool column_zero_clockwise)
{
    if (map == NULL || pose == NULL || observation == NULL ||
        !observation->valid || observation->distance_mm == 0U ||
        observation->column >= 8U || !isfinite(pose->x_m) ||
        !isfinite(pose->y_m) || !isfinite(pose->yaw_rad) ||
        !isfinite(sensor_yaw_rad) || !isfinite(horizontal_fov_rad) ||
        horizontal_fov_rad <= 0.0f) {
        return false;
    }

    float column_offset =
        (3.5f - (float)observation->column) * horizontal_fov_rad / 8.0f;
    if (!column_zero_clockwise) {
        column_offset = -column_offset;
    }
    const float bearing = pose->yaw_rad + sensor_yaw_rad + column_offset;
    const float range_m = (float)observation->distance_mm / 1000.0f;
    const float end_x_m = pose->x_m + range_m * cosf(bearing);
    const float end_y_m = pose->y_m + range_m * sinf(bearing);
    return integrate_ray(map, pose->x_m, pose->y_m, end_x_m, end_y_m);
}

bool nanolite_map_integrate_scan_point(nanolite_map_t *map,
                                       const nanolite_pose_t *pose,
                                       float local_x_m,
                                       float local_y_m)
{
    if (map == NULL || pose == NULL || !isfinite(pose->x_m) ||
        !isfinite(pose->y_m) || !isfinite(pose->yaw_rad) ||
        !isfinite(local_x_m) || !isfinite(local_y_m) ||
        (local_x_m == 0.0f && local_y_m == 0.0f)) {
        return false;
    }
    const float cosine = cosf(pose->yaw_rad);
    const float sine = sinf(pose->yaw_rad);
    const float end_x_m = pose->x_m +
                          cosine * local_x_m - sine * local_y_m;
    const float end_y_m = pose->y_m +
                          sine * local_x_m + cosine * local_y_m;
    return integrate_ray(map, pose->x_m, pose->y_m, end_x_m, end_y_m);
}

nanolite_map_cell_t nanolite_map_get_cell(const nanolite_map_t *map,
                                          uint8_t x,
                                          uint8_t y)
{
    if (map == NULL || x >= NANOLITE_MAP_SIZE || y >= NANOLITE_MAP_SIZE) {
        return NANOLITE_MAP_UNKNOWN;
    }
    const uint16_t index = cell_index(x, y);
    const uint8_t shift = (uint8_t)((index & 3U) * 2U);
    return (nanolite_map_cell_t)((map->cells[index >> 2U] >> shift) & 3U);
}

size_t nanolite_map_storage_bytes(void)
{
    return sizeof(nanolite_map_t);
}
