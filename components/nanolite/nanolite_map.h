#pragma once

#include "nanolite_core.h"
#include "nanolite_tof.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NANOLITE_MAP_SIZE 40U
#define NANOLITE_MAP_CELL_COUNT (NANOLITE_MAP_SIZE * NANOLITE_MAP_SIZE)
#define NANOLITE_MAP_PACKED_BYTES (NANOLITE_MAP_CELL_COUNT / 4U)
#define NANOLITE_MAP_RESOLUTION_M 0.5f

typedef enum {
    NANOLITE_MAP_UNKNOWN = 0,
    NANOLITE_MAP_FREE = 1,
    NANOLITE_MAP_HIT_CANDIDATE = 2,
    NANOLITE_MAP_OCCUPIED = 3,
} nanolite_map_cell_t;

typedef struct {
    uint8_t cells[NANOLITE_MAP_PACKED_BYTES];
    float anchor_x_m;
    float anchor_y_m;
    uint16_t observed_cells;
    uint16_t candidate_cells;
    uint16_t occupied_cells;
    uint32_t ray_count;
    uint32_t dropped_rays;
} nanolite_map_t;

bool nanolite_map_init(nanolite_map_t *map, float anchor_x_m, float anchor_y_m);

/* Project one reduced ToF return and integrate it into the packed grid.
 * Coordinates follow PX4 local NED in the horizontal plane: +x North,
 * +y East, and positive yaw clockwise. column_zero_clockwise describes the
 * physical sensor image orientation. */
bool nanolite_map_integrate_tof(nanolite_map_t *map,
                                const nanolite_pose_t *pose,
                                const nanolite_tof_return_t *observation,
                                float sensor_yaw_rad,
                                float horizontal_fov_rad,
                                bool column_zero_clockwise);

/* Integrate an endpoint already expressed in the pose's local frame. This is
 * used to regenerate the grid after pose-graph correction. */
bool nanolite_map_integrate_scan_point(nanolite_map_t *map,
                                       const nanolite_pose_t *pose,
                                       float point_x_m,
                                       float point_y_m);

/* Integrate an endpoint expressed in the pose's local frame. This is used to
 * regenerate the map from retained scans after graph correction. */
bool nanolite_map_integrate_scan_point(nanolite_map_t *map,
                                       const nanolite_pose_t *pose,
                                       float local_x_m,
                                       float local_y_m);

nanolite_map_cell_t nanolite_map_get_cell(const nanolite_map_t *map,
                                          uint8_t x,
                                          uint8_t y);

bool nanolite_map_world_to_cell(const nanolite_map_t *map,
                                float world_x_m,
                                float world_y_m,
                                int16_t *cell_x,
                                int16_t *cell_y);

size_t nanolite_map_storage_bytes(void);

#ifdef __cplusplus
}
#endif
