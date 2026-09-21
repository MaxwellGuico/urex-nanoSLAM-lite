#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NANOLITE_TOF_ZONE_COUNT 64U

typedef struct {
    uint16_t min_valid_range_mm;
    uint16_t map_effective_range_mm;
} nanolite_tof_config_t;

typedef struct {
    uint16_t distance_mm;
    uint8_t column;
    uint8_t sensor_id;
    uint32_t timestamp_us;
    bool valid;
} nanolite_tof_return_t;

nanolite_tof_config_t nanolite_default_tof_config(void);

bool nanolite_reduce_tof_frame(const int16_t distance_mm[NANOLITE_TOF_ZONE_COUNT],
                               const uint8_t target_status[NANOLITE_TOF_ZONE_COUNT],
                               uint8_t sensor_id,
                               uint32_t timestamp_us,
                               const nanolite_tof_config_t *config,
                               nanolite_tof_return_t *result);

#ifdef __cplusplus
}
#endif
