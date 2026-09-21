#include "nanolite_tof.h"

#include <limits.h>
#include <stddef.h>

static bool accepted_status(uint8_t status)
{
    return status == 5U || status == 9U;
}

nanolite_tof_config_t nanolite_default_tof_config(void)
{
    const nanolite_tof_config_t config = {
        .min_valid_range_mm = 50U,
        .map_effective_range_mm = 2000U,
    };
    return config;
}

bool nanolite_reduce_tof_frame(const int16_t distance_mm[NANOLITE_TOF_ZONE_COUNT],
                               const uint8_t target_status[NANOLITE_TOF_ZONE_COUNT],
                               uint8_t sensor_id,
                               uint32_t timestamp_us,
                               const nanolite_tof_config_t *config,
                               nanolite_tof_return_t *result)
{
    if (distance_mm == NULL || target_status == NULL || result == NULL ||
        sensor_id >= 8U) {
        return false;
    }

    const nanolite_tof_config_t selected =
        config == NULL ? nanolite_default_tof_config() : *config;
    if (selected.min_valid_range_mm == 0U ||
        selected.map_effective_range_mm < selected.min_valid_range_mm) {
        return false;
    }

    uint16_t best_distance = UINT16_MAX;
    uint8_t best_column = 0;
    for (uint8_t row = 3U; row <= 4U; ++row) {
        for (uint8_t column = 0; column < 8U; ++column) {
            const uint8_t zone = (uint8_t)(row * 8U + column);
            const int16_t distance = distance_mm[zone];
            if (!accepted_status(target_status[zone]) || distance < 0 ||
                (uint16_t)distance < selected.min_valid_range_mm ||
                (uint16_t)distance > selected.map_effective_range_mm) {
                continue;
            }
            if ((uint16_t)distance < best_distance) {
                best_distance = (uint16_t)distance;
                best_column = column;
            }
        }
    }

    result->distance_mm = best_distance == UINT16_MAX ? 0U : best_distance;
    result->column = best_column;
    result->sensor_id = sensor_id;
    result->timestamp_us = timestamp_us;
    result->valid = best_distance != UINT16_MAX;
    return true;
}
