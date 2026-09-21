#include "nanolite_icp.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define NANOLITE_PI 3.14159265358979323846f

nanolite_icp_config_t nanolite_default_icp_config(void)
{
    const nanolite_icp_config_t config = {
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
    };
    return config;
}

static bool valid_scan(const nanolite_scan_t *scan)
{
    if (scan == NULL || !scan->valid || scan->point_count == 0U ||
        scan->point_count > NANOLITE_MAX_SCAN_POINTS) {
        return false;
    }
    for (uint16_t index = 0; index < scan->point_count; ++index) {
        if (!isfinite(scan->points[index].x_m) ||
            !isfinite(scan->points[index].y_m)) {
            return false;
        }
    }
    return true;
}

static bool valid_pose(const nanolite_pose_t *pose)
{
    return pose != NULL && isfinite(pose->x_m) && isfinite(pose->y_m) &&
           isfinite(pose->yaw_rad);
}

static void local_to_world(const nanolite_scan_t *scan,
                           const nanolite_pose_t *pose,
                           nanolite_point_t *world)
{
    const float cosine = cosf(pose->yaw_rad);
    const float sine = sinf(pose->yaw_rad);
    for (uint16_t index = 0; index < scan->point_count; ++index) {
        world[index].x_m = pose->x_m +
                           cosine * scan->points[index].x_m -
                           sine * scan->points[index].y_m;
        world[index].y_m = pose->y_m +
                           sine * scan->points[index].x_m +
                           cosine * scan->points[index].y_m;
    }
}

static uint16_t find_correspondences(
    const nanolite_point_t *source,
    uint16_t source_count,
    const nanolite_point_t *target,
    uint16_t target_count,
    float max_distance_sq,
    nanolite_icp_workspace_t *workspace,
    float *sum_distance_sq)
{
    uint16_t pairs = 0U;
    float total = 0.0f;
    for (uint16_t source_index = 0; source_index < source_count;
         ++source_index) {
        float best_distance_sq = FLT_MAX;
        uint8_t best_target = 0U;
        for (uint16_t target_index = 0; target_index < target_count;
             ++target_index) {
            const float dx = target[target_index].x_m - source[source_index].x_m;
            const float dy = target[target_index].y_m - source[source_index].y_m;
            const float distance_sq = dx * dx + dy * dy;
            if (distance_sq < best_distance_sq) {
                best_distance_sq = distance_sq;
                best_target = (uint8_t)target_index;
            }
        }
        if (best_distance_sq <= max_distance_sq) {
            workspace->matched[source_index] = 1U;
            workspace->nearest_target[source_index] = best_target;
            total += best_distance_sq;
            ++pairs;
        } else {
            workspace->matched[source_index] = 0U;
        }
    }
    *sum_distance_sq = total;
    return pairs;
}

static nanolite_edge_t relative_edge(uint8_t from_index,
                                     uint8_t to_index,
                                     const nanolite_pose_t *from,
                                     const nanolite_pose_t *to)
{
    const float dx = to->x_m - from->x_m;
    const float dy = to->y_m - from->y_m;
    const float cosine = cosf(from->yaw_rad);
    const float sine = sinf(from->yaw_rad);
    const nanolite_edge_t edge = {
        .from_index = from_index,
        .to_index = to_index,
        .reserved = 0U,
        .dx_m = cosine * dx + sine * dy,
        .dy_m = -sine * dx + cosine * dy,
        .dyaw_rad = nanolite_wrap_angle(to->yaw_rad - from->yaw_rad),
    };
    return edge;
}

bool nanolite_icp_match(const nanolite_scan_t *source,
                        const nanolite_pose_t *source_pose,
                        const nanolite_scan_t *target,
                        const nanolite_pose_t *target_pose,
                        const nanolite_icp_config_t *config,
                        nanolite_icp_workspace_t *workspace,
                        nanolite_icp_result_t *result)
{
    if (!valid_scan(source) || !valid_scan(target) ||
        !valid_pose(source_pose) || !valid_pose(target_pose) ||
        workspace == NULL || result == NULL ||
        source->pose_index != source_pose->sequence ||
        target->pose_index != target_pose->sequence) {
        return false;
    }
    const nanolite_icp_config_t selected =
        config == NULL ? nanolite_default_icp_config() : *config;
    if (selected.max_iterations == 0U || selected.min_pairs < 3U ||
        selected.min_pairs > NANOLITE_MAX_SCAN_POINTS ||
        !isfinite(selected.max_correspondence_m) ||
        selected.max_correspondence_m <= 0.0f ||
        !isfinite(selected.min_overlap) || selected.min_overlap <= 0.0f ||
        selected.min_overlap > 1.0f ||
        !isfinite(selected.max_final_rmse_m) ||
        selected.max_final_rmse_m <= 0.0f ||
        !isfinite(selected.max_translation_correction_m) ||
        selected.max_translation_correction_m <= 0.0f ||
        !isfinite(selected.max_yaw_correction_rad) ||
        selected.max_yaw_correction_rad <= 0.0f) {
        return false;
    }

    memset(result, 0, sizeof(*result));
    result->initial_rmse_m = INFINITY;
    result->final_rmse_m = INFINITY;
    result->corrected_source_pose = *source_pose;
    local_to_world(source, source_pose, workspace->source_world);
    local_to_world(target, target_pose, workspace->target_world);

    const float max_distance_sq = selected.max_correspondence_m *
                                  selected.max_correspondence_m;
    float sum_distance_sq = 0.0f;
    uint16_t pair_count = find_correspondences(
        workspace->source_world, source->point_count,
        workspace->target_world, target->point_count, max_distance_sq,
        workspace, &sum_distance_sq);
    if (pair_count < selected.min_pairs) {
        result->pair_count = pair_count;
        return true;
    }
    result->initial_rmse_m = sqrtf(sum_distance_sq / (float)pair_count);

    for (uint8_t iteration = 0U; iteration < selected.max_iterations;
         ++iteration) {
        float source_cx = 0.0f;
        float source_cy = 0.0f;
        float target_cx = 0.0f;
        float target_cy = 0.0f;
        for (uint16_t index = 0U; index < source->point_count; ++index) {
            if (workspace->matched[index] == 0U) {
                continue;
            }
            const nanolite_point_t *source_point = &workspace->source_world[index];
            const nanolite_point_t *target_point =
                &workspace->target_world[workspace->nearest_target[index]];
            source_cx += source_point->x_m;
            source_cy += source_point->y_m;
            target_cx += target_point->x_m;
            target_cy += target_point->y_m;
        }
        const float inverse_pairs = 1.0f / (float)pair_count;
        source_cx *= inverse_pairs;
        source_cy *= inverse_pairs;
        target_cx *= inverse_pairs;
        target_cy *= inverse_pairs;

        float dot = 0.0f;
        float cross = 0.0f;
        for (uint16_t index = 0U; index < source->point_count; ++index) {
            if (workspace->matched[index] == 0U) {
                continue;
            }
            const nanolite_point_t *source_point = &workspace->source_world[index];
            const nanolite_point_t *target_point =
                &workspace->target_world[workspace->nearest_target[index]];
            const float px = source_point->x_m - source_cx;
            const float py = source_point->y_m - source_cy;
            const float qx = target_point->x_m - target_cx;
            const float qy = target_point->y_m - target_cy;
            dot += px * qx + py * qy;
            cross += px * qy - py * qx;
        }
        const float angle = atan2f(cross, dot);
        const float cosine = cosf(angle);
        const float sine = sinf(angle);
        const float translate_x = target_cx -
                                  (cosine * source_cx - sine * source_cy);
        const float translate_y = target_cy -
                                  (sine * source_cx + cosine * source_cy);

        for (uint16_t index = 0U; index < source->point_count; ++index) {
            const float x = workspace->source_world[index].x_m;
            const float y = workspace->source_world[index].y_m;
            workspace->source_world[index].x_m =
                cosine * x - sine * y + translate_x;
            workspace->source_world[index].y_m =
                sine * x + cosine * y + translate_y;
        }
        const float pose_x = result->corrected_source_pose.x_m;
        const float pose_y = result->corrected_source_pose.y_m;
        result->corrected_source_pose.x_m =
            cosine * pose_x - sine * pose_y + translate_x;
        result->corrected_source_pose.y_m =
            sine * pose_x + cosine * pose_y + translate_y;
        result->corrected_source_pose.yaw_rad = nanolite_wrap_angle(
            result->corrected_source_pose.yaw_rad + angle);
        result->iterations = (uint8_t)(iteration + 1U);

        pair_count = find_correspondences(
            workspace->source_world, source->point_count,
            workspace->target_world, target->point_count, max_distance_sq,
            workspace, &sum_distance_sq);
        if (pair_count < selected.min_pairs) {
            break;
        }
        if (hypotf(translate_x, translate_y) <=
                selected.translation_epsilon_m &&
            fabsf(angle) <= selected.yaw_epsilon_rad) {
            result->converged = true;
            break;
        }
    }

    result->pair_count = pair_count;
    result->overlap = (float)pair_count / (float)source->point_count;
    if (pair_count >= selected.min_pairs) {
        result->final_rmse_m = sqrtf(sum_distance_sq / (float)pair_count);
    }
    result->translation_correction_m = hypotf(
        result->corrected_source_pose.x_m - source_pose->x_m,
        result->corrected_source_pose.y_m - source_pose->y_m);
    result->yaw_correction_rad = fabsf(nanolite_wrap_angle(
        result->corrected_source_pose.yaw_rad - source_pose->yaw_rad));
    result->loop_edge = relative_edge(
        target->pose_index, source->pose_index, target_pose,
        &result->corrected_source_pose);

    result->accepted = pair_count >= selected.min_pairs &&
                       result->overlap >= selected.min_overlap &&
                       result->final_rmse_m <= selected.max_final_rmse_m &&
                       result->translation_correction_m <=
                           selected.max_translation_correction_m &&
                       result->yaw_correction_rad <=
                           selected.max_yaw_correction_rad &&
                       result->initial_rmse_m - result->final_rmse_m >=
                           selected.min_rmse_improvement_m;
    return true;
}
