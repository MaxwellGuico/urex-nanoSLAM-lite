#include "nanolite_optimize.h"

#include <math.h>
#include <string.h>

#define NANOLITE_PI 3.14159265358979323846f

nanolite_optimizer_config_t nanolite_default_optimizer_config(void)
{
    const nanolite_optimizer_config_t config = {
        .max_iterations = 3U,
        .odometry_weight = 1.0f,
        .loop_weight = 4.0f,
        .relaxation = 0.45f,
        .max_translation_step_m = 0.20f,
        .max_yaw_step_rad = 10.0f * NANOLITE_PI / 180.0f,
    };
    return config;
}

static nanolite_pose_t compose(const nanolite_pose_t *pose,
                               const nanolite_edge_t *edge)
{
    const float cosine = cosf(pose->yaw_rad);
    const float sine = sinf(pose->yaw_rad);
    nanolite_pose_t result = *pose;
    result.x_m += cosine * edge->dx_m - sine * edge->dy_m;
    result.y_m += sine * edge->dx_m + cosine * edge->dy_m;
    result.yaw_rad = nanolite_wrap_angle(pose->yaw_rad + edge->dyaw_rad);
    return result;
}

static nanolite_edge_t inverse_edge(const nanolite_edge_t *edge)
{
    const float cosine = cosf(edge->dyaw_rad);
    const float sine = sinf(edge->dyaw_rad);
    const nanolite_edge_t result = {
        .from_index = edge->to_index,
        .to_index = edge->from_index,
        .reserved = 0U,
        .dx_m = -cosine * edge->dx_m - sine * edge->dy_m,
        .dy_m = sine * edge->dx_m - cosine * edge->dy_m,
        .dyaw_rad = -edge->dyaw_rad,
    };
    return result;
}

static float edge_cost(const nanolite_graph_t *graph,
                       const nanolite_edge_t *edge,
                       float weight)
{
    if (edge->from_index >= graph->pose_count ||
        edge->to_index >= graph->pose_count) {
        return 0.0f;
    }
    const nanolite_pose_t predicted =
        compose(&graph->poses[edge->from_index], edge);
    const nanolite_pose_t *actual = &graph->poses[edge->to_index];
    const float dx = predicted.x_m - actual->x_m;
    const float dy = predicted.y_m - actual->y_m;
    const float dyaw = nanolite_wrap_angle(predicted.yaw_rad - actual->yaw_rad);
    return weight * (dx * dx + dy * dy + dyaw * dyaw);
}

static float graph_cost(const nanolite_graph_t *graph,
                        const nanolite_optimizer_config_t *config)
{
    float cost = 0.0f;
    for (uint8_t index = 0U; index < graph->odometry_count; ++index) {
        cost += edge_cost(graph, &graph->odometry[index],
                          config->odometry_weight);
    }
    for (uint8_t index = 0U; index < graph->loop_count; ++index) {
        cost += edge_cost(graph, &graph->loop_edges[index],
                          config->loop_weight);
    }
    return cost;
}

static float clamp_symmetric(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static void accumulate_edge(const nanolite_graph_t *graph,
                            const nanolite_edge_t *edge,
                            float edge_weight,
                            float correction_x[NANOLITE_MAX_POSES],
                            float correction_y[NANOLITE_MAX_POSES],
                            float correction_yaw[NANOLITE_MAX_POSES],
                            float weight[NANOLITE_MAX_POSES])
{
    if (edge->from_index >= graph->pose_count ||
        edge->to_index >= graph->pose_count || edge->from_index == edge->to_index) {
        return;
    }

    const nanolite_pose_t expected_to =
        compose(&graph->poses[edge->from_index], edge);
    const nanolite_pose_t *actual_to = &graph->poses[edge->to_index];
    correction_x[edge->to_index] +=
        edge_weight * (expected_to.x_m - actual_to->x_m);
    correction_y[edge->to_index] +=
        edge_weight * (expected_to.y_m - actual_to->y_m);
    correction_yaw[edge->to_index] += edge_weight * nanolite_wrap_angle(
        expected_to.yaw_rad - actual_to->yaw_rad);
    weight[edge->to_index] += edge_weight;

    const nanolite_edge_t reverse = inverse_edge(edge);
    const nanolite_pose_t expected_from =
        compose(&graph->poses[edge->to_index], &reverse);
    const nanolite_pose_t *actual_from = &graph->poses[edge->from_index];
    correction_x[edge->from_index] +=
        edge_weight * (expected_from.x_m - actual_from->x_m);
    correction_y[edge->from_index] +=
        edge_weight * (expected_from.y_m - actual_from->y_m);
    correction_yaw[edge->from_index] += edge_weight * nanolite_wrap_angle(
        expected_from.yaw_rad - actual_from->yaw_rad);
    weight[edge->from_index] += edge_weight;
}

bool nanolite_graph_optimize(nanolite_graph_t *graph,
                             const nanolite_optimizer_config_t *config,
                             nanolite_optimizer_report_t *report)
{
    if (graph == NULL || graph->pose_count == 0U || report == NULL) {
        return false;
    }
    const nanolite_optimizer_config_t selected =
        config == NULL ? nanolite_default_optimizer_config() : *config;
    if (selected.max_iterations == 0U ||
        !isfinite(selected.odometry_weight) || selected.odometry_weight <= 0.0f ||
        !isfinite(selected.loop_weight) || selected.loop_weight <= 0.0f ||
        !isfinite(selected.relaxation) || selected.relaxation <= 0.0f ||
        selected.relaxation > 1.0f ||
        !isfinite(selected.max_translation_step_m) ||
        selected.max_translation_step_m <= 0.0f ||
        !isfinite(selected.max_yaw_step_rad) ||
        selected.max_yaw_step_rad <= 0.0f) {
        return false;
    }

    memset(report, 0, sizeof(*report));
    report->initial_cost = graph_cost(graph, &selected);

    float correction_x[NANOLITE_MAX_POSES];
    float correction_y[NANOLITE_MAX_POSES];
    float correction_yaw[NANOLITE_MAX_POSES];
    float weight[NANOLITE_MAX_POSES];
    for (uint8_t iteration = 0U; iteration < selected.max_iterations;
         ++iteration) {
        memset(correction_x, 0, sizeof(correction_x));
        memset(correction_y, 0, sizeof(correction_y));
        memset(correction_yaw, 0, sizeof(correction_yaw));
        memset(weight, 0, sizeof(weight));

        for (uint8_t index = 0U; index < graph->odometry_count; ++index) {
            accumulate_edge(graph, &graph->odometry[index],
                            selected.odometry_weight, correction_x, correction_y,
                            correction_yaw, weight);
        }
        for (uint8_t index = 0U; index < graph->loop_count; ++index) {
            accumulate_edge(graph, &graph->loop_edges[index],
                            selected.loop_weight, correction_x, correction_y,
                            correction_yaw, weight);
        }

        float largest_step = 0.0f;
        float largest_yaw_step = 0.0f;
        for (uint8_t index = 1U; index < graph->pose_count; ++index) {
            if (weight[index] <= 0.0f) {
                continue;
            }
            float step_x = selected.relaxation * correction_x[index] / weight[index];
            float step_y = selected.relaxation * correction_y[index] / weight[index];
            const float translation = hypotf(step_x, step_y);
            if (translation > selected.max_translation_step_m) {
                const float scale = selected.max_translation_step_m / translation;
                step_x *= scale;
                step_y *= scale;
            }
            const float step_yaw = clamp_symmetric(
                selected.relaxation * correction_yaw[index] / weight[index],
                selected.max_yaw_step_rad);
            graph->poses[index].x_m += step_x;
            graph->poses[index].y_m += step_y;
            graph->poses[index].yaw_rad = nanolite_wrap_angle(
                graph->poses[index].yaw_rad + step_yaw);
            if (hypotf(step_x, step_y) > largest_step) {
                largest_step = hypotf(step_x, step_y);
            }
            if (fabsf(step_yaw) > largest_yaw_step) {
                largest_yaw_step = fabsf(step_yaw);
            }
            if (step_x != 0.0f || step_y != 0.0f || step_yaw != 0.0f) {
                graph->poses[index].flags |= NANOLITE_POSE_CORRECTED;
                report->changed = true;
            }
        }
        if (largest_step > report->maximum_translation_change_m) {
            report->maximum_translation_change_m = largest_step;
        }
        if (largest_yaw_step > report->maximum_yaw_change_rad) {
            report->maximum_yaw_change_rad = largest_yaw_step;
        }
        report->iterations = (uint8_t)(iteration + 1U);
        if (largest_step < 0.0001f && largest_yaw_step < 0.0001f) {
            break;
        }
    }
    report->final_cost = graph_cost(graph, &selected);
    return true;
}
