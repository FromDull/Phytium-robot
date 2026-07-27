#include "leg_kinematics.h"

#include <math.h>
#include <stddef.h>

#define LEG_EPSILON 1.0e-6f
#define LEG_POSITION_TOLERANCE_M 1.0e-4f

int leg_kinematics_config_valid(const LegKinematicsConfig *config)
{
    return config != NULL &&
        isfinite(config->base_spacing_m) && config->base_spacing_m > 0.0f &&
        isfinite(config->upper_link_m) && config->upper_link_m > 0.0f &&
        isfinite(config->lower_link_m) && config->lower_link_m > 0.0f &&
        (config->front_elbow_branch == -1 || config->front_elbow_branch == 1) &&
        (config->rear_elbow_branch == -1 || config->rear_elbow_branch == 1) &&
        (config->foot_branch == -1 || config->foot_branch == 1);
}

static int solve_chain(float pivot_x, float upper_link, float lower_link,
                       float x, float height, int branch, float *angle)
{
    float dx = x - pivot_x;
    float distance_squared = dx * dx + height * height;
    float distance;
    float cosine;

    if (!isfinite(distance_squared) || distance_squared <= LEG_EPSILON) {
        return -1;
    }
    distance = sqrtf(distance_squared);
    if (distance > upper_link + lower_link ||
        distance < fabsf(upper_link - lower_link)) {
        return -1;
    }
    cosine = (upper_link * upper_link + distance_squared -
              lower_link * lower_link) / (2.0f * upper_link * distance);
    if (cosine < -1.0f - LEG_EPSILON || cosine > 1.0f + LEG_EPSILON) {
        return -1;
    }
    cosine = fmaxf(-1.0f, fminf(1.0f, cosine));
    *angle = atan2f(height, dx) + (float)branch * acosf(cosine);
    return 0;
}

int leg_inverse_kinematics(
    const LegKinematicsConfig *config,
    float support_offset_m,
    float leg_height_m,
    LegKinematicsAngles *angles)
{
    float half_base;

    if (!leg_kinematics_config_valid(config) || angles == NULL ||
        !isfinite(support_offset_m) || !isfinite(leg_height_m) ||
        leg_height_m <= 0.0f) {
        return -1;
    }
    half_base = config->base_spacing_m * 0.5f;
    if (solve_chain(half_base, config->upper_link_m, config->lower_link_m,
                    support_offset_m, leg_height_m,
                    config->front_elbow_branch,
                    &angles->front_angle_rad) != 0 ||
        solve_chain(-half_base, config->upper_link_m, config->lower_link_m,
                    support_offset_m, leg_height_m,
                    config->rear_elbow_branch,
                    &angles->rear_angle_rad) != 0) {
        return -1;
    }
    return 0;
}

int leg_forward_kinematics(
    const LegKinematicsConfig *config,
    const LegKinematicsAngles *angles,
    float *support_offset_m,
    float *leg_height_m)
{
    float half_base;
    float front_x;
    float front_y;
    float rear_x;
    float rear_y;
    float dx;
    float dy;
    float distance;
    float half_chord;
    float mid_x;
    float mid_y;

    if (!leg_kinematics_config_valid(config) || angles == NULL ||
        support_offset_m == NULL || leg_height_m == NULL ||
        !isfinite(angles->front_angle_rad) ||
        !isfinite(angles->rear_angle_rad)) {
        return -1;
    }
    half_base = config->base_spacing_m * 0.5f;
    front_x = half_base + config->upper_link_m *
        cosf(angles->front_angle_rad);
    front_y = config->upper_link_m * sinf(angles->front_angle_rad);
    rear_x = -half_base + config->upper_link_m *
        cosf(angles->rear_angle_rad);
    rear_y = config->upper_link_m * sinf(angles->rear_angle_rad);
    dx = front_x - rear_x;
    dy = front_y - rear_y;
    distance = sqrtf(dx * dx + dy * dy);
    if (distance <= LEG_EPSILON ||
        distance > 2.0f * config->lower_link_m) {
        return -1;
    }
    half_chord = sqrtf(fmaxf(0.0f,
        config->lower_link_m * config->lower_link_m -
        distance * distance * 0.25f));
    mid_x = (front_x + rear_x) * 0.5f;
    mid_y = (front_y + rear_y) * 0.5f;
    *support_offset_m = mid_x + (float)config->foot_branch *
        (-dy / distance) * half_chord;
    *leg_height_m = mid_y + (float)config->foot_branch *
        (dx / distance) * half_chord;
    return 0;
}

int leg_kinematics_calibration_valid(
    const LegKinematicsCalibration *calibration)
{
    LegKinematicsAngles reference_physical;
    float reconstructed_offset;
    float reconstructed_height;

    if (calibration == NULL ||
        !leg_kinematics_config_valid(&calibration->geometry) ||
        !isfinite(calibration->reference_support_offset_m) ||
        !isfinite(calibration->reference_leg_height_m) ||
        calibration->reference_leg_height_m <= 0.0f ||
        !isfinite(calibration->reference_effective_angles.front_angle_rad) ||
        !isfinite(calibration->reference_effective_angles.rear_angle_rad) ||
        (calibration->front_effective_direction != -1 &&
         calibration->front_effective_direction != 1) ||
        (calibration->rear_effective_direction != -1 &&
         calibration->rear_effective_direction != 1) ||
        leg_inverse_kinematics(
            &calibration->geometry,
            calibration->reference_support_offset_m,
            calibration->reference_leg_height_m,
            &reference_physical) != 0 ||
        leg_forward_kinematics(&calibration->geometry,
                               &reference_physical,
                               &reconstructed_offset,
                               &reconstructed_height) != 0) {
        return 0;
    }
    return fabsf(reconstructed_offset -
                 calibration->reference_support_offset_m) <=
               LEG_POSITION_TOLERANCE_M &&
        fabsf(reconstructed_height -
              calibration->reference_leg_height_m) <=
            LEG_POSITION_TOLERANCE_M;
}

int leg_calibrated_inverse_kinematics(
    const LegKinematicsCalibration *calibration,
    float support_offset_m,
    float leg_height_m,
    LegKinematicsAngles *effective_angles)
{
    LegKinematicsAngles reference_physical;
    LegKinematicsAngles target_physical;
    float reconstructed_offset;
    float reconstructed_height;

    if (!leg_kinematics_calibration_valid(calibration) ||
        effective_angles == NULL ||
        leg_inverse_kinematics(
            &calibration->geometry,
            calibration->reference_support_offset_m,
            calibration->reference_leg_height_m,
            &reference_physical) != 0 ||
        leg_inverse_kinematics(&calibration->geometry,
                               support_offset_m,
                               leg_height_m,
                               &target_physical) != 0 ||
        leg_forward_kinematics(&calibration->geometry,
                               &target_physical,
                               &reconstructed_offset,
                               &reconstructed_height) != 0 ||
        fabsf(reconstructed_offset - support_offset_m) >
            LEG_POSITION_TOLERANCE_M ||
        fabsf(reconstructed_height - leg_height_m) >
            LEG_POSITION_TOLERANCE_M) {
        return -1;
    }

    effective_angles->front_angle_rad =
        calibration->reference_effective_angles.front_angle_rad +
        (float)calibration->front_effective_direction *
        (target_physical.front_angle_rad -
         reference_physical.front_angle_rad);
    effective_angles->rear_angle_rad =
        calibration->reference_effective_angles.rear_angle_rad +
        (float)calibration->rear_effective_direction *
        (target_physical.rear_angle_rad -
         reference_physical.rear_angle_rad);
    return 0;
}
