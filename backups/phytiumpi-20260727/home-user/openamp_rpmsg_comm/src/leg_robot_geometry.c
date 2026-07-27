#include "leg_robot_geometry.h"

#include <stddef.h>

#define DEG_TO_RAD 0.017453292519943295f

void leg_robot_get_measured_calibration(
    LegKinematicsCalibration *calibration)
{
    if (calibration == NULL) {
        return;
    }

    calibration->geometry.base_spacing_m = 0.090f;
    calibration->geometry.upper_link_m = 0.085f;
    calibration->geometry.lower_link_m = 0.145f;
    calibration->geometry.front_elbow_branch = -1;
    calibration->geometry.rear_elbow_branch = 1;
    calibration->geometry.foot_branch = 1;
    calibration->reference_support_offset_m = 0.0f;
    calibration->reference_leg_height_m = 0.130f;
    calibration->reference_effective_angles.front_angle_rad =
        45.0f * DEG_TO_RAD;
    calibration->reference_effective_angles.rear_angle_rad =
        45.0f * DEG_TO_RAD;
    calibration->front_effective_direction = 1;
    calibration->rear_effective_direction = -1;
}
