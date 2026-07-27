#include "leg_kinematics.h"
#include "leg_robot_geometry.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_inverse_forward_round_trip(void)
{
    const LegKinematicsConfig config = {
        .base_spacing_m = 0.10f,
        .upper_link_m = 0.10f,
        .lower_link_m = 0.15f,
        .front_elbow_branch = 1,
        .rear_elbow_branch = -1,
        .foot_branch = -1,
    };
    LegKinematicsAngles angles;
    float x;
    float height;

    assert(leg_inverse_kinematics(&config, 0.0f, 0.18f, &angles) == 0);
    assert(leg_forward_kinematics(&config, &angles, &x, &height) == 0);
    assert(fabsf(x) < 1.0e-5f);
    assert(fabsf(height - 0.18f) < 1.0e-5f);

    assert(leg_inverse_kinematics(&config, 0.02f, 0.17f, &angles) == 0);
    assert(leg_forward_kinematics(&config, &angles, &x, &height) == 0);
    assert(fabsf(x - 0.02f) < 1.0e-5f);
    assert(fabsf(height - 0.17f) < 1.0e-5f);
}

static void test_unreachable_and_invalid_geometry(void)
{
    LegKinematicsConfig config = {
        .base_spacing_m = 0.10f,
        .upper_link_m = 0.10f,
        .lower_link_m = 0.15f,
        .front_elbow_branch = 1,
        .rear_elbow_branch = -1,
        .foot_branch = -1,
    };
    LegKinematicsAngles angles;

    assert(leg_inverse_kinematics(&config, 0.0f, 0.50f, &angles) != 0);
    config.front_elbow_branch = 0;
    assert(!leg_kinematics_config_valid(&config));
}

static void test_measured_reference_calibration(void)
{
    const float rad_to_deg = 57.29577951308232f;
    LegKinematicsCalibration calibration;
    LegKinematicsAngles effective;
    LegKinematicsAngles physical;
    float reconstructed_offset;
    float reconstructed_height;

    leg_robot_get_measured_calibration(&calibration);
    assert(leg_kinematics_calibration_valid(&calibration));
    assert(leg_inverse_kinematics(&calibration.geometry,
                                  0.0f, 0.130f, &physical) == 0);
    assert(leg_forward_kinematics(&calibration.geometry,
                                  &physical,
                                  &reconstructed_offset,
                                  &reconstructed_height) == 0);
    assert(fabsf(reconstructed_offset) < 1.0e-5f);
    assert(fabsf(reconstructed_height - 0.130f) < 1.0e-5f);
    assert(leg_calibrated_inverse_kinematics(
        &calibration, 0.0f, 0.130f, &effective) == 0);
    assert(fabsf(effective.front_angle_rad * rad_to_deg - 45.0f) < 1.0e-4f);
    assert(fabsf(effective.rear_angle_rad * rad_to_deg - 45.0f) < 1.0e-4f);

    assert(leg_calibrated_inverse_kinematics(
        &calibration, 0.0f, 0.140f, &effective) == 0);
    assert(effective.front_angle_rad * rad_to_deg > 49.0f);
    assert(effective.front_angle_rad * rad_to_deg < 49.5f);
    assert(effective.rear_angle_rad * rad_to_deg > 49.0f);
    assert(effective.rear_angle_rad * rad_to_deg < 49.5f);

    assert(leg_calibrated_inverse_kinematics(
        &calibration, 0.005f, 0.130f, &effective) == 0);
    assert(effective.front_angle_rad < 45.0f / rad_to_deg);
    assert(effective.rear_angle_rad > 45.0f / rad_to_deg);

    calibration.geometry.foot_branch = -1;
    assert(!leg_kinematics_calibration_valid(&calibration));
}

static void test_measured_workspace_round_trip(void)
{
    LegKinematicsCalibration calibration;

    leg_robot_get_measured_calibration(&calibration);
    for (int x_mm = -20; x_mm <= 20; x_mm += 5) {
        for (int height_mm = 100; height_mm <= 190; height_mm += 10) {
            const float x = (float)x_mm / 1000.0f;
            const float height = (float)height_mm / 1000.0f;
            LegKinematicsAngles physical;
            LegKinematicsAngles effective;
            float reconstructed_x;
            float reconstructed_height;

            assert(leg_inverse_kinematics(&calibration.geometry,
                                          x, height, &physical) == 0);
            assert(leg_forward_kinematics(&calibration.geometry,
                                          &physical,
                                          &reconstructed_x,
                                          &reconstructed_height) == 0);
            assert(fabsf(reconstructed_x - x) < 1.0e-5f);
            assert(fabsf(reconstructed_height - height) < 1.0e-5f);
            assert(leg_calibrated_inverse_kinematics(&calibration,
                                                      x, height,
                                                      &effective) == 0);
            assert(isfinite(effective.front_angle_rad));
            assert(isfinite(effective.rear_angle_rad));
        }
    }
}

int main(void)
{
    test_inverse_forward_round_trip();
    test_unreachable_and_invalid_geometry();
    test_measured_reference_calibration();
    test_measured_workspace_round_trip();
    puts("leg_kinematics_test: PASS");
    return 0;
}
