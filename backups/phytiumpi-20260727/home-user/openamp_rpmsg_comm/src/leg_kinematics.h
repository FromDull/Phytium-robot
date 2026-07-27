#ifndef LEG_KINEMATICS_H
#define LEG_KINEMATICS_H

typedef struct {
    float base_spacing_m;
    float upper_link_m;
    float lower_link_m;
    /* Select one of the two shoulder solutions for each serial chain. */
    int front_elbow_branch;
    int rear_elbow_branch;
    /* Select one of the two lower-link circle intersections. */
    int foot_branch;
} LegKinematicsConfig;

typedef struct {
    float front_angle_rad;
    float rear_angle_rad;
} LegKinematicsAngles;

typedef struct {
    LegKinematicsConfig geometry;
    float reference_support_offset_m;
    float reference_leg_height_m;
    LegKinematicsAngles reference_effective_angles;
    int front_effective_direction;
    int rear_effective_direction;
} LegKinematicsCalibration;

int leg_kinematics_config_valid(const LegKinematicsConfig *config);
int leg_inverse_kinematics(
    const LegKinematicsConfig *config,
    float support_offset_m,
    float leg_height_m,
    LegKinematicsAngles *angles);
int leg_forward_kinematics(
    const LegKinematicsConfig *config,
    const LegKinematicsAngles *angles,
    float *support_offset_m,
    float *leg_height_m);
int leg_kinematics_calibration_valid(
    const LegKinematicsCalibration *calibration);
int leg_calibrated_inverse_kinematics(
    const LegKinematicsCalibration *calibration,
    float support_offset_m,
    float leg_height_m,
    LegKinematicsAngles *effective_angles);

#endif
