#ifndef MUJOCO_UTILS_H
#define MUJOCO_UTILS_H

#include <mujoco/mujoco.h>
#include <memory>
#include <Eigen/Dense>
#include "globals.h"

extern bool debug_mode;
extern const char* FILENAME;
extern mjModel* m;
extern mjData* d;
extern int mocap_body_id;
extern int* actuator_ids;
extern bool isPushing;
extern int grabbedBodyId;
extern mjtNum pushForce[3];
extern bool use_zero_control;
extern bool kinematic_debug_mode;

// New global variables to store one-time calculations
extern Eigen::Vector3d g_eeOffset;
extern Eigen::Matrix4d g_homeConfiguration;
extern Eigen::MatrixXd g_screwAxes;

enum ControlMode {
    HOME_CONTROL,
    TEST_CONTROL,
    DEFAULT_CONTROL
};

extern ControlMode currentControlMode;; // Set default control mode


#define DEBUG_PRINT(...) \
    do { if (debug_mode) fprintf(stderr, __VA_ARGS__); } while (0)

#define KINEMATIC_DEBUG_PRINT(...) \
    do { if (kinematic_debug_mode) std::cout << __VA_ARGS__ << std::endl; } while (0)

void init_mujoco();
void cleanup_mujoco();
void apply_force();
void update_control(const mjModel* m, mjData* d);
void print_sensor_data();
void init_actuator_ids();
void get_joint_information();
std::shared_ptr<mjtNum[]> calculate_joint_distances();
void init_control();
void get_kinematic_parameters(const mjModel* m, mjData* d);
void zero_control();
void init_control_wrapper();
// New functions for one-time calculations
void calculateAndStoreHomeParameters(const mjModel* m, mjData* d);



#endif // MUJOCO_UTILS_H