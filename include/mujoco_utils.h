#ifndef MUJOCO_UTILS_H
#define MUJOCO_UTILS_H

#include <mujoco/mujoco.h>
#include <memory>

extern const double CTRL_UPDATE_FREQ;
// Global variable for debug mode
extern bool debug_mode;

// Macro for debug printing
#define DEBUG_PRINT(...) \
    do { if (debug_mode) fprintf(stderr, __VA_ARGS__); } while (0)

// Function prototypes
void init_mujoco();
void cleanup_mujoco();
void apply_force();
void update_control(const mjModel* m, mjData* d);
void print_sensor_data();
void init_actuator_ids();
void get_joint_information();
std::shared_ptr<mjtNum[]> calculate_joint_distances();
void get_kinematic_parameters(const mjModel* m,mjData* d);


#endif // MUJOCO_UTILS_H