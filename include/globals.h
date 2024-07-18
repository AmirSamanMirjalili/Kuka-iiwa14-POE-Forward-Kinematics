#ifndef GLOBALS_H
#define GLOBALS_H

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

   struct ErrorData {
    double time;
    Eigen::Vector3d positionError;
    double orientationError;
};



extern std::vector<ErrorData> errorHistory;

extern const double CTRL_UPDATE_FREQ;


extern const char* FILENAME;
extern bool debug_mode;
extern int* actuator_ids;
extern mjvCamera cam;
extern mjvScene scn;
extern mjrContext con;
extern bool button_left, button_middle, button_right;
extern double lastx, lasty;
extern bool isPushing;
extern int grabbedBodyId;
extern mjtNum pushForce[3];
extern double pushStartX, pushStartY;
extern mjModel* m;
extern mjData* d;
extern mjvPerturb pert;
extern int mocap_body_id;
extern bool use_zero_control;
extern bool kinematic_debug_mode;
extern bool use_test_control;

#endif // GLOBALS_H