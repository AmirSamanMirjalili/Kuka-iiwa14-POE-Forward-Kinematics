#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <vector>
#include <Eigen/Dense>
#include "globals.h"


const double CTRL_UPDATE_FREQ = 100.0; 
const char* FILENAME = "/home/amir/Robotics/Mujoco/Projects/Kuka_calibration/Kuka-iiwa14-kinodyn-calibration/scene.xml";
bool debug_mode = false;
int* actuator_ids = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
bool button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;
bool isPushing = false;
int grabbedBodyId = -1;
mjtNum pushForce[3] = {0, 0, 0};
double pushStartX = 0, pushStartY = 0;
mjModel* m = nullptr;
mjData* d = nullptr;
mjvPerturb pert;
int mocap_body_id = -1;
std::vector<ErrorData> errorHistory;