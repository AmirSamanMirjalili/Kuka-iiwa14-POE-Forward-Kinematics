#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include "mujoco_utils.h"
#include "glfw_utils.h"

bool debug_mode = false;

const char* FILENAME = "/home/amir/Robotics/Mujoco/Projects/Kuka_calibration/Kuka-iiwa14-kinodyn-calibration/scene.xml";
const double CTRL_UPDATE_FREQ = 100.0;

// MuJoCo data structures
mjModel* m = NULL;
mjData* d = NULL;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
mjvPerturb pert;
int mocap_body_id;
// Actuator ID array
int* actuator_ids = nullptr;

// double T[7] = {2.5, 3.0, 4.2, 1.8, 5.5, 2.1, 3.7}; // Example periods

// Mouse interaction variables
bool button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;

// Controller variables
mjtNum last_update = 0.0;
mjtNum ctrl;

// Force application variables
bool isPushing = false;
int grabbedBodyId = -1;
mjtNum pushForce[3] = {0, 0, 0};
double pushStartX, pushStartY;

void simulate_and_render(GLFWwindow* window) {
    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0) {
            mj_step(m, d);
        
        }

        
        // Apply mouse perturbations
        mjv_applyPerturbPose(m, d, &pert, 0);

        // Render the scene
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();

        // Debugging: Print mocap position after step
    }
}

int main(int argc, const char** argv) {
    init_mujoco();
    init_glfw();
    init_actuator_ids();
    mjcb_control = update_control;


    // Main simulation loop
    

    GLFWwindow* window = glfwGetCurrentContext();
    simulate_and_render(window);

    cleanup_mujoco();
    return 0;
}