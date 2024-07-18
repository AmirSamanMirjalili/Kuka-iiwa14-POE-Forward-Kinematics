#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include "mujoco_utils.h"
#include "glfw_utils.h"
#include "globals.h"
#include "Operations.h"

// Define the global variables
// Declare global variables as extern
extern bool debug_mode;
extern const char* FILENAME;
extern mjModel* m;
extern mjData* d;
extern mjvCamera cam;
extern mjvOption opt;
extern mjvScene scn;
extern mjrContext con;
extern mjvPerturb pert;
extern int mocap_body_id;
extern int* actuator_ids;
extern bool isPushing;
extern int grabbedBodyId;
extern mjtNum pushForce[3];
extern double pushStartX, pushStartY;
extern bool use_zero_control;

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

    mr::writeErrorHistoryToFile(errorHistory, "error_history.csv");

    cleanup_mujoco();
    return 0;
}