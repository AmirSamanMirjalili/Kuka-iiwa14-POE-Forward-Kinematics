#include <stdbool.h> // for bool
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include "globals.h"





// MuJoCo data structures
mjModel* m = NULL;
mjData* d = NULL;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

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

// Function prototypes
void init_mujoco();
void init_glfw();
void cleanup();
int select_body(GLFWwindow* window, mjvScene* scn, mjvCamera* cam, mjModel* m, mjData* d);
void simulate_and_render(GLFWwindow* window);
void apply_force();
void update_control();
void setup_callbacks(GLFWwindow* window);

// Callback functions
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void scroll(GLFWwindow* window, double xoffset, double yoffset);

// Initialize MuJoCo
void init_mujoco() {
    char error[1000] = "Could not load binary model";
    m = mj_loadXML(FILENAME, 0, error, 1000);
    if (!m) mju_error_s("Load model error: %s", error);
    d = mj_makeData(m);
}

// Initialize GLFW and create window
void init_glfw() {
    if (!glfwInit()) mju_error("Could not initialize GLFW");
    GLFWwindow* window = glfwCreateWindow(1244, 700, "MuJoCo Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // Adjust near clipping plane
    scn.camera[0].frustum_near = 0.1 * m->stat.extent;
    scn.camera[1].frustum_near = 0.1 * m->stat.extent;

    setup_callbacks(window);
}

// Setup GLFW callbacks
void setup_callbacks(GLFWwindow* window) {
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
}

// Cleanup function
void cleanup() {
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
}

// Keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// Mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    glfwGetCursorPos(window, &lastx, &lasty);

    if (button == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_PRESS) {
        isPushing = true;
        grabbedBodyId = select_body(window, &scn, &cam, m, d); // Pass necessary parameters
        if (grabbedBodyId >= 0) {
            glfwGetCursorPos(window, &pushStartX, &pushStartY);
            printf("Started pushing on body ID: %d\n", grabbedBodyId);
        }
    } else if (button == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_RELEASE) {
        isPushing = false;
        memset(pushForce, 0, sizeof(pushForce));
        printf("Stopped pushing on body ID: %d\n", grabbedBodyId);
    }
}

// Mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// Scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// Function to get body ID by pointing the mouse cursor
int select_body(GLFWwindow* window, mjvScene* scn, mjvCamera* cam, mjModel* m, mjData* d) {
    // Get the current mouse position
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Get the window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // Define the viewport
    mjrRect viewport = {0, 0, width, height};

    // Convert mouse position to normalized device coordinates
    mjtNum selpos[3];
    int geomid[1], flexid[1], skinid[1];
    int selbody = mjv_select(m, d, &opt, (mjtNum)width / height, (mjtNum)xpos / width, (mjtNum)(height - ypos) / height, scn, selpos, geomid, flexid, skinid);

    // If a body is selected, print its ID
    if (selbody >= 0) {
        printf("Selected body ID: %d\n", selbody);
    }

    return selbody;
}

// Apply force to the selected body
void apply_force() {
    if (isPushing && grabbedBodyId >= 0) {
        d->xfrc_applied[6 * grabbedBodyId] = pushForce[0];
        d->xfrc_applied[6 * grabbedBodyId + 1] = pushForce[1];
        d->xfrc_applied[6 * grabbedBodyId + 2] = pushForce[2];
    }
}

// Update control signals
void update_control() {
    if (d->time - last_update >= 1.0 / CTRL_UPDATE_FREQ) {
        last_update = d->time;
        ctrl = sin(d->time);
        d->ctrl[0] = ctrl;
    }
}

// Main simulation and rendering loop
void simulate_and_render(GLFWwindow* window) {
    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0) {
            mj_step(m, d);
        }

        apply_force();
        update_control();

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

int main(int argc, const char** argv) {
    init_mujoco();
    init_glfw();

    GLFWwindow* window = glfwGetCurrentContext();
    simulate_and_render(window);

    cleanup();
    return 0;
}