#include "glfw_utils.h"
#include <stdio.h>
#include <string.h>
#include <cmath>
#include "mujoco_utils.h"

extern mjvCamera cam;
extern mjvOption opt;
extern mjvScene scn;
extern mjrContext con;
extern bool button_left, button_middle, button_right;
extern double lastx, lasty;
extern bool isPushing;
extern int grabbedBodyId;
extern mjtNum pushForce[3];
extern double pushStartX, pushStartY;
extern mjModel *m;
extern mjData *d;
extern mjvPerturb pert;
extern int mocap_body_id;

// Assuming you have a valid mjModel* m and the body name
const char *body_name = "mocap_sphere";

void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void init_glfw()
{
    if (!glfwInit())
        mju_error("Could not initialize GLFW");
    GLFWwindow *window = glfwCreateWindow(1244, 700, "MuJoCo Demo", NULL, NULL);
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

void setup_callbacks(GLFWwindow *window)
{
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
}

void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

int select_body(GLFWwindow *window, mjvScene *scn, mjvCamera *cam, mjModel *m, mjData *d)
{
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    mjrRect viewport = {0, 0, width, height};

    mjtNum selpos[3];
    int geomid[1], flexid[1], skinid[1];
    int selbody = mjv_select(m, d, &opt, (mjtNum)width / height, (mjtNum)xpos / width, (mjtNum)(height - ypos) / height, scn, selpos, geomid, flexid, skinid);

    if (selbody >= 0)
    {
        DEBUG_PRINT("Selected body ID: %d\n", selbody);
    }

    return selbody;
}

void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    glfwGetCursorPos(window, &lastx, &lasty);

    if (button == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_PRESS)
    {
        isPushing = true;

        int mocap_body_id = mj_name2id(m, mjOBJ_BODY, body_name);

        grabbedBodyId = select_body(window, &scn, &cam, m, d);
        if (grabbedBodyId == mocap_body_id)
        {
            glfwGetCursorPos(window, &pushStartX, &pushStartY);
            DEBUG_PRINT("Started pushing on body ID: %d\n", grabbedBodyId);
            mjv_initPerturb(m, d, &scn, &pert); // Initialize perturbation
            pert.select = grabbedBodyId;        // Set the selected body ID
            pert.active = mjPERT_TRANSLATE;     // Enable translational perturbations
            pert.scale = 1;                     // Set the scale of the perturbation
        }
    }
    else if (button == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_RELEASE)
    {
        isPushing = false;
        memset(pushForce, 0, sizeof(pushForce));
        DEBUG_PRINT("Stopped pushing on body ID: %d\n", grabbedBodyId);
        grabbedBodyId = 0;
    }
}



void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    if (!button_left && !button_middle && !button_right)
        return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    if (grabbedBodyId <= 0)
    {
        mjtMouse action;
        if (button_right)
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        else if (button_left)
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        else
            action = mjMOUSE_ZOOM;

        mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
    }

    if (button_left && grabbedBodyId == mocap_body_id)
    {
        // Adjust sensitivity dynamically based on window size
        double sensitivity = 0.001 * (width + height) / 2.0;

        // Apply a smoothing factor to reduce jitter
        double smoothing_factor = 0.01;
        dx = smoothing_factor * dx + (1.0 - smoothing_factor) * (xpos - lastx);
        dy = smoothing_factor * dy + (1.0 - smoothing_factor) * (ypos - lasty);

        if (!mod_shift)
        {
            // Update x and y positions
            pert.refpos[0] += dx * sensitivity;
            pert.refpos[1] += -dy * sensitivity;
        }
        else
        {
            // Update z position
            pert.refpos[2] += -dy * sensitivity;
        }

        // Clamp values to ensure they stay within reasonable bounds
        pert.refpos[0] = fmax(fmin(pert.refpos[0], 1.0), -1.0);
        pert.refpos[1] = fmax(fmin(pert.refpos[1], 1.0), -1.0);
        pert.refpos[2] = fmax(fmin(pert.refpos[2], 1.0), 0);
    }

    // Call the function to print sensor data
    // print_sensor_data();
}


