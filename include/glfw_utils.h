#ifndef GLFW_UTILS_H
#define GLFW_UTILS_H

#include <stdio.h>
#include <string.h>
#include <cmath>
#include "mujoco_utils.h"
#include <GLFW/glfw3.h>

// Declare external variables
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
extern mjModel* m;
extern mjData* d;
extern mjvPerturb pert;
extern int mocap_body_id;

// Function declarations
void init_glfw();
int select_body(GLFWwindow* window, mjvScene* scn, mjvCamera* cam, mjModel* m, mjData* d);
void setup_callbacks(GLFWwindow* window);
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void update_mocap_body_position(double dx, double dy, bool mod_shift);
void scroll(GLFWwindow *window, double xoffset, double yoffset);

#endif // GLFW_UTILS_H