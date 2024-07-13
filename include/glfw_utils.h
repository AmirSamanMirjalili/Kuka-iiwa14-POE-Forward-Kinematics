#ifndef GLFW_UTILS_H
#define GLFW_UTILS_H

#include <stdio.h>
#include <string.h>
#include <cmath>
#include "mujoco_utils.h"
#include <GLFW/glfw3.h>

// Rest of your code...

// Declare external variables
extern mjvCamera cam;

// Function declarations
void init_glfw();
int select_body(GLFWwindow* window, mjvScene* scn, mjvCamera* cam, mjModel* m, mjData* d);
void setup_callbacks(GLFWwindow* window);
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void update_mocap_body_position(double dx, double dy, bool mod_shift);

#endif // GLFW_UTILS_H