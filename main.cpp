#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>

int main() {
    // Activate MuJoCo
    // mj_activate("mjkey.txt");

    // Load MuJoCo model
    char error[1000] = "Could not load binary model";
    mjModel* m = mj_loadXML("../hello.xml", 0, error, 1000);
    if (!m) {
        std::cerr << "Load model error: " << error << std::endl;
        return 1;
    }

    // Make data
    mjData* d = mj_makeData(m);

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW" << std::endl;
        return 1;
    }

    // Create window
    GLFWwindow* window = glfwCreateWindow(1244, 700, "MuJoCo Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Main simulation loop
    while (!glfwWindowShouldClose(window)) {
        mj_step(m, d);

        // Render here

        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Clean up
    mj_deleteData(d);
    mj_deleteModel(m);
    // mj_deactivate();
    glfwTerminate();

    return 0;
}