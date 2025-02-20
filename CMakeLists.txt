cmake_minimum_required(VERSION 3.10)
project(my_mujoco_project)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find MuJoCo and GLFW
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

pkg_check_modules(MUJOCO REQUIRED mujoco)
pkg_check_modules(GLFW REQUIRED glfw3)

# Print MuJoCo paths
message(STATUS "MuJoCo include directories: ${MUJOCO_INCLUDE_DIRS}")
message(STATUS "MuJoCo library directories: ${MUJOCO_LIBRARY_DIRS}")
message(STATUS "MuJoCo libraries: ${MUJOCO_LIBRARIES}")

# Include directories
include_directories(${MUJOCO_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include)

# Link directories
link_directories(${MUJOCO_LIBRARY_DIRS} ${GLFW_LIBRARY_DIRS})


# Enable testing
enable_testing()
# Add subdirectories
add_subdirectory(src)
add_subdirectory(tests)

