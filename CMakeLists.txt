cmake_minimum_required(VERSION 3.10)
project(my_mujoco_project)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find MuJoCo and GLFW
find_package(PkgConfig REQUIRED)
# Add this line to find the Eigen package
find_package(Eigen3 REQUIRED)
pkg_check_modules(MUJOCO REQUIRED mujoco)
pkg_check_modules(GLFW REQUIRED glfw3)

#Find ModernRobotics library
find_library(MODERN_ROBOTICS_LIB
    NAMES ModernRoboticsCpp
    PATHS /usr/local/lib
    REQUIRED
)


# Print MuJoCo paths
message(STATUS "MuJoCo include directories: ${MUJOCO_INCLUDE_DIRS}")
message(STATUS "MuJoCo library directories: ${MUJOCO_LIBRARY_DIRS}")
message(STATUS "MuJoCo libraries: ${MUJOCO_LIBRARIES}")
# message(STATUS "ModernRobotics libraries: ${MODERN_ROBOTICS_LIB}")

# Include directories
include_directories(${MUJOCO_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR})

# Link directories
link_directories(${MUJOCO_LIBRARY_DIRS} ${GLFW_LIBRARY_DIRS})

# Add subdirectories
add_subdirectory(src)

# Download and unpack Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/release-1.11.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Enable testing
enable_testing()


add_subdirectory(tests build/tests)  # Specify a unique binary directory for tests
