# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

set(SOURCES
    main.cpp
    mujoco_utils.cpp
    glfw_utils.cpp
    globals.cpp
    Operations.cpp
)

# Add executable
add_executable(main ${SOURCES})

# Link libraries
target_link_libraries(main ${MUJOCO_LIBRARIES} ${GLFW_LIBRARIES} ${MODERN_ROBOTICS_LIB} Eigen3::Eigen pthread)