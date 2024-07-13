# Download and unpack googletest at configure time
include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.10.0.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Fetch the content
FetchContent_MakeAvailable(googletest)

# Add test executable
add_executable(runTests test_operations.cpp ../src/Operations.cpp ../src/mujoco_utils.cpp ../src/glfw_utils.cpp ../src/globals.cpp)
# Link Google Test libraries
target_link_libraries(runTests gtest gtest_main ${MUJOCO_LIBRARIES} ${GLFW_LIBRARIES} pthread)

# Add tests
add_test(NAME runTests COMMAND runTests)