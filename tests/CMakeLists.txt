# Add test executable
add_executable(runTests test_operations.cpp ../src/Operations.cpp)

# Link Google Test libraries
target_link_libraries(runTests GTest::gtest GTest::gtest_main ${MUJOCO_LIBRARIES} ${GLFW_LIBRARIES} pthread)

# Add tests
add_test(NAME runTests COMMAND runTests)