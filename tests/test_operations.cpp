#include <gtest/gtest.h>
#include "Operations.h"
#include <iostream>

TEST(OperationsTest, NearZeroTest) {
    std::cout << "Testing NearZero with value 1e-9" << std::endl;
    ASSERT_TRUE(mr::NearZero(1e-9)) << "1e-9 should be considered near zero";
    
    std::cout << "Testing NearZero with value 1e-3" << std::endl;
    ASSERT_FALSE(mr::NearZero(1e-3)) << "1e-3 should not be considered near zero";
}

TEST(OperationsTest, CalculateScrewAxisRevoluteTest) {
    Eigen::Vector3d axis_direction(1, 0, 0);
    Eigen::Vector3d point_on_axis(0, 0, 0);
    Eigen::VectorXd screw_axis = mr::CalculateScrewAxis("revolute", axis_direction, point_on_axis);
    
    std::cout << "Screw axis for revolute joint: " << screw_axis.transpose() << std::endl;
    
    ASSERT_EQ(screw_axis.size(), 6) << "Screw axis should be a 6-vector";
    EXPECT_DOUBLE_EQ(screw_axis(0), 1) << "First element should be 1";
    EXPECT_DOUBLE_EQ(screw_axis(1), 0) << "Second element should be 0";
    EXPECT_DOUBLE_EQ(screw_axis(2), 0) << "Third element should be 0";
    EXPECT_DOUBLE_EQ(screw_axis(3), 0) << "Fourth element should be 0";
    EXPECT_DOUBLE_EQ(screw_axis(4), 0) << "Fifth element should be 0";
    EXPECT_DOUBLE_EQ(screw_axis(5), 0) << "Sixth element should be 0";
}

TEST(OperationsTest, CalculateScrewAxisPrismaticTest) {
    Eigen::Vector3d axis_direction(0, 1, 0);
    Eigen::Vector3d point_on_axis(0, 0, 0);
    Eigen::VectorXd screw_axis = mr::CalculateScrewAxis("prismatic", axis_direction, point_on_axis);
    
    std::cout << "Screw axis for prismatic joint: " << screw_axis.transpose() << std::endl;
    
    ASSERT_EQ(screw_axis.size(), 6) << "Screw axis should be a 6-vector";
    EXPECT_DOUBLE_EQ(screw_axis(0), 0) << "First element should be 0";
    EXPECT_DOUBLE_EQ(screw_axis(1), 0) << "Second element should be 0";
    EXPECT_DOUBLE_EQ(screw_axis(2), 0) << "Third element should be 0";
    EXPECT_DOUBLE_EQ(screw_axis(3), 0) << "Fourth element should be 0";
    EXPECT_DOUBLE_EQ(screw_axis(4), 1) << "Fifth element should be 1";
    EXPECT_DOUBLE_EQ(screw_axis(5), 0) << "Sixth element should be 0";
}

TEST(OperationsTest, CalculateScrewAxisInvalidJointTypeTest) {
    Eigen::Vector3d axis_direction(0, 0, 1);
    Eigen::Vector3d point_on_axis(0, 0, 0);
    Eigen::VectorXd screw_axis = mr::CalculateScrewAxis("invalid", axis_direction, point_on_axis);
    
    SCOPED_TRACE("Testing CalculateScrewAxis with invalid joint type");
    std::cout << "Screw axis for invalid joint type: " << screw_axis.transpose() << std::endl;
    
    ASSERT_EQ(screw_axis.size(), 6) << "Screw axis should be a 6-vector";
    for (int i = 0; i < screw_axis.size(); ++i) {
        SCOPED_TRACE("Element index: " + std::to_string(i));
        EXPECT_DOUBLE_EQ(screw_axis(i), 0) << "Element " << i << " should be 0";
    }
}

TEST(OperationsTest, ScrewMatTest) {
    // Define joint axes
    std::vector<Eigen::Vector3d> joint_axes = {
        {0, 0, 1},
        {0, -1, 0},
        {1, 0, 0}
    };

    // Define points on the joint axes
    std::vector<Eigen::Vector3d> joint_points = {
        {0, 0, 0},
        {1, 0, 0},
        {0, 0, -0.5}
    };

    // Define joint types
    std::vector<std::string> joint_types = {
        "revolute",
        "revolute",
        "revolute"
    };

    // Call the ScrewMat function
    Eigen::MatrixXd Slist = mr::ScrewMat(joint_axes, joint_points, joint_types);

    // Expected result
    Eigen::MatrixXd expected_Slist(3, 6);
    expected_Slist << 0,    0,    1,    0,    0,    0,
   0,   -1,    0,    0,    0,  -1,
   1,    0,    0,    0, -0.5,   0;

    // Print the result for debugging
    std::cout << "Slist matrix:\n" << Slist << std::endl;

    // Check the size of the result
    ASSERT_EQ(Slist.rows(), 3);
    ASSERT_EQ(Slist.cols(), 6);

    // Check the values in the result
    for (int i = 0; i < Slist.rows(); ++i) {
        for (int j = 0; j < Slist.cols(); ++j) {
            EXPECT_DOUBLE_EQ(Slist(i, j), expected_Slist(i, j)) << "Mismatch at (" << i << ", " << j << ")";
        }
    }
}

// Add more tests for other functions