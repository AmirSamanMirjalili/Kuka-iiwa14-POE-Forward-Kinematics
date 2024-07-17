#include <gtest/gtest.h>
#include "Operations.h"
#include <iostream>
#include "mujoco_utils.h"
#include <cmath>


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
    Eigen::MatrixXd expected_Slist(6, 3);
    expected_Slist << 0,    0,    1,
                      0,   -1,    0,
                      1,    0,    0,
                      0,    0,    0,
                      0,    0,   -0.5,
                      0,   -1,    0;

    // Print the result for debugging
    std::cout << "Slist matrix:\n" << Slist << std::endl;

    // Check the size of the result
    ASSERT_EQ(Slist.rows(), 6);
    ASSERT_EQ(Slist.cols(), 3);

    // Check the values in the result
    for (int i = 0; i < Slist.rows(); ++i) {
        for (int j = 0; j < Slist.cols(); ++j) {
            EXPECT_NEAR(Slist(i, j), expected_Slist(i, j), 1e-10) << "Mismatch at (" << i << ", " << j << ")";
        }
    }
}
// Add more tests for other functions
TEST(OperationsTest, ScrewMatTest_UR5_6R) {
    // Define joint axes
    std::vector<Eigen::Vector3d> joint_axes = {
        {0, 0, 1},
        {0, 1, 0},
        {0, 1, 0},
        {0, 1, 0},
        {0, 0, -1},
        {0, 1, 0}
    };
    double W1=0.109, W2=0.082, L1=0.425, L2=0.392 ,H1=0.089, H2=0.095;

     // Define points on the joint axes
    std::vector<Eigen::Vector3d> joint_points = {
        {0, 0, 0},                // joint1
        {0, 0, H1},          // joint2
        {L1, 0,H1}, // joint3
        {L1+L2, 0, H1}, // joint4
        {L1+L2, W1, 0}, // joint5
        {L1+L2, 0,H1-H2} // joint6
    };


    // Define joint types
    std::vector<std::string> joint_types = {
        "revolute",
        "revolute",
        "revolute",
        "revolute",
        "revolute",
        "revolute"
    };

    // Call the ScrewMat function
    Eigen::MatrixXd Slist = mr::ScrewMat(joint_axes, joint_points, joint_types);

    // Expected result
    Eigen::MatrixXd expected_Slist(6, 6);
    expected_Slist << 0,    0,    0,   0 , 0, 0,
                      0,    1,    1,   1,  0, 1,
                      1,    0,    0,   0,  -1, 0,
                      0,    -H1,  -H1, -H1, -W1, H2-H1,
                      0,    0,   0,    0, L1+L2,0 ,
                      0,   0,    L1, L1+L2,0, L1+L2;

    // Print the result for debugging
    std::cout << "Slist matrix:\n" << Slist << std::endl;

    // Check the size of the result
    ASSERT_EQ(Slist.rows(), 6);
    ASSERT_EQ(Slist.cols(), 6);

    // Check the values in the result
    for (int i = 0; i < Slist.rows(); ++i) {
        for (int j = 0; j < Slist.cols(); ++j) {
            EXPECT_NEAR(Slist(i, j), expected_Slist(i, j), 1e-10) << "Mismatch at (" << i << ", " << j << ")";
        }
    }
}

// Function to be tested
TEST(OperationsTest, FkinSpace_UR5_6R)  {
    // UR5 6R robot arm

    // Define joint axes
    std::vector<Eigen::Vector3d> joint_axes = {
        {0, 0, 1},
        {0, 1, 0},
        {0, 1, 0},
        {0, 1, 0},
        {0, 0, -1},
        {0, 1, 0}
    };
    double W1 = 0.109, W2 = 0.082, L1 = 0.425, L2 = 0.392, H1 = 0.089, H2 = 0.095;

    Eigen::Matrix4d M;
    M << 1, 0, 0, L1 + L2,
         0, 1, 0, W1 + W2,
         0, 1, 0, H1 - H2,
         0, 0, 0, 1;

    // Define points on the joint axes
    std::vector<Eigen::Vector3d> joint_points = {
        {0, 0, 0},                // joint1
        {0, 0, H1},               // joint2
        {L1, 0, H1},              // joint3
        {L1 + L2, 0, H1},         // joint4
        {L1 + L2, W1, 0},         // joint5
        {L1 + L2, 0, H1 - H2}     // joint6
    };

    // Define joint types
    std::vector<std::string> joint_types = {
        "revolute",
        "revolute",
        "revolute",
        "revolute",
        "revolute",
        "revolute"
    };

    // Call the ScrewMat function
    Eigen::MatrixXd Slist = mr::ScrewMat(joint_axes, joint_points, joint_types);

    // Joint angles
    Eigen::VectorXd thetaList(6);
    thetaList << 0, -M_PI / 2, 0, 0, M_PI / 2, 0;

    // Compute forward kinematics
    Eigen::Matrix4d T = mr::FKinSpace(M, Slist, thetaList);

    // Print the result
    std::cout << "The end-effector configuration is:\n" << T << std::endl;

    // Expected result
    Eigen::Matrix4d expected_T;
    expected_T << 1.2326e-32, -1, 0, 0.095,
                  -1, 1.11022e-16, 0, 0.109,
                  1.11022e-16, 1, 0, 0.988,
                  0, 0, 0, 1;

    // Check the values in the result
    for (int i = 0; i < T.rows(); ++i) {
        for (int j = 0; j < T.cols(); ++j) {
            EXPECT_NEAR(T(i, j), expected_T(i, j), 1e-4) << "Mismatch at (" << i << ", " << j << ")";
        }
    }
}

TEST(OperationsTest, GetEndEffectorInfoTestAtHomePose) {
    // Initialize MuJoCo
    init_mujoco();

    // Set flag to use zero control
    use_zero_control = true;
    std::cout << "use_zero_control set to: " << std::boolalpha << use_zero_control << std::endl;

    // Call init_control to set home position
    init_control_wrapper();
    std::cout << "init_control_wrapper called" << std::endl;

    mj_step(m, d);  // Update the simulation
    std::cout << "Simulation stepped" << std::endl;

    // Call the function to test
    mr::EndEffectorInfo eeInfo = mr::getEndEffectorInfo(m, d);
    
    std::cout << "End-effector position: " << eeInfo.position.transpose() << std::endl;
    std::cout << "End-effector rotation matrix:\n" << eeInfo.rotation << std::endl;

    // Check if the position is not zero (assuming the end-effector is not at the origin)
    bool positionNotZero = !eeInfo.position.isZero();
    std::cout << "Position is not zero: " << std::boolalpha << positionNotZero << std::endl;
    EXPECT_FALSE(eeInfo.position.isZero()) << "End-effector position should not be zero";

    // Check if the rotation matrix is valid (orthogonal and determinant = 1)
    bool rotationUnitary = eeInfo.rotation.isUnitary();
    double rotationDeterminant = eeInfo.rotation.determinant();
    std::cout << "Rotation matrix is unitary: " << std::boolalpha << rotationUnitary << std::endl;
    std::cout << "Rotation matrix determinant: " << rotationDeterminant << std::endl;
    EXPECT_TRUE(eeInfo.rotation.isUnitary()) << "End-effector rotation matrix should be unitary";
    EXPECT_NEAR(eeInfo.rotation.determinant(), 1.0, 1e-6) << "Determinant of rotation matrix should be 1";

    // Check if the rotation matrix is close to identity using Angle-Axis method
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_error = eeInfo.rotation.transpose() * identity;
    Eigen::Vector4d axis_angle = mr::AxisAng3(mr::so3ToVec(mr::MatrixLog3(R_error)));
    double angle_error = axis_angle(3);
    
    double angle_threshold = 1e-6;  // Adjust this threshold as needed (in radians)
    bool rotationCloseToIdentity = std::abs(angle_error) < angle_threshold;
    
    std::cout << "Rotation angle error: " << angle_error << " radians" << std::endl;
    std::cout << "Rotation axis: " << axis_angle.head<3>().transpose() << std::endl;
    std::cout << "Rotation is close to identity: " << std::boolalpha << rotationCloseToIdentity << std::endl;
    
    EXPECT_TRUE(rotationCloseToIdentity) << "End-effector rotation should be close to identity at home pose";

    // Reset flag
    use_zero_control = false;
    std::cout << "use_zero_control reset to: " << std::boolalpha << use_zero_control << std::endl;

    // Clean up MuJoCo
    cleanup_mujoco();
    std::cout << "MuJoCo cleaned up" << std::endl;
}

TEST(OperationsTest, DefineJointInfoTest) {
    // Initialize MuJoCo
    init_mujoco();

     // // Create a mock link lengths array
    // std::shared_ptr<mjtNum[]> linkLengths(new mjtNum[m->njnt]);
    // for (int i = 0; i < m->njnt; ++i) {
    //     linkLengths[i] = 0.1 * (i + 1); // Example link lengths
    // }

    // Create a mock link lengths array
    std::shared_ptr<mjtNum[]> linkLengths(new mjtNum[m->njnt]);
    linkLengths[0] = 0.2025;
    linkLengths[1] = 0.2045;
    linkLengths[2] = 0.2155;
    linkLengths[3] = 0.1845;
    linkLengths[4] = 0.2155;
    linkLengths[5] = 0.081;

    // Call the function to test
    mr::JointInfo jointInfo = mr::defineJointInfo(m, d, linkLengths);

    // Check the axes
    std::vector<Eigen::Vector3d> expected_axes = {
        {0, 0, 1}, {0, 1, 0}, {0, 0, 1}, {0, -1, 0}, {0, 0, 1}, {0, 1, 0}, {0, 0, 1}
    };
    ASSERT_EQ(jointInfo.axes.size(), expected_axes.size());
    for (size_t i = 0; i < expected_axes.size(); ++i) {
        EXPECT_TRUE(jointInfo.axes[i].isApprox(expected_axes[i])) << "Mismatch at axis " << i;
    }

    // Check the base frame
    mjtNum baseFrame[3];
    mj_local2Global(d, baseFrame, NULL, m->jnt_pos + 3 * m->jnt_bodyid[0], NULL, m->jnt_bodyid[0], 0);
    Eigen::Vector3d expected_baseFrame(baseFrame[0], baseFrame[1], baseFrame[2]);
    EXPECT_TRUE(jointInfo.baseFrame.isApprox(expected_baseFrame)) << "Base frame mismatch";

    // Check the points
    std::vector<Eigen::Vector3d> expected_points;
    double totalLength = 0;
    for (int i = 0; i < m->njnt; ++i) {
        expected_points.push_back({0, 0, totalLength});
        totalLength += linkLengths[i];
    }
    for (auto &point : expected_points) {
        point += expected_baseFrame;
    }
    ASSERT_EQ(jointInfo.points.size(), expected_points.size());
    for (size_t i = 0; i < expected_points.size(); ++i) {
        EXPECT_TRUE(jointInfo.points[i].isApprox(expected_points[i])) << "Mismatch at point " << i;
    }

    // Check the types
    std::vector<std::string> expected_types(m->njnt, "revolute");
    ASSERT_EQ(jointInfo.types.size(), expected_types.size());
    for (size_t i = 0; i < expected_types.size(); ++i) {
        EXPECT_EQ(jointInfo.types[i], expected_types[i]) << "Mismatch at type " << i;
    }

    // Clean up MuJoCo
    cleanup_mujoco();
}

TEST(OperationsTest, CalculateEndEffectorOffsetTest) {
    // Initialize MuJoCo
    init_mujoco();

    // Set flag to use zero control
    use_zero_control = true;

    // Call init_control to set home position
    init_control_wrapper();

    // Run one step of simulation to update the robot state
    mj_step(m, d);

    // Mock end-effector position
    Eigen::Vector3d eePosition(0.5, 0.5, 0.5);

    // Call the function to test
    Eigen::Vector3d eeOffset = mr::calculateEndEffectorOffset(m, d, eePosition);

    // Get the global position of the last joint
    int last_joint_id = m->njnt - 1;
    mjtNum last_joint_pos[3];
    mj_local2Global(d, last_joint_pos, NULL, m->jnt_pos + 3 * last_joint_id, NULL, m->jnt_bodyid[last_joint_id], 0);
    Eigen::Vector3d last_joint_pos_eigen(last_joint_pos[0], last_joint_pos[1], last_joint_pos[2]);

    // Calculate the expected end-effector offset
    Eigen::Vector3d expected_eeOffset = eePosition - last_joint_pos_eigen;

    // Print the values for debugging
    std::cout << "End-effector position: " << eePosition.transpose() << std::endl;
    std::cout << "Last joint position: " << last_joint_pos_eigen.transpose() << std::endl;
    std::cout << "Calculated end-effector offset: " << eeOffset.transpose() << std::endl;
    std::cout << "Expected end-effector offset: " << expected_eeOffset.transpose() << std::endl;

    // Check if the calculated offset matches the expected offset
    EXPECT_TRUE(eeOffset.isApprox(expected_eeOffset)) << "End-effector offset mismatch";

    // Reset flag
    use_zero_control = false;

    // Clean up MuJoCo
    cleanup_mujoco();
}

#include <gtest/gtest.h>
#include "Operations.h"
#include <iostream>
#include <memory>

TEST(OperationsTest, DefineHomeConfigurationTest) {
    // Mock link lengths
    std::shared_ptr<mjtNum[]> linkLengths(new mjtNum[3]);
    linkLengths[0] = 0.5;
    linkLengths[1] = 0.3;
    linkLengths[2] = 0.2;

    // Mock base frame
    Eigen::Vector3d baseFrame(0.1, 0.2, 0.3);

    // Mock end-effector offset
    Eigen::Vector3d eeOffset(0.05, 0.05, 0.05);

    // Expected total length
    double expectedTotalLength = 0.5 + 0.3 + 0.2;

    // Expected home configuration matrix
    Eigen::Matrix4d expectedM = Eigen::Matrix4d::Identity();
    expectedM.block<3, 1>(0, 3) << 0.1 + 0.05, 0.2 + 0.05, 0.3 + expectedTotalLength + 0.05;

    // Call the function to test
    Eigen::Matrix4d M = mr::defineHomeConfiguration(linkLengths, baseFrame, eeOffset);

    // Print the result for debugging
    std::cout << "Home configuration matrix M:\n" << M << std::endl;

    // Check the values in the result
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            EXPECT_NEAR(M(i, j), expectedM(i, j), 1e-10) << "Mismatch at (" << i << ", " << j << ")";
        }
    }
}
// TEST(OperationsTest, HomeAndForwardKinematicsTest) {
//     // Initialize MuJoCo
//     init_mujoco();

//     // Call init_control
//     init_control();

//     // Check if all control inputs are zero
//     for (int i = 0; i < m->nu; i++) {
//         EXPECT_NEAR(d->ctrl[i], 0.0, 1e-10) << "Control input " << i << " is not zero";
//     }

//     // Run one step of simulation to update the robot state
//     mj_step(m, d);

//     // Verify forward kinematics
//     mr::verifyForwardKinematics(m, d);

//     // Get end-effector body ID
//     const char *eeBodyName = "endeffector";
//     int eeBodyId = mj_name2id(m, mjOBJ_BODY, eeBodyName);
//     ASSERT_GE(eeBodyId, 0) << "Could not find end-effector body";

//     // Get MuJoCo end-effector pose
//     mjtNum eePos[3], eeRotMat[9];
//     mju_copy3(eePos, d->xpos + 3 * eeBodyId);
//     mju_copy(eeRotMat, d->xmat + 9 * eeBodyId, 9);

//     // Get theoretical end-effector pose
//     Eigen::Matrix4d T = mr::FKinSpace(M, Slist, thetaList);
//     Eigen::Vector3d theoreticalEePos = T.block<3, 1>(0, 3);
//     Eigen::Matrix3d theoreticalEeRotMat = T.block<3, 3>(0, 0);

//     // Check position error
//     for (int i = 0; i < 3; i++) {
//         EXPECT_NEAR(eePos[i], theoreticalEePos(i), 1e-4) << "Position mismatch at index " << i;
//     }

//     // Check orientation error
//     Eigen::Matrix3d mujocoEeRotMat;
//     mujocoEeRotMat << eeRotMat[0], eeRotMat[1], eeRotMat[2],
//                       eeRotMat[3], eeRotMat[4], eeRotMat[5],
//                       eeRotMat[6], eeRotMat[7], eeRotMat[8];

//     Eigen::Matrix3d R_error = mujocoEeRotMat.transpose() * theoreticalEeRotMat;
//     Eigen::Vector4d axis_angle = mr::AxisAng3(mr::so3ToVec(mr::MatrixLog3(R_error)));
//     double angle_error = axis_angle(3);

//     EXPECT_NEAR(angle_error, 0.0, 1e-4) << "Orientation error is not near zero";

//     // Clean up MuJoCo
//     cleanup_mujoco();
// }

// TEST(OperationsTest, HomePositionForwardKinematicsTest) {
//     // Initialize MuJoCo
//     init_mujoco();

//     // Modify init_control to set zero control (home position)
//     auto original_init_control = init_control;
//     init_control = []() {
//         for (int i = 0; i < m->nu; i++) {
//             d->ctrl[i] = 0.0;
//         }
//     };

//     // Call init_control to set home position
//     init_control();

//     // Run one step of simulation to update the robot state
//     mj_step(m, d);

//     // Verify forward kinematics
//     mr::verifyForwardKinematics(m, d);

//     // Get end-effector body ID
//     const char *eeBodyName = "endeffector";
//     int eeBodyId = mj_name2id(m, mjOBJ_BODY, eeBodyName);
//     ASSERT_GE(eeBodyId, 0) << "Could not find end-effector body";

//     // Get MuJoCo end-effector pose
//     mjtNum eePos[3], eeRotMat[9];
//     mju_copy3(eePos, d->xpos + 3 * eeBodyId);
//     mju_copy(eeRotMat, d->xmat + 9 * eeBodyId, 9);

//     // Get theoretical end-effector pose (should be the home position)
//     Eigen::Matrix4d M = mr::getHomeMatrix(m);
//     Eigen::Vector3d theoreticalEePos = M.block<3, 1>(0, 3);
//     Eigen::Matrix3d theoreticalEeRotMat = M.block<3, 3>(0, 0);

//     // Check position error
//     for (int i = 0; i < 3; i++) {
//         EXPECT_NEAR(eePos[i], theoreticalEePos(i), 1e-4) << "Position mismatch at index " << i;
//     }

//     // Check orientation error
//     Eigen::Matrix3d mujocoEeRotMat;
//     mujocoEeRotMat << eeRotMat[0], eeRotMat[1], eeRotMat[2],
//                       eeRotMat[3], eeRotMat[4], eeRotMat[5],
//                       eeRotMat[6], eeRotMat[7], eeRotMat[8];

//     Eigen::Matrix3d R_error = mujocoEeRotMat.transpose() * theoreticalEeRotMat;
//     Eigen::Vector4d axis_angle = mr::AxisAng3(mr::so3ToVec(mr::MatrixLog3(R_error)));
//     double angle_error = axis_angle(3);

//     EXPECT_NEAR(angle_error, 0.0, 1e-4) << "Orientation error is not near zero";

//     // Restore original init_control function
//     init_control = original_init_control;

//     // Clean up MuJoCo
//     cleanup_mujoco();
// }


// TEST(OperationsTest, HomeAndForwardKinematicsTest) {
//     // Initialize MuJoCo
//     init_mujoco();

//     // Modify init_control to set zero control (home position)
//     auto original_init_control = init_control;
//     init_control = []() {
//         for (int i = 0; i < m->nu; i++) {
//             d->ctrl[i] = 0.0;
//         }
//     };

//     // Call init_control
//     init_control();

//     // Check if all control inputs are zero
//     for (int i = 0; i < m->nu; i++) {
//         EXPECT_NEAR(d->ctrl[i], 0.0, 1e-10) << "Control input " << i << " is not zero";
//     }

//     // Run one step of simulation to update the robot state
//     mj_step(m, d);

//     // Define joint axes
//     std::vector<Eigen::Vector3d> joint_axes = {
//         {0, 0, 1},  // joint 1
//         {0, 1, 0},  // joint 2
//         {0, 0, 1},  // joint 3
//         {0, -1, 0}, // joint 4
//         {0, 0, 1},  // joint 5
//         {0, 1, 0},  // joint 6
//         {0, 0, 1}   // joint 7
//     };

//     // Define joint points
//     std::shared_ptr<mjtNum[]> link_lengths = calculate_joint_distances();
//     std::vector<Eigen::Vector3d> joint_points = {
//         {0, 0, 0},
//         {0, 0, link_lengths[1]},
//         {0, 0, link_lengths[1] + link_lengths[2]},
//         {0, 0, link_lengths[1] + link_lengths[2] + link_lengths[3]},
//         {0, 0, link_lengths[1] + link_lengths[2] + link_lengths[3] + link_lengths[4]},
//         {0, 0, link_lengths[1] + link_lengths[2] + link_lengths[3] + link_lengths[4] + link_lengths[5]},
//         {0, 0, link_lengths[1] + link_lengths[2] + link_lengths[3] + link_lengths[4] + link_lengths[5] + link_lengths[6]}
//     };

//     // Define joint types
//     std::vector<std::string> joint_types = {
//         "revolute",
//         "revolute",
//         "revolute",
//         "revolute",
//         "revolute",
//         "revolute",
//         "revolute"
//     };

//     // Define the end-effector configuration at home position (M)
//     Eigen::Matrix4d M;
//     M << 1, 0, 0, 0,
//          0, 1, 0, 0,
//          0, 0, 1, link_lengths[0] + link_lengths[1] + link_lengths[2] + link_lengths[3] + link_lengths[4] + link_lengths[5] + link_lengths[6],
//          0, 0, 0, 1;

//     // Get joint angles from MuJoCo data
//     Eigen::VectorXd jointAnglesEigen(7);
//     for (int i = 0; i < 7; ++i) {
//         jointAnglesEigen[i] = d->qpos[m->jnt_qposadr[i]];
//     }

//     // Verify forward kinematics
//     mr::verifyForwardKinematics(m, d, joint_axes, joint_points, joint_types, jointAnglesEigen, M);

//     // Get end-effector body ID
//     const char *eeBodyName = "endeffector";
//     int eeBodyId = mj_name2id(m, mjOBJ_BODY, eeBodyName);
//     ASSERT_GE(eeBodyId, 0) << "Could not find end-effector body";

//     // Restore original init_control function
//     init_control = original_init_control;

//     // Clean up MuJoCo
//     cleanup_mujoco();
// }