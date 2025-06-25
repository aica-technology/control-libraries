#include "robot_model/Model.hpp"

#include <cassert>
#include <cstddef>
#include <state_representation/space/joint/JointState.hpp>

#include <gtest/gtest.h>
#include <memory>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <string>

using namespace robot_model;
using namespace state_representation;

class JointTypeTesting : public testing::Test {
protected:
  void SetUp() override {};

  std::unique_ptr<Model> robot_;
};

TEST_F(JointTypeTesting, CheckContinuousJoint) {
  // clang-format off
    auto urdf =
    "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
    "<robot name=\"continuous\">"
      "<link name=\"world\"/>"
      "<joint name=\"base_joint\" type=\"continuous\">"
        "<parent link=\"world\"/>"
        "<child link=\"base\"/>"
        "<axis xyz=\"0 0 1\"/>"
      "</joint>"
      "<link name=\"base\"/>"
    "</robot>";
  // clang-format on

  this->robot_ = std::make_unique<Model>("continuous", urdf);
  EXPECT_STREQ(this->robot_->get_robot_name().c_str(), "continuous");
  auto joint_types = this->robot_->get_joint_types();
  EXPECT_EQ(joint_types.size(), 1);
  EXPECT_EQ(joint_types.at(0), JointType::CONTINUOUS);
  EXPECT_EQ(this->robot_->get_number_of_joints(), 1);
  EXPECT_EQ(this->robot_->get_dof(), 1);
  EXPECT_EQ(this->robot_->get_configuration_dimension(), 2);

  auto& data = this->robot_->get_pinocchio_data();

  const double angle = M_PI / 3.0;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(this->robot_->get_number_of_joints());
  for (size_t i = 0; i < this->robot_->get_joint_types().size(); ++i) {
    if (this->robot_->get_joint_types().at(i) == JointType::CONTINUOUS) {
      q[i] = angle;
    }
  }
  state_representation::JointState joint_state(this->robot_->get_robot_name(), this->robot_->get_joint_frames());
  joint_state.set_positions(q);
  this->robot_->forward_kinematics(joint_state);

  const auto& M = data.oMi[1];
  Eigen::Matrix3d R_exp = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(M.rotation().isApprox(R_exp, 1e-9)) << "Rotation should be a pure Rz(pi/3)";

  Eigen::VectorXd v(1);
  v << 1.0;
  joint_state.set_velocities(v);

  auto twist = this->robot_->forward_velocity(joint_state);
  EXPECT_EQ(twist.get_reference_frame(), this->robot_->get_base_frame());
  EXPECT_EQ(twist.get_name(), "base");

  Eigen::Vector3d expected_angular(0.0, 0.0, 1.0);
  EXPECT_TRUE(twist.get_angular_velocity().isApprox(expected_angular, 1e-9))
      << "Expected angular velocity [0, 0, 1], got: " << twist.get_angular_velocity().transpose();

  EXPECT_TRUE(twist.get_linear_velocity().isZero(1e-9))
      << "Expected zero linear velocity, got: " << twist.get_linear_velocity().transpose();
}

TEST_F(JointTypeTesting, CheckPrismaticJoint) {
  // clang-format off
  auto urdf =
  "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
  "<robot name=\"prismatic\">"
    "<link name=\"world\"/>"
    "<joint name=\"fixed_base\" type=\"fixed\">"
      "<parent link=\"world\"/>"
      "<child link=\"base\"/>"
    "</joint>"
    "<link name=\"base\"/>"
    "<joint name=\"joint_0\" type=\"prismatic\">"
      "<parent link=\"base\"/>"
      "<child link=\"link_0\"/>"
      "<axis xyz=\"1 0 0\"/>"
      "<limit lower=\"0.0\" upper=\"1.0\" effort=\"10.0\" velocity=\"1.0\"/>"
    "</joint>"
    "<link name=\"link_0\"/>"
  "</robot>";
  // clang-format on
  this->robot_ = std::make_unique<Model>("prismatic", urdf);
  EXPECT_STREQ(this->robot_->get_robot_name().c_str(), "prismatic");
  EXPECT_EQ(this->robot_->get_number_of_joints(), 1);
  auto joint_types = this->robot_->get_joint_types();
  EXPECT_EQ(joint_types.size(), 1);
  EXPECT_EQ(joint_types.at(0), JointType::PRISMATIC);
}

TEST_F(JointTypeTesting, CheckFloatingJoint) {
  // clang-format off
  auto urdf =
  "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
  "<robot name=\"floating\">"
    "<link name=\"world\"/>"
    "<joint name=\"base_joint\" type=\"floating\">"
      "<parent link=\"world\"/>"
      "<child link=\"base\"/>"
    "</joint>"
    "<link name=\"base\"/>"
  "</robot>";
  // clang-format on
  this->robot_ = std::make_unique<Model>("floating", urdf);
  EXPECT_STREQ(this->robot_->get_robot_name().c_str(), "floating");
  EXPECT_EQ(this->robot_->get_number_of_joints(), 1);
  auto joint_types = this->robot_->get_joint_types();
  EXPECT_EQ(joint_types.size(), 1);
  EXPECT_EQ(joint_types.at(0), JointType::FLOATING);
}

TEST_F(JointTypeTesting, CheckRevoluteJoint) {
  // clang-format off
  auto urdf =
  "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
  "<robot name=\"revolute\">"
    "<link name=\"world\"/>"
    "<joint name=\"base_joint\" type=\"revolute\">"
      "<parent link=\"world\"/>"
      "<child link=\"base\"/>"
      "<limit lower=\"-1.57\" upper=\"1.57\" effort=\"10.0\" velocity=\"1.0\"/>"
    "</joint>"
    "<link name=\"base\"/>"
  "</robot>";
  // clang-format on
  this->robot_ = std::make_unique<Model>("revolute", urdf);
  EXPECT_STREQ(this->robot_->get_robot_name().c_str(), "revolute");
  EXPECT_EQ(this->robot_->get_number_of_joints(), 1);
  auto joint_types = this->robot_->get_joint_types();
  EXPECT_EQ(joint_types.size(), 1);
  EXPECT_EQ(joint_types.at(0), JointType::REVOLUTE);
}

TEST_F(JointTypeTesting, CheckPlanarJoint) {
  // clang-format off
  auto urdf =
  "<?xml version=\"1.0\" encoding=\"utf-8\"?>"
  "<robot name=\"foo_planar\">"
    "<link name=\"world\"/>"
    "<joint name=\"base_joint\" type=\"planar\">"
      "<parent link=\"world\"/>"
      "<child link=\"base\"/>"
      "<axis xyz=\"0 0 1\"/>"
    "</joint>"
    "<link name=\"base\"/>"
  "</robot>";
  // clang-format on
  this->robot_ = std::make_unique<Model>("planar", urdf);
  EXPECT_STREQ(this->robot_->get_robot_name().c_str(), "planar");
  EXPECT_EQ(this->robot_->get_number_of_joints(), 1);
  auto joint_types = this->robot_->get_joint_types();
  EXPECT_EQ(joint_types.size(), 1);
  EXPECT_EQ(joint_types.at(0), JointType::PLANAR);
}
