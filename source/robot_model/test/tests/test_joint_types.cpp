#include "robot_model/Model.hpp"

#include <cassert>
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
  EXPECT_EQ(this->robot_->get_number_of_joints(), 2);
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
  EXPECT_EQ(this->robot_->get_number_of_joints(), 7);
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
  EXPECT_EQ(this->robot_->get_number_of_joints(), 4);
  auto joint_types = this->robot_->get_joint_types();
  EXPECT_EQ(joint_types.size(), 1);
  EXPECT_EQ(joint_types.at(0), JointType::PLANAR);
}
