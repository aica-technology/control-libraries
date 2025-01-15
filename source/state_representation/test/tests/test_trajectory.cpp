#include "state_representation/space/cartesian/CartesianState.hpp"
#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/trajectory/CartesianTrajectory.hpp"
#include "state_representation/trajectory/JointTrajectory.hpp"

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <gtest/gtest.h>
#include <unistd.h>

TEST(TrajectoryTest, CreateTrajectory) {
  state_representation::TrajectoryBase<Eigen::VectorXd> trajectory;
  std::deque<std::chrono::nanoseconds> times = trajectory.get_times();
  EXPECT_EQ(trajectory.get_size(), 0);
  EXPECT_TRUE(times.empty());
}

TEST(TrajectoryTest, AddPoint) {
  {
    state_representation::CartesianTrajectory trajectory("world");
    EXPECT_STREQ(trajectory.get_reference_frame().c_str(), "world");

    auto point1 = state_representation::CartesianState::Random("point1");
    EXPECT_TRUE(trajectory.add_point(point1, std::chrono::nanoseconds(100)));

    auto point2 = state_representation::CartesianState::Random("point2", "foo_reference_frame");
    EXPECT_FALSE(trajectory.add_point(point2, std::chrono::nanoseconds(200)));

    auto point3 = state_representation::CartesianState::Random("point3", "world");
    EXPECT_TRUE(trajectory.add_point(point3, std::chrono::nanoseconds(300)));
  }

  {
    state_representation::JointTrajectory trajectory("joint_trajectory");

    auto point1 = state_representation::JointState::Random("foo", 25);
    EXPECT_TRUE(trajectory.add_point(point1, std::chrono::nanoseconds(100)));

    auto point2 = state_representation::JointState::Random("foo", 1);
    EXPECT_FALSE(trajectory.add_point(point2, std::chrono::nanoseconds(200)));

    auto point3 = state_representation::JointState::Random("foo", 25);
    EXPECT_TRUE(trajectory.add_point(point3, std::chrono::nanoseconds(300)));
  }
}

TEST(TrajectoryTest, ClearPoint) {
  {
    state_representation::CartesianTrajectory trajectory("world");
    auto point1 = state_representation::CartesianState::Random("point1");
    auto point2 = state_representation::CartesianState::Random("point2");
    auto point3 = state_representation::CartesianState::Random("point3");
    trajectory.add_point(point1, std::chrono::nanoseconds(100));
    trajectory.add_point(point2, std::chrono::nanoseconds(300));
    trajectory.add_point(point3, std::chrono::nanoseconds(300));
    EXPECT_EQ(trajectory.get_size(), 3);
    EXPECT_EQ(trajectory.get_times().size(), 3);
    trajectory.delete_point();
    EXPECT_EQ(trajectory.get_size(), 2);
    EXPECT_EQ(trajectory.get_times().size(), 2);
    trajectory.clear();
    EXPECT_EQ(trajectory.get_size(), 0);
    EXPECT_EQ(trajectory.get_times().size(), 0);
  }

  {
    state_representation::JointTrajectory trajectory;
    auto point1 = state_representation::JointState::Random("point1", 25);
    auto point2 = state_representation::JointState::Random("point2", 25);
    auto point3 = state_representation::JointState::Random("point3", 25);
    trajectory.add_point(point1, std::chrono::nanoseconds(100));
    trajectory.add_point(point2, std::chrono::nanoseconds(300));
    trajectory.add_point(point3, std::chrono::nanoseconds(300));
    EXPECT_EQ(trajectory.get_size(), 3);
    EXPECT_EQ(trajectory.get_times().size(), 3);
    EXPECT_EQ(trajectory.get_joint_names().size(), 25);
    trajectory.delete_point();
    EXPECT_EQ(trajectory.get_size(), 2);
    EXPECT_EQ(trajectory.get_times().size(), 2);
    EXPECT_EQ(trajectory.get_joint_names().size(), 25);
    trajectory.clear();
    EXPECT_EQ(trajectory.get_size(), 0);
    EXPECT_EQ(trajectory.get_times().size(), 0);
    EXPECT_EQ(trajectory.get_joint_names().size(), 0);
  }
}

TEST(TrajectoryTest, OverloadIndex) {
  {
    state_representation::CartesianTrajectory trajectory;
    auto point1 = state_representation::CartesianState::Random("point1");
    auto point2 = state_representation::CartesianState::Random("point2");
    trajectory.add_point(point1, std::chrono::nanoseconds(300));
    trajectory.add_point(point2, std::chrono::nanoseconds(500));
    EXPECT_TRUE(trajectory[0].second == std::chrono::nanoseconds(300));
    EXPECT_TRUE(trajectory[0].first.data() == point1.data());
    EXPECT_TRUE(trajectory[1].second == std::chrono::nanoseconds(800));
    EXPECT_TRUE(trajectory[1].first.data() == point2.data());
  }

  {
    state_representation::JointTrajectory trajectory;
    auto point1 = state_representation::JointState::Random("foo", 25);
    auto point2 = state_representation::JointState::Random("foo", 25);
    trajectory.add_point(point1, std::chrono::nanoseconds(300));
    trajectory.add_point(point2, std::chrono::nanoseconds(500));
    EXPECT_TRUE(trajectory[0].second == std::chrono::nanoseconds(300));
    EXPECT_TRUE(trajectory[0].first.data() == point1.data());
    EXPECT_TRUE(trajectory[1].second == std::chrono::nanoseconds(800));
    EXPECT_TRUE(trajectory[1].first.data() == point2.data());
  }
}

TEST(TrajectoryTest, InsertPoint) {
  {
    state_representation::CartesianTrajectory trajectory;
    auto point1 = state_representation::CartesianState::Random("point1");
    auto point2 = state_representation::CartesianState::Random("point2");
    auto point3 = state_representation::CartesianState::Random("point3");
    trajectory.add_point(point1, std::chrono::nanoseconds(300));
    trajectory.add_point(point2, std::chrono::nanoseconds(500));

    trajectory.insert_point(point3, std::chrono::nanoseconds(100), 1);
    EXPECT_EQ(trajectory.get_size(), 3);
    EXPECT_TRUE(trajectory[1].second == std::chrono::nanoseconds(400));
    EXPECT_TRUE(trajectory[1].first.data() == point3.data());
    EXPECT_TRUE(trajectory[0].second == std::chrono::nanoseconds(300));
    EXPECT_TRUE(trajectory[0].first.data() == point1.data());
    EXPECT_TRUE(trajectory[2].second == std::chrono::nanoseconds(900));
    EXPECT_TRUE(trajectory[2].first.data() == point2.data());
  }

  {
    state_representation::JointTrajectory trajectory;
    auto point1 = state_representation::JointState::Random("point1", 25);
    auto point2 = state_representation::JointState::Random("point2", 25);
    auto point3 = state_representation::JointState::Random("point3", 25);
    trajectory.add_point(point1, std::chrono::nanoseconds(300));
    trajectory.add_point(point2, std::chrono::nanoseconds(500));

    trajectory.insert_point(point3, std::chrono::nanoseconds(100), 1);
    EXPECT_EQ(trajectory.get_size(), 3);
    EXPECT_TRUE(trajectory[1].second == std::chrono::nanoseconds(400));
    EXPECT_TRUE(trajectory[1].first.data() == point3.data());
    EXPECT_TRUE(trajectory[0].second == std::chrono::nanoseconds(300));
    EXPECT_TRUE(trajectory[0].first.data() == point1.data());
    EXPECT_TRUE(trajectory[2].second == std::chrono::nanoseconds(900));
    EXPECT_TRUE(trajectory[2].first.data() == point2.data());
  }
}

TEST(TrajectoryTest, GetPoints) {
  {
    state_representation::CartesianTrajectory trajectory;
    auto point1 = state_representation::CartesianState::Random("point1");
    auto point2 = state_representation::CartesianState::Random("point2");
    auto point3 = state_representation::CartesianState::Random("point3");
    trajectory.add_point(point1, std::chrono::nanoseconds(300));
    trajectory.add_point(point2, std::chrono::nanoseconds(500));
    trajectory.add_point(point3, std::chrono::nanoseconds(100));

    auto points = trajectory.get_points();
    EXPECT_EQ(points.size(), 3);
    EXPECT_TRUE(points[0].data() == point1.data());
    EXPECT_TRUE(points[1].data() == point2.data());
    EXPECT_TRUE(points[2].data() == point3.data());

    EXPECT_TRUE(trajectory.get_point(0).data() == point1.data());
    EXPECT_TRUE(trajectory.get_point(1).data() == point2.data());
    EXPECT_TRUE(trajectory.get_point(2).data() == point3.data());
  }

  {
    state_representation::JointTrajectory trajectory;
    auto point1 = state_representation::JointState::Random("point1", 25);
    auto point2 = state_representation::JointState::Random("point2", 25);
    auto point3 = state_representation::JointState::Random("point3", 25);
    trajectory.add_point(point1, std::chrono::nanoseconds(300));
    trajectory.add_point(point2, std::chrono::nanoseconds(500));
    trajectory.add_point(point3, std::chrono::nanoseconds(100));

    auto points = trajectory.get_points();
    EXPECT_EQ(points.size(), 3);
    EXPECT_TRUE(points[0].data() == point1.data());
    EXPECT_TRUE(points[1].data() == point2.data());
    EXPECT_TRUE(points[2].data() == point3.data());

    EXPECT_TRUE(trajectory.get_point(0).data() == point1.data());
    EXPECT_TRUE(trajectory.get_point(1).data() == point2.data());
    EXPECT_TRUE(trajectory.get_point(2).data() == point3.data());
  }
}

TEST(TrajectoryTest, SetPoints) {
  {
    state_representation::CartesianTrajectory trajectory;
    auto point1 = state_representation::CartesianState::Random("point1");
    auto point2 = state_representation::CartesianState::Random("point2");
    auto point3 = state_representation::CartesianState::Random("point3");

    auto replacement1 = state_representation::CartesianState::Random("replacement1");
    auto replacement2 = state_representation::CartesianState::Random("replacement2");
    std::vector<state_representation::CartesianState> replacement_points = {replacement1, replacement2};
    std::vector<std::chrono::nanoseconds> replacement_times = {
        std::chrono::nanoseconds(100), std::chrono::nanoseconds(200)};

    trajectory.add_point(point1, std::chrono::nanoseconds(300));
    trajectory.add_point(point2, std::chrono::nanoseconds(500));
    trajectory.add_point(point3, std::chrono::nanoseconds(100));

    trajectory.set_point(1, replacement1, std::chrono::nanoseconds(50));
    EXPECT_TRUE(trajectory[1].first.data() == replacement1.data());
    EXPECT_TRUE(trajectory[1].second == std::chrono::nanoseconds(350));

    EXPECT_TRUE(trajectory.set_points(replacement_points, replacement_times));
    EXPECT_EQ(trajectory.get_size(), 2);
    EXPECT_TRUE(trajectory[0].first.data() == replacement1.data());
    EXPECT_TRUE(trajectory[0].second == std::chrono::nanoseconds(100));
    EXPECT_TRUE(trajectory[1].first.data() == replacement2.data());
    EXPECT_TRUE(trajectory[1].second == std::chrono::nanoseconds(300));
  }

  {
    state_representation::JointTrajectory trajectory;
    auto point1 = state_representation::JointState::Random("foo", 25);
    auto point2 = state_representation::JointState::Random("foo", 25);
    auto point3 = state_representation::JointState::Random("foo", 25);

    auto replacement1 = state_representation::JointState::Random("foo", 25);
    auto replacement2 = state_representation::JointState::Random("foo", 25);
    std::vector<state_representation::JointState> replacement_points = {replacement1, replacement2};
    std::vector<std::chrono::nanoseconds> replacement_times = {
        std::chrono::nanoseconds(100), std::chrono::nanoseconds(200)};

    trajectory.add_point(point1, std::chrono::nanoseconds(300));
    trajectory.add_point(point2, std::chrono::nanoseconds(500));
    trajectory.add_point(point3, std::chrono::nanoseconds(100));

    trajectory.set_point(1, replacement1, std::chrono::nanoseconds(50));
    EXPECT_TRUE(trajectory[1].first.data() == replacement1.data());
    EXPECT_TRUE(trajectory[1].second == std::chrono::nanoseconds(350));

    EXPECT_TRUE(trajectory.set_points(replacement_points, replacement_times));
    EXPECT_EQ(trajectory.get_size(), 2);
    EXPECT_TRUE(trajectory[0].first.data() == replacement1.data());
    EXPECT_TRUE(trajectory[0].second == std::chrono::nanoseconds(100));
    EXPECT_TRUE(trajectory[1].first.data() == replacement2.data());
    EXPECT_TRUE(trajectory[1].second == std::chrono::nanoseconds(300));

    replacement1.set_name("bar");
    EXPECT_FALSE(trajectory.set_point(1, replacement1, std::chrono::nanoseconds(50)));
  }
}
