#include "state_representation/exceptions/IncompatibleStatesException.hpp"
#include "state_representation/space/joint/JointState.hpp"
#include "state_representation/trajectory/CartesianTrajectory.hpp"
#include "state_representation/trajectory/JointTrajectory.hpp"

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"
#include "state_representation/space/cartesian/CartesianState.hpp"

#include <chrono>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <gtest/gtest.h>
#include <stdexcept>
#include <unistd.h>
#include <vector>

using namespace state_representation;

class TrajectoryBaseInterface : public TrajectoryBase<TrajectoryPoint> {
public:
  using TrajectoryBase<TrajectoryPoint>::TrajectoryBase;
  using TrajectoryBase<TrajectoryPoint>::add_point;
  using TrajectoryBase<TrajectoryPoint>::get_point;
  using TrajectoryBase<TrajectoryPoint>::get_points;
  using TrajectoryBase<TrajectoryPoint>::insert_point;
  using TrajectoryBase<TrajectoryPoint>::operator[];
};

TEST(TrajectoryTest, TestTrajectoryBase) {
  TrajectoryBaseInterface trajectory;
  EXPECT_EQ(trajectory.get_size(), 0);

  TrajectoryPoint point0;
  TrajectoryPoint point1;
  TrajectoryPoint point2;
  point0.data = Eigen::VectorXd::Random(3);
  point0.duration = std::chrono::nanoseconds(100);
  point1.data = Eigen::VectorXd::Random(3);
  point1.duration = std::chrono::nanoseconds(200);
  point2.data = Eigen::VectorXd::Random(3);
  point2.duration = std::chrono::nanoseconds(300);

  trajectory.add_point(point0);
  trajectory.add_point(point1);
  EXPECT_EQ(trajectory.get_size(), 2);
  EXPECT_EQ(trajectory.get_durations().size(), 2);
  EXPECT_EQ(trajectory.get_times_from_start().size(), 2);
  EXPECT_EQ(trajectory.get_point(0).data, point0.data);
  EXPECT_EQ(trajectory.get_point(1).data, point1.data);
  EXPECT_EQ(trajectory.get_times_from_start()[1], point0.duration + point1.duration);

  auto points = trajectory.get_points();
  EXPECT_EQ(points.size(), 2);
  EXPECT_EQ(points[0].data, point0.data);
  EXPECT_EQ(points[1].data, point1.data);

  EXPECT_THROW(trajectory.insert_point(point2, 10), std::out_of_range);
  EXPECT_NO_THROW(trajectory.insert_point(point2, 1));
  EXPECT_EQ(trajectory.get_size(), 3);
  EXPECT_EQ(trajectory[1].data, point2.data);
  EXPECT_EQ(trajectory.get_times_from_start()[1], point0.duration + point2.duration);
  EXPECT_EQ(trajectory.get_times_from_start()[2], point0.duration + point2.duration + point1.duration);

  trajectory.reset();
  EXPECT_EQ(trajectory.get_size(), 0);
  trajectory.reset();
  EXPECT_TRUE(trajectory.is_empty());
}

TEST(TrajectoryTest, CartesianTrajectory) {
  {
    CartesianTrajectory trajectory("foo_trajectory", CartesianState::Random("foo"), std::chrono::nanoseconds(100));
    EXPECT_EQ(trajectory.get_size(), 1);
  }

  {
    auto points = {
        CartesianState::Random("foo"),
        CartesianState::Random("bar"),
    };
    auto durations = {std::chrono::nanoseconds(100), std::chrono::nanoseconds(200)};
    CartesianTrajectory trajectory("foo_trajectory", points, durations);
    EXPECT_EQ(trajectory.get_size(), 2);
  }

  CartesianTrajectory trajectory;
  EXPECT_EQ(trajectory.get_size(), 0);

  {// incompatible sizes
    CartesianState state;
    EXPECT_THROW(trajectory.add_point(state, std::chrono::nanoseconds(100)), exceptions::EmptyStateException);
    EXPECT_EQ(trajectory.get_size(), 0);

    auto points = {
        CartesianState::Random("foo"),
        CartesianState::Random("bar"),
    };
    auto durations = {std::chrono::nanoseconds(100)};
    EXPECT_THROW(trajectory.set_points(points, durations), exceptions::IncompatibleSizeException);
    EXPECT_EQ(trajectory.get_size(), 0);
  }

  {// incompatible reference frames
    CartesianState point0("empty", "world1");
    auto point1 = CartesianState::Random("foo", "world2");
    auto point2 = CartesianState::Random("bar", "world3");
    EXPECT_THROW(trajectory.add_point(point0, std::chrono::nanoseconds(100)), exceptions::EmptyStateException);
    EXPECT_NO_THROW(trajectory.add_point(point1, std::chrono::nanoseconds(200)));
    EXPECT_THROW(
        trajectory.add_point(point2, std::chrono::nanoseconds(200)), exceptions::IncompatibleReferenceFramesException
    );
    EXPECT_STREQ(trajectory.get_reference_frame().c_str(), "world2");
    trajectory.reset();
    EXPECT_STREQ(trajectory.get_reference_frame().c_str(), "world");
  }

  auto point0 = CartesianState::Random("foo");
  auto point1 = CartesianState::Random("bar");
  auto point2 = CartesianState::Random("baz");

  trajectory.add_point(point0, std::chrono::nanoseconds(100));
  EXPECT_EQ(trajectory[0].first.data(), point0.data());
  EXPECT_EQ(trajectory[0].first.get_name(), point0.get_name());
  EXPECT_EQ(trajectory[0].first.get_reference_frame(), point0.get_reference_frame());
  trajectory.add_point(point2, std::chrono::nanoseconds(200));

  EXPECT_NO_THROW(trajectory.insert_point(point2, std::chrono::nanoseconds(300), 1));
  EXPECT_EQ(trajectory.get_size(), 3);

  point0.set_name("foofoo");
  EXPECT_NO_THROW(trajectory.set_point(point0, std::chrono::nanoseconds(50), 0));
  EXPECT_EQ(trajectory[0].first.get_name(), point0.get_name());

  std::vector<CartesianState> points = {point0, point1, point2};
  std::vector<std::chrono::nanoseconds> durations = {
      std::chrono::nanoseconds(10), std::chrono::nanoseconds(20), std::chrono::nanoseconds(30)
  };
  trajectory.set_points(points, durations);
  for (unsigned int i = 0; i < trajectory.get_size(); ++i) {
    EXPECT_EQ(trajectory[i].first.data(), points[i].data());
    EXPECT_EQ(trajectory[i].first.get_name(), points[i].get_name());
    EXPECT_EQ(trajectory[i].first.get_reference_frame(), points[i].get_reference_frame());
    EXPECT_EQ(trajectory[i].second, durations[i]);
  }
}

TEST(TrajectoryTest, JointTrajectory) {
  {
    JointTrajectory trajectory("foo_trajectory", JointState::Random("foo", 25), std::chrono::nanoseconds(100));
    EXPECT_EQ(trajectory.get_size(), 1);
  }

  {
    auto points = {
        JointState::Random("foo", 25),
        JointState::Random("foo", 25),
    };
    auto durations = {std::chrono::nanoseconds(100), std::chrono::nanoseconds(200)};
    JointTrajectory trajectory("foo_trajectory", points, durations);
    EXPECT_EQ(trajectory.get_size(), 2);
  }

  JointTrajectory trajectory("foo");
  EXPECT_EQ(trajectory.get_size(), 0);

  {// incompatible name
    auto point0 = JointState::Random("foo", 25);
    auto point1 = JointState::Random("bar", 25);
    EXPECT_NO_THROW(trajectory.add_point(point0, std::chrono::nanoseconds(100)));
    EXPECT_THROW(trajectory.add_point(point1, std::chrono::nanoseconds(200)), exceptions::IncompatibleStatesException);
    trajectory.reset();
  }

  {// incompatible sizes
    JointState state;
    EXPECT_THROW(trajectory.add_point(state, std::chrono::nanoseconds(100)), exceptions::EmptyStateException);
    EXPECT_EQ(trajectory.get_size(), 0);

    auto points = {
        JointState::Random("foo", 25),
        JointState::Random("foo", 25),
    };
    auto durations = {std::chrono::nanoseconds(100)};
    EXPECT_THROW(trajectory.set_points(points, durations), exceptions::IncompatibleSizeException);
    EXPECT_EQ(trajectory.get_size(), 0);
  }

  {// incompatible joint names
    auto states = {
        JointState::Random("foo", {"j_foo_1", "j_foo_2"}), JointState::Random("foo", {"j_bar_1", "j_bar_2"})
    };
    auto durations = {std::chrono::nanoseconds(100), std::chrono::nanoseconds(200)};
    EXPECT_THROW(trajectory.set_points(states, durations), exceptions::IncompatibleStatesException);
  }

  {// check joint names
    std::vector<std::string> joint_names = {"j_foo_1", "j_foo_2", "j_foo_3"};
    trajectory.add_point(JointState::Random("foo", joint_names), std::chrono::nanoseconds(100));
    EXPECT_EQ(trajectory.get_joint_names(), joint_names);
    trajectory.reset();
    EXPECT_EQ(trajectory.get_size(), 0);
  }

  auto point0 = JointState::Random("foo", 25);
  auto point1 = JointState::Random("foo", 25);
  auto point2 = JointState::Random("foo", 25);

  trajectory.add_point(point0, std::chrono::nanoseconds(100));

  EXPECT_EQ(trajectory[0].first.data(), point0.data());
  EXPECT_EQ(trajectory[0].first.get_name(), point0.get_name());
  EXPECT_EQ(trajectory[0].first.get_names(), trajectory.get_joint_names());
  EXPECT_EQ(trajectory[0].first.get_names(), point0.get_names());
  trajectory.add_point(point2, std::chrono::nanoseconds(200));

  EXPECT_NO_THROW(trajectory.insert_point(point2, std::chrono::nanoseconds(300), 1));
  EXPECT_EQ(trajectory.get_size(), 3);

  point0.set_name("foofoo");
  EXPECT_THROW(trajectory.set_point(point0, std::chrono::nanoseconds(50), 0), exceptions::IncompatibleStatesException);
  EXPECT_NE(trajectory[0].first.get_name(), point0.get_name());
  point0.set_name("foo");

  std::vector<JointState> points = {point0, point1, point2};
  std::vector<std::chrono::nanoseconds> durations = {
      std::chrono::nanoseconds(10), std::chrono::nanoseconds(20), std::chrono::nanoseconds(30)
  };
  trajectory.set_points(points, durations);
  for (unsigned int i = 0; i < trajectory.get_size(); ++i) {
    EXPECT_EQ(trajectory[i].first.data(), points[i].data());
    EXPECT_EQ(trajectory[i].first.get_name(), points[i].get_name());
    EXPECT_EQ(trajectory[i].first.get_names(), points[i].get_names());
    EXPECT_EQ(trajectory[i].first.get_name(), points[i].get_name());
    EXPECT_EQ(trajectory[i].second, durations[i]);
  }
}
