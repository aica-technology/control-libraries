#include <string>
#include <type_traits>

#include "state_representation_bindings.hpp"

#include <state_representation/trajectory/CartesianTrajectory.hpp>
#include <state_representation/trajectory/JointTrajectory.hpp>

template<typename StateT>
void trajectory(py::module_& m) {
  using namespace state_representation;

  std::string trajectory_type;
  if constexpr (std::is_same_v<StateT, CartesianState>) {
    trajectory_type = "CartesianTrajectory";
  } else if constexpr (std::is_same_v<StateT, JointState>) {
    trajectory_type = "JointTrajectory";
  } else {
    static_assert(false, "Unsupported type for trajectory derived");
  }
  using TrajectoryT =
      typename std::conditional<std::is_same_v<StateT, CartesianState>, CartesianTrajectory, JointTrajectory>::type;
  using PointT = typename std::conditional<
      std::is_same_v<StateT, CartesianState>, CartesianTrajectoryPoint, JointTrajectoryPoint>::type;

  // constructors
  py::class_<TrajectoryT, std::shared_ptr<TrajectoryT>> c(
      m, trajectory_type.c_str(), py::base<State>()
  );

  if constexpr (std::is_same_v<StateT, CartesianState>) {
    c.def(py::init<>(), "Empty constructor");
    c.def(
        py::init<const std::string&, const std::string&>(), "Constructor with name and reference frame provided",
        "name"_a, "reference_frame"_a = "world"
    );
  } else if constexpr (std::is_same_v<StateT, JointState>) {
    c.def(py::init<const std::string&>(), "Constructor with optional name", "name"_a = "");
  }
  c.def(
      py::init<const std::string&, const StateT&, const std::chrono::nanoseconds&>(),
      "Constructor with name, initial point, and duration provided", "name"_a, "point"_a, "duration"_a
  );
  c.def(
      py::init<const std::string&, const std::vector<StateT>&, const std::vector<std::chrono::nanoseconds>&>(),
      "Constructor with name, initial points, and durations provided", "name"_a, "points"_a, "durations"_a
  );

  // specialized member functions that are trajectory type-dependent
  if constexpr (std::is_same_v<StateT, CartesianState>) {
    c.def("get_reference_frame", &CartesianTrajectory::get_reference_frame, "Get the reference frame");
    c.def(
        "set_reference_frame", &CartesianTrajectory::set_reference_frame,
        "Set the reference frame that applies a transformation to all existing points to change the reference frame",
        "pose"_a
    );
  } else if constexpr (std::is_same_v<StateT, JointState>) {
    c.def("get_joint_names", &JointTrajectory::get_joint_names, "Get the joint names");
    c.def("set_joint_names", &JointTrajectory::set_joint_names, "Set the joint names", "joint_names"_a);
  }

  // common functions
  c.def(
      "get_duration", &TrajectoryT::get_duration, "Get the duration of the trajectory point at given index",
      "index"_a
  );
  c.def("get_durations", &TrajectoryT::get_durations, "Get list of trajectory point durations");
  c.def(
      "get_time_from_start", &TrajectoryT::get_time_from_start,
      "Get the time from start of the trajectory point at given index", "index"_a
  );
  c.def(
      "get_times_from_start", &TrajectoryT::get_times_from_start, "Get list of trajectory point times from start"
  );
  c.def(
      "get_trajectory_duration", &TrajectoryT::get_trajectory_duration, "Get the total duration of the trajectory"
  );
  c.def("get_size", &TrajectoryT::get_size, "Get number of points in trajectory");
  c.def("delete_point", py::overload_cast<>(&TrajectoryT::delete_point), "Delete the last point from trajectory");
  c.def(
      "delete_point", py::overload_cast<unsigned int>(&TrajectoryT::delete_point),
      "Delete the last point from trajectory", "index"_a
  );
  c.def("reset", &TrajectoryT::reset, "Reset trajectory");
  c.def(
      "add_point", &TrajectoryT::add_point, "Add new point and corresponding duration to trajectory", "point"_a,
      "duration"_a
  );
  c.def(
      "add_points", &TrajectoryT::add_points, "Add new points and corresponding durations to trajectory", "points"_a,
      "durations"_a
  );
  c.def(
      "insert_point", &TrajectoryT::insert_point,
      "Insert new point and corresponding duration to trajectory between two already existing points", "point"_a,
      "index"_a, "duration"_a
  );
  c.def(
      "set_point", &TrajectoryT::set_point, "Set the trajectory point at given index", "point"_a, "index"_a,
      "duration"_a
  );
  c.def("set_points", &TrajectoryT::set_points, "Set the trajectory point at given index", "points"_a, "durations"_a);
  c.def("get_points", &TrajectoryT::get_points, "Get list of trajectory points");
  c.def("get_point", &TrajectoryT::get_point, "Get the trajectory point at given index", "index"_a);
  c.def("__getitem__", &TrajectoryT::operator[], "Get the trajectory point at given index", "index"_a);
}

void bind_trajectory(py::module_& m) {
  trajectory<CartesianState>(m);
  trajectory<JointState>(m);
}
