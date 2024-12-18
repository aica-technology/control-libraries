#include "robot_model_bindings.hpp"

#include <robot_model/Model.hpp>

using namespace state_representation;

void inverse_kinematics_parameters(py::module_& m) {
  py::class_<InverseKinematicsParameters> c(m, "InverseKinematicsParameters");
  c.def(py::init());
  c.def_readwrite("damp", &InverseKinematicsParameters::damp);
  c.def_readwrite("alpha", &InverseKinematicsParameters::alpha);
  c.def_readwrite("gamma", &InverseKinematicsParameters::gamma);
  c.def_readwrite("margin", &InverseKinematicsParameters::margin);
  c.def_readwrite("tolerance", &InverseKinematicsParameters::tolerance);
  c.def_readwrite("max_number_of_iterations", &InverseKinematicsParameters::max_number_of_iterations);
}

void qp_inverse_velocity_parameters(py::module_& m) {
  py::class_<QPInverseVelocityParameters> c(m, "QPInverseVelocityParameters");
  c.def(py::init());
  c.def_readwrite("alpha", &QPInverseVelocityParameters::alpha);
  c.def_readwrite("proportional_gain", &QPInverseVelocityParameters::proportional_gain);
  c.def_readwrite("linear_velocity_limit", &QPInverseVelocityParameters::linear_velocity_limit);
  c.def_readwrite("angular_velocity_limit", &QPInverseVelocityParameters::angular_velocity_limit);
  c.def_readwrite("dt", &QPInverseVelocityParameters::dt);
}

void model(py::module_& m) {
  m.def("create_urdf_from_string", &Model::create_urdf_from_string, "Creates a URDF file with desired path and name from a string (possibly the robot description string from the ROS parameter server).", "urdf_string"_a, "desired_path"_a);

  py::class_<Model> c(m, "Model");

  c.def(py::init([](const std::string& robot_name, const std::string& urdf_path, py::object meshloader_callback) {
  std::function<std::string(const std::string&)> callback_cpp = nullptr;
  if (!meshloader_callback.is_none()) {
        callback_cpp = [meshloader_callback](const std::string& package_name) -> std::string {
              auto result = meshloader_callback(package_name).template cast<std::string>();
              return result;
        };
  }
  return new Model(robot_name, urdf_path, callback_cpp);
  }), "Constructor that creates a robot model instance with a name, URDF path, and an optional custom mesh loader callback. This constructor loads the Robot Geometries.", 
  py::arg("robot_name"), py::arg("urdf_path"), py::arg("meshloader_callback"));


  c.def(py::init<const std::string&, const std::string&>(), "Constructor that creates a robot model instance with a name and URDF path. This constructor doesn't loads the Robot Geometries.",
        py::arg("robot_name"),
        py::arg("urdf_path")
  );

  c.def(py::init<const Model&>(), "Copy constructor from another Model", "model"_a);

  c.def("get_robot_name", &Model::get_robot_name, "Getter of the robot name.");
  c.def("set_robot_name", &Model::set_robot_name, "Setter of the robot name.", "robot_name"_a);
  c.def("get_urdf_path", &Model::get_urdf_path, "Getter of the URDF path.");
  c.def("get_number_of_joints", &Model::get_number_of_joints, "Getter of the number of joints.");
  c.def("get_joint_frames", &Model::get_joint_frames, "Getter of the joint frames of the model.");
  c.def("get_frames", &Model::get_frames, "Getter of the frames of the model.");
  c.def("get_base_frame", &Model::get_base_frame, "Getter of the base frame of the model.");
  c.def("get_gravity_vector", &Model::get_gravity_vector, "Getter of the gravity vector.");
  c.def("set_gravity_vector", &Model::set_gravity_vector, "Setter of the gravity vector.", "gravity"_a);
//  c.def("get_pinocchio_model", &Model::get_pinocchio_model, "Getter of the pinocchio model.");

  
  c.def("check_collision", py::overload_cast<const JointPositions&>(&Model::check_collision), "Check if the robot is in collision at a given joint state.", "joint_positions"_a);
  c.def("compute_minimum_collision_distances", py::overload_cast<const JointPositions&>(&Model::compute_minimum_collision_distances), "Compute the minimum distances between the robot links.", "joint_positions"_a);
  c.def("get_number_of_collision_pairs", &Model::get_number_of_collision_pairs, "Get the number of collision pairs in the model.");
  c.def("is_geometry_model_initialized", &Model::is_geometry_model_initialized, "Check if the geometry model is initialized.");
  c.def(
      "compute_jacobian", py::overload_cast<const JointPositions&, const std::string&>(&Model::compute_jacobian),
      "Compute the Jacobian from a given joint state at the frame given in parameter.", "joint_positions"_a, "frame"_a = std::string(""));
  c.def(
      "compute_jacobian_time_derivative", py::overload_cast<const JointPositions&, const JointVelocities&, const std::string&>(&Model::compute_jacobian_time_derivative),
      "Compute the time derivative of the Jacobian from given joint positions and velocities at the frame in parameter.", "joint_positions"_a, "joint_velocities"_a, "frame"_a = std::string(""));
  c.def("compute_inertia_matrix", py::overload_cast<const JointPositions&>(&Model::compute_inertia_matrix), "Compute the Inertia matrix from given joint positions.", "joint_positions"_a);
  c.def(
      "compute_inertia_torques", py::overload_cast<const JointState&>(&Model::compute_inertia_torques),
      "Compute the Inertia torques, i.e the inertia matrix multiplied by the joint accelerations. Joint positions are needed as well for computations of the inertia matrix.", "joint_state"_a);
  c.def("compute_coriolis_matrix", py::overload_cast<const JointState&>(&Model::compute_coriolis_matrix), "Compute the Coriolis matrix from a given joint state.", "joint_state"_a);
  c.def(
      "compute_coriolis_torques", py::overload_cast<const JointState&>(&Model::compute_coriolis_torques),
      "Compute the Coriolis torques, i.e. the Coriolis matrix multiplied by the joint velocities and express the result as a JointTorques.", "joint_state"_a);
  c.def("compute_gravity_torques", py::overload_cast<const JointPositions&>(&Model::compute_gravity_torques), "Compute the gravity torques.", "joint_positions"_a);

  c.def("forward_kinematics", py::overload_cast<const JointPositions&, const std::vector<std::string>&>(&Model::forward_kinematics),
        "Compute the forward kinematics, i.e. the pose of certain frames from the joint positions", "joint_positions"_a, "frames"_a);
  c.def("forward_kinematics", py::overload_cast<const JointPositions&, const std::string&>(&Model::forward_kinematics),
      "Compute the forward kinematics, i.e. the pose of the frame from the joint positions", "joint_positions"_a, "frame"_a = std::string(""));

  c.def("inverse_kinematics", py::overload_cast<const CartesianPose&, const InverseKinematicsParameters&, const std::string&>(&Model::inverse_kinematics),
        "Compute the inverse kinematics, i.e. joint positions from the pose of the end-effector in an iterative manner", "cartesian_pose"_a, "parameters"_a = InverseKinematicsParameters(), "frame"_a = std::string(""));
  c.def("inverse_kinematics", py::overload_cast<const CartesianPose&, const JointPositions&, const InverseKinematicsParameters&, const std::string&>(&Model::inverse_kinematics),
        " Compute the inverse kinematics, i.e. joint positions from the pose of the end-effector", "cartesian_pose"_a, "joint_positions"_a, "parameters"_a = InverseKinematicsParameters(), "frame"_a = std::string(""));

  c.def("forward_velocity", py::overload_cast<const JointState&, const std::vector<std::string>&>(&Model::forward_velocity),
        "Compute the forward velocity kinematics, i.e. the twist of certain frames from the joint states", "joint_state"_a, "frames"_a);
  c.def("forward_velocity", py::overload_cast<const JointState&, const std::string&>(&Model::forward_velocity),
        "Compute the forward velocity kinematics, i.e. the twist of the end-effector from the joint velocities", "joint_state"_a, "frame"_a = std::string(""));

  c.def("inverse_velocity", py::overload_cast<const std::vector<CartesianTwist>&, const JointPositions&, const std::vector<std::string>&, const double>(&Model::inverse_velocity),
        "Compute the inverse velocity kinematics, i.e. joint velocities from the velocities of the frames in parameter the Jacobian", "cartesian_twists"_a, "joint_positions"_a, "frames"_a, "dls_lambda"_a = 0.0);
  c.def("inverse_velocity", py::overload_cast<const CartesianTwist&, const JointPositions&, const std::string&, const double>(&Model::inverse_velocity),
        "Compute the inverse velocity kinematics, i.e. joint velocities from the twist of the end-effector using the Jacobian", "cartesian_twist"_a, "joint_positions"_a, "frame"_a = std::string(""), "dls_lambda"_a = 0.0);
  c.def("inverse_velocity", py::overload_cast<const std::vector<CartesianTwist>&, const JointPositions&, const QPInverseVelocityParameters&, const std::vector<std::string>&>(&Model::inverse_velocity),
        "Compute the inverse velocity kinematics, i.e. joint velocities from the velocities of the frames in parameter using the QP optimization method", "cartesian_twists"_a, "joint_positions"_a, "parameters"_a, "frames"_a);
  c.def("inverse_velocity", py::overload_cast<const CartesianTwist&, const JointPositions&, const QPInverseVelocityParameters&, const std::string&>(&Model::inverse_velocity),
        "Compute the inverse velocity kinematics, i.e. joint velocities from the twist of the end-effector using the QP optimization method", "cartesian_twist"_a, "joint_positions"_a, "parameters"_a, "frame"_a = std::string(""));

  c.def("in_range", [](Model& self, const JointPositions& joint_positions) -> bool { return self.in_range(joint_positions); },
        "Check if the joint positions are inside the limits provided by the model", "joint_positions"_a);
  c.def("in_range", [](Model& self, const JointVelocities& joint_velocities) -> bool { return self.in_range(joint_velocities); },
        "Check if the joint velocities are inside the limits provided by the model", "joint_velocities"_a);
  c.def("in_range", [](Model& self, const JointTorques& joint_torques) -> bool { return self.in_range(joint_torques); },
        "Check if the joint torques are inside the limits provided by the model", "joint_torques"_a);
  c.def("in_range", [](Model& self, const JointState& joint_state) -> bool { return self.in_range(joint_state); },
        "Check if the joint state variables (positions, velocities & torques) are inside the limits provided by the model", "joint_state"_a);

  c.def(
      "clamp_in_range",
      [](Model& self, const JointState& joint_state, const JointStateVariable& state_variable_type) -> JointState {
        return self.clamp_in_range(joint_state, state_variable_type);
      },
      "Clamp the joint state variables (positions, velocities & torques) according to the limits provided by the model",
      "joint_state"_a, "state_variable_type"_a);
  c.def("clamp_in_range", [](Model& self, const JointState& joint_state) -> JointState { return self.clamp_in_range(joint_state); },
        "Clamp the joint state variables (positions, velocities & torques) according to the limits provided by the model", "joint_state"_a);
}

void bind_model(py::module_& m) {
  inverse_kinematics_parameters(m);
  qp_inverse_velocity_parameters(m);
  model(m);
}