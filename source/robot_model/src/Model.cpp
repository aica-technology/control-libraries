#include <regex>
#include <set>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include "robot_model/Model.hpp"
#include "robot_model/exceptions/FrameNotFoundException.hpp"
#include "robot_model/exceptions/InverseKinematicsNotConvergingException.hpp"
#include "robot_model/exceptions/InvalidJointStateSizeException.hpp"
#include "robot_model/exceptions/CollisionGeometryException.hpp" 

namespace robot_model {
Model::Model(const std::string& robot_name, 
             const std::string& urdf_path,
             const std::optional<std::function<std::string(const std::string&)>>& meshloader_callback
             ):
    robot_name_(robot_name),
    urdf_path_(urdf_path),
    meshloader_callback_(meshloader_callback),
    load_collision_geometries_(true)
    {
  this->init_model();
}

Model::Model(const std::string& robot_name, const std::string& urdf_path) :
    robot_name_(robot_name),
    urdf_path_(urdf_path)
    {
  this->init_model();
}


bool Model::create_urdf_from_string(const std::string& urdf_string, const std::string& desired_path) {
  std::ofstream file(desired_path);
  if (file.good() && file.is_open()) {
    file << urdf_string;
    file.close();
    return true;
  }
  return false;
}

std::vector<std::string> Model::resolve_package_paths_in_urdf(std::string& urdf) const {
  std::set<std::string> package_names;
  std::regex package_path_pattern(R"(filename=\"([^\"]+)\")");
  std::regex package_name_pattern(R"(package://([^/]+)/)");

  std::smatch matches_file;
  std::smatch matches_package_name;
  auto start = urdf.cbegin();
  auto end = urdf.cend();

  // Extract package paths
  while (std::regex_search(start, end, matches_file, package_path_pattern)) {
    std::string path = matches_file[1];
    if (std::regex_search(path, matches_package_name, package_name_pattern)) {
      package_names.insert(matches_package_name[1]);
    }
    start = matches_file[0].second;
  }

  std::vector<std::string> package_paths;
  for (const auto& package_name : package_names) {
    if (meshloader_callback_) {
      auto package_path = (*this->meshloader_callback_)(package_name);
      auto target = "package://" + package_name + "/";
      auto replacement = package_path;
      size_t start_position = 0;
      while ((start_position = urdf.find(target, start_position)) != std::string::npos) {
        // Replace the target with the replacement string
        urdf.replace(start_position, target.length(), replacement);
        // Move past the last replacement
        start_position += replacement.length();
      }
      package_paths.push_back(package_path);
    }
  }
  return package_paths;
}

void Model::init_model() {
  std::ifstream file_stream(this->get_urdf_path());
  if (!file_stream.is_open()) {
    throw std::runtime_error("Unable to open file: " + this->get_urdf_path());
  }
  std::stringstream buffer;
  buffer << file_stream.rdbuf();
  auto urdf = buffer.str();

  pinocchio::urdf::buildModelFromXML(urdf, this->robot_model_);
  this->robot_data_ = pinocchio::Data(this->robot_model_);

  if (this->load_collision_geometries_) {
    this->init_geom_model(urdf);
  }

  // get the frames
  std::vector<std::string> frames;
  for (auto& f : this->robot_model_.frames) {
    frames.push_back(f.name);
  }
  // remove universe and root_joint frame added by Pinocchio
  this->frames_ = std::vector<std::string>(frames.begin() + 2, frames.end());

  // define the QP solver
  this->qp_solver_ = std::make_unique<QPSolver>(
      this->get_number_of_joints(), this->robot_model_.lowerPositionLimit, this->robot_model_.upperPositionLimit,
      this->robot_model_.velocityLimit);
}

void Model::init_geom_model(std::string urdf) {
  try {
    auto package_paths = this->resolve_package_paths_in_urdf(urdf);
    pinocchio::urdf::buildGeom(
        this->robot_model_, std::istringstream(urdf), pinocchio::COLLISION, this->geom_model_, package_paths);
    this->geom_model_.addAllCollisionPairs();

    std::vector<pinocchio::CollisionPair> excluded_pairs = this->generate_joint_exclusion_list();

    // remove collision pairs for linked joints (i.e. parent-child joints)
    for (const auto& pair : excluded_pairs) {
      this->geom_model_.removeCollisionPair(pair);
    }

    this->geom_data_ = pinocchio::GeometryData(this->geom_model_);
  } catch (const std::exception& ex) {
    throw robot_model::exceptions::CollisionGeometryException(
        "Failed to initialize geometry model for " + this->get_robot_name() + ": " + ex.what());
  }
}

std::vector<pinocchio::CollisionPair> Model::generate_joint_exclusion_list() {
  std::vector<pinocchio::CollisionPair> excluded_pairs;
  // Iterate through all joints, except the universe joint (0), which has no parent
  for (pinocchio::JointIndex joint_id = 1u; joint_id < static_cast<pinocchio::JointIndex>(this->robot_model_.njoints);
       ++joint_id) {
    // Find the parent joint of the current joint
    pinocchio::JointIndex parent_id = this->robot_model_.parents[joint_id];

    // TODO: Replace this logic with actual geometry index lookup
    auto getGeometryIndexForJoint = [](pinocchio::JointIndex joint_id) -> int {
      return static_cast<int>(joint_id);
    };

    int geometryIndex1 = getGeometryIndexForJoint(joint_id);
    int geometryIndex2 = getGeometryIndexForJoint(parent_id);

    // Check if the geometry indices are not equal
    if (geometryIndex1 != geometryIndex2) {
      excluded_pairs.push_back(pinocchio::CollisionPair(geometryIndex2, geometryIndex1));
    }
  }
  return excluded_pairs;
}

unsigned int Model::get_number_of_collision_pairs() {
  return this->geom_model_.collisionPairs.size();
}

bool Model::is_geometry_model_initialized() {
  return !this->geom_model_.collisionPairs.empty();
}

bool Model::check_collision(const state_representation::JointPositions& joint_positions) {
  if (!this->is_geometry_model_initialized()) {
    throw robot_model::exceptions::CollisionGeometryException(
        "Geometry model not loaded for " + this->get_robot_name());
  }

  Eigen::VectorXd configuration = joint_positions.get_positions();

  pinocchio::computeCollisions(
      this->robot_model_, this->robot_data_, this->geom_model_, this->geom_data_, configuration, true);

  for (size_t pair_index = 0; pair_index < this->geom_model_.collisionPairs.size(); ++pair_index) {
    const auto& collision_result = this->geom_data_.collisionResults[pair_index];
    if (collision_result.isCollision()) {
      return true;
    }
  }
  return false;
}

Eigen::MatrixXd Model::compute_minimum_collision_distances(const state_representation::JointPositions& joint_positions) {
  if (!this->is_geometry_model_initialized()) {
    throw robot_model::exceptions::CollisionGeometryException(
        "Geometry model not loaded for " + this->get_robot_name());
  }
  Eigen::VectorXd configuration = joint_positions.get_positions();
  pinocchio::computeDistances(
      this->robot_model_, this->robot_data_, this->geom_model_, this->geom_data_, configuration);

  // nb_joints is the number of joints in the robot model
  unsigned int nb_joints = this->get_number_of_joints();

  // create a square matrix to store the distances and initialize to zero
  Eigen::MatrixXd distances = Eigen::MatrixXd::Zero(nb_joints, nb_joints);

  // iterate over the collision pairs and extract the distances
  unsigned int pair_index = 0;
  for (unsigned int row_index = 0; row_index < nb_joints; ++row_index) {
    for (unsigned int column_index = row_index + 1; column_index < nb_joints; ++column_index) {
      distances(row_index, column_index) = this->geom_data_.distanceResults[pair_index].min_distance;
      distances(column_index, row_index) = distances(row_index, column_index);
      pair_index++;
    }
  }

  return distances;
}
  
std::vector<unsigned int> Model::get_frame_ids(const std::vector<std::string>& frames) {
  std::vector<unsigned int> frame_ids;
  frame_ids.reserve(frames.size());

  for (auto& frame : frames) {
    if (frame.empty()) {
      // get last frame if none specified
      frame_ids.push_back(this->robot_model_.frames.size() - 1);
    } else {
      // throw error if specified frame does not exist
      if (!this->robot_model_.existFrame(frame)) {
        throw exceptions::FrameNotFoundException(frame);
      }
      frame_ids.push_back(this->robot_model_.getFrameId(frame));
    }
  }
  return frame_ids;
}

unsigned int Model::get_frame_id(const std::string& frame) {
  return get_frame_ids(std::vector<std::string>{frame}).back();
}

state_representation::Jacobian Model::compute_jacobian(const state_representation::JointPositions& joint_positions,
                                                       unsigned int frame_id) {
  if (joint_positions.get_size() != this->get_number_of_joints()) {
    throw exceptions::InvalidJointStateSizeException(joint_positions.get_size(), this->get_number_of_joints());
  }
  // compute the Jacobian from the joint state
  pinocchio::Data::Matrix6x J(6, this->get_number_of_joints());
  J.setZero();
  pinocchio::computeFrameJacobian(this->robot_model_,
                                  this->robot_data_,
                                  joint_positions.data(),
                                  frame_id,
                                  pinocchio::LOCAL_WORLD_ALIGNED,
                                  J);
  // the model does not have any reference frame
  return state_representation::Jacobian(this->get_robot_name(),
                                        this->get_joint_frames(),
                                        this->robot_model_.frames[frame_id].name,
                                        J,
                                        this->get_base_frame());
}

state_representation::Jacobian Model::compute_jacobian(const state_representation::JointPositions& joint_positions,
                                                       const std::string& frame) {
  auto frame_id = get_frame_id(frame);
  return this->compute_jacobian(joint_positions, frame_id);
}

Eigen::MatrixXd Model::compute_jacobian_time_derivative(const state_representation::JointPositions& joint_positions,
                                                        const state_representation::JointVelocities& joint_velocities,
                                                        unsigned int frame_id) {
  if (joint_positions.get_size() != this->get_number_of_joints()) {
    throw exceptions::InvalidJointStateSizeException(joint_positions.get_size(), this->get_number_of_joints());
  }
  if (joint_velocities.get_size() != this->get_number_of_joints()) {
    throw exceptions::InvalidJointStateSizeException(joint_velocities.get_size(), this->get_number_of_joints());
  }
  // compute the Jacobian from the joint state
  pinocchio::Data::Matrix6x dJ = Eigen::MatrixXd::Zero(6, this->get_number_of_joints());
  pinocchio::computeJointJacobiansTimeVariation(this->robot_model_,
                                                this->robot_data_,
                                                joint_positions.data(),
                                                joint_velocities.data());
  pinocchio::getFrameJacobianTimeVariation(this->robot_model_,
                                           this->robot_data_,
                                           frame_id,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           dJ);
  // the model does not have any reference frame
  return dJ;
}

Eigen::MatrixXd Model::compute_jacobian_time_derivative(const state_representation::JointPositions& joint_positions,
                                                        const state_representation::JointVelocities& joint_velocities,
                                                        const std::string& frame) {
  auto frame_id = get_frame_id(frame);
  return this->compute_jacobian_time_derivative(joint_positions, joint_velocities, frame_id);
}

Eigen::MatrixXd Model::compute_inertia_matrix(const state_representation::JointPositions& joint_positions) {
  // compute only the upper part of the triangular inertia matrix stored in robot_data_.M
  pinocchio::crba(this->robot_model_, this->robot_data_, joint_positions.data());
  // copy the symmetric lower part
  this->robot_data_.M.triangularView<Eigen::StrictlyLower>() =
      this->robot_data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  return this->robot_data_.M;
}

state_representation::JointTorques Model::compute_inertia_torques(const state_representation::JointState& joint_state) {
  Eigen::MatrixXd inertia = this->compute_inertia_matrix(joint_state);
  return state_representation::JointTorques(joint_state.get_name(),
                                            joint_state.get_names(),
                                            inertia * joint_state.get_accelerations());
}

Eigen::MatrixXd Model::compute_coriolis_matrix(const state_representation::JointState& joint_state) {
  return pinocchio::computeCoriolisMatrix(this->robot_model_,
                                          this->robot_data_,
                                          joint_state.get_positions(),
                                          joint_state.get_velocities());
}

state_representation::JointTorques
Model::compute_coriolis_torques(const state_representation::JointState& joint_state) {
  Eigen::MatrixXd coriolis_matrix = this->compute_coriolis_matrix(joint_state);
  return state_representation::JointTorques(joint_state.get_name(),
                                            joint_state.get_names(),
                                            coriolis_matrix * joint_state.get_velocities());
}

state_representation::JointTorques
Model::compute_gravity_torques(const state_representation::JointPositions& joint_positions) {
  Eigen::VectorXd gravity_torque =
      pinocchio::computeGeneralizedGravity(this->robot_model_, this->robot_data_, joint_positions.data());
  return state_representation::JointTorques(joint_positions.get_name(), joint_positions.get_names(), gravity_torque);
}

state_representation::CartesianPose Model::forward_kinematics(const state_representation::JointPositions& joint_positions,
                                                              unsigned int frame_id) {
  return this->forward_kinematics(joint_positions, std::vector<unsigned int>{frame_id}).front();
}

std::vector<state_representation::CartesianPose> Model::forward_kinematics(const state_representation::JointPositions& joint_positions,
                                                                           const std::vector<unsigned int>& frame_ids) {
  if (joint_positions.get_size() != this->get_number_of_joints()) {
    throw exceptions::InvalidJointStateSizeException(joint_positions.get_size(), this->get_number_of_joints());
  }
  std::vector<state_representation::CartesianPose> pose_vector;
  pinocchio::forwardKinematics(this->robot_model_, this->robot_data_, joint_positions.data());
  for (unsigned int id : frame_ids) {
    if (id >= static_cast<unsigned int>(this->robot_model_.nframes)) {
      throw exceptions::FrameNotFoundException(std::to_string(id));
    }
    pinocchio::updateFramePlacement(this->robot_model_, this->robot_data_, id);
    pinocchio::SE3 pose = this->robot_data_.oMf[id];
    Eigen::Vector3d translation = pose.translation();
    Eigen::Quaterniond quaternion;
    pinocchio::quaternion::assignQuaternion(quaternion, pose.rotation());
    state_representation::CartesianPose frame_pose(this->robot_model_.frames[id].name,
                                                   translation,
                                                   quaternion,
                                                   this->get_base_frame());
    pose_vector.push_back(frame_pose);
  }
  return pose_vector;
}

state_representation::CartesianPose Model::forward_kinematics(const state_representation::JointPositions& joint_positions,
                                                              const std::string& frame) {
  std::string actual_frame = frame.empty() ? this->robot_model_.frames.back().name : frame;
  return this->forward_kinematics(joint_positions, std::vector<std::string>{actual_frame}).front();
}

std::vector<state_representation::CartesianPose> Model::forward_kinematics(const state_representation::JointPositions& joint_positions,
                                                                           const std::vector<std::string>& frames) {
  auto frame_ids = get_frame_ids(frames);
  return this->forward_kinematics(joint_positions, frame_ids);
}

Eigen::MatrixXd Model::cwln_weighted_matrix(const state_representation::JointPositions& joint_positions,
                                            const double margin) {
  Eigen::MatrixXd W_b = Eigen::MatrixXd::Identity(this->robot_model_.nq, this->robot_model_.nq);
  for (int n = 0; n < this->robot_model_.nq; ++n) {
    double d = 1;
    W_b(n, n) = 1;
    if (joint_positions.data()[n] < this->robot_model_.lowerPositionLimit[n] + margin) {
      if (joint_positions.data()[n] < this->robot_model_.lowerPositionLimit[n]) {
        W_b(n, n) = 0;
      } else {
        d = (this->robot_model_.lowerPositionLimit[n] + margin - joint_positions.data()[n]) / margin;
        W_b(n, n) = -2 * d * d * d + 3 * d * d;
      }
    } else if (this->robot_model_.upperPositionLimit[n] - margin < joint_positions.data()[n]) {
      if (this->robot_model_.upperPositionLimit[n] < joint_positions.data()[n]) {
        W_b(n, n) = 0;
      } else {
        d = (joint_positions.data()[n] - (this->robot_model_.upperPositionLimit[n] - margin)) / margin;
        W_b(n, n) = -2 * d * d * d + 3 * d * d;
      }
    }
  }
  return W_b;
}

Eigen::VectorXd Model::cwln_repulsive_potential_field(const state_representation::JointPositions& joint_positions,
                                                      double margin) {
  Eigen::VectorXd Psi(this->robot_model_.nq);
  Eigen::VectorXd q = joint_positions.data();
  for (int i = 0; i < this->robot_model_.nq; ++i) {
    Psi[i] = 0;
    if (q[i] < this->robot_model_.lowerPositionLimit[i] + margin) {
      Psi[i] = this->robot_model_.upperPositionLimit[i] - margin
          - std::max(q[i], this->robot_model_.lowerPositionLimit[i]);
    } else if (this->robot_model_.upperPositionLimit[i] - margin < q[i]) {
      Psi[i] = this->robot_model_.lowerPositionLimit[i] + margin
          - std::min(q[i], this->robot_model_.upperPositionLimit[i]);
    }
  }
  return Psi;
}

state_representation::JointPositions
Model::inverse_kinematics(const state_representation::CartesianPose& cartesian_pose,
                          const state_representation::JointPositions& joint_positions,
                          const InverseKinematicsParameters& parameters,
                          const std::string& frame) {
  std::string actual_frame = frame.empty() ? this->robot_model_.frames.back().name : frame;
  if (!this->robot_model_.existFrame(actual_frame)) {
    throw exceptions::FrameNotFoundException(actual_frame);
  }
  // 1 second for the Newton-Raphson method
  const std::chrono::nanoseconds dt(static_cast<int>(1e9));
  // initialization of the loop variables
  state_representation::JointPositions q(joint_positions);
  state_representation::JointVelocities dq(this->get_robot_name(), joint_positions.get_names());
  state_representation::Jacobian J(this->get_robot_name(),
                                   this->get_joint_frames(),
                                   actual_frame,
                                   this->get_base_frame());
  Eigen::MatrixXd J_b = Eigen::MatrixXd::Zero(6, this->get_number_of_joints());
  Eigen::MatrixXd JJt(6, 6);
  Eigen::MatrixXd W_b = Eigen::MatrixXd::Identity(this->get_number_of_joints(), this->get_number_of_joints());
  Eigen::MatrixXd W_c = Eigen::MatrixXd::Identity(this->get_number_of_joints(), this->get_number_of_joints());
  Eigen::VectorXd psi(this->get_number_of_joints());
  Eigen::VectorXd err(6);
  for (unsigned int i = 0; i < parameters.max_number_of_iterations; ++i) {
    err = ((cartesian_pose - this->forward_kinematics(q, actual_frame)) / dt).data();
    // break in case of convergence
    if (err.cwiseAbs().maxCoeff() < parameters.tolerance) {
      return q;
    }
    J = this->compute_jacobian(q, actual_frame);
    W_b = this->cwln_weighted_matrix(q, parameters.margin);
    W_c = Eigen::MatrixXd::Identity(this->get_number_of_joints(), this->get_number_of_joints()) - W_b;
    psi = parameters.gamma * this->cwln_repulsive_potential_field(q, parameters.margin);
    J_b = J * W_b;
    JJt.noalias() = J_b * J_b.transpose();
    JJt.diagonal().array() += parameters.damp;
    dq.set_velocities(W_c * psi + parameters.alpha * W_b * (J_b.transpose() * JJt.ldlt().solve(err - J * W_c * psi)));
    q += dq * dt;
    q = this->clamp_in_range(q);
  }
  throw (exceptions::InverseKinematicsNotConvergingException(parameters.max_number_of_iterations,
                                                             err.array().abs().maxCoeff()));
}

state_representation::JointPositions
Model::inverse_kinematics(const state_representation::CartesianPose& cartesian_pose,
                          const InverseKinematicsParameters& parameters,
                          const std::string& frame) {
  Eigen::VectorXd q(pinocchio::neutral(this->robot_model_));
  state_representation::JointPositions positions(this->get_robot_name(), this->get_joint_frames(), q);
  return this->inverse_kinematics(cartesian_pose, positions, parameters, frame);
}

std::vector<state_representation::CartesianTwist>
Model::forward_velocity(const state_representation::JointState& joint_state,
                        const std::vector<std::string>& frames) {
  std::vector<state_representation::CartesianTwist> cartesian_twists(frames.size());
  for (std::size_t i = 0; i < frames.size(); ++i) {
    cartesian_twists.at(i) = this->compute_jacobian(joint_state, frames.at(i))
        * static_cast<state_representation::JointVelocities>(joint_state);
  }
  return cartesian_twists;
}

state_representation::CartesianTwist Model::forward_velocity(const state_representation::JointState& joint_state,
                                                             const std::string& frame) {
  return this->forward_velocity(joint_state, std::vector<std::string>{frame}).front();
}

void Model::check_inverse_velocity_arguments(const std::vector<state_representation::CartesianTwist>& cartesian_twists,
                                             const state_representation::JointPositions& joint_positions,
                                             const std::vector<std::string>& frames) {
  if (cartesian_twists.size() != frames.size()) {
    throw std::invalid_argument("The number of provided twists and frames does not match");
  }
  if (joint_positions.get_size() != this->get_number_of_joints()) {
    throw exceptions::InvalidJointStateSizeException(joint_positions.get_size(), this->get_number_of_joints());
  }
  for (auto& frame : frames) {
    if (!this->robot_model_.existFrame(frame)) {
      throw exceptions::FrameNotFoundException(frame);
    }
  }
}

state_representation::JointVelocities
Model::inverse_velocity(const std::vector<state_representation::CartesianTwist>& cartesian_twists,
                        const state_representation::JointPositions& joint_positions,
                        const std::vector<std::string>& frames,
                        const double dls_lambda) {
  // sanity check
  this->check_inverse_velocity_arguments(cartesian_twists, joint_positions, frames);

  const unsigned int nb_joints = this->get_number_of_joints();
  // the velocity vector contains position of the intermediate frame and full pose of the end-effector
  Eigen::VectorXd dX(3 * cartesian_twists.size() + 3);
  Eigen::MatrixXd jacobian(3 * cartesian_twists.size() + 3, nb_joints);
  for (unsigned int i = 0; i < cartesian_twists.size() - 1; ++i) {
    // extract only the linear velocity for intermediate points
    dX.segment<3>(3 * i) = cartesian_twists[i].get_linear_velocity();
    jacobian.block(3 * i, 0, 3 * i + 3, nb_joints) =
        this->compute_jacobian(joint_positions, frames.at(i)).data().block(0, 0, 3, nb_joints);
  }
  // full twist for the last provided frame
  dX.tail(6) = cartesian_twists.back().data();
  jacobian.bottomRows(6) = this->compute_jacobian(joint_positions, frames.back()).data();

  if (dls_lambda == 0.0) {
    return state_representation::JointVelocities(joint_positions.get_name(),
                                                 joint_positions.get_names(),
                                                 jacobian.colPivHouseholderQr().solve(dX));
  }

  // add damped least square term
  if (jacobian.rows() > jacobian.cols()) {
    Eigen::MatrixXd j_prime = jacobian.transpose() * jacobian + 
                dls_lambda * dls_lambda * Eigen::MatrixXd::Identity(jacobian.cols(), jacobian.cols());
    return state_representation::JointVelocities(joint_positions.get_name(),
                                                 joint_positions.get_names(),
                                                 j_prime.colPivHouseholderQr().solve(jacobian.transpose() * dX));
  } else {
    Eigen::MatrixXd j_prime = jacobian * jacobian.transpose() + 
                dls_lambda * dls_lambda * Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows());
    return state_representation::JointVelocities(joint_positions.get_name(),
                                                 joint_positions.get_names(),
                                                 jacobian.transpose() * j_prime.colPivHouseholderQr().solve(dX));
  }
}

state_representation::JointVelocities Model::inverse_velocity(const state_representation::CartesianTwist& cartesian_twist,
                                                              const state_representation::JointPositions& joint_positions,
                                                              const std::string& frame,
                                                              const double dls_lambda) {
  std::string actual_frame = frame.empty() ? this->robot_model_.frames.back().name : frame;
  return this->inverse_velocity(std::vector<state_representation::CartesianTwist>({cartesian_twist}),
                                joint_positions,
                                std::vector<std::string>({actual_frame}),
                                dls_lambda);
}

state_representation::JointVelocities
Model::inverse_velocity(const std::vector<state_representation::CartesianTwist>& cartesian_twists,
                        const state_representation::JointPositions& joint_positions,
                        const QPInverseVelocityParameters& parameters,
                        const std::vector<std::string>& frames) {
  using namespace state_representation;
  using namespace std::chrono;
  // sanity check
  this->check_inverse_velocity_arguments(cartesian_twists, joint_positions, frames);

  const unsigned int nb_joints = this->get_number_of_joints();
  // the velocity vector contains position of the intermediate frame and full pose of the end-effector
  Eigen::VectorXd delta_r(3 * cartesian_twists.size() + 3);
  Eigen::MatrixXd jacobian(3 * cartesian_twists.size() + 3, nb_joints);
  for (unsigned int i = 0; i < cartesian_twists.size() - 1; ++i) {
    // first convert the twist to a displacement
    CartesianPose displacement = cartesian_twists[i];
    // extract only the position for intermediate points
    delta_r.segment<3>(3 * i) = displacement.get_position();
    jacobian.block(3 * i, 0, 3 * i + 3, nb_joints) =
        this->compute_jacobian(joint_positions, frames.at(i)).data().block(0, 0, 3, nb_joints);
  }
  // extract pose for the last provided frame
  CartesianPose full_displacement = cartesian_twists.back();
  delta_r.segment<3>(3 * (cartesian_twists.size() - 1)) = full_displacement.get_position();
  delta_r.tail(3) = full_displacement.get_orientation().vec();
  jacobian.bottomRows(6) = this->compute_jacobian(joint_positions, frames.back()).data();
  // compute the Jacobian
  Eigen::MatrixXd hessian_matrix = jacobian.transpose() * jacobian;
  // set the hessian sparse matrix
  std::vector<Eigen::Triplet<double>> coefficients;
  for (unsigned int i = 0; i < nb_joints; ++i) {
    for (unsigned int j = 0; j < nb_joints; ++j) {
      coefficients.emplace_back(Eigen::Triplet<double>(i, j, hessian_matrix(i, j)));
    }
  }
  coefficients.emplace_back(Eigen::Triplet<double>(nb_joints, nb_joints, parameters.alpha));
  
  // set the matrices
  this->qp_solver_->set_matrices(coefficients, parameters, joint_positions, full_displacement, delta_r, jacobian);

  // solve the QP problem
  auto solution = this->qp_solver_->solve();

  // extract the solution
  JointPositions joint_displacement(joint_positions.get_name(),
                                    joint_positions.get_names(),
                                    solution.head(nb_joints));
  auto dt = solution.tail(1)(0);
  return JointPositions(joint_displacement) / dt;
}

state_representation::JointVelocities Model::inverse_velocity(const state_representation::CartesianTwist& cartesian_twist,
                                                              const state_representation::JointPositions& joint_positions,
                                                              const QPInverseVelocityParameters& parameters,
                                                              const std::string& frame) {
  std::string actual_frame = frame.empty() ? this->robot_model_.frames.back().name : frame;
  return this->inverse_velocity(std::vector<state_representation::CartesianTwist>({cartesian_twist}),
                                joint_positions,
                                parameters,
                                std::vector<std::string>({actual_frame}));
}

bool Model::in_range(const Eigen::VectorXd& vector,
                     const Eigen::VectorXd& lower_limits,
                     const Eigen::VectorXd& upper_limits) {
  return ((vector.array() >= lower_limits.array()).all() && (vector.array() <= upper_limits.array()).all());
}

bool Model::in_range(const state_representation::JointPositions& joint_positions) const {
  return this->in_range(joint_positions.get_positions(),
                        this->robot_model_.lowerPositionLimit,
                        this->robot_model_.upperPositionLimit);
}

bool Model::in_range(const state_representation::JointVelocities& joint_velocities) const {
  return this->in_range(joint_velocities.get_velocities(),
                        -this->robot_model_.velocityLimit,
                        this->robot_model_.velocityLimit);
}

bool Model::in_range(const state_representation::JointTorques& joint_torques) const {
  return this->in_range(joint_torques.get_torques(), -this->robot_model_.effortLimit, this->robot_model_.effortLimit);
}

bool Model::in_range(const state_representation::JointState& joint_state) const {
  return (this->in_range(static_cast<state_representation::JointPositions>(joint_state))
      && this->in_range(static_cast<state_representation::JointVelocities>(joint_state))
      && this->in_range(static_cast<state_representation::JointTorques>(joint_state)));
}

Eigen::VectorXd Model::clamp_in_range(const Eigen::VectorXd& vector,
                                      const Eigen::VectorXd& lower_limits,
                                      const Eigen::VectorXd& upper_limits) {
  return lower_limits.cwiseMax(upper_limits.cwiseMin(vector));
}

state_representation::JointState Model::clamp_in_range(const state_representation::JointState& joint_state) const {
  state_representation::JointState joint_state_clamped(joint_state);
  joint_state_clamped.set_positions(this->clamp_in_range(joint_state.get_positions(),
                                                         this->robot_model_.lowerPositionLimit,
                                                         this->robot_model_.upperPositionLimit));
  joint_state_clamped.set_velocities(this->clamp_in_range(joint_state.get_velocities(),
                                                          -this->robot_model_.velocityLimit,
                                                          this->robot_model_.velocityLimit));
  joint_state_clamped.set_torques(this->clamp_in_range(joint_state.get_torques(),
                                                       -this->robot_model_.effortLimit,
                                                       this->robot_model_.effortLimit));
  return joint_state_clamped;
}
}// namespace robot_model
