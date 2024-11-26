#pragma once

#include <string>
#include <vector>
#include <optional>

#include <OsqpEigen/OsqpEigen.h>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/geometry.hpp>

#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/parameters/ParameterInterface.hpp>
#include <state_representation/space/Jacobian.hpp>
#include <state_representation/space/joint/JointState.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>

#include "robot_model/QPSolver.hpp"

using namespace std::chrono_literals;

/**
 * @namespace robot_model
 * @brief Robot kinematics and dynamics
 */
namespace robot_model {
/**
 * @brief parameters for the inverse kinematics function
 * @param damp damping added to the diagonal of J*Jt in order to avoid the singularity
 * @param alpha alpha €]0,1], it is used to make Newthon-Raphson method less aggressive
 * @param gamma gamma €]0,1], represents the strength of the repulsive potential field in the Clamping Weighted Least-Norm method
 * @param margin the distance from the joint limit at which the joint positions should be penalized (rad)
 * @param tolerance the maximum error tolerated between the desired cartesian state and the one obtained by the returned joint positions
 * @param max_number_of_iterations the maximum number of iterations that the algorithm do for solving the inverse kinematics
 */
struct InverseKinematicsParameters {
  double damp = 1e-6;
  double alpha = 0.5;
  double gamma = 0.8;
  double margin = 0.07;
  double tolerance = 1e-3;
  unsigned int max_number_of_iterations = 1000;
};

/**
 * @class Model
 * @brief The Model class is a wrapper around pinocchio dynamic computation library with state_representation
 * encapsulations.
 */
class Model {
private:
  std::string robot_name_;         ///< name of the robot
  std::string urdf_path_;          ///< path to the urdf file
  std::vector<std::string> frames_;///< name of the frames
  pinocchio::Model robot_model_;   ///< the robot model with pinocchio
  pinocchio::Data robot_data_;     ///< the robot data with pinocchio
  std::optional<std::function<std::string(const std::string&)>>
      meshloader_callback_;               ///< callback function to resolve package paths
  pinocchio::GeometryModel geom_model_;   ///< the robot geometry model with pinocchio
  pinocchio::GeometryData geom_data_;     ///< the robot geometry data with pinocchio
  std::unique_ptr<QPSolver> qp_solver_;   ///< the QP solver for the inverse velocity kinematics
  bool load_collision_geometries_ = false;///< flag to load collision geometries

  /**
   * @brief Initialize the pinocchio model from the URDF
   */
  void init_model();

  /**
   * @brief Initialize the pinocchio geometry model from the URDF and the package paths
   */
  void init_geom_model(std::string urdf);

  /**
   * @brief Check if frames exist in robot model and return its ids
   * @param frames containing the frame names to check
   * @return the ids of the frames
   */
  std::vector<unsigned int> get_frame_ids(const std::vector<std::string>& frames);

  /**
   * @brief Check if a frame exist in robot model and return its id
   * @param frame containing the frame name to check
   * @return the id of the frame if it exists
   */
  unsigned int get_frame_id(const std::string& frame);

  /**
    * @brief Find all the package paths in the URDF and replaces them with the absolute path using meshloader_callback_
    * @param urdf string containing the URDF description of the robot
    * @return vector of the package paths
    */
  std::vector<std::string> resolve_package_paths_in_urdf(std::string& urdf) const;

  /**
   * @brief Compute the Jacobian from given joint positions at the frame in parameter
   * @param joint_positions containing the joint positions of the robot
   * @param joint_id id of the frame at which to compute the Jacobian
   * @return the Jacobian matrix
   */
  state_representation::Jacobian compute_jacobian(const state_representation::JointPositions& joint_positions,
                                                  unsigned int frame_id);

  /**
   * @brief Compute the time derivative of the Jacobian from given joint positions and velocities at the frame in parameter
   * @param joint_positions containing the joint positions of the robot
   * @param joint_velocities containing the joint positions of the robot
   * @param joint_id id of the frame at which to compute the Jacobian
   * @return the time derivative of Jacobian matrix
   */
  Eigen::MatrixXd compute_jacobian_time_derivative(const state_representation::JointPositions& joint_positions,
                                                   const state_representation::JointVelocities& joint_velocities,
                                                   unsigned int frame_id);

  /**
   * @brief Compute the forward kinematics, i.e. the pose of certain frames from the joint positions
   * @param joint_positions the joint state of the robot
   * @param frame_ids ids of the frames at which to extract the pose
   * @return the desired poses
   */
  std::vector<state_representation::CartesianPose> forward_kinematics(const state_representation::JointPositions& joint_positions,
                                                                      const std::vector<unsigned int>& frame_ids);

  /**
   * @brief Compute the forward kinematics, i.e. the pose of certain frames from the joint positions for a single frame
   * @param joint_positions the joint state of the robot
   * @param frame_id id of the frames at which to extract the pose
   * @return the desired pose
   */
  state_representation::CartesianPose forward_kinematics(const state_representation::JointPositions& joint_positions,
                                                         unsigned int frame_id);

  /**
   * @brief Check if the vector's elements are inside the parameter limits
   * @param vector the vector to check
   * @param lower_limits the lower bounds of the limits
   * @param upper_limits the upper bounds of the limits
   * @return true if all the elements are inside at their limits, false otherwise.
   */
  static bool in_range(const Eigen::VectorXd& vector,
                       const Eigen::VectorXd& lower_limits,
                       const Eigen::VectorXd& upper_limits);

  /**
   * @brief Clamp the vector's elements according to the parameter limits
   * @param vector the vector to clamp
   * @param lower_limits the lower bounds of the limits
   * @param upper_limits the upper bounds of the limits
   * @return the clamped vector
   */
  static Eigen::VectorXd clamp_in_range(const Eigen::VectorXd& vector,
                                        const Eigen::VectorXd& lower_limits,
                                        const Eigen::VectorXd& upper_limits);

  /**
   * @brief Compute the weighted matrix of the algorithm "Clamping Weighted Least-Norm"
   * @param joint_positions the joint position at the current iteration in the inverse kinematics problem
   * @param margin the distance from the joint limit at which the joint positions should be penalized
   * @return the weighted matrix
   */
  Eigen::MatrixXd cwln_weighted_matrix(const state_representation::JointPositions& joint_positions, double margin);

  /**
   * @brief Compute the repulsive potential field of the algorithm "Clamping Weighted Least-Norm"
   * @param joint_positions the joint position at the current iteration in the inverse kinematics problem
   * @param margin the distance from the joint limit at which the joint positions should be penalized
   * @return the repulsive potential field
   */
  Eigen::VectorXd cwln_repulsive_potential_field(const state_representation::JointPositions& joint_positions,
                                                 double margin);

  /**
   * @brief Check the arguments of the inverse_velocity function and throw exceptions if they are not correct
   * @param cartesian_twists vector of twist
   * @param joint_positions current joint positions, used to compute the Jacobian matrix
   * @param frames names of the frames at which to compute the twists
   */
  void check_inverse_velocity_arguments(const std::vector<state_representation::CartesianTwist>& cartesian_twists,
                                        const state_representation::JointPositions& joint_positions,
                                        const std::vector<std::string>& frames);

  /**
   * @brief Generates a list of collision pairs to exclude based on the kinematic tree of the model
   * @return the list of collision pairs to exclude 
  */
  std::vector<pinocchio::CollisionPair> generate_joint_exclusion_list();

public:
  /**
   * @brief Construct with robot name and path to URDF file
   * @details If the URDF contains references to collision geometry meshes, they will not be loaded into memory.
   * To enable collision detection, use the alternate constructor.
   * @param robot_name the name to associate with the model
   * @param urdf_path the path to the URDF file
   */
  explicit Model(const std::string& robot_name, const std::string& urdf_path);

  /**
   * @brief Construct a robot model with collision geometries from a URDF file
   * @details If the URDF contains references to collision geometry meshes, they will be loaded into memory.
   * Subsequently, the check_collision() method can be used to check for self-collisions in the robot model.
   * If geometry meshes are referenced with a relative package path using the `package://` prefix, then
   * the optional meshloader_callback function should be defined to return an absolute path to a package
   * given the package name.
   * @param robot_name the name to associate with the model
   * @param urdf_path the path to the URDF file
   * @param meshloader_callback optional callback to resolve the absolute package path from a package name
   */
  explicit Model(const std::string& robot_name, 
                   const std::string& urdf_path,
                   const std::optional<std::function<std::string(const std::string&)>>& meshloader_callback);

  /**
   * @brief Copy constructor
   * @param model the model to copy
   */
  Model(const Model& model);

  /**
   * @brief Swap the values of the two Model
   * @param model1 Model to be swapped with 2
   * @param model2 Model to be swapped with 1
   */
  friend void swap(Model& model1, Model& model2);

  /**
   * @brief Copy assignment operator that have to be defined due to the custom assignment operator
   * @param model the model with value to assign
   * @return reference to the current model with new values
   */
  Model& operator=(const Model& Model);

  /**
   * @brief Creates a URDF file with desired path and name from a string (possibly the robot description
   * string from the ROS parameter server)
   * @param urdf_string string containing the URDF description of the robot
   * @param desired_path desired path and name of the created URDF file as string
   * @return bool if operation was successful
   */
  static bool create_urdf_from_string(const std::string& urdf_string, const std::string& desired_path);

  /**
   * @brief Compute the minimum distances between the robot links
   * @details The distances are computed for each collision pair, resulting in a square matrix with
   * the same size as the number of joints. The diagonal entries are always zero.
   * @param joint_positions state_representation object containing the joint positions of the robot
   * @return the matrix containing the minimum distance between the robot links
   */
  Eigen::MatrixXd compute_minimum_collision_distances(const state_representation::JointPositions& joint_positions); 

  /**
   * @brief Check if the links of the robot are in collision
   * @param joint_positions containing the joint positions of the robot
   * @throws robot_model::exceptions::CollisionGeometryException if collision geometry is not initialized
   * @return true if the robot is in collision, false otherwise
   */
  bool check_collision(const state_representation::JointPositions& joint_positions);

  /**
   * @brief Getter of the number of collision pairs in the model
   * @return the number of collision pairs
   */
  unsigned int get_number_of_collision_pairs();

  /**
   * @brief Check if geometry model is initialized
   * @return true if the geometry model is initialized, false otherwise
   */
  bool is_geometry_model_initialized();

  /**
   * @brief Getter of the robot name
   * @return the robot name
   */
  const std::string& get_robot_name() const;

  /**
   * @brief Setter of the robot name
   * @param robot_name the new value of the robot name
   */
  void set_robot_name(const std::string& robot_name);

  /**
   * @brief Getter of the URDF path
   * @return the URDF path
   */
  const std::string& get_urdf_path() const;

  /**
   * @brief Getter of the number of joints
   * @return the number of joints
   */
  unsigned int get_number_of_joints() const;

  /**
   * @brief Getter of the joint frames from the model
   * @return the joint frames
   */
  std::vector<std::string> get_joint_frames() const;

  /**
   * @brief Getter of the frames from the model
   * @return the frame names
   */
  std::vector<std::string> get_frames() const;

  /**
   * @brief Getter of the base frame of the robot
   * @return the base frame
   */
  const std::string& get_base_frame() const;

  /**
   * @brief Getter of the gravity vector
   * @return Eigen::Vector3d the gravity vector
   */
  Eigen::Vector3d get_gravity_vector() const;

  /**
   * @brief Setter of the gravity vector
   * @param gravity the gravity vector
   */
  void set_gravity_vector(const Eigen::Vector3d& gravity);

  /**
   * @brief Getter of the pinocchio model
   * @return the pinocchio model
   */
  const pinocchio::Model& get_pinocchio_model() const;

  /**
   * @brief Compute the Jacobian from a given joint state at the frame given in parameter
   * @param joint_positions containing the joint positions of the robot
   * @param frame name of the frame at which to compute the Jacobian, if empty computed for the last frame
   * @return the Jacobian matrix
   */
  state_representation::Jacobian compute_jacobian(const state_representation::JointPositions& joint_positions,
                                                  const std::string& frame = "");

  /**
   * @brief Compute the time derivative of the Jacobian from given joint positions and velocities at the frame in parameter
   * @param joint_positions containing the joint positions of the robot
   * @param joint_velocities containing the joint positions of the robot
   * @param frame name of the frame at which to compute the Jacobian, if empty computed for the last frame
   * @return the time derivative of Jacobian matrix
   */
  Eigen::MatrixXd compute_jacobian_time_derivative(const state_representation::JointPositions& joint_positions,
                                                   const state_representation::JointVelocities& joint_velocities,
                                                   const std::string& frame = "");

  /**
   * @brief Compute the Inertia matrix from a given joint positions
   * @param joint_positions containing the joint positions values of the robot
   * @return the inertia matrix
   */
  Eigen::MatrixXd compute_inertia_matrix(const state_representation::JointPositions& joint_positions);

  /**
   * @brief Compute the Inertia torques, i.e the inertia matrix multiplied by the joint accelerations. Joint positions
   * are needed as well for computations of the inertia matrix
   * @param joint_state containing the joint positions and accelerations values of the robot
   * @return the inertia torques as a JointTorques
   */
  state_representation::JointTorques compute_inertia_torques(const state_representation::JointState& joint_state);

  /**
   * @brief Compute the Coriolis matrix from a given joint state
   * @param joint_state containing the joint positions & velocities values of the robot
   * @return the Coriolis matrix
   */
  Eigen::MatrixXd compute_coriolis_matrix(const state_representation::JointState& joint_state);

  /**
   * @brief Compute the Coriolis torques, i.e. the Coriolis matrix multiplied by the joint velocities and express the
   * result as a JointTorques
   * @param joint_state containing the joint positions & velocities values of the robot
   * @return the Coriolis torques as a JointTorques
   */
  state_representation::JointTorques compute_coriolis_torques(const state_representation::JointState& joint_state);

  /**
   * @brief Compute the gravity torques
   * @param joint_positions containing the joint positions of the robot
   * @return the gravity torque as a JointTorques
   */
  state_representation::JointTorques compute_gravity_torques(const state_representation::JointPositions& joint_positions);

  /**
   * @brief Compute the forward kinematics, i.e. the pose of certain frames from the joint positions
   * @param joint_positions the joint state of the robot
   * @param frames names of the frames at which to extract the poses
   * @return the pose of desired frames
   */
  std::vector<state_representation::CartesianPose> forward_kinematics(const state_representation::JointPositions& joint_positions,
                                                                      const std::vector<std::string>& frames);

  /**
   * @brief Compute the forward kinematics, i.e. the pose of the frame from the joint positions
   * @param joint_positions the joint state of the robot
   * @param frame name of the frame at which to extract the pose
   * @return the pose of the desired frame
   */
  state_representation::CartesianPose forward_kinematics(const state_representation::JointPositions& joint_positions,
                                                         const std::string& frame = "");

  /**
   * @brief Compute the inverse kinematics, i.e. joint positions from the pose of the end-effector in an iterative manner
   * @param cartesian_pose containing the desired pose of the end-effector
   * @param parameters parameters of the inverse kinematics algorithm (default is default values of the
   * InverseKinematicsParameters structure)
   * @param frame name of the frame at which to extract the pose
   * @return the joint positions of the robot
   */
  state_representation::JointPositions inverse_kinematics(const state_representation::CartesianPose& cartesian_pose,
                                                          const InverseKinematicsParameters& parameters = InverseKinematicsParameters(),
                                                          const std::string& frame = "");

  /**
   * @brief Compute the inverse kinematics, i.e. joint positions from the pose of the end-effector
   * @param cartesian_pose containing the desired pose of the end-effector
   * @param joint_positions current state of the robot containing the generalized position
   * @param parameters parameters of the inverse kinematics algorithm (default is default values of the
   * InverseKinematicsParameters structure)
   * @param frame name of the frame at which to extract the pose
   * @return the joint positions of the robot
   */
  state_representation::JointPositions inverse_kinematics(const state_representation::CartesianPose& cartesian_pose,
                                                          const state_representation::JointPositions& joint_positions,
                                                          const InverseKinematicsParameters& parameters = InverseKinematicsParameters(),
                                                          const std::string& frame = "");

  /**
   * @brief Compute the forward velocity kinematics, i.e. the twist of certain frames from the joint states
   * @param joint_state the joint state of the robot with positions to compute the Jacobian and velocities for the twist
   * @param frames name of the frames at which to compute the twist
   * @return the twists of the frames in parameter
   */
  std::vector<state_representation::CartesianTwist> forward_velocity(const state_representation::JointState& joint_state,
                                                                     const std::vector<std::string>& frames);

  /**
   * @brief Compute the forward velocity kinematics, i.e. the twist of the end-effector from the joint velocities
   * @param joint_state the joint state of the robot with positions to compute the Jacobian and velocities for the twist
   * @param frame name of the frame at which to compute the twist
   * @return the twist of the frame in parameter
   */
  state_representation::CartesianTwist forward_velocity(const state_representation::JointState& joint_state,
                                                        const std::string& frame = "");

  /**
   * @brief Compute the inverse velocity kinematics, i.e. joint velocities from the velocities of the frames in parameter
   * using the Jacobian
   * @param cartesian_twists vector of twist
   * @param joint_positions current joint positions, used to compute the Jacobian matrix
   * @param frames names of the frames at which to compute the twists
   * @param dls_lambda damped least square term
   * @return the joint velocities of the robot
   */
  state_representation::JointVelocities inverse_velocity(const std::vector<state_representation::CartesianTwist>& cartesian_twists,
                                                         const state_representation::JointPositions& joint_positions,
                                                         const std::vector<std::string>& frames,
                                                         const double dls_lambda = 0.0);

  /**
   * @brief Compute the inverse velocity kinematics, i.e. joint velocities from the twist of the end-effector using the
   * Jacobian
   * @param cartesian_twist containing the twist of the end-effector
   * @param joint_positions current joint positions, used to compute the Jacobian matrix
   * @param frame name of the frame at which to compute the twist
   * @param parameters parameters of the inverse velocity kinematics algorithm (default is default values of the
   * QPInverseVelocityParameters structure)
   * @param dls_lambda damped least square term
   * @return the joint velocities of the robot
   */
  state_representation::JointVelocities inverse_velocity(const state_representation::CartesianTwist& cartesian_twist,
                                                         const state_representation::JointPositions& joint_positions,
                                                         const std::string& frame = "",
                                                         const double dls_lambda = 0.0);

  /**
   * @brief Compute the inverse velocity kinematics, i.e. joint velocities from the velocities of the frames in parameter
   * using the QP optimization method
   * @param cartesian_twists vector of twist
   * @param joint_positions current joint positions, used to compute the jacobian matrix
   * @param parameters parameters of the inverse velocity kinematics algorithm (default is default values of the
   * QPInverseVelocityParameters structure)
   * @param frames names of the frames at which to compute the twists
   * @return the joint velocities of the robot
   */
  state_representation::JointVelocities inverse_velocity(const std::vector<state_representation::CartesianTwist>& cartesian_twists,
                                                         const state_representation::JointPositions& joint_positions,
                                                         const QPInverseVelocityParameters& parameters,
                                                         const std::vector<std::string>& frames);

  /**
   * @brief Compute the inverse velocity kinematics, i.e. joint velocities from the twist of the end-effector using the
   * QP optimization method
   * @param cartesian_twist containing the twist of the end-effector
   * @param joint_positions current joint positions, used to compute the Jacobian matrix
   * @param joint_positions current joint positions, used to compute the Jacobian matrix
   * @param parameters parameters of the inverse velocity kinematics algorithm (default is default values of the
   * QPInverseVelocityParameters structure)
   * @param frame name of the frame at which to compute the twist
   * @return the joint velocities of the robot
   */
  state_representation::JointVelocities inverse_velocity(const state_representation::CartesianTwist& cartesian_twist,
                                                         const state_representation::JointPositions& joint_positions,
                                                         const QPInverseVelocityParameters& parameters,
                                                         const std::string& frame = "");

  /**
   * @brief Check if the joint positions are inside the limits provided by the model
   * @param joint_positions the joint positions to check
   * @return true if the positions are inside their limits, false otherwise.
   */
  bool in_range(const state_representation::JointPositions& joint_positions) const;

  /**
   * @brief Check if the joint velocities are inside the limits provided by the model
   * @param joint_velocities the joint velocities to check
   * @return true if the velocities are inside their limits, false otherwise.
   */
  bool in_range(const state_representation::JointVelocities& joint_velocities) const;

  /**
   * @brief Check if the joint torques are inside the limits provided by the model
   * @param joint_torques the joint torques to check
   * @return true if the torques are inside their limits, false otherwise.
   */
  bool in_range(const state_representation::JointTorques& joint_torques) const;

  /**
   * @brief Check if the joint state variables (positions, velocities & torques) are inside the limits provided by
   * the model
   * @param joint_state the joint state to check
   * @return true if the state variables are inside the limits, false otherwise.
   */
  bool in_range(const state_representation::JointState& joint_state) const;

  /**
   * @brief Clamp the joint state variables of a JointStateVariable according to the limits provided by the model
   * @param joint_state the joint state to be clamped
   * @param state_variable_type the type of the joint state variable to be clamped
   * @return the clamped joint states
   */
  state_representation::JointState clamp_in_range(const state_representation::JointState& joint_state, const state_representation::JointStateVariable& state_variable_type) const;

  /**
   * @brief Clamp the joint state variables (positions, velocities & torques) according to the limits provided by
   * the model
   * @param joint_state the joint state to be clamped
   * @return the clamped joint states
   */
  state_representation::JointState clamp_in_range(const state_representation::JointState& joint_state) const;
};

inline const std::string& Model::get_robot_name() const {
  return this->robot_name_;
}

inline void swap(Model& first, Model& second) {
  using std::swap;
  swap(first.robot_name_, second.robot_name_);
  swap(first.urdf_path_, second.urdf_path_);
  swap(first.frames_, second.frames_);
  swap(first.robot_model_, second.robot_model_);
  swap(first.robot_data_, second.robot_data_);
  swap(first.meshloader_callback_, second.meshloader_callback_);
  swap(first.geom_model_, second.geom_model_);
  swap(first.geom_data_, second.geom_data_);
  swap(first.qp_solver_, second.qp_solver_);
  swap(first.load_collision_geometries_, second.load_collision_geometries_);
}

inline Model& Model::operator=(const Model& model) {
  Model tmp(model);
  swap(*this, tmp);
  return *this;
}

inline void Model::set_robot_name(const std::string& robot_name) {
  this->robot_name_ = robot_name;
}

inline const std::string& Model::get_urdf_path() const {
  return this->urdf_path_;
}

inline unsigned int Model::get_number_of_joints() const {
  return this->robot_model_.nq;
}

inline std::vector<std::string> Model::get_joint_frames() const {
  // model contains a first joint called universe that needs to be discarded
  std::vector<std::string> joint_frames(this->robot_model_.names.begin() + 1, this->robot_model_.names.end());
  return joint_frames;
}

inline const std::string& Model::get_base_frame() const {
  return this->frames_.front();
}

inline std::vector<std::string> Model::get_frames() const {
  return this->frames_;
}

inline Eigen::Vector3d Model::get_gravity_vector() const {
  return this->robot_model_.gravity.linear();
}

inline void Model::set_gravity_vector(const Eigen::Vector3d& gravity) {
  this->robot_model_.gravity.linear(gravity);
}

inline const pinocchio::Model& Model::get_pinocchio_model() const {
  return this->robot_model_;
}
}// namespace robot_model
