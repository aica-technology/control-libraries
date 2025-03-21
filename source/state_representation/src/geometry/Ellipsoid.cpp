#include "state_representation/geometry/Ellipsoid.hpp"

#include <eigen3/Eigen/Sparse>
#include <random>

#include "state_representation/exceptions/EmptyStateException.hpp"
#include "state_representation/exceptions/IncompatibleSizeException.hpp"
#include "state_representation/exceptions/NoSolutionToFitException.hpp"

namespace state_representation {

Ellipsoid::Ellipsoid() : Shape(), axis_lengths_({1., 1.}), rotation_angle_(0) {
  this->set_type(StateType::GEOMETRY_ELLIPSOID);
}

Ellipsoid::Ellipsoid(const std::string& name, const std::string& reference_frame)
    : Shape(name, reference_frame), axis_lengths_({1., 1.}), rotation_angle_(0) {
  this->set_type(StateType::GEOMETRY_ELLIPSOID);
}

Ellipsoid::Ellipsoid(const Ellipsoid& ellipsoid) : Ellipsoid(ellipsoid.get_name()) {
  if (ellipsoid) {
    this->set_center_state(ellipsoid.get_center_state());
    this->set_axis_lengths(ellipsoid.get_axis_lengths());
    this->set_rotation_angle(ellipsoid.get_rotation_angle());
  }
}

Ellipsoid Ellipsoid::Unit(const std::string& name, const std::string& reference_frame) {
  Ellipsoid unit = Ellipsoid(name, reference_frame);
  unit.set_empty(false);
  return unit;
}

Ellipsoid& Ellipsoid::operator=(const Ellipsoid& state) {
  Ellipsoid tmp(state);
  swap(*this, tmp);
  return *this;
}

const std::vector<double>& Ellipsoid::get_axis_lengths() const {
  this->assert_not_empty();
  return this->axis_lengths_;
}

double Ellipsoid::get_axis_length(unsigned int index) const {
  this->assert_not_empty();
  return this->axis_lengths_[index];
}

double Ellipsoid::get_rotation_angle() const {
  this->assert_not_empty();
  return this->rotation_angle_;
}

const CartesianPose Ellipsoid::get_rotation() const {
  this->assert_not_empty();
  Eigen::Quaterniond rotation(Eigen::AngleAxisd(this->rotation_angle_, Eigen::Vector3d::UnitZ()));
  return CartesianPose(
      this->get_center_pose().get_name() + "_rotated", Eigen::Vector3d::Zero(), rotation,
      this->get_center_pose().get_name()
  );
}

void Ellipsoid::set_axis_lengths(const std::vector<double>& axis_lengths) {
  this->axis_lengths_ = axis_lengths;
  this->set_empty(false);
}

void Ellipsoid::set_axis_lengths(unsigned int index, double axis_length) {
  this->axis_lengths_[index] = axis_length;
  this->set_empty(false);
}

void Ellipsoid::set_rotation_angle(double rotation_angle) {
  this->rotation_angle_ = rotation_angle;
  this->set_empty(false);
}

void Ellipsoid::set_data(const Eigen::VectorXd& data) {
  if (data.size() != 6) {
    throw exceptions::IncompatibleSizeException(
        "Input is of incorrect size: expected 6, given " + std::to_string(data.size())
    );
  }
  this->set_center_position(data.head(3));
  this->set_rotation_angle(data(3));
  this->set_axis_lengths({data(4), data(5)});
}

void Ellipsoid::set_data(const std::vector<double>& data) {
  this->set_data(Eigen::VectorXd::Map(data.data(), data.size()));
}

const Ellipsoid Ellipsoid::fit(
    const std::string& name, const std::list<CartesianPose>& points, const std::string& reference_frame,
    double noise_level
) {
  // define the constraint matrix
  Eigen::SparseMatrix<double> constraint_matrix(6, 6);
  constraint_matrix.insert(1, 1) = -1;
  constraint_matrix.insert(0, 2) = 2;
  constraint_matrix.insert(2, 0) = 2;

  // preprocessing normalize the data TODO using PCA to find the plan
  size_t nb_points = points.size();
  Eigen::VectorXd x_value(nb_points);
  Eigen::VectorXd y_value(nb_points);

  // retry with noise everytime delta > 0
  double delta;
  std::vector<double> coefficients(6);
  Eigen::GeneralizedEigenSolver<Eigen::Matrix<double, 6, 6>> solver;

  std::default_random_engine generator;
  std::normal_distribution<double> dist(0., noise_level);
  do {
    unsigned int i = 0;
    for (const auto& p : points) {
      x_value[i] = p.get_position()(0) + dist(generator);
      y_value[i] = p.get_position()(1) + dist(generator);
      ++i;
    }

    double xm = x_value.mean();
    double ym = y_value.mean();
    double sx = 0.5 * (x_value.maxCoeff() - x_value.minCoeff());
    double sy = 0.5 * (y_value.maxCoeff() - y_value.minCoeff());
    Eigen::VectorXd x_value_centered = (x_value.array() - xm) / sx;
    Eigen::VectorXd y_value_centered = (y_value.array() - ym) / sy;

    // compute the design matrix
    Eigen::MatrixXd design_matrix(nb_points, 6);
    design_matrix.col(0) = x_value_centered.array() * x_value_centered.array();
    design_matrix.col(1) = x_value_centered.array() * y_value_centered.array();
    design_matrix.col(2) = y_value_centered.array() * y_value_centered.array();
    design_matrix.col(3) = x_value_centered;
    design_matrix.col(4) = y_value_centered;
    design_matrix.col(5) = Eigen::VectorXd::Ones(nb_points);

    // compute the scatter matrix
    Eigen::Matrix<double, 6, 6> scatter_matrix = design_matrix.transpose() * design_matrix;

    // solve the generalized eigenvalue problem
    solver.compute(scatter_matrix, constraint_matrix);

    // solution correspon to the single positive eigenvalue
    unsigned int solution_index;
    double eigenvalue = solver.betas().maxCoeff(&solution_index);

    // no solution case
    if (eigenvalue < 0) {
      throw exceptions::NoSolutionToFitException("No solution found for the ellipse fitting");
    }

    // extract the solution
    Eigen::VectorXd solution = solver.eigenvectors().col(solution_index).real();

    // unnormalize the parameters TODO inverse PCA
    double kx = xm;
    double ky = ym;
    double sx2 = sx * sx;
    double sy2 = sy * sy;

    coefficients[0] = solution(0) * sy2;
    coefficients[1] = solution(1) * sx * sy;
    coefficients[2] = solution(2) * sx2;
    coefficients[3] = -2 * solution(0) * sy2 * kx - solution(1) * sx * sy * ky + solution(3) * sx * sy2;
    coefficients[4] = -solution(1) * sx * sy * kx - 2 * solution(2) * sx2 * ky + solution(4) * sx2 * sy;
    coefficients[5] = solution(0) * sy2 * kx * kx + solution(1) * sx * sy * kx * ky + solution(2) * sx2 * ky * ky
        - solution(3) * sx * sy2 * kx - solution(4) * sx2 * sy * ky + solution(5) * sx2 * sy2;

    delta = coefficients[1] * coefficients[1] - 4 * coefficients[0] * coefficients[2];
  } while (delta > 0);
  return from_algebraic_equation(name, coefficients, reference_frame);
}

const Ellipsoid Ellipsoid::from_algebraic_equation(
    const std::string& name, const std::vector<double>& coefficients, const std::string& reference_frame
) {
  // extract all the components from the coefficients vector
  double b2 = coefficients[1] * coefficients[1];
  double delta = b2 - 4 * coefficients[0] * coefficients[2];

  // store intermediate calculations
  double tmp1 = coefficients[0] * coefficients[4] * coefficients[4]// AE2
      + coefficients[2] * coefficients[3] * coefficients[3]        // CD2
      - coefficients[1] * coefficients[3] * coefficients[4]        // BDE
      + delta * coefficients[5];                                   // deltaF
  double tmp2 = coefficients[2] - coefficients[0];                 // C-A
  double tmp3 = sqrt(tmp2 * tmp2 + b2);

  // create the ellipsoid in the plan and set its center and axis
  Ellipsoid result(name, reference_frame);
  std::vector<double> axis_lengths(2);

  // set axis lengths
  double r1 = -sqrt(2 * tmp1 * (coefficients[0] + coefficients[2] + tmp3)) / delta;
  double r2 = -sqrt(2 * tmp1 * (coefficients[0] + coefficients[2] - tmp3)) / delta;
  axis_lengths[0] = (r1 >= r2) ? r1 : r2;
  axis_lengths[1] = (r1 >= r2) ? r2 : r1;
  result.set_axis_lengths(axis_lengths);

  // set center position
  double cx = (2 * coefficients[2] * coefficients[3] - coefficients[1] * coefficients[4]) / delta;
  double cy = (2 * coefficients[0] * coefficients[4] - coefficients[1] * coefficients[3]) / delta;
  result.set_center_position(Eigen::Vector3d(cx, cy, 0));

  // set center orientation
  double phi;
  if (abs(coefficients[1]) > 1e-4) {
    phi = atan2(tmp2 - tmp3, coefficients[1]);
  } else if (coefficients[0] < coefficients[2]) {
    phi = 0.;
  } else {
    phi = M_PI_2;
  }
  if (r1 < r2) {
    phi += M_PI_2;
  }
  result.set_rotation_angle(phi);

  return result;
}

const std::list<CartesianPose> Ellipsoid::sample_from_parameterization(unsigned int nb_samples) const {
  if (this->is_empty()) {
    throw exceptions::EmptyStateException(this->get_name() + " state is empty");
  }
  // use a linspace to have a full rotation angle between [0, 2pi]
  std::vector<double> alpha = math_tools::linspace(0, 2 * M_PI, nb_samples);

  std::list<CartesianPose> samples;
  for (unsigned int i = 0; i < nb_samples; ++i) {
    CartesianPose point(this->get_name() + "_point" + std::to_string(i), this->get_rotation().get_name());
    double a = alpha.at(i);
    Eigen::Vector3d position;
    position(0) = this->get_axis_length(0) * cos(a);
    position(1) = this->get_axis_length(1) * sin(a);
    position(2) = 0;
    point.set_position(position);
    samples.push_back(this->get_center_pose() * this->get_rotation() * point);
  }
  return samples;
}

const std::vector<double> Ellipsoid::to_std_vector() const {
  this->assert_not_empty();
  std::vector<double> representation(6);
  // position
  representation[0] = this->get_center_position()(0);
  representation[1] = this->get_center_position()(1);
  representation[2] = this->get_center_position()(2);
  // rotation angle
  representation[3] = this->get_rotation_angle();
  // axis lengths
  representation[4] = this->get_axis_length(0);
  representation[5] = this->get_axis_length(1);
  return representation;
}

std::ostream& operator<<(std::ostream& os, const Ellipsoid& ellipsoid) {
  os << ellipsoid.to_string();
  if (ellipsoid.is_empty()) {
    return os;
  }
  os << std::endl;
  os << "axis lengths: [" << ellipsoid.get_axis_length(0) << ", " << ellipsoid.get_axis_length(1) << "]";
  return os;
}
}// namespace state_representation
