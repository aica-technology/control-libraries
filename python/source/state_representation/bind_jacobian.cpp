#include "state_representation_bindings.hpp"

#include <tuple>
#include <state_representation/State.hpp>
#include <state_representation/space/Jacobian.hpp>

void bind_jacobian(py::module_& m) {
  py::class_<Jacobian, std::shared_ptr<Jacobian>, State> c(m, "Jacobian");

  c.def(py::init(), "Empty constructor for a Jacobian");
  c.def(py::init<const std::string&, unsigned int, const std::string&, const std::string&>(),
        "Constructor with name, number of joints, frame name and reference frame provided",
        "robot_name"_a, "nb_joints"_a, "frame"_a, "reference_frame"_a = "world");
  c.def(py::init<const std::string&, const std::vector<std::string>&, const std::string&, const std::string&>(),
      "Constructor with name, joint names, frame name and reference frame provided",
      "robot_name"_a, "joint_names"_a, "frame"_a, "reference_frame"_a = "world");
  c.def(py::init<const std::string&, const std::string&, const Eigen::MatrixXd&, const std::string&>(),
        "Constructor with name, frame, Jacobian matrix and reference frame provided",
        "robot_name"_a, "frame"_a, "data"_a, "reference_frame"_a = "world");
  c.def(py::init<const std::string&, const std::vector<std::string>&, const std::string&, const Eigen::MatrixXd&, const std::string&>(),
        "Constructor with name, joint names, frame name, Jacobian matrix and reference frame provided",
        "robot_name"_a, "joint_names"_a, "frame"_a, "data"_a, "reference_frame"_a = "world");
  c.def(py::init<const Jacobian&>(), "Copy constructor of a Jacobian", "jacobian"_a);

  c.def_static("Random", py::overload_cast<const std::string&, unsigned int, const std::string&, const std::string&>(&Jacobian::Random),
      "Constructor for a random Jacobian",
      "robot_name"_a, "nb_joints"_a, "frame"_a, "reference_frame"_a = "world");
  c.def_static("Random", py::overload_cast<const std::string&, const std::vector<std::string>&, const std::string&, const std::string&>(&Jacobian::Random),
      "Constructor for a random Jacobian",
      "robot_name"_a, "joint_names"_a, "frame"_a, "reference_frame"_a = "world");

  c.def("rows", &Jacobian::rows, "Getter of the number of rows attribute");
  c.def("row", &Jacobian::row, "Accessor of the row data at given index");
  c.def("cols", &Jacobian::cols, "Getter of the number of cols attribute");
  c.def("col", &Jacobian::col, "Accessor of the column data at given index");
  c.def("get_joint_names", &Jacobian::get_joint_names, "Getter of the joint_names attribute");
  c.def("get_frame", &Jacobian::get_frame, "Getter of the frame attribute");
  c.def("get_reference_frame", &Jacobian::get_reference_frame, "Getter of the reference_frame attribute");
  c.def("data", &Jacobian::data, "Getter of the data attribute");

  c.def("set_joint_names", py::overload_cast<unsigned int>(&Jacobian::set_joint_names), "Setter of the joint names attribute from the number of joints", "nb_joints"_a);
  c.def("set_joint_names", py::overload_cast<const std::vector<std::string>&>(&Jacobian::set_joint_names), "Setter of the joint names attribute from a vector of joint names", "joint_names"_a);
  c.def("set_reference_frame", &Jacobian::set_reference_frame, "Setter of the reference frame", "reference_frame"_a);
  c.def("set_data", &Jacobian::set_data, "Setter of the data attribute", "data"_a);
  c.def("set_zero", &Jacobian::set_zero, "Set the Jacobian matrix to a zero value");
  c.def("copy", &Jacobian::copy, "Return a copy of the Jacobian");

  c.def("inverse", [](const Jacobian& jacobian) {
    return jacobian.inverse();
  }, "Return the inverse of the Jacobian matrix");
  c.def("inverse", [](const Jacobian& jacobian, const Eigen::MatrixXd& matrix) {
    return jacobian.inverse(matrix);
  }, "Solve the system X = inv(J) * M to obtain X");
  c.def("inverse", [](const Jacobian& jacobian, const CartesianTwist& twist) {
    return jacobian.inverse(twist);
  }, "Transform the given Cartesian twist to joint space");
  c.def("is_incompatible", &Jacobian::is_incompatible, "Check if the Jacobian is incompatible for operations with the state given as argument", "state_a");
  c.def("pseudoinverse", [](const Jacobian& jacobian) {
    return jacobian.pseudoinverse();
  }, "Return the pseudoinverse of the Jacobian matrix");
  c.def("pseudoinverse", [](const Jacobian& jacobian, const Eigen::MatrixXd& matrix) {
    return jacobian.pseudoinverse(matrix);
  }, "Multiply the given matrix by the pseudoinverse of the Jacobian matrix");
  c.def("pseudoinverse", [](const Jacobian& jacobian, const CartesianTwist& twist) {
    return jacobian.pseudoinverse(twist);
  }, "Transform a Cartesian twist to joint space by pre-multiplying the Jacobian pseudoinverse");
  c.def("transpose", [](const Jacobian& jacobian) {
    return jacobian.transpose();
  }, "Return the transpose of the Jacobian matrix");
  c.def("transpose", [](const Jacobian& jacobian, const CartesianWrench& wrench) {
    return jacobian.transpose(wrench);
  }, "Transform a Cartesian wrench to joint space by pre-multiplying the Jacobian transpose");

  c.def("__getitem__", [](const Jacobian& jacobian, std::tuple<int, int> coefficients) {
    return jacobian(std::get<0>(coefficients), std::get<1>(coefficients));
  }, "Overload the [] operator to access the value at given (row, col)");
  c.def("__setitem__", [](Jacobian& jacobian, std::tuple<int, int> coefficients, double value) {
    jacobian(std::get<0>(coefficients), std::get<1>(coefficients)) = value;
    return jacobian;
  }, "Overload the [] operator to modify the value at given (row, col)");

  c.def(py::self * Eigen::MatrixXd());
  c.def(Eigen::Matrix<double, 6, 6>() * py::self);
  c.def(py::self * JointVelocities());
  c.def(CartesianPose() * py::self);

  c.def("__copy__", [](const Jacobian &jacobian) {
    return Jacobian(jacobian);
  });
  c.def("__deepcopy__", [](const Jacobian &jacobian, py::dict) {
    return Jacobian(jacobian);
    }, "memo"_a);
  c.def("__repr__", [](const Jacobian& jacobian) {
    std::stringstream buffer;
    buffer << jacobian;
    return buffer.str();
  });
}

