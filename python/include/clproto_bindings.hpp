#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <eigen3/Eigen/Core>

#include <clproto.hpp>

namespace py = pybind11;
using namespace pybind11::literals;

class EncodingException : public std::runtime_error {
public:
  explicit EncodingException(const std::string& msg) : std::runtime_error(msg) {}
};

void bind_clproto(py::module_& m);