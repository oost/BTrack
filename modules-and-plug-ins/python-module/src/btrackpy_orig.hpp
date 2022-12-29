#ifndef BTRACK_PY_BTRACK_ORIG_HPP
#define BTRACK_PY_BTRACK_ORIG_HPP

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "btrack_constants.hpp"

namespace py = pybind11;

void init_btrackpy_orig(py::module_ &m);

#endif // BTRACK_PY_BTRACK_ORIG_HPP