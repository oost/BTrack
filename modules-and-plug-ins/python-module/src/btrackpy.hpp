#ifndef BTRACK_PY_BTRACK_HPP
#define BTRACK_PY_BTRACK_HPP

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "btrack_constants.hpp"

namespace py = pybind11;

py::array_t<double, py::array::c_style>
btrack_trackBeats(py::array_t<double> input, int hopSize, int frameSize);
//=======================================================================
py::array_t<double, py::array::c_style>
btrack_calculateOnsetDF(py::array_t<double> input, int hopSize, int frameSize);

//=======================================================================
py::array_t<double, py::array::c_style>
btrack_trackBeatsFromOnsetDF(py::array_t<double> input, int hopSize,
                             int frameSize);

void init_btrackpy(py::module_ &m);

#endif // BTRACK_PY_BTRACK_HPP