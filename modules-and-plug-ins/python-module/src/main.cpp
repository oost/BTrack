#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "btrackpy.hpp"
#include "btrackpy_orig.hpp"
#include "build_constants.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(_btrackpy, m) {

  init_btrackpy(m);
  init_btrackpy_orig(m);

  m.def(
      "build_info", []() { return BTRACKPY_BUILDTIME; }, R"pbdoc(
        Returns info on build
    )pbdoc");

  m.doc() = R"pbdoc(
        btrackpy plugin
        -----------------------
        .. currentmodule:: btrackpy
        .. autosummary::
           :toctree: _generate
           calculateOnsetDF
           trackBeats
           trackBeatsFromOnsetDF
           calculateOnsetDFOrig
           trackBeatsOrig
           trackBeatsFromOnsetDFOrig
           build_info
    )pbdoc";
#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
