cmake_minimum_required(VERSION 3.15...3.25)


project(
  "${SKBUILD_PROJECT_NAME}"
  LANGUAGES CXX
  VERSION "${SKBUILD_PROJECT_VERSION}")

set (CMAKE_CXX_STANDARD 20)
string(TIMESTAMP BTRACKPY_BUILDTIME "%Y-%m-%d %H:%M")
configure_file(src/build_constants.h.in src/build_constants.h)
  
set(ENABLE_FFTW ON)


# vcpkg_find_acquire_program(PYTHON3)
# x_vcpkg_get_python_packages(PYTHON_EXECUTABLE "${PYTHON3}" PACKAGES numpy)
# x_vcpkg_get_python_packages(PYTHON_VERSION "3" PACKAGES numpy OUT_PYTHON_VAR "PYTHON3")

if(PROJECT_IS_TOP_LEVEL)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../src btracklib)
endif()



message(" *** Defining _btrackpy")

set(Python3_FIND_VIRTUALENV "ONLY")
set(Python3_ROOT_DIR "/opt/local")

find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(_btrackpy src/main.cpp src/btrackpy.cpp src/btrackpy_orig.cpp)

target_link_libraries(_btrackpy PRIVATE btrack btrack_orig)
target_include_directories(_btrackpy PUBLIC "${PROJECT_BINARY_DIR}/src" )
target_compile_definitions(_btrackpy
                          PRIVATE VERSION_INFO=${PROJECT_VERSION})


message("SKBUILD_PROJECT_NAME ${SKBUILD_PROJECT_NAME}")
if(DEFINED SKBUILD_PROJECT_NAME)
  message("*** Setting install config")
  install(TARGETS _btrackpy LIBRARY DESTINATION ${SKBUILD_PROJECT_NAME})
  install(FILES src/__init__.py DESTINATION ${SKBUILD_PROJECT_NAME})
endif()

