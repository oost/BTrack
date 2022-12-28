cmake_minimum_required(VERSION 3.15...3.25)


project(
  "${SKBUILD_PROJECT_NAME}"
  LANGUAGES CXX
  VERSION "${SKBUILD_PROJECT_VERSION}")

set (CMAKE_CXX_STANDARD 14)
set(ENABLE_FFTW ON)
set(Python3_FIND_VIRTUALENV "ONLY")
set(Python3_ROOT_DIR "/opt/local")
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(btrackpy src/main.cpp)

# vcpkg_find_acquire_program(PYTHON3)
# x_vcpkg_get_python_packages(PYTHON_EXECUTABLE "${PYTHON3}" PACKAGES numpy)
# x_vcpkg_get_python_packages(PYTHON_VERSION "3" PACKAGES numpy OUT_PYTHON_VAR "PYTHON3")

add_subdirectory(../.. btracklib)
message("Numpy ${Python3_NumPy_FOUND}")
target_link_libraries(btrackpy PRIVATE btrack)

target_compile_definitions(btrackpy
                           PRIVATE VERSION_INFO=${PROJECT_VERSION})

install(TARGETS btrackpy LIBRARY DESTINATION .)
