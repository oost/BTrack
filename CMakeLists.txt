cmake_minimum_required(VERSION 3.4)

project(BTrack)

set (CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++ -fno-stack-protector")

option(ENABLE_FFTW OFF)
option(BUILD_PLUGINS ON)

find_package(fmt CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(range-v3 CONFIG REQUIRED)

add_subdirectory(src)
add_subdirectory(tests)

if(BUILD_PLUGINS)
  add_subdirectory(modules-and-plug-ins)
endif()



