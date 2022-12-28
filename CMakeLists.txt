cmake_minimum_required(VERSION 3.4)

project(BTrack)

set (CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++ -fno-stack-protector")

option(ENABLE_FFTW OFF)

add_subdirectory(src)
add_subdirectory(unit-tests)
add_subdirectory(modules-and-plug-ins)



