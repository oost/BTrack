#!/bin/sh

cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=~/Programmation/projects/vcpkg/scripts/buildsystems/vcpkg.cmake -B build -S .
cmake --build build