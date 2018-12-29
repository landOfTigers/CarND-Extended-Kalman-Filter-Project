#!/bin/bash

sudo apt-get install libgtest-dev
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
sudo cp *.a /usr/lib

cd /home/workspace/CarND-Extended-Kalman-Filter-Project
cat <<EOT >> CMakeLists.txt

# GTest
cmake_minimum_required(VERSION 2.6)
 
# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
 
# Link runTests with what we want to test and the GTest and pthread library
add_executable(executeTests test/tools_test.cpp)
target_link_libraries(executeTests ${GTEST_LIBRARIES} pthread)
EOT
