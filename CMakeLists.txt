cmake_minimum_required(VERSION 3.10)

# set the project name
project(SLAMSim)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# add the executable
add_executable(SLAMSim slamsim.cxx)