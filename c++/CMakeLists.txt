cmake_minimum_required(VERSION 2.8)
project (trackerCLASS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

include_directories( BEFORE ${MY_SOURCE_DIR}/src )



add_executable(tracker src/main.cpp src/ekf.cpp)
