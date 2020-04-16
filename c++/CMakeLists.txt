cmake_minimum_required(VERSION 2.8)
project (trackerCLASS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS} -O3")

include_directories( BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/include /usr/include/python2.7 )

add_executable(tracker  src/main.cpp 
                        src/ekf.cpp
                        src/Data.cpp
                        src/plot.cpp
                        src/Tracker.cpp
                        src/Tracking.cpp
                        )
target_link_libraries(tracker python2.7)
