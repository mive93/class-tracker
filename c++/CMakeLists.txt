cmake_minimum_required(VERSION 2.8)
project (trackerCLASS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS} -O3")

include_directories( BEFORE ${MY_SOURCE_DIR}/src /usr/include/python2.7 )

add_executable(tracker  src/main.cpp 
                        src/ekf.cpp
                        src/trackutils.cpp
                        src/plot.cpp
                        src/tracker.cpp
                        )
target_link_libraries(tracker python2.7)
