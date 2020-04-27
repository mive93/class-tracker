cmake_minimum_required(VERSION 2.8)
project (class-tracker)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS} -O3")

include_directories( BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/include /usr/include/python2.7 )

file(GLOB class-tracker-SRC "src/*.cpp")
set(class-tracker-LIBS python2.7)
add_library(class-tracker SHARED ${class-tracker-SRC})
target_link_libraries(class-tracker ${class-tracker-LIBS} )

#-------------------------------------------------------------------------------
# Build executables
#-------------------------------------------------------------------------------
add_executable(tracker demo/main.cpp)
target_link_libraries(tracker class-tracker)
