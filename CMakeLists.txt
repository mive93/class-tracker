cmake_minimum_required(VERSION 2.8)
project (class-tracker)

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)
# standard compiler warnings
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS} -O3")

#-------------------------------------------------------------------------------
# Options
#-------------------------------------------------------------------------------

option(WITH_MATPLOTLIB "Compiling also matplotlib" ON)
if(WITH_MATPLOTLIB)
    add_compile_definitions(USE_MATPLOTLIB)
    include_directories(  SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/matplotlib-cpp )
endif()

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)

message("Python_FOUND:${Python3_FOUND}")
message("Python_VERSION:${Python3_VERSION}")
message("Python_Development_FOUND:${Python3_Development_FOUND}")
message("Python_LIBRARIES:${Python3_LIBRARIES}")
message("Python_INCLUDE_DIRS:${Python3_INCLUDE_DIRS}")

include_directories(    ${CMAKE_CURRENT_SOURCE_DIR}/include )
include_directories(    SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/geodetic_utils/geodetic_utils/include/geodetic_utils
                        ${EIGEN3_INCLUDE_DIR}
                        ${Python3_INCLUDE_DIRS}
                        )

file(GLOB class-tracker-SRC "src/*.cpp")
set(class-tracker-LIBS ${Python3_LIBRARIES})
add_library(class-tracker SHARED ${class-tracker-SRC})
target_link_libraries(class-tracker ${class-tracker-LIBS} project_warnings )

#-------------------------------------------------------------------------------
# Build executables
#-------------------------------------------------------------------------------
add_executable(tracker demo/main.cpp)
target_link_libraries(tracker class-tracker)
