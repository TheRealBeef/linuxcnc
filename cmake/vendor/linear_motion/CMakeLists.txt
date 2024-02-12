cmake_minimum_required(VERSION 3.5)

project(linear_motion LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the path to the include directories
include_directories(${MATPLOTLIBCPP_INCLUDE_DIRS})

include_directories(${Python3_INCLUDE_DIRS})

set(PROJECT_SOURCES
    main.cpp
    linear_motion.h
    linear_motion.cpp
)

add_executable(runtest ${PROJECT_SOURCES} )

