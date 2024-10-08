cmake_minimum_required(VERSION 3.0)
set(PROJECT "gcode")
project(${PROJECT} VERSION 0 LANGUAGES CXX C)

# Set variables.
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# Set includepaths.
include_directories(${SRC_DIR})
include_directories(${SRC_DIR}/../../src)
include_directories(${SRC_DIR}/../../src/emc/sai)
include_directories(${SRC_DIR}/../../src/emc/task)
include_directories(${SRC_DIR}/../../src/emc/pythonplugin)
include_directories(${SRC_DIR}/../../src/hal)
include_directories(${SRC_DIR}/../../src/rtapi)
include_directories(${SRC_DIR}/../../src/emc)
include_directories(${SRC_DIR}/../../src/emc/kinematics)
include_directories(${SRC_DIR}/../../src/emc/motion-logger)
include_directories(${SRC_DIR}/../../src/emc/tooldata)
include_directories(${SRC_DIR}/../../src/emc/iotask)
include_directories(${SRC_DIR}/../../src/emc/nml_intf)
include_directories(${SRC_DIR}/../../src/emc/usr_intf)
include_directories(${SRC_DIR}/../../src/emc/motion)
include_directories(${SRC_DIR}/../../src/emc/rs274ngc)
include_directories(${SRC_DIR}/../../src/emc/tp)
include_directories(${SRC_DIR}/../../src/emc/ini)
include_directories(${SRC_DIR}/../../src/libnml/rcs)
include_directories(${SRC_DIR}/../../src/libnml/nml)
include_directories(${SRC_DIR}/../../src/libnml/cms)
include_directories(${SRC_DIR}/../../src/libnml/buffer)
include_directories(${SRC_DIR}/../../src/libnml/posemath)
include_directories(${SRC_DIR}/../../src/libnml/inifile)
include_directories(${SRC_DIR}/../../src/libnml/os_intf)
include_directories(${Python3_INCLUDE_DIRS})

# Set prefix.
set(CMAKE_SHARED_LIBRARY_PREFIX "")

ADD_LIBRARY(${PROJECT} SHARED
    ${SRC_DIR}/../../src/emc/rs274ngc/gcodemodule.cc
    )

if (BUILD_USPACE)
    add_definitions(-DULAPI)
else ()
    add_definitions(-DRTAPI)
endif ()


# Set include paths for libraries to link.
add_library(rs274_ SHARED IMPORTED) # or STATIC instead of SHARED
set_target_properties(rs274_ PROPERTIES
    IMPORTED_LOCATION "${SRC_DIR}/../build/librs274/librs274.so"
    INTERFACE_INCLUDE_DIRECTORIES "${SRC_DIR}/../../include"
    )

# Link to libraries.
target_link_libraries(${PROJECT_NAME} PRIVATE rs274_
    ${PYTHON_LIB}
    )

# Add verion nr. .so.0.0.0
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})

# Set custom output name :
# set_property(TARGET  ${PROJECT} PROPERTY OUTPUT_NAME thename)

# Make install
install(TARGETS ${PROJECT} DESTINATION ${SRC_DIR}/../../lib/python)

# Notes.









