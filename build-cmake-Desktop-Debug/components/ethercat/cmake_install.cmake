# Install script for directory: /home/user/linuxcnc/cmake/components/ethercat

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib" TYPE SHARED_LIBRARY FILES "/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/lcec.so")
  if(EXISTS "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec.so"
         OLD_RPATH "/home/user/linuxcnc/cmake/components/ethercat/../../build/liblinuxcnchal:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec_conf" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec_conf")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec_conf"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec_conf")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib" TYPE EXECUTABLE FILES "/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/lcec_conf")
  if(EXISTS "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec_conf" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec_conf")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec_conf"
         OLD_RPATH "/home/user/linuxcnc/cmake/components/ethercat/../../build/liblinuxcnchal:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/user/linuxcnc/build-cmake-Desktop-Debug/components/ethercat/../../../../rtlib/lcec_conf")
    endif()
  endif()
endif()

