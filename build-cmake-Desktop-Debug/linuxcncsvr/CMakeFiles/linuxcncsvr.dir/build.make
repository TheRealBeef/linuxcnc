# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/user/linuxcnc/cmake

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/linuxcnc/build-cmake-Desktop-Debug

# Include any dependencies generated for this target.
include linuxcncsvr/CMakeFiles/linuxcncsvr.dir/depend.make

# Include the progress variables for this target.
include linuxcncsvr/CMakeFiles/linuxcncsvr.dir/progress.make

# Include the compile flags for this target's objects.
include linuxcncsvr/CMakeFiles/linuxcncsvr.dir/flags.make

linuxcncsvr/CMakeFiles/linuxcncsvr.dir/home/user/linuxcnc/src/emc/task/emcsvr.cc.o: linuxcncsvr/CMakeFiles/linuxcncsvr.dir/flags.make
linuxcncsvr/CMakeFiles/linuxcncsvr.dir/home/user/linuxcnc/src/emc/task/emcsvr.cc.o: /home/user/linuxcnc/src/emc/task/emcsvr.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object linuxcncsvr/CMakeFiles/linuxcncsvr.dir/home/user/linuxcnc/src/emc/task/emcsvr.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcncsvr && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcncsvr.dir/home/user/linuxcnc/src/emc/task/emcsvr.cc.o -c /home/user/linuxcnc/src/emc/task/emcsvr.cc

linuxcncsvr/CMakeFiles/linuxcncsvr.dir/home/user/linuxcnc/src/emc/task/emcsvr.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcncsvr.dir/home/user/linuxcnc/src/emc/task/emcsvr.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcncsvr && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/task/emcsvr.cc > CMakeFiles/linuxcncsvr.dir/home/user/linuxcnc/src/emc/task/emcsvr.cc.i

linuxcncsvr/CMakeFiles/linuxcncsvr.dir/home/user/linuxcnc/src/emc/task/emcsvr.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcncsvr.dir/home/user/linuxcnc/src/emc/task/emcsvr.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcncsvr && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/task/emcsvr.cc -o CMakeFiles/linuxcncsvr.dir/home/user/linuxcnc/src/emc/task/emcsvr.cc.s

# Object files for target linuxcncsvr
linuxcncsvr_OBJECTS = \
"CMakeFiles/linuxcncsvr.dir/home/user/linuxcnc/src/emc/task/emcsvr.cc.o"

# External object files for target linuxcncsvr
linuxcncsvr_EXTERNAL_OBJECTS =

linuxcncsvr/linuxcncsvr: linuxcncsvr/CMakeFiles/linuxcncsvr.dir/home/user/linuxcnc/src/emc/task/emcsvr.cc.o
linuxcncsvr/linuxcncsvr: linuxcncsvr/CMakeFiles/linuxcncsvr.dir/build.make
linuxcncsvr/linuxcncsvr: /home/user/linuxcnc/cmake/linuxcncsvr/../build/libnml/libnml.so
linuxcncsvr/linuxcncsvr: /home/user/linuxcnc/cmake/linuxcncsvr/../build/liblinuxcnchal/liblinuxcnchal.so
linuxcncsvr/linuxcncsvr: /home/user/linuxcnc/cmake/linuxcncsvr/../build/liblinuxcncini/liblinuxcncini.so
linuxcncsvr/linuxcncsvr: /home/user/linuxcnc/cmake/linuxcncsvr/../build/liblinuxcnc/liblinuxcnc.a
linuxcncsvr/linuxcncsvr: linuxcncsvr/CMakeFiles/linuxcncsvr.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable linuxcncsvr"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcncsvr && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/linuxcncsvr.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
linuxcncsvr/CMakeFiles/linuxcncsvr.dir/build: linuxcncsvr/linuxcncsvr

.PHONY : linuxcncsvr/CMakeFiles/linuxcncsvr.dir/build

linuxcncsvr/CMakeFiles/linuxcncsvr.dir/clean:
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcncsvr && $(CMAKE_COMMAND) -P CMakeFiles/linuxcncsvr.dir/cmake_clean.cmake
.PHONY : linuxcncsvr/CMakeFiles/linuxcncsvr.dir/clean

linuxcncsvr/CMakeFiles/linuxcncsvr.dir/depend:
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/linuxcnc/cmake /home/user/linuxcnc/cmake/linuxcncsvr /home/user/linuxcnc/build-cmake-Desktop-Debug /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcncsvr /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcncsvr/CMakeFiles/linuxcncsvr.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : linuxcncsvr/CMakeFiles/linuxcncsvr.dir/depend

