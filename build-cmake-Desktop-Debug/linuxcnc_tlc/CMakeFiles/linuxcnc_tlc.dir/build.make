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
include linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/depend.make

# Include the progress variables for this target.
include linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/progress.make

# Include the compile flags for this target's objects.
include linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/flags.make

linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/emcsh.cc.o: linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/flags.make
linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/emcsh.cc.o: /home/user/linuxcnc/src/emc/usr_intf/emcsh.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/emcsh.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcnc_tlc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/emcsh.cc.o -c /home/user/linuxcnc/src/emc/usr_intf/emcsh.cc

linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/emcsh.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/emcsh.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcnc_tlc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/usr_intf/emcsh.cc > CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/emcsh.cc.i

linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/emcsh.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/emcsh.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcnc_tlc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/usr_intf/emcsh.cc -o CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/emcsh.cc.s

linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.o: linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/flags.make
linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.o: /home/user/linuxcnc/src/emc/usr_intf/shcom.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcnc_tlc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.o -c /home/user/linuxcnc/src/emc/usr_intf/shcom.cc

linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcnc_tlc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/usr_intf/shcom.cc > CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.i

linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcnc_tlc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/usr_intf/shcom.cc -o CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.s

# Object files for target linuxcnc_tlc
linuxcnc_tlc_OBJECTS = \
"CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/emcsh.cc.o" \
"CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.o"

# External object files for target linuxcnc_tlc
linuxcnc_tlc_EXTERNAL_OBJECTS =

linuxcnc_tlc/linuxcnc.so: linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/emcsh.cc.o
linuxcnc_tlc/linuxcnc.so: linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.o
linuxcnc_tlc/linuxcnc.so: linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/build.make
linuxcnc_tlc/linuxcnc.so: /home/user/linuxcnc/cmake/linuxcnc_tlc/../build/liblinuxcncini/liblinuxcncini.so
linuxcnc_tlc/linuxcnc.so: /home/user/linuxcnc/cmake/linuxcnc_tlc/../build/liblinuxcnc/liblinuxcnc.a
linuxcnc_tlc/linuxcnc.so: /home/user/linuxcnc/cmake/linuxcnc_tlc/../build/libnml/libnml.so
linuxcnc_tlc/linuxcnc.so: linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library linuxcnc.so"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcnc_tlc && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/linuxcnc_tlc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/build: linuxcnc_tlc/linuxcnc.so

.PHONY : linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/build

linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/clean:
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcnc_tlc && $(CMAKE_COMMAND) -P CMakeFiles/linuxcnc_tlc.dir/cmake_clean.cmake
.PHONY : linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/clean

linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/depend:
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/linuxcnc/cmake /home/user/linuxcnc/cmake/linuxcnc_tlc /home/user/linuxcnc/build-cmake-Desktop-Debug /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcnc_tlc /home/user/linuxcnc/build-cmake-Desktop-Debug/linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : linuxcnc_tlc/CMakeFiles/linuxcnc_tlc.dir/depend

