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
include components/tpmod/CMakeFiles/tpmod.dir/depend.make

# Include the progress variables for this target.
include components/tpmod/CMakeFiles/tpmod.dir/progress.make

# Include the compile flags for this target's objects.
include components/tpmod/CMakeFiles/tpmod.dir/flags.make

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tpmod.c.o: components/tpmod/CMakeFiles/tpmod.dir/flags.make
components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tpmod.c.o: /home/user/linuxcnc/src/emc/tp/tpmod.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tpmod.c.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tpmod.c.o -c /home/user/linuxcnc/src/emc/tp/tpmod.c

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tpmod.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tpmod.c.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/linuxcnc/src/emc/tp/tpmod.c > CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tpmod.c.i

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tpmod.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tpmod.c.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/linuxcnc/src/emc/tp/tpmod.c -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tpmod.c.s

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tc.c.o: components/tpmod/CMakeFiles/tpmod.dir/flags.make
components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tc.c.o: /home/user/linuxcnc/src/emc/tp/tc.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tc.c.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tc.c.o -c /home/user/linuxcnc/src/emc/tp/tc.c

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tc.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tc.c.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/linuxcnc/src/emc/tp/tc.c > CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tc.c.i

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tc.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tc.c.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/linuxcnc/src/emc/tp/tc.c -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tc.c.s

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tcq.c.o: components/tpmod/CMakeFiles/tpmod.dir/flags.make
components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tcq.c.o: /home/user/linuxcnc/src/emc/tp/tcq.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tcq.c.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tcq.c.o -c /home/user/linuxcnc/src/emc/tp/tcq.c

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tcq.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tcq.c.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/linuxcnc/src/emc/tp/tcq.c > CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tcq.c.i

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tcq.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tcq.c.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/linuxcnc/src/emc/tp/tcq.c -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tcq.c.s

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tp.c.o: components/tpmod/CMakeFiles/tpmod.dir/flags.make
components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tp.c.o: /home/user/linuxcnc/src/emc/tp/tp.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tp.c.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tp.c.o -c /home/user/linuxcnc/src/emc/tp/tp.c

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tp.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tp.c.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/linuxcnc/src/emc/tp/tp.c > CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tp.c.i

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tp.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tp.c.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/linuxcnc/src/emc/tp/tp.c -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tp.c.s

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/spherical_arc.c.o: components/tpmod/CMakeFiles/tpmod.dir/flags.make
components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/spherical_arc.c.o: /home/user/linuxcnc/src/emc/tp/spherical_arc.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/spherical_arc.c.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/spherical_arc.c.o -c /home/user/linuxcnc/src/emc/tp/spherical_arc.c

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/spherical_arc.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/spherical_arc.c.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/linuxcnc/src/emc/tp/spherical_arc.c > CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/spherical_arc.c.i

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/spherical_arc.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/spherical_arc.c.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/linuxcnc/src/emc/tp/spherical_arc.c -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/spherical_arc.c.s

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/blendmath.c.o: components/tpmod/CMakeFiles/tpmod.dir/flags.make
components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/blendmath.c.o: /home/user/linuxcnc/src/emc/tp/blendmath.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/blendmath.c.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/blendmath.c.o -c /home/user/linuxcnc/src/emc/tp/blendmath.c

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/blendmath.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/blendmath.c.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/linuxcnc/src/emc/tp/blendmath.c > CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/blendmath.c.i

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/blendmath.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/blendmath.c.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/linuxcnc/src/emc/tp/blendmath.c -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/blendmath.c.s

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.o: components/tpmod/CMakeFiles/tpmod.dir/flags.make
components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.o: /home/user/linuxcnc/src/emc/nml_intf/emcpose.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.o -c /home/user/linuxcnc/src/emc/nml_intf/emcpose.c

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/linuxcnc/src/emc/nml_intf/emcpose.c > CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.i

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/linuxcnc/src/emc/nml_intf/emcpose.c -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.s

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/_posemath.c.o: components/tpmod/CMakeFiles/tpmod.dir/flags.make
components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/_posemath.c.o: /home/user/linuxcnc/src/libnml/posemath/_posemath.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/_posemath.c.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/_posemath.c.o -c /home/user/linuxcnc/src/libnml/posemath/_posemath.c

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/_posemath.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/_posemath.c.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/linuxcnc/src/libnml/posemath/_posemath.c > CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/_posemath.c.i

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/_posemath.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/_posemath.c.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/linuxcnc/src/libnml/posemath/_posemath.c -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/_posemath.c.s

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/sincos.c.o: components/tpmod/CMakeFiles/tpmod.dir/flags.make
components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/sincos.c.o: /home/user/linuxcnc/src/libnml/posemath/sincos.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/sincos.c.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/sincos.c.o -c /home/user/linuxcnc/src/libnml/posemath/sincos.c

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/sincos.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/sincos.c.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/linuxcnc/src/libnml/posemath/sincos.c > CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/sincos.c.i

components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/sincos.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/sincos.c.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/linuxcnc/src/libnml/posemath/sincos.c -o CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/sincos.c.s

# Object files for target tpmod
tpmod_OBJECTS = \
"CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tpmod.c.o" \
"CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tc.c.o" \
"CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tcq.c.o" \
"CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tp.c.o" \
"CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/spherical_arc.c.o" \
"CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/blendmath.c.o" \
"CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.o" \
"CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/_posemath.c.o" \
"CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/sincos.c.o"

# External object files for target tpmod
tpmod_EXTERNAL_OBJECTS =

components/tpmod/tpmod.so: components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tpmod.c.o
components/tpmod/tpmod.so: components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tc.c.o
components/tpmod/tpmod.so: components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tcq.c.o
components/tpmod/tpmod.so: components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/tp.c.o
components/tpmod/tpmod.so: components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/spherical_arc.c.o
components/tpmod/tpmod.so: components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/tp/blendmath.c.o
components/tpmod/tpmod.so: components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.o
components/tpmod/tpmod.so: components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/_posemath.c.o
components/tpmod/tpmod.so: components/tpmod/CMakeFiles/tpmod.dir/home/user/linuxcnc/src/libnml/posemath/sincos.c.o
components/tpmod/tpmod.so: components/tpmod/CMakeFiles/tpmod.dir/build.make
components/tpmod/tpmod.so: /home/user/linuxcnc/cmake/components/tpmod/../../build/liblinuxcnchal/liblinuxcnchal.so
components/tpmod/tpmod.so: components/tpmod/CMakeFiles/tpmod.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking C shared library tpmod.so"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tpmod.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
components/tpmod/CMakeFiles/tpmod.dir/build: components/tpmod/tpmod.so

.PHONY : components/tpmod/CMakeFiles/tpmod.dir/build

components/tpmod/CMakeFiles/tpmod.dir/clean:
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod && $(CMAKE_COMMAND) -P CMakeFiles/tpmod.dir/cmake_clean.cmake
.PHONY : components/tpmod/CMakeFiles/tpmod.dir/clean

components/tpmod/CMakeFiles/tpmod.dir/depend:
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/linuxcnc/cmake /home/user/linuxcnc/cmake/components/tpmod /home/user/linuxcnc/build-cmake-Desktop-Debug /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod /home/user/linuxcnc/build-cmake-Desktop-Debug/components/tpmod/CMakeFiles/tpmod.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : components/tpmod/CMakeFiles/tpmod.dir/depend

