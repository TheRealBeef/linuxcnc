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
include liblinuxcnc/CMakeFiles/linuxcnc.dir/depend.make

# Include the progress variables for this target.
include liblinuxcnc/CMakeFiles/linuxcnc.dir/progress.make

# Include the compile flags for this target's objects.
include liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcglb.c.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcglb.c.o: /home/user/linuxcnc/src/emc/nml_intf/emcglb.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcglb.c.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcglb.c.o -c /home/user/linuxcnc/src/emc/nml_intf/emcglb.c

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcglb.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcglb.c.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/linuxcnc/src/emc/nml_intf/emcglb.c > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcglb.c.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcglb.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcglb.c.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/linuxcnc/src/emc/nml_intf/emcglb.c -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcglb.c.s

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc.o: /home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc.o -c /home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc.s

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emc.cc.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emc.cc.o: /home/user/linuxcnc/src/emc/nml_intf/emc.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emc.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emc.cc.o -c /home/user/linuxcnc/src/emc/nml_intf/emc.cc

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emc.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emc.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/nml_intf/emc.cc > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emc.cc.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emc.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emc.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/nml_intf/emc.cc -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emc.cc.s

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.o: /home/user/linuxcnc/src/emc/nml_intf/emcpose.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.o -c /home/user/linuxcnc/src/emc/nml_intf/emcpose.c

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/linuxcnc/src/emc/nml_intf/emcpose.c > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/linuxcnc/src/emc/nml_intf/emcpose.c -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.s

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcargs.cc.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcargs.cc.o: /home/user/linuxcnc/src/emc/nml_intf/emcargs.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcargs.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcargs.cc.o -c /home/user/linuxcnc/src/emc/nml_intf/emcargs.cc

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcargs.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcargs.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/nml_intf/emcargs.cc > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcargs.cc.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcargs.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcargs.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/nml_intf/emcargs.cc -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcargs.cc.s

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcops.cc.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcops.cc.o: /home/user/linuxcnc/src/emc/nml_intf/emcops.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcops.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcops.cc.o -c /home/user/linuxcnc/src/emc/nml_intf/emcops.cc

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcops.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcops.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/nml_intf/emcops.cc > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcops.cc.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcops.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcops.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/nml_intf/emcops.cc -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcops.cc.s

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/canon_position.cc.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/canon_position.cc.o: /home/user/linuxcnc/src/emc/nml_intf/canon_position.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/canon_position.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/canon_position.cc.o -c /home/user/linuxcnc/src/emc/nml_intf/canon_position.cc

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/canon_position.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/canon_position.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/nml_intf/canon_position.cc > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/canon_position.cc.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/canon_position.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/canon_position.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/nml_intf/canon_position.cc -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/canon_position.cc.s

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/interpl.cc.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/interpl.cc.o: /home/user/linuxcnc/src/emc/nml_intf/interpl.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/interpl.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/interpl.cc.o -c /home/user/linuxcnc/src/emc/nml_intf/interpl.cc

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/interpl.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/interpl.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/nml_intf/interpl.cc > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/interpl.cc.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/interpl.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/interpl.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/nml_intf/interpl.cc -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/interpl.cc.s

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/emcIniFile.cc.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/emcIniFile.cc.o: /home/user/linuxcnc/src/emc/ini/emcIniFile.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/emcIniFile.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/emcIniFile.cc.o -c /home/user/linuxcnc/src/emc/ini/emcIniFile.cc

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/emcIniFile.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/emcIniFile.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/ini/emcIniFile.cc > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/emcIniFile.cc.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/emcIniFile.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/emcIniFile.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/ini/emcIniFile.cc -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/emcIniFile.cc.s

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/iniaxis.cc.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/iniaxis.cc.o: /home/user/linuxcnc/src/emc/ini/iniaxis.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/iniaxis.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/iniaxis.cc.o -c /home/user/linuxcnc/src/emc/ini/iniaxis.cc

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/iniaxis.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/iniaxis.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/ini/iniaxis.cc > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/iniaxis.cc.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/iniaxis.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/iniaxis.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/ini/iniaxis.cc -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/iniaxis.cc.s

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inijoint.cc.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inijoint.cc.o: /home/user/linuxcnc/src/emc/ini/inijoint.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inijoint.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inijoint.cc.o -c /home/user/linuxcnc/src/emc/ini/inijoint.cc

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inijoint.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inijoint.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/ini/inijoint.cc > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inijoint.cc.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inijoint.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inijoint.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/ini/inijoint.cc -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inijoint.cc.s

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inispindle.cc.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inispindle.cc.o: /home/user/linuxcnc/src/emc/ini/inispindle.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inispindle.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inispindle.cc.o -c /home/user/linuxcnc/src/emc/ini/inispindle.cc

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inispindle.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inispindle.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/ini/inispindle.cc > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inispindle.cc.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inispindle.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inispindle.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/ini/inispindle.cc -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inispindle.cc.s

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/initraj.cc.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/initraj.cc.o: /home/user/linuxcnc/src/emc/ini/initraj.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/initraj.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/initraj.cc.o -c /home/user/linuxcnc/src/emc/ini/initraj.cc

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/initraj.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/initraj.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/ini/initraj.cc > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/initraj.cc.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/initraj.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/initraj.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/ini/initraj.cc -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/initraj.cc.s

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inihal.cc.o: liblinuxcnc/CMakeFiles/linuxcnc.dir/flags.make
liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inihal.cc.o: /home/user/linuxcnc/src/emc/ini/inihal.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inihal.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inihal.cc.o -c /home/user/linuxcnc/src/emc/ini/inihal.cc

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inihal.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inihal.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/ini/inihal.cc > CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inihal.cc.i

liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inihal.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inihal.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/ini/inihal.cc -o CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inihal.cc.s

# Object files for target linuxcnc
linuxcnc_OBJECTS = \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcglb.c.o" \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc.o" \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emc.cc.o" \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.o" \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcargs.cc.o" \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcops.cc.o" \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/canon_position.cc.o" \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/interpl.cc.o" \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/emcIniFile.cc.o" \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/iniaxis.cc.o" \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inijoint.cc.o" \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inispindle.cc.o" \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/initraj.cc.o" \
"CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inihal.cc.o"

# External object files for target linuxcnc
linuxcnc_EXTERNAL_OBJECTS =

liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcglb.c.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/rs274ngc/modal_state.cc.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emc.cc.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcpose.c.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcargs.cc.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/emcops.cc.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/canon_position.cc.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/nml_intf/interpl.cc.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/emcIniFile.cc.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/iniaxis.cc.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inijoint.cc.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inispindle.cc.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/initraj.cc.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/home/user/linuxcnc/src/emc/ini/inihal.cc.o
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/build.make
liblinuxcnc/liblinuxcnc.a: liblinuxcnc/CMakeFiles/linuxcnc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Linking CXX static library liblinuxcnc.a"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && $(CMAKE_COMMAND) -P CMakeFiles/linuxcnc.dir/cmake_clean_target.cmake
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/linuxcnc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
liblinuxcnc/CMakeFiles/linuxcnc.dir/build: liblinuxcnc/liblinuxcnc.a

.PHONY : liblinuxcnc/CMakeFiles/linuxcnc.dir/build

liblinuxcnc/CMakeFiles/linuxcnc.dir/clean:
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc && $(CMAKE_COMMAND) -P CMakeFiles/linuxcnc.dir/cmake_clean.cmake
.PHONY : liblinuxcnc/CMakeFiles/linuxcnc.dir/clean

liblinuxcnc/CMakeFiles/linuxcnc.dir/depend:
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/linuxcnc/cmake /home/user/linuxcnc/cmake/liblinuxcnc /home/user/linuxcnc/build-cmake-Desktop-Debug /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc /home/user/linuxcnc/build-cmake-Desktop-Debug/liblinuxcnc/CMakeFiles/linuxcnc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : liblinuxcnc/CMakeFiles/linuxcnc.dir/depend

