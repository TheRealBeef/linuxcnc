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
include schedrmt/CMakeFiles/schedrmt.dir/depend.make

# Include the progress variables for this target.
include schedrmt/CMakeFiles/schedrmt.dir/progress.make

# Include the compile flags for this target's objects.
include schedrmt/CMakeFiles/schedrmt.dir/flags.make

schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc.o: schedrmt/CMakeFiles/schedrmt.dir/flags.make
schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc.o: /home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/schedrmt && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc.o -c /home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc

schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/schedrmt && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc > CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc.i

schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/schedrmt && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc -o CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc.s

schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/emcsched.cc.o: schedrmt/CMakeFiles/schedrmt.dir/flags.make
schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/emcsched.cc.o: /home/user/linuxcnc/src/emc/usr_intf/emcsched.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/emcsched.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/schedrmt && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/emcsched.cc.o -c /home/user/linuxcnc/src/emc/usr_intf/emcsched.cc

schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/emcsched.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/emcsched.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/schedrmt && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/usr_intf/emcsched.cc > CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/emcsched.cc.i

schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/emcsched.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/emcsched.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/schedrmt && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/usr_intf/emcsched.cc -o CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/emcsched.cc.s

schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.o: schedrmt/CMakeFiles/schedrmt.dir/flags.make
schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.o: /home/user/linuxcnc/src/emc/usr_intf/shcom.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/schedrmt && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.o -c /home/user/linuxcnc/src/emc/usr_intf/shcom.cc

schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/schedrmt && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/linuxcnc/src/emc/usr_intf/shcom.cc > CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.i

schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/schedrmt && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/linuxcnc/src/emc/usr_intf/shcom.cc -o CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.s

# Object files for target schedrmt
schedrmt_OBJECTS = \
"CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc.o" \
"CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/emcsched.cc.o" \
"CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.o"

# External object files for target schedrmt
schedrmt_EXTERNAL_OBJECTS =

schedrmt/schedrmt: schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/schedrmt.cc.o
schedrmt/schedrmt: schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/emcsched.cc.o
schedrmt/schedrmt: schedrmt/CMakeFiles/schedrmt.dir/home/user/linuxcnc/src/emc/usr_intf/shcom.cc.o
schedrmt/schedrmt: schedrmt/CMakeFiles/schedrmt.dir/build.make
schedrmt/schedrmt: /home/user/linuxcnc/cmake/schedrmt/../build/libnml/libnml.so
schedrmt/schedrmt: /home/user/linuxcnc/cmake/schedrmt/../build/liblinuxcnchal/liblinuxcnchal.so
schedrmt/schedrmt: /home/user/linuxcnc/cmake/schedrmt/../build/liblinuxcncini/liblinuxcncini.so
schedrmt/schedrmt: /home/user/linuxcnc/cmake/schedrmt/../build/liblinuxcnc/liblinuxcnc.a
schedrmt/schedrmt: schedrmt/CMakeFiles/schedrmt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable schedrmt"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/schedrmt && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/schedrmt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
schedrmt/CMakeFiles/schedrmt.dir/build: schedrmt/schedrmt

.PHONY : schedrmt/CMakeFiles/schedrmt.dir/build

schedrmt/CMakeFiles/schedrmt.dir/clean:
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/schedrmt && $(CMAKE_COMMAND) -P CMakeFiles/schedrmt.dir/cmake_clean.cmake
.PHONY : schedrmt/CMakeFiles/schedrmt.dir/clean

schedrmt/CMakeFiles/schedrmt.dir/depend:
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/linuxcnc/cmake /home/user/linuxcnc/cmake/schedrmt /home/user/linuxcnc/build-cmake-Desktop-Debug /home/user/linuxcnc/build-cmake-Desktop-Debug/schedrmt /home/user/linuxcnc/build-cmake-Desktop-Debug/schedrmt/CMakeFiles/schedrmt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : schedrmt/CMakeFiles/schedrmt.dir/depend

