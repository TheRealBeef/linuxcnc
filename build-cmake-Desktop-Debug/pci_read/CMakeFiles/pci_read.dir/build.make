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
include pci_read/CMakeFiles/pci_read.dir/depend.make

# Include the progress variables for this target.
include pci_read/CMakeFiles/pci_read.dir/progress.make

# Include the compile flags for this target's objects.
include pci_read/CMakeFiles/pci_read.dir/flags.make

pci_read/CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/upci.c.o: pci_read/CMakeFiles/pci_read.dir/flags.make
pci_read/CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/upci.c.o: /home/user/linuxcnc/src/hal/utils/upci.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object pci_read/CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/upci.c.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/pci_read && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/upci.c.o -c /home/user/linuxcnc/src/hal/utils/upci.c

pci_read/CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/upci.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/upci.c.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/pci_read && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/linuxcnc/src/hal/utils/upci.c > CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/upci.c.i

pci_read/CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/upci.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/upci.c.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/pci_read && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/linuxcnc/src/hal/utils/upci.c -o CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/upci.c.s

pci_read/CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/pci_read.c.o: pci_read/CMakeFiles/pci_read.dir/flags.make
pci_read/CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/pci_read.c.o: /home/user/linuxcnc/src/hal/utils/pci_read.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object pci_read/CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/pci_read.c.o"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/pci_read && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/pci_read.c.o -c /home/user/linuxcnc/src/hal/utils/pci_read.c

pci_read/CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/pci_read.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/pci_read.c.i"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/pci_read && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/user/linuxcnc/src/hal/utils/pci_read.c > CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/pci_read.c.i

pci_read/CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/pci_read.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/pci_read.c.s"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/pci_read && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/user/linuxcnc/src/hal/utils/pci_read.c -o CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/pci_read.c.s

# Object files for target pci_read
pci_read_OBJECTS = \
"CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/upci.c.o" \
"CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/pci_read.c.o"

# External object files for target pci_read
pci_read_EXTERNAL_OBJECTS =

pci_read/pci_read-0: pci_read/CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/upci.c.o
pci_read/pci_read-0: pci_read/CMakeFiles/pci_read.dir/home/user/linuxcnc/src/hal/utils/pci_read.c.o
pci_read/pci_read-0: pci_read/CMakeFiles/pci_read.dir/build.make
pci_read/pci_read-0: pci_read/CMakeFiles/pci_read.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/linuxcnc/build-cmake-Desktop-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable pci_read"
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/pci_read && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pci_read.dir/link.txt --verbose=$(VERBOSE)
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/pci_read && $(CMAKE_COMMAND) -E cmake_symlink_executable pci_read-0 pci_read

pci_read/pci_read: pci_read/pci_read-0


# Rule to build all files generated by this target.
pci_read/CMakeFiles/pci_read.dir/build: pci_read/pci_read

.PHONY : pci_read/CMakeFiles/pci_read.dir/build

pci_read/CMakeFiles/pci_read.dir/clean:
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug/pci_read && $(CMAKE_COMMAND) -P CMakeFiles/pci_read.dir/cmake_clean.cmake
.PHONY : pci_read/CMakeFiles/pci_read.dir/clean

pci_read/CMakeFiles/pci_read.dir/depend:
	cd /home/user/linuxcnc/build-cmake-Desktop-Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/linuxcnc/cmake /home/user/linuxcnc/cmake/pci_read /home/user/linuxcnc/build-cmake-Desktop-Debug /home/user/linuxcnc/build-cmake-Desktop-Debug/pci_read /home/user/linuxcnc/build-cmake-Desktop-Debug/pci_read/CMakeFiles/pci_read.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pci_read/CMakeFiles/pci_read.dir/depend

