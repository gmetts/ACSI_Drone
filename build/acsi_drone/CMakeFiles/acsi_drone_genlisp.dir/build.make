# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/cerlabcontrol/Desktop/Drone Rewrite/src"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/cerlabcontrol/Desktop/Drone Rewrite/build"

# Utility rule file for acsi_drone_genlisp.

# Include the progress variables for this target.
include acsi_drone/CMakeFiles/acsi_drone_genlisp.dir/progress.make

acsi_drone_genlisp: acsi_drone/CMakeFiles/acsi_drone_genlisp.dir/build.make

.PHONY : acsi_drone_genlisp

# Rule to build all files generated by this target.
acsi_drone/CMakeFiles/acsi_drone_genlisp.dir/build: acsi_drone_genlisp

.PHONY : acsi_drone/CMakeFiles/acsi_drone_genlisp.dir/build

acsi_drone/CMakeFiles/acsi_drone_genlisp.dir/clean:
	cd "/home/cerlabcontrol/Desktop/Drone Rewrite/build/acsi_drone" && $(CMAKE_COMMAND) -P CMakeFiles/acsi_drone_genlisp.dir/cmake_clean.cmake
.PHONY : acsi_drone/CMakeFiles/acsi_drone_genlisp.dir/clean

acsi_drone/CMakeFiles/acsi_drone_genlisp.dir/depend:
	cd "/home/cerlabcontrol/Desktop/Drone Rewrite/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/cerlabcontrol/Desktop/Drone Rewrite/src" "/home/cerlabcontrol/Desktop/Drone Rewrite/src/acsi_drone" "/home/cerlabcontrol/Desktop/Drone Rewrite/build" "/home/cerlabcontrol/Desktop/Drone Rewrite/build/acsi_drone" "/home/cerlabcontrol/Desktop/Drone Rewrite/build/acsi_drone/CMakeFiles/acsi_drone_genlisp.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : acsi_drone/CMakeFiles/acsi_drone_genlisp.dir/depend

