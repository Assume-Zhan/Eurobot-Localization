# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/ubuntu/Localization2023_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/Localization2023_ws/build

# Utility rule file for run_tests_phidgets_magnetometer.

# Include the progress variables for this target.
include phidgets_drivers/phidgets_magnetometer/CMakeFiles/run_tests_phidgets_magnetometer.dir/progress.make

run_tests_phidgets_magnetometer: phidgets_drivers/phidgets_magnetometer/CMakeFiles/run_tests_phidgets_magnetometer.dir/build.make

.PHONY : run_tests_phidgets_magnetometer

# Rule to build all files generated by this target.
phidgets_drivers/phidgets_magnetometer/CMakeFiles/run_tests_phidgets_magnetometer.dir/build: run_tests_phidgets_magnetometer

.PHONY : phidgets_drivers/phidgets_magnetometer/CMakeFiles/run_tests_phidgets_magnetometer.dir/build

phidgets_drivers/phidgets_magnetometer/CMakeFiles/run_tests_phidgets_magnetometer.dir/clean:
	cd /home/ubuntu/Localization2023_ws/build/phidgets_drivers/phidgets_magnetometer && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_phidgets_magnetometer.dir/cmake_clean.cmake
.PHONY : phidgets_drivers/phidgets_magnetometer/CMakeFiles/run_tests_phidgets_magnetometer.dir/clean

phidgets_drivers/phidgets_magnetometer/CMakeFiles/run_tests_phidgets_magnetometer.dir/depend:
	cd /home/ubuntu/Localization2023_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/Localization2023_ws/src /home/ubuntu/Localization2023_ws/src/phidgets_drivers/phidgets_magnetometer /home/ubuntu/Localization2023_ws/build /home/ubuntu/Localization2023_ws/build/phidgets_drivers/phidgets_magnetometer /home/ubuntu/Localization2023_ws/build/phidgets_drivers/phidgets_magnetometer/CMakeFiles/run_tests_phidgets_magnetometer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : phidgets_drivers/phidgets_magnetometer/CMakeFiles/run_tests_phidgets_magnetometer.dir/depend

