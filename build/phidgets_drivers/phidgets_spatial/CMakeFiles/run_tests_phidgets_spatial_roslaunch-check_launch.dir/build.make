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

# Utility rule file for run_tests_phidgets_spatial_roslaunch-check_launch.

# Include the progress variables for this target.
include phidgets_drivers/phidgets_spatial/CMakeFiles/run_tests_phidgets_spatial_roslaunch-check_launch.dir/progress.make

phidgets_drivers/phidgets_spatial/CMakeFiles/run_tests_phidgets_spatial_roslaunch-check_launch:
	cd /home/ubuntu/Localization2023_ws/build/phidgets_drivers/phidgets_spatial && ../../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/run_tests.py /home/ubuntu/Localization2023_ws/build/test_results/phidgets_spatial/roslaunch-check_launch.xml "/usr/bin/cmake -E make_directory /home/ubuntu/Localization2023_ws/build/test_results/phidgets_spatial" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/ubuntu/Localization2023_ws/build/test_results/phidgets_spatial/roslaunch-check_launch.xml\" \"/home/ubuntu/Localization2023_ws/src/phidgets_drivers/phidgets_spatial/launch\" "

run_tests_phidgets_spatial_roslaunch-check_launch: phidgets_drivers/phidgets_spatial/CMakeFiles/run_tests_phidgets_spatial_roslaunch-check_launch
run_tests_phidgets_spatial_roslaunch-check_launch: phidgets_drivers/phidgets_spatial/CMakeFiles/run_tests_phidgets_spatial_roslaunch-check_launch.dir/build.make

.PHONY : run_tests_phidgets_spatial_roslaunch-check_launch

# Rule to build all files generated by this target.
phidgets_drivers/phidgets_spatial/CMakeFiles/run_tests_phidgets_spatial_roslaunch-check_launch.dir/build: run_tests_phidgets_spatial_roslaunch-check_launch

.PHONY : phidgets_drivers/phidgets_spatial/CMakeFiles/run_tests_phidgets_spatial_roslaunch-check_launch.dir/build

phidgets_drivers/phidgets_spatial/CMakeFiles/run_tests_phidgets_spatial_roslaunch-check_launch.dir/clean:
	cd /home/ubuntu/Localization2023_ws/build/phidgets_drivers/phidgets_spatial && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_phidgets_spatial_roslaunch-check_launch.dir/cmake_clean.cmake
.PHONY : phidgets_drivers/phidgets_spatial/CMakeFiles/run_tests_phidgets_spatial_roslaunch-check_launch.dir/clean

phidgets_drivers/phidgets_spatial/CMakeFiles/run_tests_phidgets_spatial_roslaunch-check_launch.dir/depend:
	cd /home/ubuntu/Localization2023_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/Localization2023_ws/src /home/ubuntu/Localization2023_ws/src/phidgets_drivers/phidgets_spatial /home/ubuntu/Localization2023_ws/build /home/ubuntu/Localization2023_ws/build/phidgets_drivers/phidgets_spatial /home/ubuntu/Localization2023_ws/build/phidgets_drivers/phidgets_spatial/CMakeFiles/run_tests_phidgets_spatial_roslaunch-check_launch.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : phidgets_drivers/phidgets_spatial/CMakeFiles/run_tests_phidgets_spatial_roslaunch-check_launch.dir/depend

