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

# Utility rule file for clean_test_results_phidgets_accelerometer.

# Include the progress variables for this target.
include phidgets_drivers/phidgets_accelerometer/CMakeFiles/clean_test_results_phidgets_accelerometer.dir/progress.make

phidgets_drivers/phidgets_accelerometer/CMakeFiles/clean_test_results_phidgets_accelerometer:
	cd /home/ubuntu/Localization2023_ws/build/phidgets_drivers/phidgets_accelerometer && /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/remove_test_results.py /home/ubuntu/Localization2023_ws/build/test_results/phidgets_accelerometer

clean_test_results_phidgets_accelerometer: phidgets_drivers/phidgets_accelerometer/CMakeFiles/clean_test_results_phidgets_accelerometer
clean_test_results_phidgets_accelerometer: phidgets_drivers/phidgets_accelerometer/CMakeFiles/clean_test_results_phidgets_accelerometer.dir/build.make

.PHONY : clean_test_results_phidgets_accelerometer

# Rule to build all files generated by this target.
phidgets_drivers/phidgets_accelerometer/CMakeFiles/clean_test_results_phidgets_accelerometer.dir/build: clean_test_results_phidgets_accelerometer

.PHONY : phidgets_drivers/phidgets_accelerometer/CMakeFiles/clean_test_results_phidgets_accelerometer.dir/build

phidgets_drivers/phidgets_accelerometer/CMakeFiles/clean_test_results_phidgets_accelerometer.dir/clean:
	cd /home/ubuntu/Localization2023_ws/build/phidgets_drivers/phidgets_accelerometer && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_phidgets_accelerometer.dir/cmake_clean.cmake
.PHONY : phidgets_drivers/phidgets_accelerometer/CMakeFiles/clean_test_results_phidgets_accelerometer.dir/clean

phidgets_drivers/phidgets_accelerometer/CMakeFiles/clean_test_results_phidgets_accelerometer.dir/depend:
	cd /home/ubuntu/Localization2023_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/Localization2023_ws/src /home/ubuntu/Localization2023_ws/src/phidgets_drivers/phidgets_accelerometer /home/ubuntu/Localization2023_ws/build /home/ubuntu/Localization2023_ws/build/phidgets_drivers/phidgets_accelerometer /home/ubuntu/Localization2023_ws/build/phidgets_drivers/phidgets_accelerometer/CMakeFiles/clean_test_results_phidgets_accelerometer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : phidgets_drivers/phidgets_accelerometer/CMakeFiles/clean_test_results_phidgets_accelerometer.dir/depend

