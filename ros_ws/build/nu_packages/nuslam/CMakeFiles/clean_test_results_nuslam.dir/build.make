# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/ricojia/coffee_bot/ros_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ricojia/coffee_bot/ros_ws/build

# Utility rule file for clean_test_results_nuslam.

# Include the progress variables for this target.
include nu_packages/nuslam/CMakeFiles/clean_test_results_nuslam.dir/progress.make

nu_packages/nuslam/CMakeFiles/clean_test_results_nuslam:
	cd /home/ricojia/coffee_bot/ros_ws/build/nu_packages/nuslam && /usr/bin/python2 /opt/ros/melodic/share/catkin/cmake/test/remove_test_results.py /home/ricojia/coffee_bot/ros_ws/build/test_results/nuslam

clean_test_results_nuslam: nu_packages/nuslam/CMakeFiles/clean_test_results_nuslam
clean_test_results_nuslam: nu_packages/nuslam/CMakeFiles/clean_test_results_nuslam.dir/build.make

.PHONY : clean_test_results_nuslam

# Rule to build all files generated by this target.
nu_packages/nuslam/CMakeFiles/clean_test_results_nuslam.dir/build: clean_test_results_nuslam

.PHONY : nu_packages/nuslam/CMakeFiles/clean_test_results_nuslam.dir/build

nu_packages/nuslam/CMakeFiles/clean_test_results_nuslam.dir/clean:
	cd /home/ricojia/coffee_bot/ros_ws/build/nu_packages/nuslam && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_nuslam.dir/cmake_clean.cmake
.PHONY : nu_packages/nuslam/CMakeFiles/clean_test_results_nuslam.dir/clean

nu_packages/nuslam/CMakeFiles/clean_test_results_nuslam.dir/depend:
	cd /home/ricojia/coffee_bot/ros_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ricojia/coffee_bot/ros_ws/src /home/ricojia/coffee_bot/ros_ws/src/nu_packages/nuslam /home/ricojia/coffee_bot/ros_ws/build /home/ricojia/coffee_bot/ros_ws/build/nu_packages/nuslam /home/ricojia/coffee_bot/ros_ws/build/nu_packages/nuslam/CMakeFiles/clean_test_results_nuslam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : nu_packages/nuslam/CMakeFiles/clean_test_results_nuslam.dir/depend
