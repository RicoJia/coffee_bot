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

# Utility rule file for phidgets_msgs_generate_messages_cpp.

# Include the progress variables for this target.
include onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/CMakeFiles/phidgets_msgs_generate_messages_cpp.dir/progress.make

onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/CMakeFiles/phidgets_msgs_generate_messages_cpp: /home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs/EncoderDecimatedSpeed.h
onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/CMakeFiles/phidgets_msgs_generate_messages_cpp: /home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs/SetDigitalOutput.h


/home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs/EncoderDecimatedSpeed.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
/home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs/EncoderDecimatedSpeed.h: /home/ricojia/coffee_bot/ros_ws/src/onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/msg/EncoderDecimatedSpeed.msg
/home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs/EncoderDecimatedSpeed.h: /opt/ros/melodic/share/std_msgs/msg/Header.msg
/home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs/EncoderDecimatedSpeed.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ricojia/coffee_bot/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from phidgets_msgs/EncoderDecimatedSpeed.msg"
	cd /home/ricojia/coffee_bot/ros_ws/src/onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs && /home/ricojia/coffee_bot/ros_ws/build/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/ricojia/coffee_bot/ros_ws/src/onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/msg/EncoderDecimatedSpeed.msg -Iphidgets_msgs:/home/ricojia/coffee_bot/ros_ws/src/onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p phidgets_msgs -o /home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs -e /opt/ros/melodic/share/gencpp/cmake/..

/home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs/SetDigitalOutput.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
/home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs/SetDigitalOutput.h: /home/ricojia/coffee_bot/ros_ws/src/onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/srv/SetDigitalOutput.srv
/home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs/SetDigitalOutput.h: /opt/ros/melodic/share/gencpp/msg.h.template
/home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs/SetDigitalOutput.h: /opt/ros/melodic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ricojia/coffee_bot/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating C++ code from phidgets_msgs/SetDigitalOutput.srv"
	cd /home/ricojia/coffee_bot/ros_ws/src/onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs && /home/ricojia/coffee_bot/ros_ws/build/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/ricojia/coffee_bot/ros_ws/src/onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/srv/SetDigitalOutput.srv -Iphidgets_msgs:/home/ricojia/coffee_bot/ros_ws/src/onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p phidgets_msgs -o /home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs -e /opt/ros/melodic/share/gencpp/cmake/..

phidgets_msgs_generate_messages_cpp: onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/CMakeFiles/phidgets_msgs_generate_messages_cpp
phidgets_msgs_generate_messages_cpp: /home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs/EncoderDecimatedSpeed.h
phidgets_msgs_generate_messages_cpp: /home/ricojia/coffee_bot/ros_ws/devel/include/phidgets_msgs/SetDigitalOutput.h
phidgets_msgs_generate_messages_cpp: onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/CMakeFiles/phidgets_msgs_generate_messages_cpp.dir/build.make

.PHONY : phidgets_msgs_generate_messages_cpp

# Rule to build all files generated by this target.
onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/CMakeFiles/phidgets_msgs_generate_messages_cpp.dir/build: phidgets_msgs_generate_messages_cpp

.PHONY : onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/CMakeFiles/phidgets_msgs_generate_messages_cpp.dir/build

onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/CMakeFiles/phidgets_msgs_generate_messages_cpp.dir/clean:
	cd /home/ricojia/coffee_bot/ros_ws/build/onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs && $(CMAKE_COMMAND) -P CMakeFiles/phidgets_msgs_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/CMakeFiles/phidgets_msgs_generate_messages_cpp.dir/clean

onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/CMakeFiles/phidgets_msgs_generate_messages_cpp.dir/depend:
	cd /home/ricojia/coffee_bot/ros_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ricojia/coffee_bot/ros_ws/src /home/ricojia/coffee_bot/ros_ws/src/onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs /home/ricojia/coffee_bot/ros_ws/build /home/ricojia/coffee_bot/ros_ws/build/onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs /home/ricojia/coffee_bot/ros_ws/build/onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/CMakeFiles/phidgets_msgs_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : onboard_pkgs/phidget_imu/phidgets_drivers/phidgets_msgs/CMakeFiles/phidgets_msgs_generate_messages_cpp.dir/depend

