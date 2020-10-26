# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "nuturtlebot: 2 messages, 0 services")

set(MSG_I_FLAGS "-Inuturtlebot:/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(nuturtlebot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/SensorData.msg" NAME_WE)
add_custom_target(_nuturtlebot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nuturtlebot" "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/SensorData.msg" ""
)

get_filename_component(_filename "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/WheelCommands.msg" NAME_WE)
add_custom_target(_nuturtlebot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "nuturtlebot" "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/WheelCommands.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(nuturtlebot
  "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/SensorData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nuturtlebot
)
_generate_msg_cpp(nuturtlebot
  "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/WheelCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nuturtlebot
)

### Generating Services

### Generating Module File
_generate_module_cpp(nuturtlebot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nuturtlebot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(nuturtlebot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(nuturtlebot_generate_messages nuturtlebot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/SensorData.msg" NAME_WE)
add_dependencies(nuturtlebot_generate_messages_cpp _nuturtlebot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/WheelCommands.msg" NAME_WE)
add_dependencies(nuturtlebot_generate_messages_cpp _nuturtlebot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nuturtlebot_gencpp)
add_dependencies(nuturtlebot_gencpp nuturtlebot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nuturtlebot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(nuturtlebot
  "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/SensorData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nuturtlebot
)
_generate_msg_eus(nuturtlebot
  "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/WheelCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nuturtlebot
)

### Generating Services

### Generating Module File
_generate_module_eus(nuturtlebot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nuturtlebot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(nuturtlebot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(nuturtlebot_generate_messages nuturtlebot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/SensorData.msg" NAME_WE)
add_dependencies(nuturtlebot_generate_messages_eus _nuturtlebot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/WheelCommands.msg" NAME_WE)
add_dependencies(nuturtlebot_generate_messages_eus _nuturtlebot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nuturtlebot_geneus)
add_dependencies(nuturtlebot_geneus nuturtlebot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nuturtlebot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(nuturtlebot
  "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/SensorData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nuturtlebot
)
_generate_msg_lisp(nuturtlebot
  "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/WheelCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nuturtlebot
)

### Generating Services

### Generating Module File
_generate_module_lisp(nuturtlebot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nuturtlebot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(nuturtlebot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(nuturtlebot_generate_messages nuturtlebot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/SensorData.msg" NAME_WE)
add_dependencies(nuturtlebot_generate_messages_lisp _nuturtlebot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/WheelCommands.msg" NAME_WE)
add_dependencies(nuturtlebot_generate_messages_lisp _nuturtlebot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nuturtlebot_genlisp)
add_dependencies(nuturtlebot_genlisp nuturtlebot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nuturtlebot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(nuturtlebot
  "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/SensorData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nuturtlebot
)
_generate_msg_nodejs(nuturtlebot
  "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/WheelCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nuturtlebot
)

### Generating Services

### Generating Module File
_generate_module_nodejs(nuturtlebot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nuturtlebot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(nuturtlebot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(nuturtlebot_generate_messages nuturtlebot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/SensorData.msg" NAME_WE)
add_dependencies(nuturtlebot_generate_messages_nodejs _nuturtlebot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/WheelCommands.msg" NAME_WE)
add_dependencies(nuturtlebot_generate_messages_nodejs _nuturtlebot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nuturtlebot_gennodejs)
add_dependencies(nuturtlebot_gennodejs nuturtlebot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nuturtlebot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(nuturtlebot
  "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/SensorData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nuturtlebot
)
_generate_msg_py(nuturtlebot
  "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/WheelCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nuturtlebot
)

### Generating Services

### Generating Module File
_generate_module_py(nuturtlebot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nuturtlebot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(nuturtlebot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(nuturtlebot_generate_messages nuturtlebot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/SensorData.msg" NAME_WE)
add_dependencies(nuturtlebot_generate_messages_py _nuturtlebot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ricojia/coffee_bot/ros_ws/src/default_coffeebot_pkgs/nuturtle/msg/WheelCommands.msg" NAME_WE)
add_dependencies(nuturtlebot_generate_messages_py _nuturtlebot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(nuturtlebot_genpy)
add_dependencies(nuturtlebot_genpy nuturtlebot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS nuturtlebot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nuturtlebot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/nuturtlebot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nuturtlebot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/nuturtlebot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nuturtlebot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/nuturtlebot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nuturtlebot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/nuturtlebot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nuturtlebot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nuturtlebot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/nuturtlebot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
