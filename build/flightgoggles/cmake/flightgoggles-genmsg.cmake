# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "flightgoggles: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iflightgoggles:/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(flightgoggles_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg" NAME_WE)
add_custom_target(_flightgoggles_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flightgoggles" "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg" "std_msgs/String"
)

get_filename_component(_filename "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarkerArray.msg" NAME_WE)
add_custom_target(_flightgoggles_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "flightgoggles" "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarkerArray.msg" "flightgoggles/IRMarker:std_msgs/Header:std_msgs/String"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(flightgoggles
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flightgoggles
)
_generate_msg_cpp(flightgoggles
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flightgoggles
)

### Generating Services

### Generating Module File
_generate_module_cpp(flightgoggles
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flightgoggles
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(flightgoggles_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(flightgoggles_generate_messages flightgoggles_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg" NAME_WE)
add_dependencies(flightgoggles_generate_messages_cpp _flightgoggles_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarkerArray.msg" NAME_WE)
add_dependencies(flightgoggles_generate_messages_cpp _flightgoggles_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flightgoggles_gencpp)
add_dependencies(flightgoggles_gencpp flightgoggles_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flightgoggles_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(flightgoggles
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flightgoggles
)
_generate_msg_eus(flightgoggles
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flightgoggles
)

### Generating Services

### Generating Module File
_generate_module_eus(flightgoggles
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flightgoggles
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(flightgoggles_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(flightgoggles_generate_messages flightgoggles_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg" NAME_WE)
add_dependencies(flightgoggles_generate_messages_eus _flightgoggles_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarkerArray.msg" NAME_WE)
add_dependencies(flightgoggles_generate_messages_eus _flightgoggles_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flightgoggles_geneus)
add_dependencies(flightgoggles_geneus flightgoggles_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flightgoggles_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(flightgoggles
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flightgoggles
)
_generate_msg_lisp(flightgoggles
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flightgoggles
)

### Generating Services

### Generating Module File
_generate_module_lisp(flightgoggles
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flightgoggles
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(flightgoggles_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(flightgoggles_generate_messages flightgoggles_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg" NAME_WE)
add_dependencies(flightgoggles_generate_messages_lisp _flightgoggles_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarkerArray.msg" NAME_WE)
add_dependencies(flightgoggles_generate_messages_lisp _flightgoggles_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flightgoggles_genlisp)
add_dependencies(flightgoggles_genlisp flightgoggles_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flightgoggles_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(flightgoggles
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flightgoggles
)
_generate_msg_nodejs(flightgoggles
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flightgoggles
)

### Generating Services

### Generating Module File
_generate_module_nodejs(flightgoggles
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flightgoggles
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(flightgoggles_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(flightgoggles_generate_messages flightgoggles_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg" NAME_WE)
add_dependencies(flightgoggles_generate_messages_nodejs _flightgoggles_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarkerArray.msg" NAME_WE)
add_dependencies(flightgoggles_generate_messages_nodejs _flightgoggles_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flightgoggles_gennodejs)
add_dependencies(flightgoggles_gennodejs flightgoggles_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flightgoggles_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(flightgoggles
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarkerArray.msg"
  "${MSG_I_FLAGS}"
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flightgoggles
)
_generate_msg_py(flightgoggles
  "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flightgoggles
)

### Generating Services

### Generating Module File
_generate_module_py(flightgoggles
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flightgoggles
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(flightgoggles_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(flightgoggles_generate_messages flightgoggles_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarker.msg" NAME_WE)
add_dependencies(flightgoggles_generate_messages_py _flightgoggles_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robotclass/fastsim/src/flightgoggles/flightgoggles/msg/IRMarkerArray.msg" NAME_WE)
add_dependencies(flightgoggles_generate_messages_py _flightgoggles_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(flightgoggles_genpy)
add_dependencies(flightgoggles_genpy flightgoggles_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS flightgoggles_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flightgoggles)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/flightgoggles
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(flightgoggles_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flightgoggles)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/flightgoggles
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(flightgoggles_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flightgoggles)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/flightgoggles
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(flightgoggles_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flightgoggles)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/flightgoggles
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(flightgoggles_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flightgoggles)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flightgoggles\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/flightgoggles
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(flightgoggles_generate_messages_py std_msgs_generate_messages_py)
endif()
