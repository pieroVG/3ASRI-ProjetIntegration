# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "motoman_msgs: 5 messages, 8 services")

set(MSG_I_FLAGS "-Imotoman_msgs:/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg;-Iindustrial_msgs:/opt/ros/noetic/share/industrial_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/noetic/share/trajectory_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(motoman_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg" NAME_WE)
add_custom_target(_motoman_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motoman_msgs" "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg" "motoman_msgs/DynamicJointsGroup"
)

get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg" NAME_WE)
add_custom_target(_motoman_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motoman_msgs" "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg" ""
)

get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg" NAME_WE)
add_custom_target(_motoman_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motoman_msgs" "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg" "std_msgs/Header:motoman_msgs/DynamicJointPoint:motoman_msgs/DynamicJointsGroup"
)

get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectoryFeedback.msg" NAME_WE)
add_custom_target(_motoman_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motoman_msgs" "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectoryFeedback.msg" "std_msgs/Header:motoman_msgs/DynamicJointState"
)

get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg" NAME_WE)
add_custom_target(_motoman_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motoman_msgs" "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg" ""
)

get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/CmdJointTrajectoryEx.srv" NAME_WE)
add_custom_target(_motoman_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motoman_msgs" "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/CmdJointTrajectoryEx.srv" "std_msgs/Header:industrial_msgs/ServiceReturnCode:motoman_msgs/DynamicJointPoint:motoman_msgs/DynamicJointsGroup:motoman_msgs/DynamicJointTrajectory"
)

get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadGroupIO.srv" NAME_WE)
add_custom_target(_motoman_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motoman_msgs" "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadGroupIO.srv" ""
)

get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadMRegister.srv" NAME_WE)
add_custom_target(_motoman_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motoman_msgs" "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadMRegister.srv" ""
)

get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadSingleIO.srv" NAME_WE)
add_custom_target(_motoman_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motoman_msgs" "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadSingleIO.srv" ""
)

get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/SelectTool.srv" NAME_WE)
add_custom_target(_motoman_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motoman_msgs" "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/SelectTool.srv" ""
)

get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteGroupIO.srv" NAME_WE)
add_custom_target(_motoman_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motoman_msgs" "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteGroupIO.srv" ""
)

get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteMRegister.srv" NAME_WE)
add_custom_target(_motoman_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motoman_msgs" "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteMRegister.srv" ""
)

get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteSingleIO.srv" NAME_WE)
add_custom_target(_motoman_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motoman_msgs" "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteSingleIO.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg"
  "${MSG_I_FLAGS}"
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
)
_generate_msg_cpp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
)
_generate_msg_cpp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
)
_generate_msg_cpp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
)
_generate_msg_cpp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
)

### Generating Services
_generate_srv_cpp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/CmdJointTrajectoryEx.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/industrial_msgs/cmake/../msg/ServiceReturnCode.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_cpp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadGroupIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_cpp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadMRegister.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_cpp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadSingleIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_cpp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/SelectTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_cpp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteGroupIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_cpp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteMRegister.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_cpp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteSingleIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
)

### Generating Module File
_generate_module_cpp(motoman_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(motoman_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(motoman_msgs_generate_messages motoman_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_cpp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_cpp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_cpp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectoryFeedback.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_cpp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_cpp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/CmdJointTrajectoryEx.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_cpp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadGroupIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_cpp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadMRegister.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_cpp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadSingleIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_cpp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/SelectTool.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_cpp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteGroupIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_cpp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteMRegister.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_cpp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteSingleIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_cpp _motoman_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motoman_msgs_gencpp)
add_dependencies(motoman_msgs_gencpp motoman_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motoman_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg"
  "${MSG_I_FLAGS}"
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
)
_generate_msg_eus(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
)
_generate_msg_eus(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
)
_generate_msg_eus(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
)
_generate_msg_eus(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
)

### Generating Services
_generate_srv_eus(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/CmdJointTrajectoryEx.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/industrial_msgs/cmake/../msg/ServiceReturnCode.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
)
_generate_srv_eus(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadGroupIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
)
_generate_srv_eus(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadMRegister.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
)
_generate_srv_eus(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadSingleIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
)
_generate_srv_eus(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/SelectTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
)
_generate_srv_eus(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteGroupIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
)
_generate_srv_eus(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteMRegister.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
)
_generate_srv_eus(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteSingleIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
)

### Generating Module File
_generate_module_eus(motoman_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(motoman_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(motoman_msgs_generate_messages motoman_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_eus _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_eus _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_eus _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectoryFeedback.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_eus _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_eus _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/CmdJointTrajectoryEx.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_eus _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadGroupIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_eus _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadMRegister.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_eus _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadSingleIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_eus _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/SelectTool.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_eus _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteGroupIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_eus _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteMRegister.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_eus _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteSingleIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_eus _motoman_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motoman_msgs_geneus)
add_dependencies(motoman_msgs_geneus motoman_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motoman_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg"
  "${MSG_I_FLAGS}"
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
)
_generate_msg_lisp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
)
_generate_msg_lisp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
)
_generate_msg_lisp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
)
_generate_msg_lisp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
)

### Generating Services
_generate_srv_lisp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/CmdJointTrajectoryEx.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/industrial_msgs/cmake/../msg/ServiceReturnCode.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_lisp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadGroupIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_lisp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadMRegister.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_lisp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadSingleIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_lisp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/SelectTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_lisp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteGroupIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_lisp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteMRegister.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
)
_generate_srv_lisp(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteSingleIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
)

### Generating Module File
_generate_module_lisp(motoman_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(motoman_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(motoman_msgs_generate_messages motoman_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_lisp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_lisp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_lisp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectoryFeedback.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_lisp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_lisp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/CmdJointTrajectoryEx.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_lisp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadGroupIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_lisp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadMRegister.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_lisp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadSingleIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_lisp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/SelectTool.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_lisp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteGroupIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_lisp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteMRegister.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_lisp _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteSingleIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_lisp _motoman_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motoman_msgs_genlisp)
add_dependencies(motoman_msgs_genlisp motoman_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motoman_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg"
  "${MSG_I_FLAGS}"
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
)
_generate_msg_nodejs(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
)
_generate_msg_nodejs(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
)
_generate_msg_nodejs(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
)
_generate_msg_nodejs(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
)

### Generating Services
_generate_srv_nodejs(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/CmdJointTrajectoryEx.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/industrial_msgs/cmake/../msg/ServiceReturnCode.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
)
_generate_srv_nodejs(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadGroupIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
)
_generate_srv_nodejs(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadMRegister.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
)
_generate_srv_nodejs(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadSingleIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
)
_generate_srv_nodejs(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/SelectTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
)
_generate_srv_nodejs(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteGroupIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
)
_generate_srv_nodejs(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteMRegister.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
)
_generate_srv_nodejs(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteSingleIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
)

### Generating Module File
_generate_module_nodejs(motoman_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(motoman_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(motoman_msgs_generate_messages motoman_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_nodejs _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_nodejs _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_nodejs _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectoryFeedback.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_nodejs _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_nodejs _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/CmdJointTrajectoryEx.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_nodejs _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadGroupIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_nodejs _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadMRegister.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_nodejs _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadSingleIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_nodejs _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/SelectTool.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_nodejs _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteGroupIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_nodejs _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteMRegister.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_nodejs _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteSingleIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_nodejs _motoman_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motoman_msgs_gennodejs)
add_dependencies(motoman_msgs_gennodejs motoman_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motoman_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg"
  "${MSG_I_FLAGS}"
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
)
_generate_msg_py(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
)
_generate_msg_py(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
)
_generate_msg_py(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
)
_generate_msg_py(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
)

### Generating Services
_generate_srv_py(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/CmdJointTrajectoryEx.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/industrial_msgs/cmake/../msg/ServiceReturnCode.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg;/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
)
_generate_srv_py(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadGroupIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
)
_generate_srv_py(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadMRegister.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
)
_generate_srv_py(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadSingleIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
)
_generate_srv_py(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/SelectTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
)
_generate_srv_py(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteGroupIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
)
_generate_srv_py(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteMRegister.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
)
_generate_srv_py(motoman_msgs
  "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteSingleIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
)

### Generating Module File
_generate_module_py(motoman_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(motoman_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(motoman_msgs_generate_messages motoman_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointPoint.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_py _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointState.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_py _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectory.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_py _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointTrajectoryFeedback.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_py _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/msg/DynamicJointsGroup.msg" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_py _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/CmdJointTrajectoryEx.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_py _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadGroupIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_py _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadMRegister.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_py _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/ReadSingleIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_py _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/SelectTool.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_py _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteGroupIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_py _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteMRegister.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_py _motoman_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/etudiant/catkin_ws/src/motoman/motoman_msgs/srv/WriteSingleIO.srv" NAME_WE)
add_dependencies(motoman_msgs_generate_messages_py _motoman_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motoman_msgs_genpy)
add_dependencies(motoman_msgs_genpy motoman_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motoman_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motoman_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET industrial_msgs_generate_messages_cpp)
  add_dependencies(motoman_msgs_generate_messages_cpp industrial_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(motoman_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(motoman_msgs_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motoman_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET industrial_msgs_generate_messages_eus)
  add_dependencies(motoman_msgs_generate_messages_eus industrial_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(motoman_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(motoman_msgs_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motoman_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET industrial_msgs_generate_messages_lisp)
  add_dependencies(motoman_msgs_generate_messages_lisp industrial_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(motoman_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(motoman_msgs_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motoman_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET industrial_msgs_generate_messages_nodejs)
  add_dependencies(motoman_msgs_generate_messages_nodejs industrial_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(motoman_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(motoman_msgs_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motoman_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET industrial_msgs_generate_messages_py)
  add_dependencies(motoman_msgs_generate_messages_py industrial_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(motoman_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(motoman_msgs_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
