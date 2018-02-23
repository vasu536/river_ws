# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "phidgets_ik: 0 messages, 1 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(phidgets_ik_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/vasu536/catkin_ws/src/phidgets_drivers/phidgets_ik/srv/SetDigitalOutput.srv" NAME_WE)
add_custom_target(_phidgets_ik_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phidgets_ik" "/home/vasu536/catkin_ws/src/phidgets_drivers/phidgets_ik/srv/SetDigitalOutput.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(phidgets_ik
  "/home/vasu536/catkin_ws/src/phidgets_drivers/phidgets_ik/srv/SetDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phidgets_ik
)

### Generating Module File
_generate_module_cpp(phidgets_ik
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phidgets_ik
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(phidgets_ik_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(phidgets_ik_generate_messages phidgets_ik_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vasu536/catkin_ws/src/phidgets_drivers/phidgets_ik/srv/SetDigitalOutput.srv" NAME_WE)
add_dependencies(phidgets_ik_generate_messages_cpp _phidgets_ik_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(phidgets_ik_gencpp)
add_dependencies(phidgets_ik_gencpp phidgets_ik_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS phidgets_ik_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(phidgets_ik
  "/home/vasu536/catkin_ws/src/phidgets_drivers/phidgets_ik/srv/SetDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/phidgets_ik
)

### Generating Module File
_generate_module_eus(phidgets_ik
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/phidgets_ik
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(phidgets_ik_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(phidgets_ik_generate_messages phidgets_ik_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vasu536/catkin_ws/src/phidgets_drivers/phidgets_ik/srv/SetDigitalOutput.srv" NAME_WE)
add_dependencies(phidgets_ik_generate_messages_eus _phidgets_ik_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(phidgets_ik_geneus)
add_dependencies(phidgets_ik_geneus phidgets_ik_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS phidgets_ik_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(phidgets_ik
  "/home/vasu536/catkin_ws/src/phidgets_drivers/phidgets_ik/srv/SetDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phidgets_ik
)

### Generating Module File
_generate_module_lisp(phidgets_ik
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phidgets_ik
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(phidgets_ik_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(phidgets_ik_generate_messages phidgets_ik_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vasu536/catkin_ws/src/phidgets_drivers/phidgets_ik/srv/SetDigitalOutput.srv" NAME_WE)
add_dependencies(phidgets_ik_generate_messages_lisp _phidgets_ik_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(phidgets_ik_genlisp)
add_dependencies(phidgets_ik_genlisp phidgets_ik_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS phidgets_ik_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(phidgets_ik
  "/home/vasu536/catkin_ws/src/phidgets_drivers/phidgets_ik/srv/SetDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/phidgets_ik
)

### Generating Module File
_generate_module_nodejs(phidgets_ik
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/phidgets_ik
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(phidgets_ik_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(phidgets_ik_generate_messages phidgets_ik_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vasu536/catkin_ws/src/phidgets_drivers/phidgets_ik/srv/SetDigitalOutput.srv" NAME_WE)
add_dependencies(phidgets_ik_generate_messages_nodejs _phidgets_ik_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(phidgets_ik_gennodejs)
add_dependencies(phidgets_ik_gennodejs phidgets_ik_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS phidgets_ik_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(phidgets_ik
  "/home/vasu536/catkin_ws/src/phidgets_drivers/phidgets_ik/srv/SetDigitalOutput.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phidgets_ik
)

### Generating Module File
_generate_module_py(phidgets_ik
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phidgets_ik
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(phidgets_ik_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(phidgets_ik_generate_messages phidgets_ik_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vasu536/catkin_ws/src/phidgets_drivers/phidgets_ik/srv/SetDigitalOutput.srv" NAME_WE)
add_dependencies(phidgets_ik_generate_messages_py _phidgets_ik_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(phidgets_ik_genpy)
add_dependencies(phidgets_ik_genpy phidgets_ik_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS phidgets_ik_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phidgets_ik)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phidgets_ik
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/phidgets_ik)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/phidgets_ik
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phidgets_ik)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phidgets_ik
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/phidgets_ik)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/phidgets_ik
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phidgets_ik)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phidgets_ik\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phidgets_ik
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
