cmake_minimum_required(VERSION 2.8.3)
project(ichthus_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_config_msgs
  autoware_build_flags
  amathutils_lib
  roscpp
  std_msgs
  message_generation
  ichthus_can
  )

add_message_files(FILES
  #can_msg.msg
  #obd_msg.msg
)

add_service_files(FILES
  con_msg.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  )

catkin_package(CATKIN_DEPENDS
  autoware_config_msgs
  amathutils_lib
  roscpp
  std_msgs
  message_runtime
  ichthus_can
  )

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})
if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

###########################
### build configuration ###
###########################
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -no-pie${CMAKE_CXX_FALGS}")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

#add_executable(ichthus_controller
#  src/ichthus_controller_core.cpp
#  src/ichthus_controller_node.cpp
#  src/task_graph.cpp
#  )

#target_link_libraries(ichthus_controller
#  ${catkin_LIBRARIES}
#  )

add_executable(ichthus_controller_server
  src/ichthus_controller_server_node.cpp
  src/ichthus_controller_server_core.cpp
  src/controllers.cpp
  src/ethercat.cpp
  src/util.cpp
  )

target_link_libraries(ichthus_controller_server
  ${catkin_LIBRARIES}
  ${CMAKE_SOURCE_DIR}/src/libethercat.a
  )

add_executable(ichthus_controller_client
  src/ichthus_controller_client.cpp
  )

target_link_libraries(ichthus_controller_client
  ${catkin_LIBRARIES}
  )

install(TARGETS ichthus_controller_server ichthus_controller_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
install(PROGRAMS src/ichthus_controller.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
