cmake_minimum_required(VERSION 2.8.3)
project(anrobot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    kdl_conversions
    kdl_parser
    roscpp
    rospy
    roslib
    sensor_msgs
    std_msgs
    visualization_msgs
    std_srvs
    tf
    urdf
    message_generation
    )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FwdKinematics.srv
  InvKinematics.srv
  SetTarget.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs sensor_msgs std_srvs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES anrobot
    CATKIN_DEPENDS roscpp rospy geometry_msgs message_runtime sensor_msgs std_msgs std_srvs tf urdf visualization_msgs
    #  DEPENDS system_lib
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
    ${catkin_INCLUDE_DIRS}
    )

## Declare a C++ library
add_library(trajectory
    src/trajectory.cpp
    )

set(LIBS ${catkin_LIBRARIES} trajectory)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(anrobot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(trajectory ${anrobot_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(dh_to_urdf src/dh_to_urdf.cpp)
target_link_libraries(dh_to_urdf ${LIBS})

add_executable(gen_trajectory src/gen_trajectory.cpp)
target_link_libraries(gen_trajectory ${LIBS})

add_executable(kdl_fwd_kinematics src/kdl_fwd_kinematics.cpp)
target_link_libraries(kdl_fwd_kinematics ${LIBS})
add_dependencies(kdl_fwd_kinematics ${anrobot_EXPORTED_TARGETS})

add_executable(simple_fwd_kinematics src/simple_fwd_kinematics.cpp)
target_link_libraries(simple_fwd_kinematics ${LIBS})
add_dependencies(simple_fwd_kinematics  ${anrobot_EXPORTED_TARGETS})

add_executable(set_param src/set_param.cpp)
target_link_libraries(set_param ${LIBS})

add_executable(inv_kinematics src/inv_kinematics.cpp)
target_link_libraries(inv_kinematics ${LIBS})
add_dependencies(inv_kinematics ${anrobot_EXPORTED_TARGETS})

add_executable(trajectory_test src/trajectory_test.cpp)
target_link_libraries(trajectory_test ${LIBS})
add_dependencies(trajectory_test ${anrobot_EXPORTED_TARGETS})

add_executable(inv_gen_trajectory src/inv_gen_trajectory.cpp)
target_link_libraries(inv_gen_trajectory ${LIBS})
add_dependencies(inv_gen_trajectory ${anrobot_EXPORTED_TARGETS})

add_executable(trajectory_marker_pub src/trajectory_marker_pub.cpp)
target_link_libraries(trajectory_marker_pub ${LIBS})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(anrobot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS dh_to_urdf simple_fwd_kinematics kdl_fwd_kinematics set_param gen_trajectory inv_gen_trajectory inv_kinematics trajectory_marker_pub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.launch"
    )

install(
    DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.urdf"
    )

install(
    DIRECTORY rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.rviz"
    )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_anrobot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
