cmake_minimum_required(VERSION 3.8)
project(wj_716N_lidar)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_lint_auto)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

# ################################################
# ## Declare ROS dynamic reconfigure parameters ##
# ################################################

# ## To declare and build dynamic reconfigure parameters within this
# ## package, follow these steps:
# ## * In the file package.xml:
# ##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# ## * In this file (CMakeLists.txt):
# ##   * add "dynamic_reconfigure" to
# ##     find_package(catkin REQUIRED COMPONENTS ...)
# ##   * uncomment the "generate_dynamic_reconfigure_options" section below
# ##     and list every .cfg file to be processed

# ## Generate dynamic reconfigure parameters in the 'cfg' folder
#  generate_dynamic_reconfigure_options(
#     cfg/wj_716N_lidar.cfg

#   )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wj_716N_lidar
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
#)
ament_package(

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations#include_directories(
#   include ${catkin_INCLUDE_DIRS}
# # include
#   ${catkin_INCLUDE_DIRS}
# )

add_executable(wj_716N_lidar
   src/wj_716N_lidar_01.cpp src/async_client.cpp src/wj_716N_lidar_protocol.cpp )

target_include_directories(wj_716N_lidar PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)

target_compile_features(wj_716N_lidar PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(wj_716N_lidar
  "rclcpp"
  "std_msgs"
  "rcl_interfaces"
  "sensor_msgs"
  "Boost"
)
install(TARGETS wj_716N_lidar
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


