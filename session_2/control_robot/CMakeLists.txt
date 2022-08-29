cmake_minimum_required(VERSION 3.0.2)
project(control_robot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)

########################################
## Configuracion especifica de catkin ##
########################################

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(proportional_controller src/proportional_controller.cpp)
target_link_libraries(proportional_controller ${catkin_LIBRARIES})

add_executable(control_robot src/control_robot.cpp)
target_link_libraries(control_robot
  proportional_controller
  ${catkin_LIBRARIES}
)
