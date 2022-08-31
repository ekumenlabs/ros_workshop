cmake_minimum_required(VERSION 3.0.2)
project(move_robot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

########################################
## Configuracion especifica de catkin ##
########################################

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_robot src/move_robot.cpp)
target_link_libraries(move_robot ${catkin_LIBRARIES})

add_executable(move_robot_listener src/move_robot_listener.cpp)
target_link_libraries(move_robot_listener ${catkin_LIBRARIES})

add_executable(move_robot_square src/move_robot_square.cpp)
target_link_libraries(move_robot_square ${catkin_LIBRARIES})
