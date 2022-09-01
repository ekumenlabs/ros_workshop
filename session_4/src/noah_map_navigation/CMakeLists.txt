cmake_minimum_required(VERSION 2.8.3)

project(noah_map_navigation)

find_package(catkin REQUIRED COMPONENTS
  map_server
  amcl
  move_base_utils
)

catkin_package()

roslaunch_add_file_check(launch)

install(DIRECTORY launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
