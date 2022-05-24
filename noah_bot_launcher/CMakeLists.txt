cmake_minimum_required(VERSION 3.0.2)
project(noah_bot_launcher)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
    launch
)

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)