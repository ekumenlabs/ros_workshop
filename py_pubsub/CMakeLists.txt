cmake_minimum_required(VERSION 3.10.2)
project(py_pubsub)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

catkin_package(CATKIN_DEPENDS rospy std_msgs)

catkin_install_python(PROGRAMS
  scripts/talker.py
  scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_rostest(test/test-talker-listener.launch)
endif()
