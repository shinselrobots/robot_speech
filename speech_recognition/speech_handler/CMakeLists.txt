cmake_minimum_required(VERSION 2.8.3)
project(speech_handler)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)

catkin_package(
  CATKIN_DEPENDS message_runtime 
)

install(DIRECTORY launch config 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/intent_extraction.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
