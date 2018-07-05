cmake_minimum_required(VERSION 2.8.3)
project(audio_and_speech_common)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs message_generation)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  speech.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs
)

## Define catkin exports
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp actionlib actionlib_msgs message_generation
#  DEPENDS 
  LIBRARIES audio_and_speech_common
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_definitions(-std=c++11)

## Utility library
add_library(${PROJECT_NAME} SHARED src/audio_and_speech_common.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
