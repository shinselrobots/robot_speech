cmake_minimum_required(VERSION 2.8.3)
project(speech_recognition_common)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    std_msgs 
    message_generation
)

#catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
    FILES
    speech_recognition_intent.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
#    speech_recognition_common
)



## Define catkin exports
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS 
#  DEPENDS 
#  LIBRARIES speech_recognition_common
)

# include_directories(include ${catkin_INCLUDE_DIRS})

# add_definitions(-std=c++11)

## Utility library
#add_library(${PROJECT_NAME} SHARED src/audio_and_speech_common.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
