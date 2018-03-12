cmake_minimum_required(VERSION 2.8.3)
project(grounded_dialogue_agent_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    std_msgs
    std_srvs
    actionlib
    actionlib_msgs
    message_generation
)

add_action_files(
  FILES
    ConductDialogue.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    roscpp
    rospy
    std_msgs
)


include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/dialogue_provider
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)