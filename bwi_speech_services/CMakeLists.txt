cmake_minimum_required(VERSION 2.8.3)
project(bwi_speech_services)

find_package(catkin REQUIRED COMPONENTS
    #roscpp
    rospy
    roslib
    std_msgs
    #std_srvs
    #actionlib
    #actionlib_msgs
    message_generation
)

add_service_files(
    DIRECTORY
    srv
    FILES
    RequestSoundTranscript.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    #actionlib_msgs
    #roscpp
    rospy
    std_msgs
    #std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(kb_script_interpreter
#    src/kb_script_interpreter.cc)

#target_link_libraries(kb_script_interpreter
#  ${catkin_LIBRARIES}
#  mysqlcppconn)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

catkin_install_python(PROGRAMS
  scripts/sound_transcript_server.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sound_transcript_server.py
)

foreach(dir domain launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

#install(TARGETS
#        kb_script_interpreter
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
