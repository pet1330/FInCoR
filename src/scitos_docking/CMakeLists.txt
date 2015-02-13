cmake_minimum_required(VERSION 2.8.3)
project(scitos_docking)

find_package(catkin REQUIRED COMPONENTS 
    dynamic_reconfigure
    geometry_msgs
    image_transport 
    message_generation
    roscpp
    rospy 
    sensor_msgs
    std_msgs
    tf)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(GSL REQUIRED gsl)
include_directories(${GSL_INCLUDE_DIRS})
link_directories(${GSL_LIBRARY_DIRS})

add_service_files(
   FILES
   Charging.srv
)

add_action_files(
   DIRECTORY action 
   FILES Charging.action
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  actionlib_msgs
)

catkin_package(
   INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Libraries
add_library(CCircleDetect src/scitos_docking/CCircleDetect.cpp)
add_library(CRawImage src/scitos_docking/CRawImage.cpp)
add_library(CTransformation src/scitos_docking/CTransformation.cpp)

add_dependencies(CTransformation ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#add_dependencies(CChargingClient ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#add_dependencies(CPtuClient ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(CTransformation ${GSL_LIBRARIES} ${catkin_LIBRARIES})

# Executables
add_executable(visual_charging src/scitos_docking/docking.cpp)

add_dependencies(visual_charging ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries (visual_charging 
    CCircleDetect
    CRawImage
    CTransformation
    ${catkin_LIBRARIES}
    )
#target_link_libraries (visual_charging_client  ${catkin_LIBRARIES})
#target_link_libraries (simple_move_client  ${catkin_LIBRARIES})
#target_link_libraries (simple_ptu_client  ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS 
    CCircleDetect 
    CRawImage 
    CTransformation 
    visual_charging 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

 install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

install(DIRECTORY doc
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
