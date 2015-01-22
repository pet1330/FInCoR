cmake_minimum_required(VERSION 2.8.3)
project(camera_setup)
find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rospy sensor_msgs std_msgs message_filters camera_calibration)
find_package(PCL REQUIRED)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(calibrate_cameras src/calibrate.cpp)
target_link_libraries(calibrate_cameras ${catkin_LIBRARIES} ${PCL_LIBRARIES}) 

# add_executable(cal src/cal.cpp)
# target_link_libraries(cal ${catkin_LIBRARIES} ${PCL_LIBRARIES})