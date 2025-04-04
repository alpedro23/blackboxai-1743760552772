cmake_minimum_required(VERSION 3.0.2)
project(drone_control_system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros
  mavros_msgs
  tf2_ros
  message_generation
  image_transport
  cv_bridge
)

find_package(wiringPi REQUIRED)

add_message_files(
  FILES
  SensorFusion.msg
  ServoCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drone_control_system
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    mavros_msgs
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_processor src/waypoint_processor.cpp)
target_link_libraries(waypoint_processor ${catkin_LIBRARIES})

add_executable(sensor_processor src/sensor_processor.cpp)
target_link_libraries(sensor_processor ${catkin_LIBRARIES})

add_executable(servo_control src/servo_control.cpp)
target_link_libraries(servo_control ${catkin_LIBRARIES} wiringPi)

add_executable(video_streamer src/video_streamer.cpp)
target_link_libraries(video_streamer ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(object_tracker src/object_tracker.cpp)
target_link_libraries(object_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(soil_sampler src/soil_sampler.cpp)
target_link_libraries(soil_sampler ${catkin_LIBRARIES} wiringPi)

install(TARGETS waypoint_processor sensor_processor servo_control video_streamer object_tracker soil_sampler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)