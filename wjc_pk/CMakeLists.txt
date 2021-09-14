cmake_minimum_required(VERSION 3.0.2)
project(wjc_pk)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  std_msgs
  message_generation
  pcl_conversions
  pcl_ros
  sensor_msgs
)

find_package(OpenCV 3 REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")


# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Num.msg
  CustomMsg.msg
  CustomPoint.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTwoInts.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wjc_pk
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
#3.head file path，头文件目录
include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "/home/wjc/ros_ws/src/wjc_pk/include"
)

## Add new .cpp here for compiling. Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker wjc_pk_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener wjc_pk_generate_messages_cpp)

add_executable(writer_example src/writer_example.cpp)
target_link_libraries(writer_example ${catkin_LIBRARIES})
add_dependencies(writer_example wjc_pk_generate_messages_cpp)

add_executable(reader_example src/reader_example.cpp)
target_link_libraries(reader_example ${catkin_LIBRARIES})
add_dependencies(reader_example wjc_pk_generate_messages_cpp)

add_executable(ros_bag_reader src/ros_bag_reader.cpp)
target_link_libraries(ros_bag_reader ${catkin_LIBRARIES} ${OpenCV_LIBS} pcl_visualization)
add_dependencies(ros_bag_reader wjc_pk_generate_messages_cpp)

add_executable(pcd_reader src/pcd_reader.cpp)
target_link_libraries(pcd_reader ${catkin_LIBRARIES} ${OpenCV_LIBS} pcl_visualization)
add_dependencies(pcd_reader wjc_pk_generate_messages_cpp)

add_executable(testlab src/testlab.cpp)
target_link_libraries(testlab ${catkin_LIBRARIES} ${OpenCV_LIBS} pcl_visualization)
add_dependencies(testlab wjc_pk_generate_messages_cpp)
