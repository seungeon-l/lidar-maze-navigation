# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 3.8)
project(lidar_maze_navigation)
# Load dependencies required for this package (ROS 2)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(create_fundamentals REQUIRED)
find_package(visualization_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)	   
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_srvs REQUIRED)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
## ROS 2 build type is configured via colcon; leave empty here.

#uncomment if you have defined messages
#add_message_files(FILES
#)
#uncomment if you have defined services
#add_service_files(FILES
#)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)

#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecutePlan.srv"
  "msg/Line.msg"
  "msg/Ransac.msg"
  "msg/Cell.msg"
  "msg/Grid.msg"
  "msg/Pose.msg"
  "msg/Row.msg"
  "srv/PotentialField.srv"
  "msg/ExactPose.msg"
  "msg/Motion.msg"
  "srv/Dijkstra.srv"
  "srv/MoveToPosition.srv"
  "msg/Point.msg"
  "msg/PointArray.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(wanderer src/wanderer.cpp)
ament_target_dependencies(wanderer rclcpp sensor_msgs create_fundamentals)

add_executable(square_no_sensors src/square_no_sensors.cpp)
ament_target_dependencies(square_no_sensors rclcpp sensor_msgs create_fundamentals)

add_executable(square_with_encoders src/square_with_encoders.cpp)
ament_target_dependencies(square_with_encoders rclcpp sensor_msgs create_fundamentals)

add_executable(align src/align.cpp)
ament_target_dependencies(align rclcpp sensor_msgs create_fundamentals)

add_executable(plan_executor_srv src/plan_executor_srv.cpp)
ament_target_dependencies(plan_executor_srv rclcpp sensor_msgs create_fundamentals)

add_executable(smooth_plan_srv src/smooth_plan_srv.cpp)
ament_target_dependencies(smooth_plan_srv rclcpp sensor_msgs create_fundamentals)

add_executable(ransac_node src/ransac_node.cpp)
ament_target_dependencies(ransac_node rclcpp sensor_msgs create_fundamentals)

# localization as library to be used in tracking node
add_library(localization_lib src/localization.cpp)
ament_target_dependencies(localization_lib rclcpp sensor_msgs create_fundamentals std_srvs)

add_executable(localization src/localization.cpp)
ament_target_dependencies(localization rclcpp sensor_msgs create_fundamentals std_srvs)

add_executable(tracking src/tracking.cpp)
ament_target_dependencies(tracking rclcpp sensor_msgs create_fundamentals std_srvs)

add_executable(potential_field src/potential_field.cpp)
ament_target_dependencies(potential_field rclcpp sensor_msgs create_fundamentals)

add_executable(dij_shortest_path src/dij_shortest_path.cpp)
ament_target_dependencies(dij_shortest_path rclcpp sensor_msgs create_fundamentals)

add_executable(dij_gold_order src/dij_gold_order.cpp)
ament_target_dependencies(dij_gold_order rclcpp sensor_msgs create_fundamentals)


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(plan_executor_srv "${cpp_typesupport_target}")
target_link_libraries(smooth_plan_srv "${cpp_typesupport_target}")
target_link_libraries(ransac_node "${cpp_typesupport_target}")
target_link_libraries(localization "${cpp_typesupport_target}")
target_link_libraries(potential_field "${cpp_typesupport_target}")
target_link_libraries(tracking "${cpp_typesupport_target}")
target_link_libraries(localization_lib "${cpp_typesupport_target}")
target_link_libraries(localization "${cpp_typesupport_target}")
target_link_libraries(tracking localization_lib "${cpp_typesupport_target}")
target_link_libraries(dij_shortest_path "${cpp_typesupport_target}")
target_link_libraries(dij_gold_order "${cpp_typesupport_target}")



#rosidl_target_interfaces(plan_executor_srv ${PROJECT_NAME} "rosidl_typesupport_cpp")
#rosidl_target_interfaces(smooth_plan_srv ${PROJECT_NAME} "rosidl_typesupport_cpp")


#add_executable(plan_executor_client src/plan_executor_client.cpp)
#ament_target_dependencies(plan_executor_client rclcpp sensor_msgs create_fundamentals)
#rosidl_target_interfaces(plan_executor_client ${PROJECT_NAME} "rosidl_typesupport_cpp")
# add_executable(example_moving src/example_moving.cpp)
# ament_target_dependencies(example_moving rclcpp sensor_msgs create_fundamentals)

#target_link_libraries(example ${PROJECT_NAME})

## Generate added messages and services with any dependencies listed here
#generate_messages(
#    DEPENDENCIES std_msgs create_fundamentals
#)

install(PROGRAMS
  src/example_laser.py
  src/example_moving.py
  src/example_song.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/publish_map.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME publish_map
)


install(FILES
  src/map.txt
  src/gold.txt 
  src/pickup.txt 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  wanderer
  square_no_sensors
  square_with_encoders
  align
  plan_executor_srv
  smooth_plan_srv
  ransac_node
  localization
  tracking
  potential_field
  dij_shortest_path
  dij_gold_order
  # plan_executor_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
