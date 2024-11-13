cmake_minimum_required(VERSION 3.8)
project(tof)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)

add_executable(serial_node src/serial_node.cpp)
ament_target_dependencies(serial_node rclcpp std_msgs)
target_link_libraries(serial_node 
    ${Boost_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${visualization_msgs_LIBRARIES}  # 追加
    ${geometry_msgs_LIBRARIES}  # 追加
   )

add_executable(multiarray_to_pointcloud src/multiarray_to_pointcloud.cpp)
ament_target_dependencies(multiarray_to_pointcloud rclcpp std_msgs sensor_msgs visualization_msgs geometry_msgs)
target_link_libraries(multiarray_to_pointcloud ${Boost_LIBRARIES})
# add_executable(pointcloud_publisher src/pointcloud_publisher.cpp)
# ament_target_dependencies(pointcloud_publisher rclcpp std_msgs sensor_msgs visualization_msgs geometry_msgs)

add_executable(servo_controller src/servo_controller.cpp)
ament_target_dependencies(servo_controller rclcpp std_msgs)

add_executable(servo_tf_broadcaster src/servo_tf_broadcaster.cpp)
ament_target_dependencies(servo_tf_broadcaster rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs std_msgs)

install(TARGETS
  serial_node
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS
  multiarray_to_pointcloud
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

install(DIRECTORY launch urdf 
  DESTINATION share/${PROJECT_NAME})  
 #  install(TARGETS
 #  pointcloud_publisher
  # DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS
  servo_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  servo_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# ament_export_dependencies(rclcpp sensor_msgs visualization_msgs geometry_msgs std_msgs)
ament_package()
