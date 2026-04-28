cmake_minimum_required(VERSION 3.5)
project(encoder_odometry)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# بناء النود
add_executable(encoder_odom_node src/encoder_odom_node.cpp)
ament_target_dependencies(encoder_odom_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# 1. تثبيت الملف التنفيذي للنود
install(TARGETS
  encoder_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

# 2. تثبيت مجلد الـ launch (لازم قبل ament_package)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
#rviz
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# 3. ده لازم يكون آخر سطر في الملف
ament_package()