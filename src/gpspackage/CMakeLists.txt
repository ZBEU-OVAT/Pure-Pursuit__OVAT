cmake_minimum_required(VERSION 3.8)
project(gpspackage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Bagimliliklari Bul (Dependencies)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

# 2. Sofor Node (Pure Pursuit)
add_executable(pure_pursuit_node src/pure_pursuit.cpp)
# DIKKAT: sensor_msgs buraya eklendi, yoksa RTK GPS kodu derlenmezdi!
# YENI: tf2 eklendi ki Quaternion.h hatasi cozulsun!
ament_target_dependencies(pure_pursuit_node rclcpp nav_msgs geometry_msgs std_msgs std_srvs visualization_msgs sensor_msgs tf2)

# 3. Motor Ustasi Node (Teensy Comm)
add_executable(teensy_comm_node src/teensy_comm.cpp)
ament_target_dependencies(teensy_comm_node rclcpp std_msgs std_srvs)

# 4. Kurulum (Install) Hedefleri - Eski Haritaci (waypoint_logger) buradan kaldirildi
install(TARGETS
  pure_pursuit_node
  teensy_comm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# BU SATIR HER ZAMAN EN ALTTA OLMALI!
ament_package()
