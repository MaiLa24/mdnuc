cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(plugin_trajectory_follower)


# Encontrar los paquetes de ROS2 necesarios
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-sim7 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport12 REQUIRED)


# Incluir las cabeceras de ROS 2 y Gazebo
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# Crear el plugin
add_library(MyTrajectoryFollowerPlugin SHARED
  src/MyTrajectoryFollowerPlugin.cc
)


# Enlazar con las bibliotecas de Gazebo necesarias
target_link_libraries(MyTrajectoryFollowerPlugin PUBLIC
  gz-sim${gz-sim7_VERSION_MAJOR}::core
  gz-common${gz-common5_VERSION_MAJOR}::gz-common${gz-common5_VERSION_MAJOR}
  gz-math${gz-math7_VERSION_MAJOR}
  gz-transport${gz-transport12_VERSION_MAJOR}::gz-transport${gz-transport12_VERSION_MAJOR}
  gz-plugin${gz-plugin2_VERSION_MAJOR}::gz-plugin${gz-plugin2_VERSION_MAJOR}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)



# Instalar el plugin
install(
  TARGETS MyTrajectoryFollowerPlugin
  DESTINATION lib
)


ament_package()
