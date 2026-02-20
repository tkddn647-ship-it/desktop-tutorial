cmake_minimum_required(VERSION 3.8)
project(localization_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cartographer REQUIRED)
find_package(cartographer_ros REQUIRED)

ament_package()

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
