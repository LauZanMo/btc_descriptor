cmake_minimum_required(VERSION 3.8)
project(btc_desc)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  cv_bridge
  pcl_conversions
  pcl_ros
  sensor_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_library(utils_lib src/utils.cpp)
ament_target_dependencies(utils_lib ${dependencies})

add_library(btc_utils_lib src/btc.cpp)
target_link_libraries(btc_utils_lib
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        tbb
        utils_lib)
ament_target_dependencies(btc_utils_lib ${dependencies})

add_executable(btc_place_recognition example/place_recognition.cpp)
target_link_libraries(btc_place_recognition
        btc_utils_lib)
ament_target_dependencies(btc_place_recognition ${dependencies})

find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(btc_test test/btc_test.cpp)
target_link_libraries(btc_test btc_utils_lib)
ament_target_dependencies(btc_test ${dependencies})

install(TARGETS
  btc_place_recognition
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  utils_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  config
  launch
  rviz_cfg
  poses
  DESTINATION share/${PROJECT_NAME})

ament_package()

