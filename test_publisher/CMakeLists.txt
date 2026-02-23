cmake_minimum_required(VERSION 3.8)
project(test_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(TARGET_NAME test_subscriber)

add_executable(${TARGET_NAME} src/test_subscriber.cpp)
add_executable(dummy_publisher src/dummy_publisher.cpp)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)
target_compile_features(dummy_publisher PUBLIC c_std_99 cxx_std_17)

target_link_libraries(${TARGET_NAME} PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

target_link_libraries(dummy_publisher PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

install(TARGETS ${TARGET_NAME} dummy_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
