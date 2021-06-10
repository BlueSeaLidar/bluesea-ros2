cmake_minimum_required(VERSION 3.8)
project(bluesea2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(bluesea_node src/bluesea_node.cpp src/parser.cpp src/uart_reader.cpp src/udp_reader.cpp src/tcp_reader.cpp src/uart.c)
ament_target_dependencies(bluesea_node
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
)


target_include_directories(bluesea_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(bluesea_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS bluesea_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params 
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
