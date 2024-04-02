cmake_minimum_required(VERSION 3.8)
project(bluesea2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Get ROS2 version information (initial letter) from environment variables
string(SUBSTRING $ENV{ROS_DISTRO} 0 1 tmp)
#MESSAGE("ROS_DISTRO2:${tmp}")

string(COMPARE LESS_EQUAL ${tmp} "e" flag)
#MESSAGE("flag:${flag}")


if( ${flag} STREQUAL 1)
add_definitions(-DROS_DISTRO_E)
else()
add_definitions(-DROS_DISTRO_F)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME} "srv/Control.srv" "srv/DefenceZone.srv")
# ament_export_dependencies(rosidl_default_runtime)
include_directories(${sensor_msgs_INCLUDE_DIRS})

set(BLUESEA_SDK_PATH "./sdk/")
FILE(GLOB BLUESEA_SDK_SRC  "${BLUESEA_SDK_PATH}/src/*.cpp", "${BLUESEA_SDK_PATH}/src/*.c")

add_executable(${PROJECT_NAME}_node  ./src/node.cpp ${BLUESEA_SDK_SRC})
ament_target_dependencies(${PROJECT_NAME}_node
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "std_srvs"
)
add_executable(${PROJECT_NAME}_client ./src/client.cpp)
ament_target_dependencies(${PROJECT_NAME}_client rclcpp std_srvs)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_client ${catkin_LIBRARIES})



target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(${PROJECT_NAME}_client PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_client
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

