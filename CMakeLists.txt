cmake_minimum_required(VERSION 3.8)
project(delta_lidar)

set(delta_lidar_SDK_PATH "sdk")

FILE(GLOB source_files 
  "${delta_lidar_SDK_PATH}/src/*.cpp"
)
set (CMAKE_CXX_FLAGS "-lrt")

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_export_include_directories(include)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(node src/node.cpp ${source_files})
target_include_directories(node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
include_directories(
  ${delta_lidar_SDK_PATH}/include
  ${delta_lidar_SDK_PATH}/src
  ${rclcpp_INCLUDE_DIRS}
)
target_compile_features(node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(node rclcpp sensor_msgs)
#target_link_libraries( -lrt)

install(TARGETS node
  DESTINATION lib/${PROJECT_NAME})

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
ament_export_dependencies(${DEPENDENCIES})
ament_package()
