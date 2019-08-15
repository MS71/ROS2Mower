cmake_minimum_required(VERSION 3.5)

project(r2m_sensors)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(libgps REQUIRED IMPORTED_TARGET libgps)
#pkg_check_modules(libi2c REQUIRED IMPORTED_TARGET libi2c)

add_executable(r2m_laser src/r2m_laser.cpp)
ament_target_dependencies(r2m_laser
  "rclcpp"
  "sensor_msgs"
)

add_executable(r2m_gps src/r2m_gps.cpp)
ament_target_dependencies(r2m_gps
  "rclcpp"
  "sensor_msgs"
)

target_link_libraries(r2m_gps
        gps)

add_executable(r2m_powersupply src/r2m_powersupply.cpp)
ament_target_dependencies(r2m_powersupply
  "rclcpp"
  "std_msgs"
)

target_link_libraries(r2m_powersupply
        i2c)

add_executable(r2m_joint_states src/r2m_joint_states.cpp)
ament_target_dependencies(r2m_joint_states
  "rclcpp"
  "sensor_msgs")

install(TARGETS
  r2m_joint_states
  r2m_laser
  r2m_gps
  r2m_powersupply
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
