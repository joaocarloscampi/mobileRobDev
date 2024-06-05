cmake_minimum_required(VERSION 3.8)
project(robovisor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options()
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig)

pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE) 
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

#
# instalação do move_base
#

set(${PROJECT_NAME}_MOVE_BASE
    src/robovisor/robot_main.cpp
    src/robovisor/robot_system.cpp
    src/robovisor/robot.cpp   
  )

add_executable(robovisor_node ${${PROJECT_NAME}_MOVE_BASE})
ament_target_dependencies(robovisor_node 
    rclcpp 
    rclcpp_action
    tf2_ros
    nav_msgs
    geometry_msgs
    tf2_geometry_msgs
    std_srvs
)

#
# instalação do zlac
#

set(${PROJECT_NAME}_ZLAC
    src/zlac/zlac.cpp
    src/zlac/robot_main.cpp
    src/zlac/robot_system.cpp
    src/zlac/robot.cpp
)

add_executable(zlac ${${PROJECT_NAME}_ZLAC})
ament_target_dependencies(zlac
    rclcpp 
    rclcpp_action
    tf2_ros
    nav_msgs
    geometry_msgs
    tf2_geometry_msgs
    std_srvs
)

#
# instalação dos scripts
#

add_executable(odom_dump scripts/odom_dump.cpp)
ament_target_dependencies(odom_dump
    rclcpp
    nav_msgs
    geometry_msgs
    std_srvs
)

install(DIRECTORY
  launch
  urdf
  rviz
  maps
  worlds
  config
  src
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

target_include_directories(zlac
  PUBLIC
  ${LIBMODBUS_INCLUDE_DIRS}
)

target_link_libraries(zlac
  ${LIBMODBUS_LIBRARIES}
)

install(TARGETS
  robovisor_node
  zlac
  odom_dump
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
