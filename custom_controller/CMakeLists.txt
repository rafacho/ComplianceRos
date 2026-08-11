cmake_minimum_required(VERSION 3.8...3.28)
project(custom_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(joint_limits REQUIRED)
find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_messages REQUIRED)
find_package(custom_services REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/actuator_position_controller.cpp
  src/actuator_velocity_controller.cpp
  src/controller_utils.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_interface
  pluginlib
  realtime_tools
  joint_limits
  urdf
  sensor_msgs
  std_srvs
  custom_messages
  custom_services
)

# ROS distro switch note (2026-08-06): ament_target_dependencies() was
# removed from ament_cmake in Lyrical Luth (it built fine on Jazzy). The
# current recommended replacement is linking each dependency's own exported
# target list (populated into <pkg>_TARGETS by ament_cmake_export_targets
# when find_package(<pkg>) is called), rather than the old convenience macro.
foreach(_dep ${THIS_PACKAGE_INCLUDE_DEPENDS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${${_dep}_TARGETS})
endforeach()

pluginlib_export_plugin_description_file(controller_interface custom_controller_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
