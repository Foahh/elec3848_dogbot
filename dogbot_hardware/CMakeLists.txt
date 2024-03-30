cmake_minimum_required(VERSION 3.16)
project(dogbot_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#find dependency - serial
set(CMAKE_INSTALL_RPATH /usr/local/lib)
find_package(serial REQUIRED)

## COMPILE
add_library(
  dogbot_hardware
  SHARED
  hardware/dogbot_system.cpp
)
target_compile_features(dogbot_hardware PUBLIC cxx_std_17)
target_include_directories(dogbot_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/dogbot_hardware>
)
ament_target_dependencies(
  dogbot_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  serial
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DOGBOT_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface dogbot_hardware.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/dogbot_hardware
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/dogbot_hardware
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/dogbot_hardware
)
install(TARGETS dogbot_hardware
  EXPORT export_dogbot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_dogbot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
