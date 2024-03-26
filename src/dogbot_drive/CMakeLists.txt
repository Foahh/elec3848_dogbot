cmake_minimum_required(VERSION 3.16)
project(dogbot_drive LANGUAGES CXX)

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


## COMPILE
add_library(
  dogbot_drive
  SHARED
  hardware/dogbot_system.cpp
)
target_compile_features(dogbot_drive PUBLIC cxx_std_17)
target_include_directories(dogbot_drive PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/dogbot_drive>
)
ament_target_dependencies(
  dogbot_drive PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(dogbot_drive PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DOGBOT_DRIVE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface dogbot_drive.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/dogbot_drive
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/dogbot_drive
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/dogbot_drive
)
install(TARGETS dogbot_drive
  EXPORT export_dogbot_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_dogbot_drive HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
