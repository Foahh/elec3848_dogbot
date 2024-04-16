cmake_minimum_required(VERSION 3.16)
project(dogbot_servo_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  dogbot_interfaces
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(dogbot_servo_controller_parameters
  src/dogbot_servo_controller_parameter.yaml
)

add_library(dogbot_servo_controller SHARED
  src/dogbot_servo_controller.cpp
)
target_compile_features(dogbot_servo_controller PUBLIC cxx_std_17)
target_include_directories(dogbot_servo_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/dogbot_servo_controller>
)
target_link_libraries(dogbot_servo_controller PUBLIC dogbot_servo_controller_parameters)
ament_target_dependencies(dogbot_servo_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(dogbot_servo_controller PRIVATE "dogbot_servo_controller_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface dogbot_servo_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/dogbot_servo_controller
)
install(TARGETS dogbot_servo_controller dogbot_servo_controller_parameters
  EXPORT export_dogbot_servo_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_dogbot_servo_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()