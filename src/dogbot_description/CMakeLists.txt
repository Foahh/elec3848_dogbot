cmake_minimum_required(VERSION 3.5)
project(dogbot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY dogbot/urdf dogbot/rviz
  DESTINATION share/${PROJECT_NAME}/dogbot
)

ament_package()