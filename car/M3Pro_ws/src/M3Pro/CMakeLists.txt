cmake_minimum_required(VERSION 3.5)

project(M3Pro)

find_package(ament_cmake REQUIRED)


install(
  DIRECTORY launch meshes config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
