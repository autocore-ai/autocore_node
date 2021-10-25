cmake_minimum_required(VERSION 3.5)
project(autocore_node)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED src/node.cpp)

ament_auto_package()
